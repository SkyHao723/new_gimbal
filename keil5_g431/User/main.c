#include "stm32g4xx_hal.h"
#include "board.h"
#include "board_config.h"
#include "can_transport.h"
#include "as5600.h"
#include "current_sense.h"
#include "current_limit.h"
#include "bench_current.h"
#include "motor.h"
#include "motor_config.h"
#include "diagnostics.h"
#include <stdio.h>

static Motor motor;
static CurrentLimit limit;
volatile G431Diagnostics g431_diagnostics;

static void publish_current_diagnostics(void)
{
    g431_diagnostics.cs_valid = (uint32_t)current_sense_valid();
    g431_diagnostics.cs_samples = current_sense_samples();
    g431_diagnostics.cs_trips = current_sense_trips();
    g431_diagnostics.cs_blind = limit.blind;
    g431_diagnostics.cs_peak = limit.peak_a;
    g431_diagnostics.cs_allowed = limit.allowed_v;
    g431_diagnostics.cs_thermal = limit.thermal;
    g431_diagnostics.cs_amp_mid = current_sense_valid() ? current_sense_amp(1)
                                                        : current_sense_offset(1);
    g431_diagnostics.cs_trig = (uint32_t)current_sense_trigger();
    g431_diagnostics.cs_probe_a = current_sense_raw(0);
    g431_diagnostics.cs_probe_b = current_sense_raw(1);
    g431_diagnostics.cs_probe_c = current_sense_raw(2);
}

int main(void)
{
    char command[BOARD_COMMAND_LENGTH];
    uint32_t last_snapshot = 0u;
    uint32_t last_limit_us;
    uint32_t last_retry = 0u;
    float requested_voltage = MOTOR_VOLTAGE_LIMIT;
    int sense_ok;

    g431_diagnostics.magic = 0x47343331u;
    g431_diagnostics.stage = 1u;
    board_init();
    g431_diagnostics.core_clock = SystemCoreClock;
    g431_diagnostics.stage = 2u;
#if G431_BENCH_MODE
    /* Bench builds run without a CAN adapter, so skip the transport: a bus-off
       would otherwise keep requesting a stop while the motor is exercised. */
#else
    if (!can_transport_init()) {
        board_driver_set(0);
        g431_diagnostics.stage = 0xe001u;
        for (;;) board_led((board_millis() / 100u) & 1u);
    }
#endif
    printf("\r\nKeil5 / G431CB + DRV8316C + AS5600\r\n");
    printf("SysClk=%lu PWM=PA8/9/10 nSLEEP=PB0 DRVOFF->PGND nFAULT=PB2\r\n", (unsigned long)SystemCoreClock);
    printf("CAN=PB8/9 STB=PB5 500k RX=0x%03X TX=0x%03X SW-I2C=PB6/7 LED=PA5\r\n",
           BOARD_CAN_COMMAND_ID, BOARD_CAN_REPLY_ID);
    as5600_init();
    g431_diagnostics.stage = 3u;

    /* Standstill offset calibration: the driver is awake, MOE is clear, so the
       three SOx channels only carry their own bias. */
    if (!board_driver_wake())
        printf("DRV faults: nFAULT low (VM missing or latched fault)\r\n");
    sense_ok = current_sense_init();
    current_limit_init(&limit);
    if (sense_ok) {
        printf("I-sense OK off=%.0f/%.0f/%.0f cnt  %.1fA FS  %.4fV/cnt\r\n",
               (double)current_sense_offset(0), (double)current_sense_offset(1),
               (double)current_sense_offset(2),
               (double)(CS_ADC_COUNTS * CS_AMP_PER_COUNT * 0.5f),
               (double)CS_V_PER_COUNT);
    } else {
        printf("I-sense FAILED (%s): bias=%.0f/%.0f/%.0f cnt, voltage pinned to floor %.2fV\r\n",
               current_sense_error(),
               (double)current_sense_offset(0), (double)current_sense_offset(1),
               (double)current_sense_offset(2),
               (double)CURRENT_LIMIT_VOLTAGE_FLOOR);
    }
    printf("I-limit rated=%.2fA fold>%.2fA trip>%.2fA floor=%.2fV ceil=%.2fV\r\n",
           (double)CURRENT_LIMIT_RATED_A, (double)CURRENT_LIMIT_SOFT_A,
           (double)CURRENT_LIMIT_TRIP_A, (double)CURRENT_LIMIT_VOLTAGE_FLOOR,
           (double)MOTOR_MAX_VOLTAGE);
    printf("I-probe raw=%u/%u/%u cnt trig=%d (1=TIM1_CC4 2=TIM1_TRGO)\r\n",
           (unsigned)current_sense_probe_raw(0), (unsigned)current_sense_probe_raw(1),
           (unsigned)current_sense_probe_raw(2), current_sense_trigger());

    motor_init(&motor);
    g431_diagnostics.stage = 4u;
#if G431_BENCH_CURRENT_TEST
    /* Bench build: no CAN adapter is available, so validate the current chain
       here. Results land in bench_current_result[] and are read over SWD. */
    bench_current_run();
    printf("bench current test done: worst=%.3fA trips=%.0f code=%.0f\r\n",
           (double)bench_current_result[24], (double)bench_current_result[25],
           (double)bench_current_result[28]);
#endif
#if G431_CLOSEDLOOP_TEST
    /* Bring-up step 3: how far the voltage ceiling can be raised before phase
       current (and heat) becomes a concern. 3 rad steps, ceiling swept over
       2.5 / 4 / 6 / 8 V; one summary row per ceiling in bench_cl_table[]. */
    (void)bench_cl_run(&motor, 2.5f, 3.0f, 800u, 2000u, 3);
    bench_cl_save(0);
    (void)bench_cl_run(&motor, 4.0f, 3.0f, 800u, 2000u, 3);
    bench_cl_save(1);
    (void)bench_cl_run(&motor, 6.0f, 3.0f, 800u, 2000u, 3);
    bench_cl_save(2);
    (void)bench_cl_run(&motor, 8.0f, 3.0f, 800u, 2000u, 3);
    bench_cl_save(3);
#endif
#if G431_ALIGN_TEST
    /* Bring-up step 2a: settle the rotor onto a known d-axis at three electrical
       angles so the zero-electric offset and direction can be derived. Results
       land in bench_align_result[] and are read over SWD. */
    (void)bench_align_run(0, 1.5f, 0.0f, 2500u);
    (void)bench_align_run(1, 1.5f, 0.0f, 2500u);
    (void)bench_align_run(2, 1.5f, 1.5707963f, 2500u);
    (void)bench_align_run(3, 1.5f, 3.1415927f, 2500u);
#endif
#if G431_SPIN_TEST
    /* Bring-up step 1: open-loop spin once at boot. Results land in
       bench_spin_result[] and are read over SWD. */
    (void)bench_spin_run(0.8f, 6.2831853f, 4000u);
    printf("spin test: code=%.0f delta=%.3f rad worst=%.3fA\r\n",
           (double)bench_spin_result[4], (double)bench_spin_result[10],
           (double)bench_spin_result[3]);
#endif
#if G431_CAPTURE_TEST
    /* Jitter diagnosis: cap the trajectory speed so the capture window sits in
       the constant-velocity part of a long move, then record the loop waveform
       into bench_capture[] for an SWD dump + FFT. */
    (void)motor_command(&motor, "L2");
    (void)bench_capture_run(&motor, 2.5f, 6.0f, 600u, 2600u);
    printf("capture: code=%.0f n=%.0f uq_last=%.3f worst=%.3fA\r\n",
           (double)bench_capture_meta[0], (double)bench_capture_meta[1],
           (double)bench_capture[(int)bench_capture_meta[1] * BENCH_CAPTURE_COLS - 1],
           (double)bench_capture_meta[6]);
#endif
#if G431_CURRENT_LOCK_TEST
    /* One-shot gain calibration: repeatedly hold A-B at 0.6 V for 30 s with a
       20 s quiet gap, so a multimeter can be clipped on at any time. Read
       bench_lock_result[] over SWD afterwards. */
    printf("lock test: A-B 0.6V for 30s / 20s pause, repeating\r\n");
    for (;;) {
        (void)bench_lock_run(0.3f, 30000u);
        HAL_Delay(20000u);
    }
#endif
    printf("commands: E D T<n> V<n> L<n> R ? H (single CAN frame, T/V/L + float32 LE)\r\n");
    printf("V is a request: the current limiter may hold it lower (see ? )\r\n");
    last_limit_us = board_micros();
    for (;;) {
        uint32_t now_us;
        float dt;
        int sampling_ok;
        int stop = board_take_stop();
        if (stop) {
            motor_disable(&motor);
            current_sense_arm(0);
            board_discard_commands();
            /* nSLEEP low means we put the driver to sleep ourselves, so its
               low nFAULT is an artifact of that, not a real fault report. */
            if ((stop & BOARD_STOP_DRIVER) && (GPIOB->IDR & DRIVER_SLEEP_PIN))
                printf("FAULT nFAULT low, motor DISABLED\r\n");
            if (stop & BOARD_STOP_CAN) printf("FAULT CAN, motor DISABLED\r\n");
            if (stop & BOARD_STOP_CURRENT)
                printf("FAULT overcurrent pk=%.2fA, motor DISABLED\r\n", (double)limit.peak_a);
            printf("EN OFF (stop)\r\n");
        }

        /* Current budget: fold the voltage request back before the next step. */
        now_us = board_micros();
        dt = (float)(uint32_t)(now_us - last_limit_us) * 1e-6f;
        last_limit_us = now_us;
        if (dt > 0.05f) dt = 0.05f;
        /* A sleeping or faulted driver leaves the SOx pins without a valid
           bias, so treat that as "no measurement" rather than "zero current". */
        sampling_ok = current_sense_recent() &&
                      (board_pwm_low_window() >= CS_CONV_COUNTS) &&
                      ((GPIOB->IDR & DRIVER_FAULT_PIN) != 0u);
        current_sense_arm(motor.enabled);
        if (!motor.enabled && ((GPIOB->IDR & DRIVER_FAULT_PIN) != 0u) &&
            !(TIM1->BDTR & TIM_BDTR_MOE)) {
            /* Bridge off: no current can flow, so the sampled value is the bias. */
            current_sense_track_zero();
        }
        current_limit_update(&limit, &current_limit_defaults,
                             current_sense_peak(), current_sense_mean_square(),
                             dt, sampling_ok, motor.enabled);
        if (limit.tripped) {
            current_limit_clear_trip(&limit);
            board_request_stop(BOARD_STOP_CURRENT);
        }
        motor.voltage_limit = current_limit_voltage(&limit, &current_limit_defaults,
                                                    requested_voltage);

        motor_step(&motor);
        if (board_get_command(command)) {
            motor_command(&motor, command);
            if (command[0] == '?') {
                printf("CAN txdrop=%lu nFAULT=%u\r\n", can_transport_dropped_bytes(),
                       (GPIOB->IDR & DRIVER_FAULT_PIN) ? 1u : 0u);
                printf("I pk=%.2fA allowed=%.2fV req=%.2fV th=%.2f smp=%lu trips=%lu ok=%u\r\n",
                       (double)limit.peak_a, (double)limit.allowed_v,
                       (double)requested_voltage, (double)limit.thermal,
                       (unsigned long)current_sense_samples(),
                       (unsigned long)current_sense_trips(),
                       (unsigned)sampling_ok);
            } else if (command[0] == 'V') {
                /* motor_command already clamped the request to MOTOR_MAX_VOLTAGE. */
                requested_voltage = motor.voltage_limit;
            }
        }
        can_transport_service();
        board_led((board_millis() / 400u) & 1u);
        ++g431_diagnostics.loops;
        /* Recovery path while the motor is idle: a fault latch (or a UVLO at
           power-up) clears on a nSLEEP low -> high transition, so wake the
           driver and re-learn the bias. Also retried when the bias is stale
           but the device happens to be reporting a fault, otherwise the offsets
           would stay frozen at whatever they were when the driver last slept.
           Calibration and zero tracking both need MOE clear. */
        if (!motor.enabled && current_sense_hw_ok() &&
            (uint32_t)(board_millis() - last_retry) >= 500u &&
            (!current_sense_valid() ||
             (GPIOB->IDR & DRIVER_FAULT_PIN) == 0u)) {
            last_retry = board_millis();
            if (board_driver_wake() && current_sense_init()) {
                printf("I-sense recalibrated off=%.0f/%.0f/%.0f cnt\r\n",
                       (double)current_sense_offset(0),
                       (double)current_sense_offset(1),
                       (double)current_sense_offset(2));
            }
        }
        if ((uint32_t)(board_millis() - last_snapshot) >= 50u) {
            last_snapshot = board_millis();
            g431_diagnostics.uptime_ms = last_snapshot;
            g431_diagnostics.enabled = motor.enabled;
            g431_diagnostics.sleep_high = (GPIOB->ODR & DRIVER_SLEEP_PIN) ? 1u : 0u;
            g431_diagnostics.off_high = (GPIOB->ODR & DRIVER_OFF_PIN) ? 1u : 0u;
            g431_diagnostics.fault_high = (GPIOB->IDR & DRIVER_FAULT_PIN) ? 1u : 0u;
            g431_diagnostics.sensor_valid = motor.sensor.valid;
            g431_diagnostics.sensor_errors = motor.sensor.errors;
            g431_diagnostics.can_psr = FDCAN1->PSR;
            g431_diagnostics.can_ecr = FDCAN1->ECR;
            g431_diagnostics.tx_dropped = can_transport_dropped_bytes();
            g431_diagnostics.angle = motor.angle;
            g431_diagnostics.velocity = motor.velocity;
            publish_current_diagnostics();
        }
    }
}
