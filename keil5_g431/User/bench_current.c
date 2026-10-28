/* Bench validation of the three low-side current channels.
 *
 * Why this exists: the board has no USB-CAN adapter on the bench, so the
 * CAN-driven application cannot be told to enable the bridge. This routine
 * enables the bridge on its own for a bounded time and applies a very small
 * DC voltage between two phases (a "DC lock" test). The current is low enough
 * that the motor cannot overheat, yet large enough to check the whole chain:
 *
 *   Kirchhoff   the two driven phases must read +I and -I, the third ~0
 *   Symmetry    both phases of a pair must show the same |I|
 *   Gain        the same channel is exercised with +I and -I, and every phase
 *               appears in two different pairs, so a gain error shows up as a
 *               mismatch between the two pairs that share that channel
 *   Linearity   I must scale with the applied voltage
 *
 * Test 0 runs at 0 V with the bridge enabled: it measures the bias shift that
 * enabling the outputs adds. Everything else is judged after subtracting it.
 *
 * The results are written to bench_current_result[] so they can be read over
 * SWD without any serial or CAN link.
 *
 * Safety: every step holds for at most BENCH_HOLD_MS, the peak current is
 * checked by the ADC interrupt (2.6 A soft / 4.5 A hard trip), and the bridge
 * is disabled after every step. 0.6 V across a 3.8 ohm winding is 0.16 A and
 * 0.1 W, far below any thermal limit.
 */
#include "bench_current.h"
#include "stm32g4xx_hal.h"
#include "board.h"
#include "board_config.h"
#include "current_sense.h"
#include "as5600.h"
#include "foc_math.h"
#include <stdio.h>
#include <math.h>

#define BENCH_SUPPLY_V    24.0f
#define BENCH_HOLD_MS     300u
#define BENCH_SETTLE_MS   60u    /* ignore the enable transient */
#define BENCH_REST_MS     400u
#define BENCH_TEST_V      0.60f
/* Abort the long DC lock if the measured current stays above this for more
   than a few dozen samples. The lock voltage is chosen so the expected current
   is ~0.16 A, so 1.2 A already means something is badly wrong. */
#define BENCH_LOCK_TRIP_A 1.20f

/* Phase voltages in volts, summing to zero; duty = 0.5 + v / Vbus, so the
   terminal-to-terminal voltage between two phases equals v_a - v_b. */
typedef struct {
    float a;
    float b;
    float c;
} bench_step;

static const bench_step bench_steps[BENCH_STEPS] = {
    {0.0f, 0.0f, 0.0f},                       /* 0: bias with bridge on */
    {BENCH_TEST_V / 2, -BENCH_TEST_V / 2, 0}, /* 1: A-B + */
    {-BENCH_TEST_V / 2, BENCH_TEST_V / 2, 0}, /* 2: A-B - */
    {0.0f, BENCH_TEST_V / 2, -BENCH_TEST_V / 2}, /* 3: B-C + */
    {0.0f, -BENCH_TEST_V / 2, BENCH_TEST_V / 2}, /* 4: B-C - */
    {-BENCH_TEST_V / 2, 0.0f, BENCH_TEST_V / 2}, /* 5: C-A + */
    {BENCH_TEST_V / 2, 0.0f, -BENCH_TEST_V / 2}  /* 6: C-A - */
};

volatile float bench_current_result[BENCH_RESULT_WORDS];

static void apply_phases(bench_step v)
{
    board_pwm(0.5f + v.a / BENCH_SUPPLY_V,
              0.5f + v.b / BENCH_SUPPLY_V,
              0.5f + v.c / BENCH_SUPPLY_V);
}

static void wait_ms(uint32_t ms)
{
    uint32_t start = board_millis();
    while ((uint32_t)(board_millis() - start) < ms) {
    }
}

void bench_current_run(void)
{
    int step;
    int i;
    float worst = 0.0f;

    for (i = 0; i < BENCH_RESULT_WORDS; ++i) bench_current_result[i] = 0.0f;
    for (i = 0; i < 3; ++i) bench_current_result[0 + i] = current_sense_offset(i);

    board_pwm(0.5f, 0.5f, 0.5f);
    current_sense_arm(1);

    for (step = 0; step < BENCH_STEPS; ++step) {
        float sum0 = 0.0f, sum1 = 0.0f, sum2 = 0.0f;
        uint32_t samples = 0u;
        uint32_t start;

        /* A fault latched before this routine ran (e.g. the nFAULT glitch at
           power-up) would make board_driver_set() refuse to enable. Record what
           was latched, then clear it so the enable attempt below is honest. */
        bench_current_result[26] += (float)board_take_stop();
        if (!board_driver_wake()) {
            bench_current_result[28] = -1.0f;   /* driver will not wake */
            break;
        }
        apply_phases(bench_steps[step]);
        bench_current_result[27] = (float)((GPIOB->IDR & DRIVER_FAULT_PIN) ? 1u : 0u);
        if (!board_driver_set(1)) {
            bench_current_result[28] = -2.0f;   /* board_driver_set refused */
            board_pwm(0.5f, 0.5f, 0.5f);
            break;
        }
        wait_ms(BENCH_SETTLE_MS);
        start = board_millis();
        while ((uint32_t)(board_millis() - start) < BENCH_HOLD_MS) {
            float a0 = current_sense_amp(0);
            float a1 = current_sense_amp(1);
            float a2 = current_sense_amp(2);
            sum0 += a0;
            sum1 += a1;
            sum2 += a2;
            if (a0 > worst) worst = a0;
            if (-a0 > worst) worst = -a0;
            if (a1 > worst) worst = a1;
            if (-a1 > worst) worst = -a1;
            if (a2 > worst) worst = a2;
            if (-a2 > worst) worst = -a2;
            ++samples;
        }
        board_pwm(0.5f, 0.5f, 0.5f);
        board_driver_set(0);
        if (samples != 0u) {
            float inv = 1.0f / (float)samples;
            bench_current_result[3 + step * 3 + 0] = sum0 * inv;
            bench_current_result[3 + step * 3 + 1] = sum1 * inv;
            bench_current_result[3 + step * 3 + 2] = sum2 * inv;
        }
        wait_ms(BENCH_REST_MS);
    }

    board_pwm(0.5f, 0.5f, 0.5f);
    board_driver_set(0);
    current_sense_arm(0);
    bench_current_result[24] = worst;
    bench_current_result[25] = (float)current_sense_trips();
    for (i = 0; i < 3; ++i) bench_current_result[29 + i] = current_sense_offset(i);
    (void)board_take_stop();
}

volatile float bench_lock_result[8];

int bench_lock_run(float phase_v, uint32_t hold_ms)
{
    float sum0 = 0.0f, sum1 = 0.0f, sum2 = 0.0f;
    float worst = 0.0f;
    uint32_t samples = 0u;
    uint32_t start;
    int over = 0;
    int i;

    for (i = 0; i < 8; ++i) bench_lock_result[i] = 0.0f;
    bench_lock_result[4] = -1.0f;

    board_take_stop();
    board_pwm(0.5f, 0.5f, 0.5f);
    if (!board_driver_wake()) return -1;

    board_pwm(0.5f + phase_v / BENCH_SUPPLY_V,
              0.5f - phase_v / BENCH_SUPPLY_V,
              0.5f);
    current_sense_arm(1);
    if (!board_driver_set(1)) {
        board_pwm(0.5f, 0.5f, 0.5f);
        current_sense_arm(0);
        bench_lock_result[4] = -2.0f;
        return -2;
    }
    bench_lock_result[5] = (float)((GPIOB->IDR & DRIVER_FAULT_PIN) ? 1u : 0u);

    start = board_millis();
    while ((uint32_t)(board_millis() - start) < hold_ms) {
        float a0 = current_sense_amp(0);
        float a1 = current_sense_amp(1);
        float a2 = current_sense_amp(2);
        float m = a0 < 0.0f ? -a0 : a0;
        if (a1 < 0.0f ? -a1 > m : a1 > m) m = a1 < 0.0f ? -a1 : a1;
        if (a2 < 0.0f ? -a2 > m : a2 > m) m = a2 < 0.0f ? -a2 : a2;
        if (m > worst) worst = m;
        if (worst > BENCH_LOCK_TRIP_A) {
            if (++over > 50) {          /* > 50 samples well above the cap */
                bench_lock_result[4] = -3.0f;
                break;
            }
        } else {
            over = 0;
        }
        sum0 += a0;
        sum1 += a1;
        sum2 += a2;
        ++samples;
        bench_lock_result[0] = sum0 / (float)samples;
        bench_lock_result[1] = sum1 / (float)samples;
        bench_lock_result[2] = sum2 / (float)samples;
        bench_lock_result[3] = worst;
    }

    board_pwm(0.5f, 0.5f, 0.5f);
    board_driver_set(0);
    current_sense_arm(0);
    bench_lock_result[6] = (float)samples;
    bench_lock_result[7] = (float)current_sense_trips();
    if (bench_lock_result[4] == -1.0f) bench_lock_result[4] = 0.0f;
    return (int)bench_lock_result[4];
}

volatile float bench_spin_result[BENCH_SPIN_WORDS];

static void spin_phases(float theta, float amp_v)
{
    float ua = amp_v * cosf(theta);
    float ub = amp_v * cosf(theta - 2.0943951f);
    float uc = amp_v * cosf(theta + 2.0943951f);
    board_pwm(0.5f + ua / BENCH_SUPPLY_V,
              0.5f + ub / BENCH_SUPPLY_V,
              0.5f + uc / BENCH_SUPPLY_V);
}

int bench_spin_run(float amp_v, float elec_rate, uint32_t hold_ms)
{
    Encoder sensor = {0};
    float sum0 = 0.0f, sum1 = 0.0f, sum2 = 0.0f;
    float worst = 0.0f;
    float theta = 0.0f;
    uint32_t samples = 0u;
    uint32_t start, last_us;
    int over = 0;
    int i;

    for (i = 0; i < BENCH_SPIN_WORDS; ++i) bench_spin_result[i] = 0.0f;
    bench_spin_result[4] = -1.0f;
    bench_spin_result[11] = elec_rate;

    board_take_stop();
    board_pwm(0.5f, 0.5f, 0.5f);
    as5600_init();
    if (as5600_update(&sensor)) {
        bench_spin_result[8] = sensor.mechanical;
        bench_spin_result[9] = sensor.mechanical;
    }
    if (!board_driver_wake()) return -1;

    current_sense_arm(1);
    if (!board_driver_set(1)) {
        board_pwm(0.5f, 0.5f, 0.5f);
        current_sense_arm(0);
        bench_spin_result[4] = -2.0f;
        return -2;
    }
    bench_spin_result[5] = (float)((GPIOB->IDR & DRIVER_FAULT_PIN) ? 1u : 0u);

    last_us = board_micros();
    start = board_millis();
    while ((uint32_t)(board_millis() - start) < hold_ms && over <= 50) {
        uint32_t now = board_micros();
        float dt = (float)(uint32_t)(now - last_us) * 1e-6f;
        float a0, a1, a2, m;
        last_us = now;
        if (dt > 0.02f) dt = 0.02f;      /* tolerate a slow iteration */
        theta += elec_rate * dt;
        while (theta > 6.2831853f) theta -= 6.2831853f;
        spin_phases(theta, amp_v);

        a0 = current_sense_amp(0);
        a1 = current_sense_amp(1);
        a2 = current_sense_amp(2);
        m = a0 < 0.0f ? -a0 : a0;
        if ((a1 < 0.0f ? -a1 : a1) > m) m = a1 < 0.0f ? -a1 : a1;
        if ((a2 < 0.0f ? -a2 : a2) > m) m = a2 < 0.0f ? -a2 : a2;
        if (m > worst) worst = m;
        if (worst > BENCH_LOCK_TRIP_A) {
            ++over;
        } else {
            over = 0;
        }
        sum0 += a0;
        sum1 += a1;
        sum2 += a2;
        ++samples;
        bench_spin_result[0] = sum0 / (float)samples;
        bench_spin_result[1] = sum1 / (float)samples;
        bench_spin_result[2] = sum2 / (float)samples;
        bench_spin_result[3] = worst;
        bench_spin_result[6] = (float)samples;
    }
    if (over > 50) bench_spin_result[4] = -3.0f;

    board_pwm(0.5f, 0.5f, 0.5f);
    board_driver_set(0);
    current_sense_arm(0);
    if (as5600_update(&sensor)) {
        float delta = sensor.mechanical - bench_spin_result[8];
        bench_spin_result[9] = sensor.mechanical;
        bench_spin_result[10] = delta + 6.2831853f * (float)sensor.turns;
    }
    bench_spin_result[7] = (float)current_sense_trips();
    if (bench_spin_result[4] == -1.0f) bench_spin_result[4] = 0.0f;
    board_take_stop();
    return (int)bench_spin_result[4];
}

volatile float bench_align_result[BENCH_ALIGN_ROWS * BENCH_ALIGN_COLS];

/* ---------------------------------------------------------------------------
 * Zero-electric-angle calibration.
 *
 * A d-axis field pulls the rotor flux onto the commanded electrical angle, so
 * the mechanical angle measured at that moment defines "zero".  Friction makes
 * the rotor stop short of the target by a hysteresis angle whose sign depends
 * on the direction the field arrived from, which is why a single pull is only
 * good for a few tens of electrical degrees.  Every row therefore does two
 * pulls - one swinging up from theta - BENCH_ALIGN_SWING, one swinging down
 * from theta + BENCH_ALIGN_SWING - and stores both settled angles, so the
 * offline fit can average the hysteresis out.
 * ------------------------------------------------------------------------- */

#define BENCH_ALIGN_SWING    0.9f  /* electrical radians */
#define BENCH_ALIGN_STEP_MS  10u

static void align_apply(float d_v, float theta)
{
    float duty[3];

    foc_svpwm(0.0f, d_v, theta, BENCH_SUPPLY_V, duty);
    board_pwm(duty[0], duty[1], duty[2]);
}

/* Returns 1 once the phase current has stayed over the trip for 50 polls. */
static int align_over(float *worst, int *over)
{
    float a0 = current_sense_amp(0);
    float a1 = current_sense_amp(1);
    float a2 = current_sense_amp(2);
    float m = fabsf(a0);

    if (fabsf(a1) > m) m = fabsf(a1);
    if (fabsf(a2) > m) m = fabsf(a2);
    if (m > *worst) *worst = m;
    if (*worst > BENCH_LOCK_TRIP_A) {
        if (++(*over) > 50) return 1;
    } else {
        *over = 0;
    }
    return 0;
}

/* Swing the field from `from` to `theta` over ramp_ms, then hold settle_ms and
   report the mechanical angle averaged over the last 200 ms of that hold. */
static int align_pull(float d_v, float from, float theta, uint32_t ramp_ms,
                      uint32_t settle_ms, float *worst, int *over, float *out)
{
    Encoder sensor = {0};
    uint32_t steps = (ramp_ms + BENCH_ALIGN_STEP_MS - 1u) / BENCH_ALIGN_STEP_MS;
    float sum = 0.0f;
    uint32_t samples = 0u;
    uint32_t start;
    uint32_t i;

    if (steps == 0u) steps = 1u;

    align_apply(d_v, from);
    start = board_millis();
    while ((uint32_t)(board_millis() - start) < 120u) {
        if (align_over(worst, over)) return -3;
    }
    for (i = 1u; i <= steps; ++i) {
        while ((uint32_t)(board_millis() - start) <
               (uint32_t)(120u + BENCH_ALIGN_STEP_MS * i)) {
            if (align_over(worst, over)) return -3;
        }
        align_apply(d_v, from + (theta - from) * (float)i / (float)steps);
    }

    start = board_millis();
    while ((uint32_t)(board_millis() - start) < settle_ms) {
        uint32_t elapsed = board_millis() - start;
        if (align_over(worst, over)) return -3;
        if ((uint32_t)(elapsed + 200u) >= settle_ms && as5600_update(&sensor)) {
            sum += sensor.mechanical;
            ++samples;
        }
    }
    if (samples == 0u) return -4;
    *out = sum / (float)samples;
    return 0;
}

int bench_align_run(int row, float d_v, float theta, uint32_t hold_ms)
{
    Encoder sensor = {0};
    volatile float *r;
    float worst = 0.0f;
    float m_up = 0.0f;
    float m_down = 0.0f;
    uint32_t ramp_ms = hold_ms / 4u;
    uint32_t settle_ms = hold_ms / 2u;
    int over = 0;
    int code;
    int ok;
    int try;
    int i;

    if (row < 0 || row >= BENCH_ALIGN_ROWS) return -2;
    r = &bench_align_result[row * BENCH_ALIGN_COLS];
    for (i = 0; i < BENCH_ALIGN_COLS; ++i) r[i] = 0.0f;
    r[4] = -1.0f;
    r[7] = theta;

    board_take_stop();
    board_pwm(0.5f, 0.5f, 0.5f);
    as5600_init();
    if (as5600_update(&sensor)) {
        r[0] = sensor.mechanical;
        r[1] = sensor.mechanical;
    }
    if (!board_driver_wake()) return -1;

    current_sense_arm(1);
    /* Stopping the driver at the end of the previous row pulls nSLEEP low, so
       the DRV latches a (harmless) nFAULT falling edge that races with the next
       enable through EXTI2. Retry the enable a few times instead of giving up:
       the flag is cleared and the latch pulsed away on every attempt. */
    ok = 0;
    for (try = 0; try < 5 && !ok; ++try) {
        board_take_stop();
        ok = board_driver_set(1);
        if (!ok) HAL_Delay(20u);
    }
    if (!ok) {
        board_pwm(0.5f, 0.5f, 0.5f);
        current_sense_arm(0);
        r[4] = -2.0f;
        return -2;
    }
    r[5] = (float)((GPIOB->IDR & DRIVER_FAULT_PIN) ? 1u : 0u);
    if (ramp_ms < 200u) ramp_ms = 200u;
    if (settle_ms < 400u) settle_ms = 400u;

    code = align_pull(d_v, theta - BENCH_ALIGN_SWING, theta, ramp_ms, settle_ms,
                      &worst, &over, &m_up);
    if (code == 0) {
        code = align_pull(d_v, theta + BENCH_ALIGN_SWING, theta, ramp_ms,
                          settle_ms, &worst, &over, &m_down);
    }

    board_pwm(0.5f, 0.5f, 0.5f);
    board_driver_set(0);
    current_sense_arm(0);
    r[3] = worst;
    r[4] = (float)code;
    if (code == 0) {
        r[1] = m_up;
        r[2] = m_up - r[0];
        r[6] = m_down;
    }
    board_take_stop();
    return code;
}

volatile float bench_cl_result[BENCH_CL_ROWS * BENCH_CL_COLS];
volatile float bench_cl_summary[8];

#define BENCH_CL_TRIP_A 1.20f

/* Runs motor_step() until `ms` have elapsed, tracking the worst phase current.
   Returns 0 normally, -3 if the current stays above the trip for a while. */
static int cl_service(Motor *motor, float v_limit, uint32_t ms, float *worst)
{
    uint32_t start = board_millis();
    int over = 0;
    while ((uint32_t)(board_millis() - start) < ms) {
        float a0, a1, a2, m;
        motor->voltage_limit = v_limit;
        motor_step(motor);
        a0 = current_sense_amp(0);
        a1 = current_sense_amp(1);
        a2 = current_sense_amp(2);
        m = a0 < 0.0f ? -a0 : a0;
        if ((a1 < 0.0f ? -a1 : a1) > m) m = a1 < 0.0f ? -a1 : a1;
        if ((a2 < 0.0f ? -a2 : a2) > m) m = a2 < 0.0f ? -a2 : a2;
        if (m > *worst) *worst = m;
        if (*worst > BENCH_CL_TRIP_A) {
            if (++over > 50) return -3;
        } else {
            over = 0;
        }
        if (!motor->enabled) return -2;
    }
    return 0;
}

int bench_cl_run(Motor *motor, float v_limit, float step_rad,
                 uint32_t settle_ms, uint32_t hold_ms, int steps)
{
    float worst = 0.0f;
    float start_angle;
    int i, done = 0, code = 0;

    for (i = 0; i < BENCH_CL_ROWS * BENCH_CL_COLS; ++i) bench_cl_result[i] = 0.0f;
    for (i = 0; i < 8; ++i) bench_cl_summary[i] = 0.0f;
    bench_cl_summary[5] = v_limit;
    if (steps > BENCH_CL_ROWS) steps = BENCH_CL_ROWS;

    /* A latched stop (power-up nFAULT edge) refuses the enable once; retry a few
       times so a spurious stop cannot abort the whole test. */
    for (i = 0; i < 5 && !motor->enabled; ++i) {
        board_take_stop();
        motor_command(motor, "E");
        if (!motor->enabled) HAL_Delay(60u);
    }
    if (!motor->enabled) {
        bench_cl_summary[0] = -2.0f;
        bench_cl_summary[1] = worst;
        return -2;
    }
    code = cl_service(motor, v_limit, settle_ms, &worst);
    start_angle = motor->angle;
    bench_cl_summary[6] = start_angle;

    for (i = 0; code == 0 && i < steps; ++i) {
        char command[24];
        volatile float *row = &bench_cl_result[i * BENCH_CL_COLS];
        snprintf(command, sizeof command, "T%.5f", (double)(start_angle + step_rad * (float)(i + 1)));
        motor_command(motor, command);
        code = cl_service(motor, v_limit, hold_ms, &worst);
        row[0] = motor->target;
        row[1] = motor->angle;
        row[2] = motor->target - motor->angle;
        row[3] = motor->velocity;
        row[4] = motor->voltage_limit;
        if (code == 0) ++done;
    }

    bench_cl_summary[0] = (float)code;
    bench_cl_summary[1] = worst;
    bench_cl_summary[2] = (float)current_sense_trips();
    bench_cl_summary[4] = (float)done;
    bench_cl_summary[7] = motor->angle;
    motor_command(motor, "D");
    bench_cl_summary[3] = (float)motor->enabled;
    board_take_stop();
    return code;
}

volatile float bench_cl_table[BENCH_CL_SLOTS * 8];

void bench_cl_save(int slot)
{
    int i;
    volatile float *row;

    if (slot < 0 || slot >= BENCH_CL_SLOTS) {
        return;
    }
    row = &bench_cl_table[slot * 8];
    for (i = 0; i < 8; ++i) {
        row[i] = bench_cl_summary[i];
    }
}
