#include "motor.h"
#include "motor_config.h"
#include "board.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static void apply_voltage(float q, float electrical_angle)
{
    float duty[3];
    foc_svpwm(q, 0.0f, electrical_angle, MOTOR_SUPPLY_VOLTAGE, duty);
    board_pwm(duty[0], duty[1], duty[2]);
}

static int update_sensor(Motor *motor)
{
    if (!as5600_update(&motor->sensor)) {
        if (motor->sensor_failures < 255u) ++motor->sensor_failures;
        /* Do not energize using stale electrical position, even on one failure. */
        if (motor->enabled) {
            motor_disable(motor);
            printf("FAULT sensor read, motor DISABLED\r\n");
        }
        return 0;
    }
    motor->sensor_failures = 0u;
    motor->angle = MOTOR_SENSOR_DIRECTION * motor->sensor.angle;
    motor->velocity = MOTOR_SENSOR_DIRECTION * motor->sensor.velocity;
    return 1;
}

static int enable_here(Motor *motor)
{
    if (!update_sensor(motor)) return 0;
    motor->target = motor->ramp_target = motor->angle;
    memset(&motor->pi, 0, sizeof(motor->pi));
    motor->step_us = board_micros();
    apply_voltage(0.0f, 0.0f);
    motor->enabled = (uint8_t)board_driver_set(1);
    return motor->enabled;
}

void motor_disable(Motor *motor)
{
    board_driver_set(0);
    motor->enabled = motor->startup = 0u;
    memset(&motor->pi, 0, sizeof(motor->pi));
    motor->target = motor->ramp_target = motor->angle;
}

void motor_init(Motor *motor)
{
    memset(motor, 0, sizeof(*motor));
    board_driver_set(0);
    motor->voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor->velocity_limit = MOTOR_VELOCITY_LIMIT;
    motor->step_us = board_micros();
    if (!update_sensor(motor)) {
        printf("FAULT sensor init, motor DISABLED\r\n");
        return;
    }
    motor->target = motor->ramp_target = motor->angle;
    printf("fixed calibration: pp=%d dir=%d zero=%.4f (no sweep)\r\n",
           MOTOR_POLE_PAIRS, MOTOR_SENSOR_DIRECTION, (double)MOTOR_ZERO_ELECTRIC);
#if STARTUP_ROTATE_ONCE
    if (enable_here(motor)) {
        motor->startup = 1u;
        motor->startup_target = motor->angle + FOC_TWO_PI;
        motor->target = motor->startup_target;
        /* Match SimpleFOC velocityOpenloop: synthetic angle starts at shaft angle. */
        motor->openloop_angle = motor->angle;
        motor->startup_ms = board_millis();
        printf("startup rotate +360deg, target=%.4f\r\n", (double)motor->target);
    }
#else
    printf("motor DISABLED - send E to enable\r\n");
#endif
}

void motor_step(Motor *motor)
{
    uint32_t now = board_micros();
    float dt = (uint32_t)(now-motor->step_us)*1e-6f;
    float error, max_step, velocity_sp, q, electrical;
    motor->step_us = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;
    if (!update_sensor(motor) || !motor->enabled) return;
    if (motor->startup) {
        if (motor->startup_target-motor->angle <= STARTUP_ROTATE_TOLERANCE) {
            motor_disable(motor);
            printf("startup rotate DONE (+360deg), motor DISABLED\r\n");
            return;
        }
        if ((uint32_t)(board_millis()-motor->startup_ms) >= STARTUP_ROTATE_TIMEOUT_MS) {
            motor_disable(motor);
            printf("startup rotate TIMEOUT, motor DISABLED\r\n");
            return;
        }
        motor->openloop_angle = foc_normalize(motor->openloop_angle+STARTUP_ROTATE_VELOCITY*dt);
        apply_voltage(motor->voltage_limit, motor->openloop_angle*MOTOR_POLE_PAIRS);
        return;
    }
    max_step = fminf(MOTOR_TARGET_SLEW, motor->velocity_limit)*dt;
    error = motor->target-motor->ramp_target;
    motor->ramp_target += foc_clamp(error, -max_step, max_step);
    velocity_sp = foc_clamp(MOTOR_ANGLE_P*(motor->ramp_target-motor->angle),
                            -motor->velocity_limit, motor->velocity_limit);
    q = foc_velocity_pi(&motor->pi, velocity_sp-motor->velocity, dt, motor->voltage_limit);
    electrical = MOTOR_SENSOR_DIRECTION*MOTOR_POLE_PAIRS*motor->sensor.mechanical-MOTOR_ZERO_ELECTRIC;
    apply_voltage(q, electrical);
}

void motor_status(const Motor *motor)
{
    /* Keep the existing Web Serial parser's field names and EN format. */
    printf("EN=%s  angle=%.4f  vel=%.4f  tgt=%.4f  Vlim=%.2f  Vbus=%.1f  dir=%d  pp=FIXED  zero=%.4f\r\n",
           motor->enabled ? "ON" : "OFF", (double)motor->angle, (double)motor->velocity,
           (double)motor->target, (double)motor->voltage_limit, (double)MOTOR_SUPPLY_VOLTAGE,
           MOTOR_SENSOR_DIRECTION, (double)MOTOR_ZERO_ELECTRIC);
}

static int read_number(const char *text, float *value)
{
    char *end;
    double parsed = strtod(text, &end);
    if (end == text) {
        printf("ERR invalid numeric argument\r\n");
        return 0;
    }
    while (*end == ' ' || *end == '\t') ++end;
    if (*end != '\0' || !isfinite(parsed) || fabs(parsed) > 1000000.0) {
        printf("ERR invalid numeric argument\r\n");
        return 0;
    }
    *value = (float)parsed;
    return 1;
}

void motor_command(Motor *motor, const char *command)
{
    float value;
    switch (command[0]) {
    case 'E':
        if (motor->enabled) printf("EN ON (already enabled)\r\n");
        else if (enable_here(motor)) printf("EN ON  target=%.4f\r\n", (double)motor->target);
        else printf("EN OFF (sensor unavailable or stop pending)\r\n");
        break;
    case 'D':
        motor_disable(motor);
        printf("EN OFF (stop)\r\n");
        break;
    case 'T':
        if (!read_number(command+1, &value)) break;
        if (motor->startup) {
            motor->startup = 0u;
            motor->ramp_target = motor->angle;
            memset(&motor->pi, 0, sizeof(motor->pi));
            printf("startup rotate ABORTED by target command\r\n");
        }
        motor->target = value;
        printf("target=%.4f\r\n", (double)value);
        break;
    case 'V':
        if (!read_number(command+1, &value)) break;
        if (value <= 0.0f) value = MOTOR_VOLTAGE_LIMIT;
        motor->voltage_limit = fminf(value, MOTOR_MAX_VOLTAGE);
        printf("Vlim=%.2f V\r\n", (double)motor->voltage_limit);
        break;
    case 'L':
        if (!read_number(command+1, &value)) break;
        if (value <= 0.0f || value > MOTOR_VELOCITY_LIMIT) {
            printf("ERR velocity range: 0 < L <= 20\r\n");
            break;
        }
        motor->velocity_limit = value;
        printf("Vellim=%.2f\r\n", (double)value);
        break;
    case 'R':
        printf("sensor angle=%.4f  vel=%.4f  mech=%.4f\r\n",
               (double)motor->sensor.angle, (double)motor->sensor.velocity, (double)motor->sensor.mechanical);
        break;
    case '?': motor_status(motor); break;
    case 'H': printf("E D T<n> V<n> L<n> R ?\r\n"); break;
    default: printf("? for help\r\n"); break;
    }
}
