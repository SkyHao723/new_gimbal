#ifndef MOTOR_H
#define MOTOR_H
#include "as5600.h"
#include "foc_math.h"

typedef struct {
    Encoder sensor;
    VelocityPI pi;
    float angle, velocity, target, ramp_target;
    float voltage_limit, velocity_limit;
    float openloop_angle, startup_target;
    uint32_t step_us, startup_ms;
    uint8_t enabled, startup, sensor_failures;
} Motor;

void motor_init(Motor *motor);
void motor_step(Motor *motor);
void motor_command(Motor *motor, const char *command);
void motor_disable(Motor *motor);
void motor_status(const Motor *motor);
#endif
