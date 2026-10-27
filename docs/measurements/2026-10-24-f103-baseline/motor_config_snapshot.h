#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* Same motor, phase wiring and magnet mounting as foc_official/src/main.cpp. */
#define MOTOR_POLE_PAIRS          11
#define MOTOR_SENSOR_DIRECTION    1
#define MOTOR_ZERO_ELECTRIC       0.92f
#define MOTOR_SUPPLY_VOLTAGE      24.0f
#define MOTOR_VOLTAGE_LIMIT       3.0f
#define MOTOR_MAX_VOLTAGE         5.0f
#define MOTOR_VELOCITY_LIMIT      20.0f
/* Keep the validated velocity damping/filter. Faster integral recovery and
   position correction reduce the measured low-speed overshoot near zero. */
#define MOTOR_VELOCITY_P          0.6f
#define MOTOR_VELOCITY_I          5.0f
#define MOTOR_VOLTAGE_RAMP        1000.0f
#define MOTOR_VELOCITY_FILTER_TF  0.02f
#define MOTOR_ANGLE_P             40.0f
#define MOTOR_TARGET_SLEW         6.0f
#define MOTOR_TARGET_ACCEL        8.0f  /* rad/s^2; 0 -> 6 rad/s in 0.75s */
#define MOTOR_TARGET_DECEL        4.0f  /* retain the validated gentle braking */

/* Set to 0 to boot disabled without moving. No alignment sweep is performed. */
#define STARTUP_ROTATE_ONCE       1
#define STARTUP_ROTATE_VELOCITY   4.0f
#define STARTUP_ROTATE_TOLERANCE  0.03f
#define STARTUP_ROTATE_TIMEOUT_MS 10000u
#define MOTOR_PWM_HZ             25000u
#define ENCODER_I2C_HZ           400000u

#endif
