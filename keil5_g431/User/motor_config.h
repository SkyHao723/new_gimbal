#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* Inherited motor parameters. Recheck direction/zero after changing phases or magnet. */
#define MOTOR_POLE_PAIRS          11
#define MOTOR_SENSOR_DIRECTION -1
#define MOTOR_ZERO_ELECTRIC     1.697f
#define MOTOR_SUPPLY_VOLTAGE      24.0f
/* Voltage command and the absolute software ceiling for the V command. The
   ceiling is only reachable while the measured phase current stays near
   CURRENT_LIMIT_SOFT_A; User/current_limit.c folds the command back otherwise.
   Without usable current feedback the limiter pins the command to
   CURRENT_LIMIT_VOLTAGE_FLOOR. */
#define MOTOR_VOLTAGE_LIMIT       4.0f
#define MOTOR_MAX_VOLTAGE         8.0f

/* Current budget, measured through the DRV8316C low-side shunts.
   Rated 1.4 A continuous; fold-back starts slightly above that; 2.6 A is an
   instantaneous instruction to stop; the winding may sit at the rated current
   for CURRENT_LIMIT_THERMAL_TAU_S before the I^2t budget removes the drive. */
#define CURRENT_LIMIT_RATED_A        1.4f
#define CURRENT_LIMIT_SOFT_A        1.6f
#define CURRENT_LIMIT_TRIP_A        2.6f
#define CURRENT_LIMIT_FOLD_V_PER_A   10.0f
#define CURRENT_LIMIT_VOLTAGE_FLOOR  1.5f
#define CURRENT_LIMIT_PEAK_DECAY_S   0.05f
#define CURRENT_LIMIT_THERMAL_TAU_S  8.0f
#define MOTOR_VELOCITY_LIMIT      20.0f
/* Keep the validated velocity damping/filter. Faster integral recovery and
   position correction reduce the measured low-speed overshoot near zero. */
#define MOTOR_VELOCITY_P 0.6f
#define MOTOR_VELOCITY_I 5.0f
#define MOTOR_VOLTAGE_RAMP        1000.0f
#define MOTOR_VELOCITY_FILTER_TF 0.02f
#define MOTOR_ANGLE_P 40.0f
#define MOTOR_TARGET_SLEW         6.0f
#define MOTOR_TARGET_ACCEL        8.0f  /* rad/s^2; 0 -> 6 rad/s in 0.75s */
#define MOTOR_TARGET_DECEL        4.0f  /* retain the validated gentle braking */

/* Set to 0 to boot disabled without moving. No alignment sweep is performed. */
#define STARTUP_ROTATE_ONCE       0
#define STARTUP_ROTATE_VELOCITY   4.0f
#define STARTUP_ROTATE_TOLERANCE  0.03f
#define STARTUP_ROTATE_TIMEOUT_MS 10000u
#define MOTOR_PWM_HZ             25000u
#define ENCODER_I2C_HZ           100000u

/* Set to 1 for a bench build that runs User/bench_current.c once at start-up:
   a bounded low-voltage DC lock that validates the three current channels
   without needing a CAN adapter. Keep 0 for the delivered firmware. */
#define G431_BENCH_CURRENT_TEST 0

/* Set to 1 to hold one A-B DC lock (0.6 V terminal-to-terminal) for 45 s at
   boot, so a multimeter can be read on the 24 V input and across the motor
   leads. Used once to calibrate the current-sense gain against a real meter;
   must be 0 in the delivered firmware. */
#define G431_CURRENT_LOCK_TEST 0

/* Bring-up step 1: spin the motor open loop once at start-up (0.8 V amplitude,
   1 Hz electrical, 4 s) and leave the bridge off. The travelled angle lands in
   bench_spin_result[] and is read over SWD. Keep 0 for the delivered firmware. */
/* Bring-up step 2: settle onto three electrical d-axes to derive the zero offset. */
#define G431_ALIGN_TEST 0
/* Bring-up step 2b: closed-loop position steps once at start-up. */
#define G431_CLOSEDLOOP_TEST 0
#define G431_SPIN_TEST 0
/* Jitter diagnosis: capture the control loop waveform into RAM (read it back
   over SWD and FFT it). Keep 0 for the delivered firmware. */
#define G431_CAPTURE_TEST 0

/* Encoder software-I2C half period in microseconds. The control loop runs at
   roughly 1/(45 bit times * 2 * half period); 25 us gives ~460 Hz, which is
   too coarse for a 12-bit encoder (one LSB per loop is 0.7 rad/s of velocity
   quantisation). 5 us should reach ~1.5 kHz; watch sensor_errors for I2C
   errors before trusting it. */
#define ENCODER_I2C_HALF_US 25u

/* Velocity = angle change over the last N loop periods instead of one.  With a
   12-bit encoder and a 1.9 kHz loop a single loop period is one count = 2.9
   rad/s, so v(t) is dominated by quantisation noise that the velocity PI turns
   into torque ripple.  N=4 -> 0.7 rad/s per count; the F103 build leaves the
   macro undefined and keeps the one-period difference. */
#define ENCODER_VELOCITY_WINDOW 1

/* Any bench self-test implies no CAN adapter is present: main() then skips
   can_transport_init() so a bus-off cannot request a motor stop mid-test. */
#if (G431_BENCH_CURRENT_TEST + G431_CURRENT_LOCK_TEST + G431_ALIGN_TEST + G431_CLOSEDLOOP_TEST + G431_SPIN_TEST + G431_CAPTURE_TEST) > 0
#define G431_BENCH_MODE 1
#else
#define G431_BENCH_MODE 0
#endif

#endif
