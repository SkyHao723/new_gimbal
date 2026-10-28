#ifndef CALIB_H
#define CALIB_H

#include "board_config.h"

/* Per-board calibration of the motor that is bolted to that board.
   A magnet ring is glued onto the rotor by hand, and the three phase wires can
   be plugged in either order, so both the electrical zero and the sensor
   direction differ from board to board even with identical motors.
   Measure them per board with the align self-test (G431_ALIGN_TEST = 1 in
   User/motor_config.h): bench_align_result[] holds start/stable/delta/worst/
   code/fault/samples/theta for theta = 0, 0, pi/2, pi. Fit pp and direction by
   requiring the derived zero to be identical in all four rows (see
   docs/g431-闭环打通-2026-10-27.md). */
#if BOARD_CAN_NODE == 1
#define CALIB_ZERO_ELECTRIC      1.697f
#define CALIB_SENSOR_DIRECTION   (-1)
#elif BOARD_CAN_NODE == 2
/* 2026-10-28 align test on board 2 with its own motor: pp=11, dir=-1.
   The phase order on this board is opposite to board 3 - with board 3's
   dir=+1 the drive vector landed on the rotor flux axis, so 1.07 A flowed
   while the rotor did not move at all.
   Zero was then measured with the bidirectional align test (see
   docs/g431-闭环打通-2026-10-27.md): each row pulls the d-axis from both
   sides so the friction hysteresis cancels. Four rows fit zero = 0.122 rad
   with rms 5.1 deg; rows at theta=0 agree to 5e-5 rad, the residual spread
   comes from the hand-glued magnet ring (about 0.4 deg mechanical). */
#define CALIB_ZERO_ELECTRIC      0.122f
#define CALIB_SENSOR_DIRECTION   (-1)
#elif BOARD_CAN_NODE == 3
/* 2026-10-27 align test: pp=11, dir=+1, zero=0.566 rad (residual 2.4 deg). */
#define CALIB_ZERO_ELECTRIC      0.566f
#define CALIB_SENSOR_DIRECTION   1
#else
#define CALIB_ZERO_ELECTRIC      1.697f
#define CALIB_SENSOR_DIRECTION   (-1)
#endif

#endif
