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
/* 2026-10-28: board 3's motor + magnet ring is temporarily bolted to board 2
   while the board 2 power stage is being repaired, so it inherits board 3's
   calibration. Re-measure with G431_ALIGN_TEST once board 2 gets its own motor
   back.
   Re-measured on board 2 with that motor on 2026-10-28: the phase order on this
   board is opposite to board 3 (it needs dir = -1), and with board 3's dir = +1
   the drive vector landed on the rotor flux axis: 1.07 A flowed while the rotor
   did not move at all. Fit of the four align rows: pp=11, dir=-1,
   zero=0.274 rad (max deviation 9.7 deg). */
#define CALIB_ZERO_ELECTRIC      0.274f
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
