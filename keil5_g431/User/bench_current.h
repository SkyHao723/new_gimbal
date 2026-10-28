#ifndef BENCH_CURRENT_H
#define BENCH_CURRENT_H
#include <stdint.h>

#include "motor.h"

/* Number of DC-lock steps applied by bench_current_run(): one 0 V bias step
   plus six +/-0.6 V steps covering the three phase pairs A-B, B-C, C-A. */
#define BENCH_STEPS        7
/* Result array size; keeps the SWD reader simple (32 words = 128 bytes). */
#define BENCH_RESULT_WORDS 32

/* Result layout (floats):
     [0..2]        zero offsets in ADC counts, learnt before the bridge was on
     [3+3S+c]      mean phase current in amps at step S (S = 0..6), channel c
     Step 0 is 0 V with the bridge enabled: subtract [3..5] from the other
     steps to remove the bias shift that enabling the outputs adds.
     Steps 1/2 = A-B +/-, 3/4 = B-C +/-, 5/6 = C-A +/-.
     [24]    worst |current| seen during the run, in amps
     [25]    ADC-interrupt trip counter at the end of the run
     [26]    sum of stop flags cleared before the enable attempts
     [27]    nFAULT state (1 = released) just before the last enable attempt
     [28]    error code: 0 = ran to completion, -1 = driver would not wake,
             -2 = board_driver_set(1) refused the enable
     [29..31] zero offsets re-read after the run (detects bias drift) */
extern volatile float bench_current_result[BENCH_RESULT_WORDS];

/* Runs the whole sequence and leaves the bridge disabled. Blocking; takes
   roughly BENCH_STEPS * 0.8 s. */
void bench_current_run(void);

/* Holds one A-B DC lock so a multimeter can be read while it lasts. phase_v is
   the half-difference: the terminal-to-terminal voltage is 2 * phase_v.
   Results (floats):
     [0..2] running mean phase current A/B/C, amps (live during the hold)
     [3]    worst |current| during the hold, amps
     [4]    result code: 0 = held for the full time, -1 = driver would not wake,
            -2 = enable refused, -3 = aborted early on over-current
     [5]    nFAULT state after enable (1 = released)
     [6]    number of averaged samples
     [7]    ADC-interrupt trip counter at the end
   Returns the result code. Leaves the bridge disabled. */
extern volatile float bench_lock_result[8];
int bench_lock_run(float phase_v, uint32_t hold_ms);

/* Open-loop spin (step 1 of the bring-up plan): rotates the stator field at
   elec_rate rad/s electrical with a sine amplitude of amp_v volts, then stops.
   amp_v is a phase voltage amplitude; 0.8 V gives roughly 0.2 A on this motor.
   Results (floats):
     [0..2] mean phase current A/B/C, amps
     [3]    worst |current|, amps
     [4]    result code: 0 = spun for the full time, -1 = driver would not wake,
            -2 = enable refused, -3 = aborted early on over-current
     [5]    nFAULT state after enable (1 = released)
     [6]    number of current samples taken
     [7]    ADC-interrupt trip counter at the end
     [8]    encoder mechanical angle before the spin, radians
     [9]    encoder mechanical angle after the spin, radians
     [10]   travelled mechanical angle in radians (turns included)
     [11]   electrical rate that was commanded, rad/s
   Leaves the bridge disabled. */
#define BENCH_SPIN_WORDS 12
extern volatile float bench_spin_result[BENCH_SPIN_WORDS];
int bench_spin_run(float amp_v, float elec_rate, uint32_t hold_ms);

/* Zero-electric-angle calibration (step 2 of the bring-up plan). Applies a pure
   d-axis voltage vector at electrical angle theta and lets the rotor settle, so
   the mechanical angle at that moment defines the zero.  Every row does two
   pulls, swinging the field up from theta - BENCH_ALIGN_SWING and down from
   theta + BENCH_ALIGN_SWING, so the fit can average the friction hysteresis
   out (a single pull is only good for a few tens of electrical degrees).
   Each call fills one row of bench_align_result[]:
     [0] encoder mechanical angle before the pull, radians
     [1] mechanical angle settled after the swing-up pull, radians
     [2] delta = [1] - [0], radians
     [3] worst |current|, amps
     [4] result code: 0 = settled, -1 = driver would not wake, -2 = enable
         refused, -3 = aborted early on over-current, -4 = no encoder samples
     [5] nFAULT state after enable (1 = released)
     [6] mechanical angle settled after the swing-down pull, radians
     [7] electrical angle that was commanded, radians
   Align at theta = 0, 0, pi/2 and pi: the repeated row 0 checks repeatability,
   and the derived zero has to agree across all four rows. */
#define BENCH_ALIGN_ROWS 4
#define BENCH_ALIGN_COLS 8
extern volatile float bench_align_result[BENCH_ALIGN_ROWS * BENCH_ALIGN_COLS];
int bench_align_run(int row, float d_v, float theta, uint32_t hold_ms);

/* Closed-loop step test (bring-up step 2b). Enables the motor with a fixed
   voltage ceiling, commands `steps` consecutive position steps of step_rad each,
   and records the response. No CAN adapter is involved.
   bench_cl_result[] holds one row per step:
     [0] commanded target, rad      [1] measured angle, rad
     [2] target - angle, rad        [3] velocity, rad/s
     [4] voltage ceiling in force, V
   bench_cl_summary[]:
     [0] result code: 0 = all steps ran, -2 = enable refused, -3 = aborted on
         over-current, -4 = sensor lost
     [1] worst |phase current|, amps   [2] ADC trip counter at the end
     [3] motor->enabled at the end     [4] steps completed
     [5] voltage ceiling used, V       [6] angle before the first step
     [7] angle after the last step */
#define BENCH_CL_ROWS 3
#define BENCH_CL_COLS 5
extern volatile float bench_cl_result[BENCH_CL_ROWS * BENCH_CL_COLS];
extern volatile float bench_cl_summary[8];

/* Position-dependent jitter trace: sweep across mechanical angle pi and store
   one row per control step as (angle, velocity, uq). Decode over SWD.
   meta = [0] code [1] n [2] vlim [3] start_angle [4] pi_target
          [5] worst current [6] trips [7] duration s */
#define BENCH_TRACE_N 1200
#define BENCH_TRACE_COLS 3
extern volatile float bench_trace[BENCH_TRACE_N * BENCH_TRACE_COLS];
extern volatile float bench_trace_meta[8];
int bench_trace_run(Motor *motor, float v_limit, uint32_t settle_ms, uint32_t cap_ms);

/* Voltage-ceiling sweep (bring-up step 3): one summary row per ceiling so a
   single boot can compare several limits. Uses the same layout as
   bench_cl_summary[]. */
#define BENCH_CL_SLOTS 4
extern volatile float bench_cl_table[BENCH_CL_SLOTS * 8];
void bench_cl_save(int slot);
int bench_cl_run(Motor *motor, float v_limit, float step_rad,
                 uint32_t settle_ms, uint32_t hold_ms, int steps);

#endif
