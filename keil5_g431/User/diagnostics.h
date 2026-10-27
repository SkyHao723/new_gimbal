#ifndef G431_DIAGNOSTICS_H
#define G431_DIAGNOSTICS_H
#include <stdint.h>
/* Read-only SWD bring-up snapshot, 28 words; no motor commands via memory.
   tools/g431_probe.py reads the symbol size from the map file, so appending
   words keeps older tools working. */
typedef struct {
    uint32_t magic, stage, core_clock, uptime_ms, loops;
    uint32_t enabled, sleep_high, off_high, fault_high;
    uint32_t sensor_valid, sensor_errors, can_psr, can_ecr, tx_dropped;
    float angle, velocity;
    /* Current sensing / limiting (BSP/current_sense.c, User/current_limit.c). */
    uint32_t cs_valid, cs_samples, cs_trips, cs_blind;
    float cs_peak, cs_allowed, cs_thermal, cs_amp_mid;
    /* Trigger route and the software-triggered self test raw values. */
    uint32_t cs_trig, cs_probe_a, cs_probe_b, cs_probe_c;
} G431Diagnostics;
extern volatile G431Diagnostics g431_diagnostics;
#endif
