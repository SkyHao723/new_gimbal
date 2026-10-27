#ifndef CURRENT_LIMIT_H
#define CURRENT_LIMIT_H
#include <stdint.h>

/* Host-testable current budget: turns the measured phase current into a
   voltage allowance. Kept free of STM32 headers so Tests/test_current_limit.c
   can compile it with a native compiler.

   Order of protection:
     1. BSP/current_sense.c  - instantaneous trip inside the ADC interrupt
     2. this module          - fold back the voltage command before the limit
     3. this module          - I^2t budget so a long overload cannot cook the
                               winding before the current peak is reached
     4. the OCP inside DRV8316C, configured by hardware (OCP/SR tied to PGND) */

typedef struct {
    float rated_a;         /* continuous phase current the winding tolerates */
    float soft_a;          /* fold-back starts above this phase current */
    float trip_a;          /* latch a stop at or above this peak current */
    float fold_v_per_a;    /* volts shed per amp over soft_a */
    float voltage_floor;   /* lowest voltage the limiter will allow */
    float voltage_ceiling; /* absolute ceiling, normally MOTOR_MAX_VOLTAGE */
    float peak_decay_s;    /* release time constant of the peak detector */
    float thermal_tau_s;   /* winding heating time constant */
} CurrentLimitConfig;

typedef struct {
    float peak_a;     /* released peak current */
    float thermal;    /* 0 = cold, 1 = the whole thermal budget is spent */
    float allowed_v;  /* voltage the limiter currently permits */
    float mean_square_a;
    uint32_t trips;
    uint8_t tripped;
    uint8_t blind;    /* current feedback unusable -> only voltage_floor */
} CurrentLimit;

/* Defaults come from User/motor_config.h. */
extern const CurrentLimitConfig current_limit_defaults;

void current_limit_init(CurrentLimit *lim);
void current_limit_update(CurrentLimit *lim, const CurrentLimitConfig *cfg,
                          float peak_a, float mean_square_a, float dt_s,
                          int sampling_ok, int enabled);
float current_limit_voltage(const CurrentLimit *lim, const CurrentLimitConfig *cfg,
                            float requested_v);
void current_limit_clear_trip(CurrentLimit *lim);

#endif
