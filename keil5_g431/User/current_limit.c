#include "current_limit.h"
#include "motor_config.h"

const CurrentLimitConfig current_limit_defaults = {
    CURRENT_LIMIT_RATED_A,
    CURRENT_LIMIT_SOFT_A,
    CURRENT_LIMIT_TRIP_A,
    CURRENT_LIMIT_FOLD_V_PER_A,
    CURRENT_LIMIT_VOLTAGE_FLOOR,
    MOTOR_MAX_VOLTAGE,
    CURRENT_LIMIT_PEAK_DECAY_S,
    CURRENT_LIMIT_THERMAL_TAU_S
};

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void current_limit_init(CurrentLimit *lim)
{
    lim->peak_a = 0.0f;
    lim->thermal = 0.0f;
    lim->allowed_v = current_limit_defaults.voltage_ceiling;
    lim->mean_square_a = 0.0f;
    lim->trips = 0u;
    lim->tripped = 0u;
    lim->blind = 0u;
}

void current_limit_clear_trip(CurrentLimit *lim) { lim->tripped = 0u; }

void current_limit_update(CurrentLimit *lim, const CurrentLimitConfig *cfg,
                          float peak_a, float mean_square_a, float dt_s,
                          int sampling_ok, int enabled)
{
    float released, heat, allowed, ref;

    if (dt_s < 0.0f) dt_s = 0.0f;

    if (!enabled) {
        /* Idle: drain the thermal budget and the peak detector, no limit. */
        lim->peak_a = 0.0f;
        lim->mean_square_a = 0.0f;
        lim->thermal = clampf(lim->thermal - dt_s / cfg->thermal_tau_s, 0.0f, 1.0f);
        lim->allowed_v = cfg->voltage_ceiling;
        lim->blind = 0u;
        return;
    }

    if (!sampling_ok) {
        /* Without current feedback the only honest allowance is the floor. */
        lim->blind = 1u;
        lim->peak_a = 0.0f;
        lim->allowed_v = cfg->voltage_floor;
        return;
    }
    lim->blind = 0u;

    /* Peak detector with a linear release, so one spike cannot hold the
       voltage down for long but a sustained overload keeps it down. */
    released = lim->peak_a - lim->peak_a * (dt_s / cfg->peak_decay_s);
    lim->peak_a = peak_a > released ? peak_a : released;
    lim->mean_square_a = mean_square_a;

    /* I^2t style budget: heating when the mean square exceeds rated^2. */
    ref = cfg->rated_a * cfg->rated_a;
    heat = (ref > 0.0f) ? (mean_square_a / ref - 1.0f) : 0.0f;
    lim->thermal = clampf(lim->thermal + heat * dt_s / cfg->thermal_tau_s, 0.0f, 1.0f);

    /* Fold the voltage command back towards the floor as the peak rises. */
    allowed = cfg->voltage_ceiling;
    if (lim->peak_a > cfg->soft_a) {
        allowed -= (lim->peak_a - cfg->soft_a) * cfg->fold_v_per_a;
    }
    allowed *= (1.0f - lim->thermal);
    lim->allowed_v = clampf(allowed, cfg->voltage_floor, cfg->voltage_ceiling);

    if (lim->peak_a >= cfg->trip_a) {
        lim->tripped = 1u;
        ++lim->trips;
    }
}

float current_limit_voltage(const CurrentLimit *lim, const CurrentLimitConfig *cfg,
                            float requested_v)
{
    float v = requested_v;
    if (v > lim->allowed_v) v = lim->allowed_v;
    if (v > cfg->voltage_ceiling) v = cfg->voltage_ceiling;
    if (v < 0.0f) v = 0.0f;
    return v;
}
