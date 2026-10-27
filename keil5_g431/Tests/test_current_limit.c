#include "current_limit.h"
#include <math.h>
#include <stdio.h>

static int failures;

static void check(int cond, const char *what)
{
    if (!cond) {
        printf("FAIL: %s\n", what);
        ++failures;
    }
}

static void check_near(float got, float want, const char *what)
{
    if (fabsf(got - want) > 1e-3f) {
        printf("FAIL: %s (got %.4f want %.4f)\n", what, (double)got, (double)want);
        ++failures;
    }
}

int main(void)
{
    CurrentLimit lim;
    const CurrentLimitConfig *cfg = &current_limit_defaults;

    /* The defaults must mirror User/motor_config.h. */
    check_near(cfg->rated_a, 1.4f, "rated");
    check_near(cfg->soft_a, 1.6f, "soft");
    check_near(cfg->trip_a, 2.6f, "trip");
    check_near(cfg->voltage_floor, 1.5f, "floor");
    check_near(cfg->voltage_ceiling, 8.0f, "ceiling");

    /* 1. Init leaves the full ceiling available. */
    current_limit_init(&lim);
    check_near(lim.allowed_v, 8.0f, "init allowed");
    check_near(lim.peak_a, 0.0f, "init peak");
    check(lim.tripped == 0u && lim.blind == 0u && lim.trips == 0u, "init flags");

    /* 2. Disabled: nothing is limited, the peak detector is drained. */
    current_limit_init(&lim);
    lim.thermal = 0.5f;
    current_limit_update(&lim, cfg, 5.0f, 100.0f, 0.1f, 1, 0);
    check_near(lim.allowed_v, 8.0f, "disabled allowed");
    check_near(lim.peak_a, 0.0f, "disabled peak");
    check(lim.tripped == 0u, "disabled does not trip");
    check_near(lim.thermal, 0.4875f, "disabled thermal decay");

    /* 3. No usable sampling: only the floor is honest. */
    current_limit_init(&lim);
    current_limit_update(&lim, cfg, 0.0f, 0.0f, 0.01f, 0, 1);
    check(lim.blind == 1u, "blind set");
    check_near(lim.allowed_v, 1.5f, "blind floor");

    /* 4. Below the soft threshold nothing is folded. */
    current_limit_init(&lim);
    current_limit_update(&lim, cfg, 1.0f, 1.0f, 0.01f, 1, 1);
    check(lim.blind == 0u, "blind cleared");
    check_near(lim.allowed_v, 8.0f, "below soft");

    /* 5. Above soft the ceiling is folded at fold_v_per_a. */
    current_limit_init(&lim);
    current_limit_update(&lim, cfg, 2.0f, 1.96f, 0.01f, 1, 1);
    check_near(lim.allowed_v, 4.0f, "fold at 2.0 A");
    check(lim.tripped == 0u, "2.0 A does not trip");

    /* 6. At the trip level the stop latches and the allowance hits the floor. */
    current_limit_init(&lim);
    current_limit_update(&lim, cfg, 2.6f, 1.96f, 0.01f, 1, 1);
    check(lim.tripped == 1u, "trip latched");
    check(lim.trips == 1u, "trip counted");
    check_near(lim.allowed_v, 1.5f, "trip floor");
    current_limit_clear_trip(&lim);
    check(lim.tripped == 0u, "trip cleared");
    check(lim.trips == 1u, "trip count kept");

    /* 7. The I^2t budget removes voltage even when the peak is small. */
    current_limit_init(&lim);
    current_limit_update(&lim, cfg, 0.0f, 7.84f, 0.1f, 1, 1);
    check_near(lim.allowed_v, 7.7f, "thermal fold");

    /* 8. A single spike releases again; half the decay time halves the peak. */
    current_limit_init(&lim);
    current_limit_update(&lim, cfg, 2.0f, 1.96f, 0.0f, 1, 1);
    check_near(lim.peak_a, 2.0f, "peak held");
    current_limit_update(&lim, cfg, 0.0f, 0.0f, 0.025f, 1, 1);
    check_near(lim.peak_a, 1.0f, "peak half released");
    current_limit_update(&lim, cfg, 0.0f, 0.0f, 0.05f, 1, 1);
    check_near(lim.peak_a, 0.0f, "peak fully released");
    check_near(lim.allowed_v, 8.0f, "ceiling restored");

    /* 9. The voltage gate never exceeds the allowance or the ceiling. */
    current_limit_init(&lim);
    check_near(current_limit_voltage(&lim, cfg, 8.0f), 8.0f, "gate 8");
    check_near(current_limit_voltage(&lim, cfg, 2.0f), 2.0f, "gate 2");
    check_near(current_limit_voltage(&lim, cfg, 20.0f), 8.0f, "gate ceiling");
    check_near(current_limit_voltage(&lim, cfg, -1.0f), 0.0f, "gate non-negative");
    lim.allowed_v = 1.5f;
    check_near(current_limit_voltage(&lim, cfg, 4.0f), 1.5f, "gate floor");

    if (failures != 0) {
        printf("%d current_limit check(s) failed\n", failures);
        return 1;
    }
    printf("current_limit: all checks passed\n");
    return 0;
}
