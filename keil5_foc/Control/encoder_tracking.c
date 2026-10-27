#include "encoder_tracking.h"
#include "foc_math.h"
#include "motor_config.h"

void encoder_accept_raw(Encoder *sensor, uint16_t raw, uint32_t now)
{
    float mechanical = raw * (FOC_TWO_PI / 4096.0f);
    float delta, dt, velocity;
    if (!sensor->valid) {
        sensor->mechanical = mechanical;
        sensor->angle = mechanical;
        sensor->velocity = 0.0f;
        sensor->timestamp = now;
        sensor->valid = 1u;
        return;
    }
    delta = mechanical - sensor->mechanical;
    if (delta > 0.8f*FOC_TWO_PI) { --sensor->turns; delta -= FOC_TWO_PI; }
    else if (delta < -0.8f*FOC_TWO_PI) { ++sensor->turns; delta += FOC_TWO_PI; }
    dt = (uint32_t)(now-sensor->timestamp)*1e-6f;
    if (dt > 0.0f && dt <= 0.5f) {
#if defined(ENCODER_VELOCITY_WINDOW) && (ENCODER_VELOCITY_WINDOW > 1)
        /* Rolling baseline: velocity over the last N loop periods.  One LSB of
           a 12-bit encoder over 0.52 ms is ~2.9 rad/s, which is pure noise in
           the velocity loop; summing N deltas cuts that by N. */
        sensor->vel_index = (uint8_t)((sensor->vel_index + 1u) % ENCODER_VELOCITY_WINDOW);
        sensor->vel_sum += delta - sensor->vel_hist[sensor->vel_index];
        sensor->vel_hist[sensor->vel_index] = delta;
        velocity = sensor->vel_sum / (dt * (float)ENCODER_VELOCITY_WINDOW);
#else
        velocity = delta/dt;
#endif
        sensor->velocity += dt/(MOTOR_VELOCITY_FILTER_TF+dt)*(velocity-sensor->velocity);
    } else sensor->velocity = 0.0f;
    sensor->mechanical = mechanical;
    sensor->angle = sensor->turns*FOC_TWO_PI + mechanical;
    sensor->timestamp = now;
}
