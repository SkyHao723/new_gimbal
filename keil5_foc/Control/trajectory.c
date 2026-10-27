#include "trajectory.h"
#include <math.h>
#include <string.h>

void trajectory_reset(Trajectory *path, float position)
{
    memset(path, 0, sizeof(*path));
    path->position = path->target = position;
}

static void append_segment(Trajectory *path, float duration, float acceleration)
{
    if (duration <= 0.0f) return;
    path->duration[path->count] = duration;
    path->acceleration[path->count] = acceleration;
    ++path->count;
}

void trajectory_plan(Trajectory *path, float target, float velocity_limit,
                     float acceleration_limit, float deceleration_limit)
{
    float direction, distance, velocity, peak, accel_distance, brake_distance, cruise_distance;
    float vmax = velocity_limit;
    float amax = acceleration_limit;
    float bmax = deceleration_limit;
    path->target = target;
    path->segment = path->count = 0u;
    path->segment_position = path->position;
    path->segment_velocity = path->velocity;
    path->segment_elapsed = 0.0;
    distance = target-path->position;
    direction = distance >= 0.0f ? 1.0f : -1.0f;
    distance = fabsf(distance);
    velocity = direction*path->velocity;

    /* Retargeting keeps the current planned velocity. Brake first if moving
       away from the new target, rather than reversing velocity in one loop. */
    if (velocity < 0.0f) {
        append_segment(path, -velocity/bmax, direction*bmax);
        distance += velocity*velocity/(2.0f*bmax);
        velocity = 0.0f;
    }
    brake_distance = velocity*velocity/(2.0f*bmax);
    if (brake_distance > distance) {
        /* Target lies inside stopping distance: stop beyond it, then return. */
        append_segment(path, velocity/bmax, -direction*bmax);
        distance = brake_distance-distance;
        direction = -direction;
        velocity = 0.0f;
    }
    if (velocity > vmax) {
        /* A lowered L limit decelerates smoothly instead of clipping speed. */
        append_segment(path, (velocity-vmax)/bmax, -direction*bmax);
        distance -= (velocity*velocity-vmax*vmax)/(2.0f*bmax);
        velocity = vmax;
    }
    peak = sqrtf(fmaxf(0.0f, (2.0f*amax*bmax*distance+bmax*velocity*velocity)/(amax+bmax)));
    peak = fmaxf(velocity, fminf(vmax, peak));
    accel_distance = (peak*peak-velocity*velocity)/(2.0f*amax);
    brake_distance = peak*peak/(2.0f*bmax);
    cruise_distance = fmaxf(0.0f, distance-accel_distance-brake_distance);
    append_segment(path, (peak-velocity)/amax, direction*amax);
    if (peak > 0.0f) append_segment(path, cruise_distance/peak, 0.0f);
    append_segment(path, peak/bmax, -direction*bmax);
}

void trajectory_step(Trajectory *path, float dt)
{
    double remaining = dt;
    while (remaining > 0.0 && path->segment < path->count) {
        double left = path->duration[path->segment]-path->segment_elapsed;
        double step = remaining < left ? remaining : left;
        double acceleration = path->acceleration[path->segment];
        double elapsed = path->segment_elapsed+step;
        double position = path->segment_position+path->segment_velocity*elapsed
                          +0.5*acceleration*elapsed*elapsed;
        double velocity = path->segment_velocity+acceleration*elapsed;
        path->position = (float)position;
        path->velocity = (float)velocity;
        path->segment_elapsed = elapsed;
        remaining -= step;
        if (step == left) {
            path->segment_position = position;
            path->segment_velocity = velocity;
            path->segment_elapsed = 0.0;
            ++path->segment;
        }
    }
    if (path->segment == path->count) {
        path->position = path->target;
        path->velocity = 0.0f;
    }
}
