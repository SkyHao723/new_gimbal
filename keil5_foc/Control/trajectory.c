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

void trajectory_plan(Trajectory *path, float target, float velocity_limit, float acceleration_limit)
{
    float direction, distance, velocity, peak, accel_distance, brake_distance, cruise_distance;
    float vmax = velocity_limit;
    float amax = acceleration_limit;
    path->target = target;
    path->segment = path->count = 0u;
    path->segment_left = 0.0f;
    distance = target-path->position;
    direction = distance >= 0.0f ? 1.0f : -1.0f;
    distance = fabsf(distance);
    velocity = direction*path->velocity;

    /* Retargeting keeps the current planned velocity. Brake first if moving
       away from the new target, rather than reversing velocity in one loop. */
    if (velocity < 0.0f) {
        append_segment(path, -velocity/amax, direction*amax);
        distance += velocity*velocity/(2.0f*amax);
        velocity = 0.0f;
    }
    brake_distance = velocity*velocity/(2.0f*amax);
    if (brake_distance > distance) {
        /* Target lies inside stopping distance: stop beyond it, then return. */
        append_segment(path, velocity/amax, -direction*amax);
        distance = brake_distance-distance;
        direction = -direction;
        velocity = 0.0f;
    }
    if (velocity > vmax) {
        /* A lowered L limit decelerates smoothly instead of clipping speed. */
        append_segment(path, (velocity-vmax)/amax, -direction*amax);
        distance -= (velocity*velocity-vmax*vmax)/(2.0f*amax);
        velocity = vmax;
    }
    peak = sqrtf(fmaxf(0.0f, amax*distance+0.5f*velocity*velocity));
    peak = fmaxf(velocity, fminf(vmax, peak));
    accel_distance = (peak*peak-velocity*velocity)/(2.0f*amax);
    brake_distance = peak*peak/(2.0f*amax);
    cruise_distance = fmaxf(0.0f, distance-accel_distance-brake_distance);
    append_segment(path, (peak-velocity)/amax, direction*amax);
    if (peak > 0.0f) append_segment(path, cruise_distance/peak, 0.0f);
    append_segment(path, peak/amax, -direction*amax);
    if (path->count) path->segment_left = path->duration[0];
}

void trajectory_step(Trajectory *path, float dt)
{
    while (dt > 0.0f && path->segment < path->count) {
        float step = fminf(dt, path->segment_left);
        float acceleration = path->acceleration[path->segment];
        path->position += path->velocity*step + 0.5f*acceleration*step*step;
        path->velocity += acceleration*step;
        dt -= step;
        path->segment_left -= step;
        if (path->segment_left <= 0.0f) {
            ++path->segment;
            if (path->segment < path->count)
                path->segment_left = path->duration[path->segment];
        }
    }
    if (path->segment == path->count) {
        path->position = path->target;
        path->velocity = 0.0f;
    }
}
