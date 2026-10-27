#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <stdint.h>

/* At most one initial braking segment followed by accel/cruise/decel. */
typedef struct {
    float position, velocity, target;
    float duration[4], acceleration[4];
    /* Evaluate each segment from its start, avoiding drift from repeatedly
       adding tiny float displacements to a multi-turn absolute position. */
    double segment_position, segment_velocity, segment_elapsed;
    uint8_t segment, count;
} Trajectory;

void trajectory_reset(Trajectory *path, float position);
/* Limits must be finite and strictly positive. Planning preserves position
   and velocity; a reduced speed limit is reached by bounded deceleration. */
void trajectory_plan(Trajectory *path, float target, float velocity_limit,
                     float acceleration_limit, float deceleration_limit);
void trajectory_step(Trajectory *path, float dt);
#endif
