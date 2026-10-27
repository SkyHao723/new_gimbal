#ifndef AS5600_H
#define AS5600_H
#include <stdint.h>
#include "motor_config.h"

/* Velocity from a single 12-bit LSB pair is far too coarse: one count per loop
   period is 2.9 rad/s at 1.9 kHz.  Sum the last ENCODER_VELOCITY_WINDOW angle
   deltas so the velocity baseline is window*period instead of one period. */
#ifndef ENCODER_VELOCITY_WINDOW
#define ENCODER_VELOCITY_WINDOW 1
#endif

typedef struct {
    float mechanical;
    float angle;
    float velocity;
    float vel_hist[ENCODER_VELOCITY_WINDOW];
    float vel_sum;
    uint8_t vel_index;
    int32_t turns;
    uint32_t timestamp;
    uint32_t errors;
    uint8_t valid;
} Encoder;

void as5600_init(void);
int as5600_update(Encoder *sensor);
#endif
