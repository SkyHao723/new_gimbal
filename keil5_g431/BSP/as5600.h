#ifndef AS5600_H
#define AS5600_H
#include <stdint.h>

typedef struct {
    float mechanical;
    float angle;
    float velocity;
    int32_t turns;
    uint32_t timestamp;
    uint32_t errors;
    uint8_t valid;
} Encoder;

void as5600_init(void);
int as5600_update(Encoder *sensor);
#endif
