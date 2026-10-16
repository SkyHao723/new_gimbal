#ifndef ENCODER_TRACKING_H
#define ENCODER_TRACKING_H
#include "as5600.h"
void encoder_accept_raw(Encoder *sensor, uint16_t raw, uint32_t now);
#endif
