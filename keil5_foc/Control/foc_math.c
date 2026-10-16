/* C adaptation of SimpleFOC 2.4.0: foc_utils.cpp, pid.cpp, BLDCMotor.cpp.
 * Copyright (c) 2020 Antun Skuric. MIT license: ../LICENSE-SimpleFOC.txt. */
#include "foc_math.h"
#include "motor_config.h"
#include <math.h>
#include <stdint.h>

float foc_clamp(float x, float low, float high)
{
    return x < low ? low : (x > high ? high : x);
}

float foc_normalize(float angle)
{
    float a = fmodf(angle, FOC_TWO_PI);
    return a < 0.0f ? a + FOC_TWO_PI : a;
}

float foc_sin(float angle)
{
    static const uint16_t table[65] = {
        0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,
        10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,
        18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,
        24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,
        29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,
        32286,32413,32522,32610,32679,32729,32758,32768
    };
    uint32_t index = (uint32_t)(angle * (65536.0f / FOC_TWO_PI));
    int32_t fraction = (int32_t)(index & 255u);
    int32_t a, b;
    index = (index >> 8) & 255u;
    if (index < 64u) { a = table[index]; b = table[index + 1u]; }
    else if (index < 128u) { a = table[128u-index]; b = table[127u-index]; }
    else if (index < 192u) { a = -(int32_t)table[index-128u]; b = -(int32_t)table[index-127u]; }
    else { a = -(int32_t)table[256u-index]; b = -(int32_t)table[255u-index]; }
    return (a + (((b-a)*fraction) >> 8)) / 32768.0f;
}

void foc_svpwm(float q, float d, float electrical_angle, float supply, float duty[3])
{
    float angle = foc_normalize(electrical_angle);
    float s = foc_sin(angle);
    float c = foc_sin(foc_normalize(angle + 1.57079632679f));
    float alpha = c*d - s*q;
    float beta = s*d + c*q;
    float a = alpha;
    float b = -0.5f*alpha + 0.86602540378f*beta;
    float e = -0.5f*alpha - 0.86602540378f*beta;
    float low = a < b ? a : b;
    float high = a > b ? a : b;
    float center;
    if (e < low) low = e;
    if (e > high) high = e;
    center = supply*0.5f - (high+low)*0.5f;
    duty[0] = foc_clamp((a+center)/supply, 0.0f, 1.0f);
    duty[1] = foc_clamp((b+center)/supply, 0.0f, 1.0f);
    duty[2] = foc_clamp((e+center)/supply, 0.0f, 1.0f);
}

float foc_velocity_pi(VelocityPI *pi, float error, float dt, float limit)
{
    float output, step;
    pi->integral += MOTOR_VELOCITY_I * dt * 0.5f * (error+pi->error_prev);
    pi->integral = foc_clamp(pi->integral, -limit, limit);
    output = foc_clamp(MOTOR_VELOCITY_P*error + pi->integral, -limit, limit);
    step = MOTOR_VOLTAGE_RAMP * dt;
    output = foc_clamp(output, pi->output_prev-step, pi->output_prev+step);
    /* Also enforce a newly lowered runtime voltage limit immediately. */
    output = foc_clamp(output, -limit, limit);
    pi->output_prev = output;
    pi->error_prev = error;
    return output;
}
