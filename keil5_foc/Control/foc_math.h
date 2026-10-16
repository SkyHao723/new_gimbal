#ifndef FOC_MATH_H
#define FOC_MATH_H
#define FOC_TWO_PI 6.28318530718f

typedef struct {
    float integral;
    float error_prev;
    float output_prev;
} VelocityPI;

float foc_clamp(float x, float low, float high);
float foc_normalize(float angle);
float foc_sin(float angle);
void foc_svpwm(float q, float d, float electrical_angle, float supply, float duty[3]);
float foc_velocity_pi(VelocityPI *pi, float error, float dt, float limit);
#endif
