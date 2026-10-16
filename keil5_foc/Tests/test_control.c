/* Host-side checks: analytical PWM reference, encoder rollover, and lifecycle.
 * No hardware access; mock sensor samples exercise the production controller. */
#include "motor.h"
#include "encoder_tracking.h"
#include "board.h"
#include "motor_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t fake_us, fake_ms;
static int driver_enabled, sensor_ok=1;
static uint16_t fake_raw=1000u;
static float last_duty[3];

uint32_t board_micros(void) { return fake_us; }
uint32_t board_millis(void) { return fake_ms; }
int board_driver_set(int enable) { driver_enabled=enable; return enable; }
void board_pwm(float a, float b, float c) { last_duty[0]=a; last_duty[1]=b; last_duty[2]=c; }
int as5600_update(Encoder *s)
{
    if (!sensor_ok) return 0;
    encoder_accept_raw(s, fake_raw, fake_us);
    return 1;
}

static void tick(Motor *m)
{
    fake_us+=1000u;
    fake_ms+=1u;
    motor_step(m);
}

static void test_pwm(void)
{
    int i, j;
    float max_error=0.0f;
    for (i=-600; i<=600; ++i) {
        float a=i*0.021f;
        for (j=-5; j<=5; ++j) {
            float q=(float)j, d=0.3f*j, duty[3], actual_alpha, actual_beta;
            float ref_alpha=cosf(a)*d-sinf(a)*q;
            float ref_beta=sinf(a)*d+cosf(a)*q;
            float error;
            foc_svpwm(q,d,a,24.0f,duty);
            assert(duty[0]>=0 && duty[0]<=1 && duty[1]>=0 && duty[1]<=1 && duty[2]>=0 && duty[2]<=1);
            /* Reconstruct the voltage vector from line voltages: common mode cancels. */
            actual_alpha=24.0f*(2*duty[0]-duty[1]-duty[2])/3.0f;
            actual_beta=24.0f*(duty[1]-duty[2])/sqrtf(3.0f);
            error=fmaxf(fabsf(actual_alpha-ref_alpha),fabsf(actual_beta-ref_beta));
            if (error>max_error) max_error=error;
            assert(error<0.002f);
        }
    }
    printf("PWM analytical vector sweep: 13211 cases, max error %.6f V\n",max_error);
}

static void test_encoder(void)
{
    Encoder e={0};
    uint32_t now=0xfffff000u;
    int i;
    encoder_accept_raw(&e,4000u,now);
    for (i=4001; i<=4000+8192; ++i) {
        now+=1000u;
        encoder_accept_raw(&e,(uint16_t)(i%4096),now);
    }
    assert(e.turns==2);
    assert(fabsf(e.angle-(4000.0f/4096.0f+2.0f)*FOC_TWO_PI)<0.00001f);
    for (i=4000+8191; i>=4000; --i) {
        now+=1000u;
        encoder_accept_raw(&e,(uint16_t)(i%4096),now);
    }
    assert(e.turns==0);
    assert(fabsf(e.angle-4000.0f/4096.0f*FOC_TWO_PI)<0.00001f);
    for (i=0; i<1000; ++i) { now+=1000u; encoder_accept_raw(&e,4000u,now); }
    assert(fabsf(e.velocity)<0.0001f);
    puts("Encoder: forward/reverse two turns, timestamp wrap, zero-speed decay PASS");
}

static void test_pi(void)
{
    VelocityPI pi={0};
    int i;
    for (i=0; i<10000; ++i) {
        float q=foc_velocity_pi(&pi,200.0f,0.001f,3.0f);
        assert(q<=3.0f && q>=-3.0f && fabsf(pi.integral)<=3.0f);
    }
    assert(fabsf(foc_velocity_pi(&pi,200.0f,0.001f,0.25f))<=0.25f);
    puts("PI: sustained saturation/anti-windup and lowered voltage limit PASS");
}

static void test_motor(void)
{
    Motor m;
    float start, integral;
    int i;
    motor_init(&m);
    assert(m.startup && driver_enabled);
    start=m.angle;
    for (i=1; i<=4096 && m.enabled; ++i) {
        fake_raw=(uint16_t)((1000+i)%4096);
        tick(&m);
    }
    assert(!m.enabled && !m.startup && !driver_enabled);
    assert(m.angle-start>=FOC_TWO_PI-STARTUP_ROTATE_TOLERANCE-0.002f);
    assert(m.angle-start<FOC_TWO_PI+0.002f);
    motor_command(&m,"T100");
    motor_command(&m,"E");
    assert(m.enabled && m.target==m.angle);
    motor_command(&m,"T8");
    tick(&m);
    integral=m.pi.integral;
    motor_command(&m,"E");
    assert(m.target==8.0f && m.pi.integral==integral);
    assert(fabsf(m.ramp_target-m.angle)<=0.0041f);
    motor_command(&m,"Tnan"); assert(m.target==8.0f);
    motor_command(&m,"T1junk"); assert(m.target==8.0f);
    motor_command(&m,"T   "); assert(m.target==8.0f);
    motor_command(&m,"V99"); assert(m.voltage_limit==5.0f);
    motor_command(&m,"L-1"); assert(m.velocity_limit==20.0f);
    motor_command(&m,"D"); tick(&m); assert(!m.enabled && !driver_enabled);
    motor_command(&m,"E"); sensor_ok=0; tick(&m);
    assert(!m.enabled && !driver_enabled);
    motor_command(&m,"E"); assert(!m.enabled);
    sensor_ok=1;
    motor_init(&m);
    fake_ms+=10000u; tick(&m); assert(!m.enabled && !driver_enabled);
    puts("Motor: measured 360deg, timeout, E idempotence, slew, parsing, limits, D, sensor fault PASS");
}

int main(void)
{
    test_pwm(); test_encoder(); test_pi(); test_motor();
    puts("ALL HOST CHECKS PASSED (not a hardware test)");
    return 0;
}
