/* Standalone DRVOFF-grounded bring-up with AS5600 feedback diagnostics.
 * No boot motion. SWD requests: 1=wake/check, 2=1.5V pulse, 3=stop,
 * 4=2.0V pulse, 6=software I2C recovery.
 * 9=explicit 1.5V open-loop magnetic-warning diagnostic (MD required).
 * Every pulse ends independently after 1200ms in SysTick. Normal commands
 * require healthy magnet; command 9 tolerates ML for open-loop diagnosis only.
 * Encoder communication is required; this is not calibrated closed-loop FOC.
 * PB0/nSLEEP low and TIM1.MOE clear are the actual stop paths; PB1 is input.
 * Only for the independently powered MCU configuration confirmed by user. */
#include "stm32g4xx_hal.h"
#include "clock.h"
#include "foc_math.h"

typedef struct {
    uint32_t magic, request, ack, stage, clock_hz, uptime_ms, loops;
    uint32_t sleep_high, fault_high, moe, active, stop_reason;
    uint32_t started_ms, ended_ms, pulses, updates, gpiob_idr;
    uint32_t cfsr, hfsr;
    float uq, electrical_angle;
    uint32_t pwm_period, heartbeat;
    uint32_t sensor_ok, sensor_reads, sensor_errors, raw, magnet_status;
    uint32_t magnet_errors, sensor_ms;
    float sensor_angle, sensor_velocity, motion_start_angle, motion_end_angle;
    int32_t encoder_counts;
    uint32_t magnet_agc, magnet_magnitude, magnet_data_ms;
} BringupDiag;
volatile BringupDiag bringup_diag;
static volatile uint32_t active, started, stop_reason;
static uint16_t period;
static uint32_t wake_ms;
static uint32_t last_sensor_ms, previous_raw, have_sample;
static uint32_t soft_mode;
static uint32_t warning_pulse;
static uint32_t last_magnet_ms;

static void soft_wait(void)
{
    uint32_t t = TIM2->CNT;
    while ((uint32_t)(TIM2->CNT-t) < 25u) {}
}
static int soft_clock_high(void)
{
    uint32_t t = TIM2->CNT;
    GPIOB->BSRR = GPIO_PIN_6;
    while (!(GPIOB->IDR & GPIO_PIN_6))
        if ((uint32_t)(TIM2->CNT-t)>250u) return 0;
    soft_wait(); return 1;
}
static void soft_stop(void)
{
    GPIOB->BSRR = (uint32_t)GPIO_PIN_7<<16;
    soft_wait(); (void)soft_clock_high();
    GPIOB->BSRR = GPIO_PIN_7; soft_wait();
}
static int soft_start(void)
{
    GPIOB->BSRR = GPIO_PIN_7; soft_wait();
    if (!soft_clock_high() || !(GPIOB->IDR & GPIO_PIN_7)) return 0;
    GPIOB->BSRR = (uint32_t)GPIO_PIN_7<<16; soft_wait();
    GPIOB->BSRR = (uint32_t)GPIO_PIN_6<<16; return 1;
}
static int soft_write(uint8_t value)
{
    unsigned int bit;
    int ack;
    for (bit=0; bit<8; ++bit) {
        GPIOB->BSRR = value&0x80u ? GPIO_PIN_7 : (uint32_t)GPIO_PIN_7<<16;
        soft_wait(); if (!soft_clock_high()) return 0;
        GPIOB->BSRR = (uint32_t)GPIO_PIN_6<<16; value <<= 1;
    }
    GPIOB->BSRR = GPIO_PIN_7; soft_wait();
    if (!soft_clock_high()) return 0;
    ack = !(GPIOB->IDR & GPIO_PIN_7);
    GPIOB->BSRR = (uint32_t)GPIO_PIN_6<<16;
    return ack;
}
static int soft_read_byte(uint8_t *value, int ack)
{
    unsigned int bit;
    *value = 0; GPIOB->BSRR = GPIO_PIN_7;
    for (bit=0; bit<8; ++bit) {
        soft_wait(); if (!soft_clock_high()) return 0;
        *value = (uint8_t)((*value<<1) | !!(GPIOB->IDR & GPIO_PIN_7));
        GPIOB->BSRR = (uint32_t)GPIO_PIN_6<<16;
    }
    GPIOB->BSRR = ack ? (uint32_t)GPIO_PIN_7<<16 : GPIO_PIN_7;
    soft_wait(); if (!soft_clock_high()) return 0;
    GPIOB->BSRR = (uint32_t)GPIO_PIN_6<<16;
    GPIOB->BSRR = GPIO_PIN_7; return 1;
}
static int soft_read_bytes(uint8_t reg, uint8_t *data, unsigned int count)
{
    unsigned int i;
    int ok = soft_start() && soft_write(0x6cu) && soft_write(reg) &&
             soft_start() && soft_write(0x6du);
    for (i=0; ok && i<count; ++i) ok = soft_read_byte(data+i, i+1<count);
    soft_stop(); return ok;
}
static void soft_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    unsigned int i;
    GPIOB->BSRR = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7; gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    for (i=0; i<9; ++i) {
        GPIOB->BSRR = (uint32_t)GPIO_PIN_6<<16; soft_wait();
        (void)soft_clock_high();
    }
    soft_stop(); soft_mode = 1u;
}

static void stop_drive(uint32_t reason)
{
    GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
    active = 0;
    warning_pulse = 0;
    stop_reason = reason;
    bringup_diag.ended_ms = HAL_GetTick();
    bringup_diag.uq = 0.0f;
    bringup_diag.motion_end_angle = bringup_diag.sensor_angle;
}

static void encoder_poll(uint32_t now)
{
    uint8_t data[3];
    uint32_t raw, elapsed = now-last_sensor_ms;
    int32_t delta;
    float velocity, dt;
    if (elapsed < 4u) return; /* Nominal 250 samples/s. */
    last_sensor_ms = now;
    if (!soft_read_bytes(0x0bu, data, 3u)) {
        bringup_diag.sensor_ok = 0; ++bringup_diag.sensor_errors;
        if (active) stop_drive(8u);
        return;
    }
    ++bringup_diag.sensor_reads;
    raw = ((data[1]&15u)<<8) | data[2];
    bringup_diag.raw = raw; bringup_diag.magnet_status = data[0];
    /* Reuse project encoder_test.c MD/ML/MH decoding. */
    if ((data[0]&0x38u) != 0x20u) {
        bringup_diag.sensor_ok = 0; ++bringup_diag.magnet_errors;
        /* Only command 9 tolerates ML; missing magnet or MH still stops. */
        if (active && !(warning_pulse && (data[0]&0x38u)==0x30u)) stop_drive(9u);
    } else bringup_diag.sensor_ok = 1;
    if (!have_sample) {
        bringup_diag.encoder_counts = (int32_t)raw; have_sample = 1u;
    } else {
        delta = (int32_t)raw-(int32_t)previous_raw;
        if (delta > 2048) delta -= 4096;
        else if (delta < -2048) delta += 4096;
        bringup_diag.encoder_counts += delta;
        dt = (now-bringup_diag.sensor_ms)*0.001f;
        velocity = delta*(FOC_TWO_PI/4096.0f)/dt;
        bringup_diag.sensor_velocity += dt/(0.02f+dt)*(velocity-bringup_diag.sensor_velocity);
    }
    previous_raw = raw; bringup_diag.sensor_ms = now;
    bringup_diag.sensor_angle = bringup_diag.encoder_counts*(FOC_TWO_PI/4096.0f);
    if (soft_mode && (uint32_t)(now-last_magnet_ms)>=100u) {
        last_magnet_ms = now;
        if (soft_read_bytes(0x0bu, data, 1u)) bringup_diag.magnet_status = data[0];
        if (soft_read_bytes(0x1au, data, 3u)) {
            bringup_diag.magnet_agc = data[0];
            bringup_diag.magnet_magnitude = ((data[1]&15u)<<8) | data[2];
            bringup_diag.magnet_data_ms = now;
        }
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (active && (uint32_t)(HAL_GetTick()-started) >= 1200u) stop_drive(1u);
}
void EXTI2_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2)) {
        if (active) stop_drive(2u);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    }
}
static void fatal(void)
{
    stop_drive(4u);
    bringup_diag.stage = 0xe002u;
    bringup_diag.cfsr = SCB->CFSR;
    bringup_diag.hfsr = SCB->HFSR;
    for (;;) {} /* independent watchdog resets back into disabled boot */
}
void NMI_Handler(void) { fatal(); }
void HardFault_Handler(void) { fatal(); }
void MemManage_Handler(void) { fatal(); }
void BusFault_Handler(void) { fatal(); }
void UsageFault_Handler(void) { fatal(); }

static void pwm(float q, float angle)
{
    float duty[3];
    uint32_t irq;
    foc_svpwm(q, 0.0f, angle, 24.0f, duty);
    irq = __get_PRIMASK();
    __disable_irq();
    /* The timeout/fault ISR owns disable: never restore output after it. */
    if (active) {
        TIM1->CCR1 = (uint16_t)(period*duty[0]);
        TIM1->CCR2 = (uint16_t)(period*duty[1]);
        TIM1->CCR3 = (uint16_t)(period*duty[2]);
        bringup_diag.uq = q;
        bringup_diag.electrical_angle = angle;
        ++bringup_diag.updates;
    }
    __set_PRIMASK(irq);
}

int main(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t last_us, last_diag = 0, req;
    float electrical = 0.0f, voltage = 1.5f;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    /* PB1 has no physical DRVOFF connection on this board. */
    gpio.Pin = GPIO_PIN_1; gpio.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &gpio);
    bringup_diag.magic = 0x42555033u;
    bringup_diag.stage = 1u;
    if (!clock_init()) { bringup_diag.stage = 0xe001u; for (;;) {} }
    /* No USB-C in this board. Avoid reset-default CC pull-down interaction. */
    HAL_PWREx_DisableUCPDDeadBattery();
    bringup_diag.clock_hz = SystemCoreClock;
    gpio.Pin = GPIO_PIN_2; gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP; HAL_GPIO_Init(GPIOB, &gpio);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0u, 0u); HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    __HAL_RCC_TIM1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP; gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH; gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
    period = (uint16_t)(HAL_RCC_GetPCLK2Freq()/(2u*25000u));
    TIM1->PSC = 0; TIM1->ARR = period;
    TIM1->CCMR1 = (6u<<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                    (6u<<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6u<<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CEN;
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = HAL_RCC_GetPCLK1Freq()/1000000u-1u;
    TIM2->ARR = 0xffffffffu; TIM2->EGR = TIM_EGR_UG; TIM2->CR1 = TIM_CR1_CEN;
    gpio.Pin = GPIO_PIN_5; gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* About 0.5s nominal, clock tolerance does not govern the 1.2s pulse. */
    IWDG->KR = 0x5555; IWDG->PR = 4u; IWDG->RLR = 250u;
    IWDG->KR = 0xaaaa; IWDG->KR = 0xcccc;
    stop_drive(0u);
    /* Use the verified PB6/PB7 software bus by default; no PA15 jumper. */
    soft_init();
    bringup_diag.stage = 2u;
    bringup_diag.pwm_period = period;
    last_us = TIM2->CNT;
    for (;;) {
        uint32_t now = HAL_GetTick(), us = TIM2->CNT;
        float dt = (uint32_t)(us-last_us)*1e-6f;
        last_us = us;
        IWDG->KR = 0xaaaa;
        encoder_poll(now);
        ++bringup_diag.loops;
        req = bringup_diag.request;
        /* Leave request stable so the SWD programmer can verify its write. */
        if (req != 0u && req != bringup_diag.ack) {
            if (req == 3u) { stop_drive(3u); bringup_diag.stage = 2u; }
            else if ((req == 5u || req == 8u) && !active) {
                /* Reject obsolete hardware-I2C diagnostics on unsupported PB6. */
                stop_drive(10u); bringup_diag.stage = 2u;
            }
            else if (req == 6u && !active) {
                stop_drive(0u); soft_init(); bringup_diag.stage = 2u;
            }
            else if (req == 1u && !active) {
                stop_drive(0u);
                GPIOB->BSRR = GPIO_PIN_0;
                wake_ms = now; bringup_diag.stage = 3u;
            } else if ((req == 2u || req == 4u || req == 9u) && !active &&
                       bringup_diag.stage == 3u && (uint32_t)(now-wake_ms)>=100u &&
                       (bringup_diag.sensor_ok || (req == 9u && soft_mode && have_sample &&
                        (bringup_diag.magnet_status&0x38u)==0x30u)) &&
                       (uint32_t)(now-bringup_diag.sensor_ms)<20u &&
                       (GPIOB->IDR & GPIO_PIN_2) && (GPIOB->ODR & GPIO_PIN_0)) {
                uint32_t irq = __get_PRIMASK();
                __disable_irq();
                TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = period/2u;
                TIM1->EGR = TIM_EGR_UG;
                started = now; stop_reason = 0u; electrical = 0.0f;
                voltage = req == 4u ? 2.0f : 1.5f;
                warning_pulse = req == 9u;
                active = 1u; TIM1->BDTR |= TIM_BDTR_MOE;
                bringup_diag.started_ms = now; ++bringup_diag.pulses;
                bringup_diag.motion_start_angle = bringup_diag.sensor_angle;
                bringup_diag.stage = 4u;
                __set_PRIMASK(irq);
            } else { if (!active) stop_drive(5u); }
            bringup_diag.ack = req;
        }
        if (active) {
            float elapsed = (now-started)*0.001f;
            float ramp = elapsed < 0.15f ? elapsed/0.15f :
                         (elapsed > 1.05f ? (1.2f-elapsed)/0.15f : 1.0f);
            float speed = elapsed < 0.15f ? 0.0f :
                          (elapsed < 0.45f ? (elapsed-0.15f)/0.3f : 1.0f);
            if (!(GPIOB->IDR & GPIO_PIN_2)) stop_drive(2u);
            else if (dt <= 0.0f || dt > 0.01f) stop_drive(6u);
            else {
                electrical = foc_normalize(electrical+11.0f*speed*dt);
                pwm(voltage*foc_clamp(ramp,0.0f,1.0f), electrical);
            }
        } else if (bringup_diag.stage == 4u) bringup_diag.stage = 5u;
        /* A wake-only check never remains energized indefinitely. */
        if (!active && bringup_diag.stage == 3u && (uint32_t)(now-wake_ms)>=15000u) {
            stop_drive(7u); bringup_diag.stage = 2u;
        }
        if ((uint32_t)(now-last_diag)>=5u) {
            last_diag = now;
            bringup_diag.uptime_ms = now;
            bringup_diag.sleep_high = !!(GPIOB->ODR & GPIO_PIN_0);
            bringup_diag.fault_high = !!(GPIOB->IDR & GPIO_PIN_2);
            bringup_diag.moe = !!(TIM1->BDTR & TIM_BDTR_MOE);
            bringup_diag.active = active; bringup_diag.stop_reason = stop_reason;
            bringup_diag.gpiob_idr = GPIOB->IDR;
            bringup_diag.cfsr = SCB->CFSR; bringup_diag.hfsr = SCB->HFSR;
            ++bringup_diag.heartbeat;
            GPIOA->BSRR = (now/400u)&1u ? GPIO_PIN_5 : (uint32_t)GPIO_PIN_5<<16;
        }
    }
}
