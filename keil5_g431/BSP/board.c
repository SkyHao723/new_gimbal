#include "board.h"
#include "board_config.h"
#include "clock.h"
#include "current_sense.h"
#include "motor_config.h"
#include "diagnostics.h"
#include <stdio.h>

static volatile uint8_t stop_requested;
static uint16_t pwm_period;
static uint32_t pwm_low_window;
/* nSLEEP -> ready time of the DRV8316C (charge pump and gate driver). */
#define DRIVER_WAKE_MS 2u

static void driver_off(void)
{
    /* DRVOFF is strapped to PGND on this board, so clearing MOE leaves the
       bridge braking through its low-side FETs, not off. Pulling nSLEEP low is
       the only way to reach real high impedance. */
    DRIVER_PORT->BSRR = DRIVER_OFF_PIN | ((uint32_t)DRIVER_SLEEP_PIN << 16);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

void board_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* Set the output latch before changing the GPIO mode. */
    DRIVER_PORT->BSRR = DRIVER_OFF_PIN | ((uint32_t)DRIVER_SLEEP_PIN << 16);
    gpio.Pin = DRIVER_SLEEP_PIN | DRIVER_OFF_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DRIVER_PORT, &gpio);
    if (!clock_init()) for (;;) {}

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    gpio.Pin = DRIVER_FAULT_PIN;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DRIVER_PORT, &gpio);
    HAL_NVIC_SetPriority(DRIVER_FAULT_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(DRIVER_FAULT_IRQn);

    __HAL_RCC_TIM1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* At 170 MHz, ARR=3400 gives exactly 25 kHz center-aligned PWM.
       MOE stays clear until a successful explicit E command. */
    pwm_period = (uint16_t)(HAL_RCC_GetPCLK2Freq() / (2u * MOTOR_PWM_HZ));
    TIM1->PSC = 0;
    TIM1->ARR = pwm_period;
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    /* OC3/OC4 are PWM mode 1/2: the high sides are on around CNT = 0, so OC4
       rising inside the all-low-side window is the current-sense trigger.
       PA11 is not routed to TIM1, so CC4E only enables the internal signal. */
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (7u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = pwm_period / 2u;
    TIM1->CCR4 = pwm_period / 2u;
    pwm_low_window = (uint32_t)(2u * (pwm_period - pwm_period / 2u)) - CS_TRIG_GUARD;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CEN;

    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = HAL_RCC_GetPCLK1Freq() / 1000000u - 1u;
    TIM2->ARR = 0xffffffffu;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;

    /* PA5 LED: +3.3V -> resistor -> LED -> PA5 (active low). */
    GPIOA->BSRR = GPIO_PIN_5;
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Wake the driver once with MOE still clear: the SOx outputs only carry a
       meaningful bias while the device is awake, and current_sense_init()
       measures those biases at standstill before any drive is possible. */
    DRIVER_PORT->BSRR = DRIVER_SLEEP_PIN;
    HAL_Delay(10u);
}

uint32_t board_micros(void) { return TIM2->CNT; }
uint32_t board_millis(void) { return HAL_GetTick(); }
void board_led(int on) { GPIOA->BSRR = on ? (uint32_t)GPIO_PIN_5 << 16 : GPIO_PIN_5; }

int board_driver_set(int enabled)
{
    uint32_t irq;
    int ok;

    if (!enabled || stop_requested) {
        irq = __get_PRIMASK();
        __disable_irq();
        driver_off();
        __set_PRIMASK(irq);
        return 0;
    }

    /* nFAULT latches low at power-up (VM ramping) and after any real fault. A
       low pulse on nSLEEP is the only reset, so retry the wake cycle once
       instead of refusing the enable outright. */
    if (!(DRIVER_PORT->IDR & DRIVER_FAULT_PIN) && !board_driver_wake()) {
        irq = __get_PRIMASK();
        __disable_irq();
        driver_off();
        __set_PRIMASK(irq);
        return 0;
    }

    /* nSLEEP has to rise before the bridge can follow PWM, and the wake-up
       delay needs SysTick, so it runs outside the critical section that
       protects MOE. */
    DRIVER_PORT->BSRR = DRIVER_SLEEP_PIN;
    HAL_Delay(DRIVER_WAKE_MS);

    irq = __get_PRIMASK();
    __disable_irq();
    ok = 0;
    if (!stop_requested && (DRIVER_PORT->IDR & DRIVER_FAULT_PIN)) {
        /* Load neutral preloaded compares before exposing PWM to the bridge. */
        TIM1->EGR = TIM_EGR_UG;
        TIM1->BDTR |= TIM_BDTR_MOE;
        DRIVER_PORT->BSRR = (uint32_t)DRIVER_OFF_PIN << 16;
        ok = 1;
        if (!(DRIVER_PORT->IDR & DRIVER_FAULT_PIN)) {
            driver_off();
            stop_requested |= BOARD_STOP_DRIVER;
            ok = 0;
        }
    } else {
        driver_off();
    }
    __set_PRIMASK(irq);
    return ok;
}

/* Wake the driver without enabling PWM, for a standstill current-sense
   calibration. nSLEEP low is also the fault latch reset, so a latched fault is
   pulsed away first. Returns 1 when the driver is awake and fault free. */
int board_driver_wake(void)
{
    if (!(DRIVER_PORT->IDR & DRIVER_FAULT_PIN)) {
        driver_off();
        HAL_Delay(1u);
    }
    DRIVER_PORT->BSRR = DRIVER_SLEEP_PIN;
    HAL_Delay(DRIVER_WAKE_MS);
    return (DRIVER_PORT->IDR & DRIVER_FAULT_PIN) ? 1 : 0;
}

void board_pwm(float a, float b, float c){
    uint16_t ca = (uint16_t)(a * pwm_period);
    uint16_t cb = (uint16_t)(b * pwm_period);
    uint16_t cc = (uint16_t)(c * pwm_period);
    uint16_t top = ca;
    uint16_t trig;

    if (cb > top) top = cb;
    if (cc > top) top = cc;
    TIM1->CCR1 = ca;
    TIM1->CCR2 = cb;
    TIM1->CCR3 = cc;
    /* Park the current-sense trigger right after the last high-side turn-off.
       A trigger above ARR never fires; current_limit treats a silent trigger
       as a blind sensor and drops to the voltage floor. */
    trig = (uint16_t)(top + CS_TRIG_GUARD);
    TIM1->CCR4 = (trig > pwm_period) ? (uint16_t)(pwm_period + 1u) : trig;
    pwm_low_window = (top < pwm_period) ? (uint32_t)(2u * (pwm_period - top)) : 0u;
    pwm_low_window = (pwm_low_window > CS_TRIG_GUARD)
                   ? pwm_low_window - CS_TRIG_GUARD : 0u;
}

uint32_t board_pwm_low_window(void) { return pwm_low_window; }

int board_take_stop(void)
{
    uint32_t irq = __get_PRIMASK();
    int stop;
    __disable_irq();
    stop = stop_requested;
    stop_requested = 0u;
    __set_PRIMASK(irq);
    return stop;
}

void board_request_stop(unsigned int reason)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    driver_off();
    stop_requested |= (uint8_t)reason;
    __set_PRIMASK(irq);
}

void EXTI2_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(DRIVER_FAULT_PIN)) {
        board_request_stop(BOARD_STOP_DRIVER);
        __HAL_GPIO_EXTI_CLEAR_IT(DRIVER_FAULT_PIN);
    }
}

void SysTick_Handler(void) { HAL_IncTick(); }
static void fault_stop(void)
{
    driver_off();
    g431_diagnostics.stage = 0xe002u;
    g431_diagnostics.off_high = 1u;
    for (;;) {}
}
void NMI_Handler(void) { fault_stop(); }
void HardFault_Handler(void) { fault_stop(); }
void MemManage_Handler(void) { fault_stop(); }
void BusFault_Handler(void) { fault_stop(); }
void UsageFault_Handler(void) { fault_stop(); }
