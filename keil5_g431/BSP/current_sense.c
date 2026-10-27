#include "stm32g4xx_hal.h"
#include "current_sense.h"
#include "board.h"

/* Three-phase low-side current sensing for the DRV8316C on the G431 board.
 *
 * Clocking: ADC12 kernel clock = PLLP (170 MHz) / ADC_CCR_PRESC(4) = 42.5 MHz,
 * inside the 60 MHz limit. Sampling 12.5 cycles + 12.5 conversion cycles =
 * 25 cycles per channel, 75 cycles for the injected group = 1.76 us.
 *
 * Triggering: TIM1 CC4 is configured by board.c as PWM mode 2 with
 * CCR4 = max(CCR1..3) + CS_TRIG_GUARD. In centre-aligned mode 1 the high side
 * is on around CNT = 0 and off around CNT = ARR, so OC4REF rising at CCR4
 * opens the all-low-side window: every phase current then flows through its
 * low-side shunt and the SOx outputs are valid.
 *
 * Offset: measured once at standstill with the driver awake but MOE clear. A
 * failed calibration is not fatal, but it forces current_limit to the
 * conservative voltage floor. */

static volatile float s_amp[3];
static volatile float s_offset[3];
static volatile float s_peak;
static volatile float s_msq;
static volatile uint16_t s_raw[3];
static volatile uint32_t s_samples;
static volatile uint32_t s_trips;
static volatile uint32_t s_last_sample_us;
static volatile uint32_t s_cal_sum[3];
static volatile uint32_t s_cal_count;
static volatile uint8_t s_state;   /* 0 idle, 1 calibrating, 2 running */
static volatile uint8_t s_valid;
static volatile uint8_t s_armed;
static volatile uint8_t s_hw_ok;
static volatile uint8_t s_trigger;  /* 1 = TIM1_CC4, 2 = TIM1_TRGO(OC4REF) */
static volatile uint16_t s_probe[3];
static uint8_t s_over_count;
static const char *s_error = "";

static float fabs_f(float v) { return v < 0.0f ? -v : v; }

static void spin_us(uint32_t us)
{
    uint32_t start = board_micros();
    while ((uint32_t)(board_micros() - start) < us) {
    }
}

static void adc_pins_analog(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* One calibration attempt with whatever ADC12 kernel clock is selected now.
   ADCAL is cleared by hardware when the calibration engine finishes; it never
   finishes without a kernel clock, so a timeout here is also a clock probe. */
static int adc_calibrate(void)
{
    uint32_t guard = 200000u;
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->CR & ADC_CR_ADCAL) && --guard) {
    }
    return guard ? 1 : 0;
}

/* Trigger the injected group directly from the TIM1 CC4 event. */
static void adc_trigger_cc4(void)
{
    MODIFY_REG(ADC1->JSQR, ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN,
               (1u << ADC_JSQR_JEXTSEL_Pos) | (1u << ADC_JSQR_JEXTEN_Pos));
    s_trigger = 1u;
}

/* Fallback route: TIM1 TRGO = OC4REF (CR2.MMS = 111) and the injected group
   triggers on TIM1_TRGO (JEXTSEL = 0, rising edge). */
static void adc_trigger_trgo(void)
{
    MODIFY_REG(TIM1->CR2, TIM_CR2_MMS, 7u << TIM_CR2_MMS_Pos);
    MODIFY_REG(ADC1->JSQR, ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN,
               (1u << ADC_JSQR_JEXTEN_Pos));
    s_trigger = 2u;
}

/* One software-started injected sequence at standstill: proves the ADC core and
   the analog path work, and its raw values show where the SOx bias sits. */
static int adc_probe(void)
{
    uint32_t guard;

    MODIFY_REG(ADC1->JSQR, ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN, 0u);
    ADC1->CR |= ADC_CR_JADSTP;
    guard = 200000u;
    while ((ADC1->CR & ADC_CR_JADSTP) && --guard) {
    }
    ADC1->CR |= ADC_CR_JADSTART;
    guard = 200000u;
    while (!(ADC1->ISR & ADC_ISR_JEOC) && --guard) {
    }
    if (!(ADC1->ISR & ADC_ISR_JEOC)) return 0;
    ADC1->ISR = ADC_ISR_JEOC;
    s_probe[0] = (uint16_t)ADC1->JDR1;
    s_probe[1] = (uint16_t)ADC1->JDR2;
    s_probe[2] = (uint16_t)ADC1->JDR3;
    return 1;
}

/* Power up and calibrate ADC1, then set the injected 3-channel group to be
   triggered by TIM1 CC4 on every PWM period. */
static int adc_start(void)
{
    uint32_t guard;

    adc_pins_analog();

    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    (void)RCC->AHB2ENR;                                  /* keep the clock gating visible */
    /* Divide the ADC12 kernel clock by four inside the ADC: 170 MHz / 4 =
       42.5 MHz, inside the 60 MHz limit. */
    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_PRESC, ADC_CCR_PRESC_1);

    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    spin_us(20u);                                        /* t_ADCVREG_STUP */

    /* RM0440: ADCAL requires ADEN = 0, i.e. calibrate right after power-up.
       PLLP is the documented kernel clock, but the ADC hangs forever in ADCAL
       if that output is not actually running, so fall back to SYSCLK. The
       stuck calibration finishes as soon as a clock arrives. */
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_ADC12SEL, RCC_CCIPR_ADC12SEL_0);
    if (!adc_calibrate()) {
        MODIFY_REG(RCC->CCIPR, RCC_CCIPR_ADC12SEL, RCC_CCIPR_ADC12SEL_1);
        if (!adc_calibrate()) {
            s_error = "ADC clock";
            return 0;
        }
    }

    /* Queue disabled (JQDIS = 1) is the classic single-sequence behaviour: one
       trigger converts the whole JSQR sequence, and a software start works with
       JEXTEN = 0. With the queue enabled (JQDIS = 0) the HAL itself rejects
       JEXTEN = 0 as "empty queue", and a software start then only arms the group
       forever - which is exactly the stuck JADSTART seen on the bench. Must be
       written while ADSTART and JADSTART are both clear. */
    ADC1->CFGR = ADC_CFGR_JQDIS;
    ADC1->SMPR1 = (2u << ADC_SMPR1_SMP1_Pos) |           /* 12.5 cycles */
                  (2u << ADC_SMPR1_SMP2_Pos) |
                  (2u << ADC_SMPR1_SMP3_Pos);
    ADC1->JSQR = (2u << ADC_JSQR_JL_Pos) |               /* JL = 2 -> 3 channels */
                 (1u << ADC_JSQR_JSQ1_Pos) |             /* IN1 = PA0 = SOA */
                 (2u << ADC_JSQR_JSQ2_Pos) |             /* IN2 = PA1 = SOB */
                 (3u << ADC_JSQR_JSQ3_Pos);              /* IN3 = PA2 = SOC */

    ADC1->CR |= ADC_CR_ADEN;
    guard = 200000u;
    while (!(ADC1->ISR & ADC_ISR_ADRDY) && --guard) {
    }
    if (!guard) {
        s_error = "ADC enable";
        return 0;
    }

    /* Prove the analog path once with a software start before blaming the
       timer trigger: the raw values also tell whether SOx sits at mid-scale. */
    if (!adc_probe()) {
        s_error = "ADC conv";
        return 0;
    }
    adc_trigger_cc4();
    ADC1->CR |= ADC_CR_JADSTART;

    ADC1->ISR = ADC_ISR_ADRDY | ADC_ISR_JEOC | ADC_ISR_JQOVF;
    ADC1->IER |= ADC_IER_JEOCIE;
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    s_hw_ok = 1u;
    return 1;
}

void ADC1_2_IRQHandler(void)
{
    float a0, a1, a2, peak;
    uint16_t r0, r1, r2;

    if (!(ADC1->ISR & ADC_ISR_JEOC)) {
        /* Overrun / spurious entry: acknowledge so the interrupt cannot storm. */
        ADC1->ISR = ADC_ISR_JQOVF;
        return;
    }
    ADC1->ISR = ADC_ISR_JEOC;
    r0 = (uint16_t)ADC1->JDR1;
    r1 = (uint16_t)ADC1->JDR2;
    r2 = (uint16_t)ADC1->JDR3;

    if (s_state == 1u) {
        s_cal_sum[0] += r0;
        s_cal_sum[1] += r1;
        s_cal_sum[2] += r2;
        ++s_cal_count;
        return;
    }
    if (s_state != 2u) return;

    s_raw[0] = r0;
    s_raw[1] = r1;
    s_raw[2] = r2;
    s_last_sample_us = board_micros();

    a0 = ((float)r0 - s_offset[0]) * CS_AMP_PER_COUNT;
    a1 = ((float)r1 - s_offset[1]) * CS_AMP_PER_COUNT;
    a2 = ((float)r2 - s_offset[2]) * CS_AMP_PER_COUNT;
    s_amp[0] = a0;
    s_amp[1] = a1;
    s_amp[2] = a2;
    s_msq = (a0 * a0 + a1 * a1 + a2 * a2) / 3.0f;
    peak = fabs_f(a0);
    if (fabs_f(a1) > peak) peak = fabs_f(a1);
    if (fabs_f(a2) > peak) peak = fabs_f(a2);
    s_peak = peak;
    ++s_samples;

    if (!s_armed) return;
    if (!(TIM1->BDTR & TIM_BDTR_MOE)) return;
    if (peak > CS_ISR_HARD_TRIP_A) {
        ++s_trips;
        board_request_stop(BOARD_STOP_CURRENT);
    } else if (peak > CS_ISR_TRIP_A) {
        /* Two consecutive samples above the soft trip: noise alone is gone by
           the first one, a real overcurrent survives both. */
        if (++s_over_count >= 2u) {
            s_over_count = 0u;
            ++s_trips;
            board_request_stop(BOARD_STOP_CURRENT);
        }
    } else {
        s_over_count = 0u;
    }
}

static int calibrate_offsets(void)
{
    int i;
    int in_range = 1;
    uint32_t start;

    s_cal_sum[0] = s_cal_sum[1] = s_cal_sum[2] = 0u;
    s_cal_count = 0u;
    s_valid = 0u;
    s_state = 1u;
    if (!(ADC1->CR & ADC_CR_JADSTART)) ADC1->CR |= ADC_CR_JADSTART;
    start = board_micros();
    while (s_cal_count < CS_CAL_SAMPLES &&
           (uint32_t)(board_micros() - start) < 20000u) {
    }
    if (s_cal_count < CS_CAL_SAMPLES) {
        s_state = 0u;
        s_error = "no JEOC";
        return 0;
    }
    for (i = 0; i < 3; ++i) {
        s_offset[i] = (float)s_cal_sum[i] / (float)s_cal_count;
        /* A bias outside the middle 80 % of the input range means the pin is
           not connected to a live SOx output; keep the numbers for the log but
           do not trust the reading. */
        if (s_offset[i] < 0.1f * CS_ADC_COUNTS || s_offset[i] > 0.9f * CS_ADC_COUNTS) {
            in_range = 0;
        }
    }
    if (!in_range) {
        s_state = 0u;
        s_error = "SOx bias";
        return 0;
    }
    s_valid = 1u;
    s_state = 2u;
    s_error = "";
    return 1;
}

int current_sense_init(void)
{
    if (!s_hw_ok && !adc_start()) return 0;
    if (calibrate_offsets()) return 1;
    /* The direct CC4 trigger input produced no samples: retry on the timer TRGO
       output, which reaches the ADC through a different route on silicon. */
    if (s_trigger == 1u) {
        adc_trigger_trgo();
        if (calibrate_offsets()) return 1;
    }
    return 0;
}

int current_sense_hw_ok(void) { return s_hw_ok ? 1 : 0; }

int current_sense_trigger(void) { return (int)s_trigger; }

uint16_t current_sense_probe_raw(int index)
{
    return (index >= 0 && index < 3) ? s_probe[index] : 0u;
}

const char *current_sense_error(void) { return s_error; }

void current_sense_arm(int armed) { s_armed = armed ? 1u : 0u; }

int current_sense_valid(void) { return s_valid ? 1 : 0; }

int current_sense_recent(void)
{
    if (!s_valid) return 0;
    return (uint32_t)(board_micros() - s_last_sample_us) < CS_SAMPLE_TIMEOUT_US;
}

float current_sense_peak(void) { return s_peak; }
float current_sense_mean_square(void) { return s_msq; }

float current_sense_offset(int index)
{
    return (index >= 0 && index < 3) ? s_offset[index] : 0.0f;
}

float current_sense_amp(int index)
{
    return (index >= 0 && index < 3) ? s_amp[index] : 0.0f;
}

uint16_t current_sense_raw(int index)
{
    return (index >= 0 && index < 3) ? s_raw[index] : 0u;
}

uint32_t current_sense_samples(void) { return s_samples; }
uint32_t current_sense_trips(void) { return s_trips; }

void current_sense_track_zero(void)
{
    /* The DRV8316C bias (VREF/2) is present only while the device is awake, and
       it moves when the device sleeps or comes back. Track it slowly so a
       standstill reading stays at 0 A, but refuse a step larger than the soft
       trip: a real current must never be absorbed into the offset. */
    static const float alpha = 0.05f;
    int i;

    if (!s_valid || s_state != 2u) return;
    for (i = 0; i < 3; ++i) {
        float raw = (float)s_raw[i];
        float delta = raw - s_offset[i];
        if (delta > CS_ISR_TRIP_A / CS_AMP_PER_COUNT) continue;
        if (delta < -CS_ISR_TRIP_A / CS_AMP_PER_COUNT) continue;
        s_offset[i] += delta * alpha;
    }
}
