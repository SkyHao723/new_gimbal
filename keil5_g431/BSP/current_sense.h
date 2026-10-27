#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H
#include <stdint.h>

/* DRV8316C current sense, per schematic docs/hardware/g431-current-2026-10-20:
     SOA/SOB/SOC (U4 pin 40/39/38) -> PA0/PA1/PA2 = ADC1_IN1/IN2/IN3,
     straight wires, no RC filter and no series resistor.
     GAIN pin (31) tied to PGND            -> 0.15 V/A
     VREF/ILIM pin (37) tied to AVDD (3V3) -> bipolar output centred on 1.65 V
   Only the datasheet slope is assumed here; the real zero offset of every
   channel is measured at standstill before the bridge is ever enabled, so a
   wrong bias model cannot shift the measured current. */

#define CS_GAIN_V_PER_A     0.15f
#define CS_VREF_V           3.3f
#define CS_ADC_COUNTS       4096.0f
#define CS_V_PER_COUNT      (CS_VREF_V / CS_ADC_COUNTS)          /* 0.000806 V */
#define CS_AMP_PER_COUNT    (CS_V_PER_COUNT / CS_GAIN_V_PER_A)   /* 0.005371 A */

/* TIM1 CC4 is the ADC1 injected trigger. board_pwm() parks CCR4 inside the
   all-low-side window (CCR4 = max(CCR1..3) + CS_TRIG_GUARD) so the low-side
   shunt current of all three phases is sampled after the switching edge has
   settled. */
#define CS_TRIG_GUARD  700u   /* 4.12 us at 170 MHz */
#define CS_CONV_COUNTS      300u   /* 3 x 25 ADC cycles at 42.5 MHz, in 170 MHz counts */

/* Instantaneous protection inside the ADC interrupt. The slower fold-back and
   the I^2t budget live in User/current_limit.c. */
#define CS_ISR_TRIP_A       2.6f
#define CS_ISR_HARD_TRIP_A  4.5f

#define CS_CAL_SAMPLES      32u
#define CS_SAMPLE_TIMEOUT_US 5000u

int  current_sense_init(void);
int  current_sense_hw_ok(void);
int  current_sense_trigger(void);      /* 1 = TIM1_CC4, 2 = TIM1_TRGO */
const char *current_sense_error(void);
void current_sense_arm(int armed);
/* Re-learn the zero bias from live samples. Only call while the bridge output is
   disabled (MOE = 0) so no current can flow through the shunts. */
void current_sense_track_zero(void);
int  current_sense_valid(void);
int  current_sense_recent(void);
float current_sense_peak(void);
float current_sense_mean_square(void);
float current_sense_offset(int index);
float current_sense_amp(int index);
uint16_t current_sense_raw(int index);
uint16_t current_sense_probe_raw(int index);   /* software-trigger self test */
uint32_t current_sense_samples(void);
uint32_t current_sense_trips(void);

#endif
