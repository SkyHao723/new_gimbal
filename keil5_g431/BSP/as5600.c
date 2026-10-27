#include "board_config.h"
#include "as5600.h"
#include "board.h"
#include "encoder_tracking.h"

#define I2C_HALF_PERIOD_US 25u
#define I2C_STRETCH_TIMEOUT_US 250u
#define AS5600_ADDRESS_WRITE 0x6cu
#define AS5600_ADDRESS_READ  0x6du
#define AS5600_CONF_REG       0x07u
#define AS5600_CONF_PMOD_MASK 0x03u
#define AS5600_PMOD_REFRESH   1024u

/* Diagnostics for SWD: [15:8] = CONF read before the fix, [7:0] = CONF read
   back after the fix (0xFF when the write did not stick). */
volatile uint32_t as5600_conf_log;
/* [0]=angle updates seen, [1]=reads, [2]=last raw, [3]=min us between updates,
   [4]=max us between updates, [5]=CONF readback. */
volatile uint32_t as5600_stats[6];

static int initialized;
static uint32_t pmod_tick;
static uint16_t last_raw;
static uint32_t last_change_us;
static int have_last;

static void pause_us(uint32_t delay)
{
    uint32_t start = board_micros();
    while ((uint32_t)(board_micros() - start) < delay) {}
}

static void scl_low(void)
{
    BOARD_ENCODER_PORT->BSRR = (uint32_t)BOARD_ENCODER_SCL_PIN << 16;
}

static void scl_release(void)
{
    BOARD_ENCODER_PORT->BSRR = BOARD_ENCODER_SCL_PIN;
}

static int scl_wait_high(void)
{
    uint32_t start = board_micros();
    scl_release();
    while ((BOARD_ENCODER_PORT->IDR & BOARD_ENCODER_SCL_PIN) == 0u) {
        if ((uint32_t)(board_micros() - start) >= I2C_STRETCH_TIMEOUT_US) return 0;
    }
    pause_us(I2C_HALF_PERIOD_US);
    return 1;
}

static void sda_low(void)
{
    BOARD_ENCODER_PORT->BSRR = (uint32_t)BOARD_ENCODER_SDA_PIN << 16;
}

static void sda_release(void)
{
    BOARD_ENCODER_PORT->BSRR = BOARD_ENCODER_SDA_PIN;
}

static void i2c_stop(void)
{
    sda_low();
    pause_us(I2C_HALF_PERIOD_US);
    if (scl_wait_high()) {
        sda_release();
        pause_us(I2C_HALF_PERIOD_US);
    } else {
        sda_release();
    }
}

static int i2c_start(void)
{
    sda_release();
    pause_us(I2C_HALF_PERIOD_US);
    if (!scl_wait_high()) return 0;
    if ((BOARD_ENCODER_PORT->IDR & BOARD_ENCODER_SDA_PIN) == 0u) return 0;
    sda_low();
    pause_us(I2C_HALF_PERIOD_US);
    scl_low();
    return 1;
}

static int i2c_write_byte(uint8_t value)
{
    unsigned int bit;
    int ack;
    for (bit = 0u; bit < 8u; ++bit) {
        if (value & 0x80u) sda_release();
        else sda_low();
        pause_us(I2C_HALF_PERIOD_US);
        if (!scl_wait_high()) return 0;
        scl_low();
        value <<= 1;
    }
    sda_release();
    pause_us(I2C_HALF_PERIOD_US);
    if (!scl_wait_high()) return 0;
    ack = (BOARD_ENCODER_PORT->IDR & BOARD_ENCODER_SDA_PIN) == 0u;
    scl_low();
    return ack;
}

static int i2c_read_byte(uint8_t *value, int ack)
{
    unsigned int bit;
    uint8_t result = 0u;
    sda_release();
    for (bit = 0u; bit < 8u; ++bit) {
        result <<= 1;
        pause_us(I2C_HALF_PERIOD_US);
        if (!scl_wait_high()) return 0;
        if (BOARD_ENCODER_PORT->IDR & BOARD_ENCODER_SDA_PIN) result |= 1u;
        scl_low();
    }
    if (ack) sda_low();
    else sda_release();
    pause_us(I2C_HALF_PERIOD_US);
    if (!scl_wait_high()) return 0;
    scl_low();
    sda_release();
    *value = result;
    return 1;
}

static int read_registers(uint8_t reg, uint8_t *data, unsigned int count)
{
    unsigned int i;
    int ok = i2c_start() &&
             i2c_write_byte(AS5600_ADDRESS_WRITE) &&
             i2c_write_byte(reg) &&
             i2c_start() &&
             i2c_write_byte(AS5600_ADDRESS_READ);
    for (i = 0u; ok && i < count; ++i) {
        ok = i2c_read_byte(data + i, i + 1u < count);
    }
    i2c_stop();
    return ok;
}

static int write_register(uint8_t reg, uint8_t value)
{
    int ok = i2c_start() &&
             i2c_write_byte(AS5600_ADDRESS_WRITE) &&
             i2c_write_byte(reg) &&
             i2c_write_byte(value);
    i2c_stop();
    return ok;
}

/* The AS5600 watches for magnet movement; when the magnet has not moved for a
   while it falls back to a low power mode (PMOD = LPM3, 100 ms polling) and the
   angle registers then only refresh at ~10 Hz.  Reading CONF shows what was
   programmed, not which state machine is actually running, so the value is
   re-written unconditionally instead of only when the readback looks wrong. */
static int force_normal_power_mode(void)
{
    uint8_t conf;
    if (!read_registers(AS5600_CONF_REG, &conf, 1u)) {
        return 0;
    }
    as5600_stats[5] = conf;
    return write_register(AS5600_CONF_REG,
                          (uint8_t)(conf & (uint8_t)~AS5600_CONF_PMOD_MASK));
}

static void recover_bus(void)
{
    unsigned int i;
    sda_release();
    scl_release();
    for (i = 0u; i < 9u; ++i) {
        scl_low();
        pause_us(I2C_HALF_PERIOD_US);
        (void)scl_wait_high();
    }
    i2c_stop();
}

void as5600_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    initialized = 0;
    BOARD_ENCODER_GPIO_CLOCK_ENABLE();
    /* PB6 is also UCPD1_CC1; this board does not use USB-C/UCPD. */
    HAL_PWREx_DisableUCPDDeadBattery();
    gpio.Pin = BOARD_ENCODER_SCL_PIN | BOARD_ENCODER_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL; /* External 3.3 V pull-ups are fitted. */
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_ENCODER_PORT, &gpio);
    sda_release();
    scl_release();
    recover_bus();
    initialized = ((BOARD_ENCODER_PORT->IDR &
                    (BOARD_ENCODER_SCL_PIN | BOARD_ENCODER_SDA_PIN)) ==
                   (BOARD_ENCODER_SCL_PIN | BOARD_ENCODER_SDA_PIN));
    if (initialized) {
        uint8_t before = 0u;
        uint8_t after = 0u;
        int fixed;
        (void)read_registers(AS5600_CONF_REG, &before, 1u);
        fixed = force_normal_power_mode();
        (void)read_registers(AS5600_CONF_REG, &after, 1u);
        as5600_conf_log = ((uint32_t)before << 8) | (fixed ? after : 0xFFu);
        pmod_tick = 0u;
        as5600_stats[0] = 0u;
        as5600_stats[1] = 0u;
        as5600_stats[2] = 0u;
        as5600_stats[3] = 0xFFFFFFFFu;
        as5600_stats[4] = 0u;
        as5600_stats[5] = 0u;
        have_last = 0;
        last_raw = 0u;
        last_change_us = 0u;
    }
}

uint32_t as5600_ms_since_change(void)
{
    if (!have_last) {
        return 0u;
    }
    return (uint32_t)(board_micros() - last_change_us) / 1000u;
}

int as5600_update(Encoder *sensor)
{
    uint8_t data[2];
    uint16_t raw;
    uint32_t now;
    if (!initialized || !read_registers(0x0cu, data, 2u)) {
        /* Never continue control with a stale angle. */
        board_driver_set(0);
        sensor->errors++;
        recover_bus();
        initialized = ((BOARD_ENCODER_PORT->IDR &
                        (BOARD_ENCODER_SCL_PIN | BOARD_ENCODER_SDA_PIN)) ==
                       (BOARD_ENCODER_SCL_PIN | BOARD_ENCODER_SDA_PIN));
        return 0;
    }
    raw = (uint16_t)(((data[0] & 15u) << 8) | data[1]);
    now = board_micros();
    as5600_stats[1]++;
    if (!have_last || raw != last_raw) {
        if (have_last) {
            uint32_t dt = now - last_change_us;
            if (dt < as5600_stats[3]) {
                as5600_stats[3] = dt;
            }
            if (dt > as5600_stats[4]) {
                as5600_stats[4] = dt;
            }
        }
        last_raw = raw;
        last_change_us = now;
        have_last = 1;
        as5600_stats[0]++;
        as5600_stats[2] = raw;
    }
    encoder_accept_raw(sensor, raw, now);
    if (++pmod_tick >= AS5600_PMOD_REFRESH) {
        pmod_tick = 0u;
        (void)force_normal_power_mode();
    }
    return 1;
}
