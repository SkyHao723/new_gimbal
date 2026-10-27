#include "board_config.h"
#include "as5600.h"
#include "board.h"
#include "encoder_tracking.h"

#define I2C_HALF_PERIOD_US 25u
#define I2C_STRETCH_TIMEOUT_US 250u
#define AS5600_ADDRESS_WRITE 0x6cu
#define AS5600_ADDRESS_READ  0x6du

static int initialized;

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
}

int as5600_update(Encoder *sensor)
{
    uint8_t data[2];
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
    encoder_accept_raw(sensor,
        (uint16_t)(((data[0] & 15u) << 8) | data[1]), board_micros());
    return 1;
}
