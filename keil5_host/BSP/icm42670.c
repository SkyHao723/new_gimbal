#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_pwr_ex.h"
#include "icm42670.h"
#include "host_config.h"
#include <string.h>

static const char *last_error = "";
static uint32_t i2c_core_clock_hz;

/* ICM-42670-P bank0 寄存器 */
#define REG_TEMP_DATA1   0x09u
#define REG_PWR_MGMT0    0x1Fu
#define REG_GYRO_CONFIG0 0x20u
#define REG_ACCEL_CONFIG0 0x21u
#define REG_GYRO_DATA_X1 0x11u
#define REG_ACCEL_DATA_X1 0x0Bu

#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_HALF_PERIOD_US IMU_I2C_HALF_PERIOD_US
#define I2C_STRETCH_TIMEOUT_US 250u

const char *imu_last_error(void) { return last_error; }

static void i2c_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (i2c_core_clock_hz / 1000000u) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { }
}

static void i2c_gpio_init(void)
{
    uint32_t mode;

    /* PB6 is UCPD1_CC1 on STM32G431. Disable its dead-battery pull-down before
       using it as an open-drain I2C clock. */
    HAL_PWREx_DisableUCPDDeadBattery();
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    /* PB6 has no I2C1_SCL alternate function on STM32G431. Bit-bang both
       lines as open-drain GPIO instead of routing them to the I2C peripheral. */
    mode = GPIOB->MODER & ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOB->MODER = mode | (1u << GPIO_MODER_MODE6_Pos) |
                          (1u << GPIO_MODER_MODE7_Pos);
    GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7)) |
                   GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
    GPIOB->BSRR = I2C_SCL_PIN | I2C_SDA_PIN;
}

int imu_i2c_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    i2c_core_clock_hz = HAL_RCC_GetHCLKFreq();
    i2c_gpio_init();
    last_error = "";
    return 1;
}

static void scl_low(void) { GPIOB->BSRR = (uint32_t)I2C_SCL_PIN << 16; }
static void scl_release(void) { GPIOB->BSRR = I2C_SCL_PIN; }
static void sda_low(void) { GPIOB->BSRR = (uint32_t)I2C_SDA_PIN << 16; }
static void sda_release(void) { GPIOB->BSRR = I2C_SDA_PIN; }

static int scl_release_and_wait(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t timeout = (i2c_core_clock_hz / 1000000u) * I2C_STRETCH_TIMEOUT_US;
    scl_release();
    while ((GPIOB->IDR & I2C_SCL_PIN) == 0u) {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout) return 0;
    }
    i2c_delay_us(I2C_HALF_PERIOD_US);
    return 1;
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay_us(I2C_HALF_PERIOD_US);
    if (scl_release_and_wait()) {
        sda_release();
        i2c_delay_us(I2C_HALF_PERIOD_US);
    } else {
        sda_release();
    }
}

static int i2c_start(void)
{
    sda_release();
    i2c_delay_us(I2C_HALF_PERIOD_US);
    if (!scl_release_and_wait()) return 0;
    if ((GPIOB->IDR & I2C_SDA_PIN) == 0u) return 0;
    sda_low();
    i2c_delay_us(I2C_HALF_PERIOD_US);
    scl_low();
    i2c_delay_us(I2C_HALF_PERIOD_US);
    return 1;
}

static int i2c_write_byte(uint8_t value)
{
    unsigned int bit;
    int ack;
    for (bit = 0u; bit < 8u; ++bit) {
        if (value & 0x80u) sda_release(); else sda_low();
        i2c_delay_us(I2C_HALF_PERIOD_US);
        if (!scl_release_and_wait()) return 0;
        scl_low();
        value <<= 1;
    }
    sda_release();
    i2c_delay_us(I2C_HALF_PERIOD_US);
    if (!scl_release_and_wait()) return 0;
    ack = (GPIOB->IDR & I2C_SDA_PIN) == 0u;
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
        i2c_delay_us(I2C_HALF_PERIOD_US);
        if (!scl_release_and_wait()) return 0;
        if (GPIOB->IDR & I2C_SDA_PIN) result |= 1u;
        scl_low();
    }
    if (ack) sda_low(); else sda_release();
    i2c_delay_us(I2C_HALF_PERIOD_US);
    if (!scl_release_and_wait()) return 0;
    scl_low();
    sda_release();
    *value = result;
    return 1;
}

static int i2c_probe(uint8_t addr)
{
    int ack;
    if (!i2c_start()) return 0;
    ack = i2c_write_byte((uint8_t)(addr << 1));
    i2c_stop();
    return ack;
}

static int mem_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    int ok = i2c_start() && i2c_write_byte((uint8_t)(addr << 1)) &&
             i2c_write_byte(reg) && i2c_start() &&
             i2c_write_byte((uint8_t)((addr << 1) | 1u));
    for (i = 0u; ok && i < len; ++i) {
        ok = i2c_read_byte(&buf[i], i + 1u < len);
    }
    i2c_stop();
    if (!ok) last_error = "I2C read NACK or SCL timeout";
    return ok;
}

static int mem_write(uint8_t addr, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    int ok = i2c_start() && i2c_write_byte((uint8_t)(addr << 1)) &&
             i2c_write_byte(reg);
    for (i = 0u; ok && i < len; ++i) ok = i2c_write_byte(buf[i]);
    i2c_stop();
    if (!ok) last_error = "I2C write NACK or SCL timeout";
    return ok;
}

int imu_scan(uint8_t *found, int max)
{
    int count = 0;
    for (uint16_t addr = HOST_IMU_ADDR_SCAN_FIRST; addr <= HOST_IMU_ADDR_SCAN_LAST; ++addr) {
        if (i2c_probe((uint8_t)addr)) {
            if (count < max) found[count] = (uint8_t)addr;
            ++count;
        }
    }
    if (count == 0) last_error = "no I2C device at 10k GPIO-I2C";
    return count;
}

int imu_init(ImuInfo *imu)
{
    uint8_t who = 0;
    uint8_t addr = HOST_IMU_ADDR_PRIMARY;

    memset(imu, 0, sizeof(*imu));
    /* Address strap selects 0x68 or 0x69; WHO_AM_I is a separate register. */
    for (int a = 0; a < 2; ++a) {
        addr = (a == 0) ? HOST_IMU_ADDR_PRIMARY : HOST_IMU_ADDR_SECONDARY;
        if (mem_read(addr, HOST_IMU_WHO_AM_I_REG, &who, 1) && who == HOST_IMU_WHO_AM_I_VALUE) {
            imu->address = addr;
            imu->who_am_i = who;
            imu->speed_code = 0u;
            /* Bank0: accelerometer and gyroscope low-noise mode. */
            uint8_t pwr = 0x0Fu;
            if (!mem_write(addr, REG_PWR_MGMT0, &pwr, 1)) {
                last_error = "PWR_MGMT0 write failed";
                return 0;
            }
            HAL_Delay(50u);                            /* datasheet minimum is 45 ms */
            uint8_t cfg = 0x06u;                       /* gyro: ±2000 dps, 800 Hz */
            if (!mem_write(addr, REG_GYRO_CONFIG0, &cfg, 1)) return 0;
            cfg = 0x06u;                                /* accel: ±16 g, 800 Hz */
            if (!mem_write(addr, REG_ACCEL_CONFIG0, &cfg, 1)) return 0;
            last_error = "";
            return 1;
        }
    }
    if (!last_error[0]) last_error = "no answer at 0x68/0x69 or WHO_AM_I mismatch";
    return 0;
}

int imu_read_raw(const ImuInfo *imu, int16_t accel[3], int16_t gyro[3], int16_t *temp_raw)
{
    uint8_t buf[6];
    if (!mem_read(imu->address, REG_ACCEL_DATA_X1, buf, 6)) return 0;    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    if (!mem_read(imu->address, REG_GYRO_DATA_X1, buf, 6)) return 0;
    gyro[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyro[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyro[2] = (int16_t)((buf[4] << 8) | buf[5]);
    if (temp_raw) {
        uint8_t t[2];
        if (mem_read(imu->address, REG_TEMP_DATA1, t, 2)) {
            *temp_raw = (int16_t)((t[0] << 8) | t[1]);
        } else {
            *temp_raw = 0;
        }
    }
    return 1;
}

/* ------------------------------------------------------------------ *
 *  Bring-up bus diagnostics (results readable over SWD, no console).  *
 * ------------------------------------------------------------------ */

volatile uint32_t bus_diag[8];

#define BUS_SCL GPIO_IDR_ID6
#define BUS_SDA GPIO_IDR_ID7

static void bus_delay(void)
{
    for (volatile uint32_t i = 0; i < 600u; ++i) {
        __NOP();
    }
}

/* mode: 0 = input, 1 = output open-drain. pupd is used only in input mode. */
static void bus_pins_mode(uint32_t mode, uint32_t pupd)
{
    uint32_t m;

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;

    m = GPIOB->MODER;
    m &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    m |= (mode << GPIO_MODER_MODE6_Pos) | (mode << GPIO_MODER_MODE7_Pos);
    GPIOB->MODER = m;

    if (mode == 0u) {
        GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
        GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7)) | pupd;
    } else {
        GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
        GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
    }
    bus_delay();
}

static void bus_scl_write(int high)
{
    GPIOB->BSRR = high ? GPIO_BSRR_BS6 : GPIO_BSRR_BR6;
}

static void bus_sda_write(int high)
{
    GPIOB->BSRR = high ? GPIO_BSRR_BS7 : GPIO_BSRR_BR7;
}

/* ---- software I2C probe: bypasses the peripheral, tests the wiring ---- */
volatile uint32_t sw_scan[4];   /* [0] hits (SCL=PB6/SDA=PB7), [1] first addr,
                                   [2] hits (swapped),          [3] first addr   */

static void sw_delay(void)
{
    for (volatile uint32_t i = 0; i < 400u; ++i) {
        __NOP();
    }
}

static void sw_pins(void)
{
    uint32_t m;

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    m = GPIOB->MODER;
    m &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    m |= (1u << GPIO_MODER_MODE6_Pos) | (1u << GPIO_MODER_MODE7_Pos);
    GPIOB->MODER = m;
    GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7)) |
                   GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
    GPIOB->BSRR = GPIO_BSRR_BS6 | GPIO_BSRR_BS7;      /* release both */
    sw_delay();
}

/* swap = 0: SCL=PB6 SDA=PB7 ; swap = 1: SCL=PB7 SDA=PB6 */
static void sw_write(int swap, int scl_high, int sda_high)
{
    uint32_t set = 0u, clr = 0u;

    if (swap) {
        if (scl_high) set |= GPIO_BSRR_BS7; else clr |= GPIO_BSRR_BR7;
        if (sda_high) set |= GPIO_BSRR_BS6; else clr |= GPIO_BSRR_BR6;
    } else {
        if (scl_high) set |= GPIO_BSRR_BS6; else clr |= GPIO_BSRR_BR6;
        if (sda_high) set |= GPIO_BSRR_BS7; else clr |= GPIO_BSRR_BR7;
    }
    GPIOB->BSRR = set | clr;
}

static int sw_read_sda(int swap)
{
    return swap ? ((GPIOB->IDR & GPIO_IDR_ID6) != 0u)
                : ((GPIOB->IDR & GPIO_IDR_ID7) != 0u);
}

static void sw_start(int swap)
{
    sw_write(swap, 1, 1); sw_delay();
    sw_write(swap, 1, 0); sw_delay();
    sw_write(swap, 0, 0); sw_delay();
}

static void sw_stop(int swap)
{
    sw_write(swap, 0, 0); sw_delay();
    sw_write(swap, 1, 0); sw_delay();
    sw_write(swap, 1, 1); sw_delay();
}

/* returns 1 when the slave pulled SDA low on the ACK bit */
static int sw_write_byte(int swap, uint8_t b)
{
    int i, ack;

    for (i = 0; i < 8; ++i) {
        int bit = (b & 0x80u) ? 1 : 0;
        sw_write(swap, 0, bit); sw_delay();
        sw_write(swap, 1, bit); sw_delay();
        sw_write(swap, 0, bit); sw_delay();
        b = (uint8_t)(b << 1);
    }
    sw_write(swap, 0, 1); sw_delay();          /* release SDA */
    sw_write(swap, 1, 1); sw_delay();
    ack = sw_read_sda(swap) ? 0 : 1;
    sw_write(swap, 0, 1); sw_delay();
    return ack;
}

static int sw_probe(int swap, uint8_t addr7)
{
    int ack;

    sw_start(swap);
    ack = sw_write_byte(swap, (uint8_t)(addr7 << 1));
    sw_stop(swap);
    return ack;
}

static void sw_scan_bus(int swap, volatile uint32_t *out_count, volatile uint32_t *out_first)
{
    uint16_t a;
    uint32_t n = 0u, first = 0u;

    for (a = HOST_IMU_ADDR_SCAN_FIRST; a <= HOST_IMU_ADDR_SCAN_LAST; ++a) {
        if (sw_probe(swap, (uint8_t)a)) {
            if (n == 0u) first = (uint32_t)a;
            ++n;
        }
    }
    *out_count = n;
    *out_first = first;
}

void imu_bus_diag(void)
{
    int i;

    bus_diag[0] = 0xFFFFFFFFu;
    bus_diag[1] = 0xFFFFFFFFu;
    bus_diag[2] = 0u;
    bus_diag[3] = 0xFFFFFFFFu;

    /* 1) internal pull-up: a healthy idle I2C bus reads high on both lines */
    bus_pins_mode(0u, GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);
    bus_diag[0] = GPIOB->IDR & (BUS_SCL | BUS_SDA);

    /* 2) internal pull-down: an external pull-up (2k2..10k) keeps the line high,
     *    so 1 here proves the on-board pull-up exists; 0 means it is absent */
    bus_pins_mode(0u, GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1);
    bus_diag[1] = GPIOB->IDR & (BUS_SCL | BUS_SDA);

    /* 3) bus-clear: 9 clocks with SDA released, then a STOP condition */
    bus_pins_mode(1u, 0u);
    bus_sda_write(1);
    bus_delay();
    for (i = 0; i < 9; ++i) {
        bus_scl_write(0);
        bus_delay();
        bus_scl_write(1);
        bus_delay();
    }
    bus_scl_write(0);          /* STOP: SDA low while SCL low, then SCL high */
    bus_delay();
    bus_sda_write(0);
    bus_delay();
    bus_scl_write(1);
    bus_delay();
    bus_sda_write(1);
    bus_delay();
    bus_diag[2] = 9u;

    /* 4) software-I2C probe, both pin orders: this tests wiring/address/power
     *    without depending on the I2C peripheral configuration at all */
    sw_pins();
    sw_scan_bus(0, &sw_scan[0], &sw_scan[1]);
    sw_scan_bus(1, &sw_scan[2], &sw_scan[3]);

    /* 5) levels again, then start the real peripheral */
    bus_pins_mode(0u, GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);
    bus_diag[3] = GPIOB->IDR & (BUS_SCL | BUS_SDA);

    (void)imu_i2c_init();
    bus_diag[6] = GPIOB->MODER;
    bus_diag[7] = PWR->CR3;
}

void imu_bus_diag_scan_result(int found_count, const uint8_t *found){
    bus_diag[4] = (found_count > 0) ? (uint32_t)found_count : 0u;
    if (found_count > 0) {
        uint32_t v = (uint32_t)found[0];
        if (found_count > 1) v |= ((uint32_t)found[1] << 8);
        bus_diag[5] = v;
    } else {
        bus_diag[5] = 0u;
    }
    bus_diag[6] = GPIOB->MODER;
    bus_diag[7] = PWR->CR3;
}

#if HOST_I2C_PIN_DRIVE_TEST
/* Hardware check aid: drive PB6/PB7 as plain push-pull outputs, 5 s low then
 * 5 s high, forever. Measure the IMU SDA/SCL pins with a multimeter: if the
 * voltage follows, the connection MCU -> IMU is good; if it stays fixed, the
 * net / solder joint is broken. */
void imu_i2c_pin_drive_test(void)
{
    uint32_t m;

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    m = GPIOB->MODER;
    m &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    m |= (1u << GPIO_MODER_MODE6_Pos) | (1u << GPIO_MODER_MODE7_Pos);
    GPIOB->MODER = m;
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);        /* push-pull */
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);

    for (;;) {
        GPIOB->BSRR = GPIO_BSRR_BR6 | GPIO_BSRR_BR7;              /* both LOW  */
        HAL_Delay(5000u);
        GPIOB->BSRR = GPIO_BSRR_BS6 | GPIO_BSRR_BS7;              /* both HIGH */
        HAL_Delay(5000u);
    }
}
#endif
