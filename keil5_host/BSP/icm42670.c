#include "stm32g4xx_hal.h"
#include "icm42670.h"
#include "host_config.h"
#include <string.h>

static I2C_HandleTypeDef i2c;
static const char *last_error = "";

/* ICM-42670-P bank0 寄存器 */
#define REG_TEMP_DATA1   0x1Du
#define REG_PWR_MGMT0    0x1Fu
#define REG_GYRO_CONFIG0 0x20u
#define REG_ACCEL_CONFIG0 0x21u
#define REG_GYRO_DATA_X1 0x25u
#define REG_ACCEL_DATA_X1 0x2Fu

const char *imu_last_error(void) { return last_error; }

static void i2c_gpio_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    /* PB6 = SCL, PB7 = SDA, AF4 (I2C1), 开漏 + 上拉 */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7)) |
                   (2u << GPIO_MODER_MODE6_Pos) | (2u << GPIO_MODER_MODE7_Pos);
    GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7;
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7)) |
                   GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~((0xFu << 24) | (0xFu << 28))) |
                    (4u << 24) | (4u << 28);               /* AF4 */
}

static void i2c_apply_timing(uint32_t timing)
{
    (void)HAL_I2C_DeInit(&i2c);
    memset(&i2c, 0, sizeof(i2c));
    i2c.Instance = HOST_I2C_INSTANCE;
    i2c.Init.Timing = timing;
    i2c.Init.OwnAddress1 = 0;
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c.Init.OwnAddress2 = 0;
    i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    (void)HAL_I2C_Init(&i2c);
}

int imu_i2c_init(void)
{
    i2c_gpio_init();
    __HAL_RCC_I2C1_CLK_ENABLE();
    i2c_apply_timing(HOST_I2C_TIMING_100K);
    /* GPIO 之外再做一次显式引脚配置，确保 HAL 不会重置成默认模式 */
    i2c_gpio_init();
    return 1;
}

static int mem_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&i2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                         buf, len, 20u) == HAL_OK) return 1;
    /* 总线卡住（SDA 被从机拉住）时释放一次：模拟 9 个时钟 + STOP */
    (void)HAL_I2C_DeInit(&i2c);
    i2c_gpio_init();
    i2c_apply_timing(i2c.Init.Timing);
    return 0;
}

static int mem_write(uint8_t addr, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Write(&i2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)buf, len, 20u) == HAL_OK;
}

int imu_scan(uint8_t *found, int max)
{
    int count = 0;
    static const uint32_t speeds[3] = { HOST_I2C_TIMING_100K, HOST_I2C_TIMING_50K,
                                        HOST_I2C_TIMING_10K };
    for (int s = 0; s < 3; ++s) {
        i2c_apply_timing(speeds[s]);
        count = 0;
        for (uint16_t addr = HOST_IMU_ADDR_SCAN_FIRST; addr <= HOST_IMU_ADDR_SCAN_LAST; ++addr) {
            /* 单次探测 3 ms：总线上没器件时整轮扫描要在 1 秒内结束，不然开机看着像死机 */
            if (HAL_I2C_IsDeviceReady(&i2c, (uint16_t)(addr << 1), 1u, 3u) == HAL_OK) {
                if (count < max) found[count] = (uint8_t)addr;
                ++count;
            }
        }
        if (count > 0) return count;
        last_error = "no I2C device at 100k/50k/10k";
    }
    return count;
}

int imu_init(ImuInfo *imu)
{
    uint8_t who = 0;
    uint8_t addr = HOST_IMU_ADDR_PRIMARY;

    memset(imu, 0, sizeof(*imu));
    /* 先按 0x68 找，不行再试 0x69；两档 I2C 速度都试，便于判断是速度问题还是接线问题 */
    static const uint32_t speeds[3] = { HOST_I2C_TIMING_100K, HOST_I2C_TIMING_50K,
                                        HOST_I2C_TIMING_10K };
    for (int s = 0; s < 3; ++s) {
        i2c_apply_timing(speeds[s]);
        for (int a = 0; a < 2; ++a) {
            addr = (a == 0) ? HOST_IMU_ADDR_PRIMARY : HOST_IMU_ADDR_SECONDARY;
            if (mem_read(addr, HOST_IMU_WHO_AM_I_REG, &who, 1) && who == HOST_IMU_WHO_AM_I_VALUE) {
                imu->address = addr;
                imu->who_am_i = who;
                imu->speed_code = (uint8_t)s;
                /* bank0 + 加速度/角速度都进低噪声模式 */
                uint8_t pwr = 0x0Fu;
                if (!mem_write(addr, REG_PWR_MGMT0, &pwr, 1)) {
                    last_error = "PWR_MGMT0 write failed";
                    return 0;
                }
                HAL_Delay(40u);                            /* 陀螺需要 ~35 ms 起振 */
                uint8_t cfg = 0x06u;                       /* gyro: ±2000 dps, 1 kHz ODR */
                (void)mem_write(addr, REG_GYRO_CONFIG0, &cfg, 1);
                cfg = 0x06u;                                /* accel: ±16 g, 1 kHz ODR */
                (void)mem_write(addr, REG_ACCEL_CONFIG0, &cfg, 1);
                last_error = "";
                return 1;
            }
            if (who != 0 && who != HOST_IMU_WHO_AM_I_VALUE) {
                last_error = "WHO_AM_I mismatch (expect 0x67)";
            }
        }
    }
    if (!last_error[0]) last_error = "no answer at 0x68/0x69";
    return 0;
}

int imu_read_raw(const ImuInfo *imu, int16_t accel[3], int16_t gyro[3], int16_t *temp_raw)
{
    uint8_t buf[6];
    if (!mem_read(imu->address, REG_ACCEL_DATA_X1, buf, 6)) return 0;
    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
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
