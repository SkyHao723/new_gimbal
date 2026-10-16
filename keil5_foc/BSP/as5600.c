/* STM32F1 two-byte I2C receive sequence, RM0008: POS=1 before ADDR,
 * clear ACK while clearing ADDR, wait BTF, STOP then read both bytes. */
#include "stm32f10x.h"
#include "as5600.h"
#include "board.h"
#include "motor_config.h"
#include "foc_math.h"
#include "encoder_tracking.h"

static void pause_us(uint32_t delay)
{
    uint32_t start = board_micros();
    while ((uint32_t)(board_micros()-start) < delay) {}
}

void as5600_init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &gpio);
    I2C_DeInit(I2C1);
    I2C_StructInit(&i2c);
    i2c.I2C_ClockSpeed = ENCODER_I2C_HZ;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

static void recover_bus(void)
{
    GPIO_InitTypeDef gpio;
    unsigned int i;
    I2C1->CR1 &= ~I2C_CR1_PE;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &gpio);
    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
    for (i=0; i<9; ++i) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_6); pause_us(5);
        GPIO_SetBits(GPIOB, GPIO_Pin_6); pause_us(5);
    }
    GPIO_ResetBits(GPIOB, GPIO_Pin_7); pause_us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_6); pause_us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_7); pause_us(5);
    as5600_init();
}

static int wait_sr1(uint16_t mask)
{
    uint32_t start = board_micros();
    while (!(I2C1->SR1 & mask)) {
        if (I2C1->SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO)) return 0;
        if ((uint32_t)(board_micros()-start) > 2000u) return 0;
    }
    return 1;
}

static int read_raw(uint16_t *raw)
{
    uint32_t start, irq;
    uint8_t high, low;
    start = board_micros();
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((uint32_t)(board_micros()-start) > 2000u) goto fail;
    }
    I2C1->CR1 = (I2C1->CR1 & ~I2C_CR1_POS) | I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_sr1(I2C_SR1_SB)) goto fail;
    I2C1->DR = 0x6cu;
    if (!wait_sr1(I2C_SR1_ADDR)) goto fail;
    (void)I2C1->SR2;
    I2C1->DR = 0x0cu;
    if (!wait_sr1(I2C_SR1_BTF)) goto fail;
    I2C1->CR1 |= I2C_CR1_POS | I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_sr1(I2C_SR1_SB)) goto fail;
    I2C1->DR = 0x6du;
    if (!wait_sr1(I2C_SR1_ADDR)) goto fail;
    irq = __get_PRIMASK();
    __disable_irq();
    (void)I2C1->SR2;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    __set_PRIMASK(irq);
    if (!wait_sr1(I2C_SR1_BTF)) goto fail;
    irq = __get_PRIMASK();
    __disable_irq();
    I2C1->CR1 |= I2C_CR1_STOP;
    high = (uint8_t)I2C1->DR;
    low = (uint8_t)I2C1->DR;
    __set_PRIMASK(irq);
    start = board_micros();
    while (I2C1->CR1 & I2C_CR1_STOP) {
        if ((uint32_t)(board_micros()-start) > 2000u) goto fail;
    }
    I2C1->CR1 = (I2C1->CR1 & ~I2C_CR1_POS) | I2C_CR1_ACK;
    *raw = (uint16_t)(((high & 15u) << 8) | low);
    return 1;
fail:
    I2C1->CR1 |= I2C_CR1_STOP;
    recover_bus();
    return 0;
}

int as5600_update(Encoder *sensor)
{
    uint16_t raw;
    if (!read_raw(&raw)) { sensor->errors++; return 0; }
    encoder_accept_raw(sensor, raw, board_micros());
    return 1;
}
