/* ============================================================================
 * encoder_fast.c —— AS5600 加速版驱动（阶段 2 性能改进，待独立验证）
 *
 * 为什么单独一个文件：
 *   encoder.c 已通过阶段 1 压测（40000 次读零失败），是【可信基线】。
 *   本文件是它的性能改进版，必须独立重跑压测后才能替代。
 *   出问题立刻切回 encoder.c。
 *
 * 与 encoder.c 的唯一区别：
 *   encoder.c 读角度分两次独立传输（各含 地址/寄存器号/重复起始/停止）
 *   → 每次约 0.5ms，主循环被压到 1kHz
 *
 *   本文件用【一次 2 字节读】：控制字节地址自增（0x0C -> 0x0C,0x0D）
 *   → 传输次数减半，预计主循环可到 ~2kHz
 *
 * ⚠️ 两字节读是 I2C 最容易写错的时序，重点在：
 *   1. 清 ADDR 之前必须先置 ACK（否则会立刻 NACK 第一个字节）
 *   2. 读第 1 字节后清 ACK、置 STOP，再读第 2 字节
 *   3. 等 BTF（而不是 RXNE）才表明两个字节都收完
 * ==========================================================================*/
#include "encoder.h"

/* 统计（与 encoder.c 分离，便于独立对比） */
EncoderStats_t encf_stat;

#define CR1_PE     (1u<<0)
#define CR1_START  (1u<<8)
#define CR1_STOP   (1u<<9)
#define CR1_ACK    (1u<<10)
#define CR1_POS    (1u<<11)
#define SR1_SB     (1u<<0)
#define SR1_ADDR   (1u<<1)
#define SR1_BTF    (1u<<2)
#define SR1_RXNE   (1u<<6)
#define SR1_TXE    (1u<<7)
#define SR1_AF     (1u<<10)

#define FTMO  20000

static void f_clear_err(void)
{
	I2C1->SR1 &= ~(SR1_AF|(1u<<8)|(1u<<9));
}

static void f_abort(void)
{
	I2C1->CR1 |= CR1_STOP;
	{
		volatile uint32_t t = FTMO;
		while((I2C1->CR1 & CR1_STOP) && --t);
	}
	I2C1->CR1 &= ~CR1_POS;
	I2C1->CR1 |=  CR1_ACK;
	f_clear_err();
	encf_stat.abort++;
}

void encoder_fast_init(uint32_t speed_hz)
{
	GPIO_InitTypeDef gi;
	I2C_InitTypeDef ii;

	encf_stat.speed_hz = speed_hz;
	if(encf_stat.speed_hz < 10000)  encf_stat.speed_hz = 10000;
	if(encf_stat.speed_hz > 400000) encf_stat.speed_hz = 400000;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	gi.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	gi.GPIO_Mode = GPIO_Mode_AF_OD;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gi);

	I2C_DeInit(I2C1);
	ii.I2C_Mode = I2C_Mode_I2C;
	ii.I2C_DutyCycle = I2C_DutyCycle_2;
	ii.I2C_OwnAddress1 = 0;
	ii.I2C_Ack = I2C_Ack_Enable;
	ii.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	ii.I2C_ClockSpeed = encf_stat.speed_hz;
	I2C_Init(I2C1, &ii);
	I2C_Cmd(I2C1, ENABLE);
}

/* 一次传输读 2 字节（寄存器地址自增） */
static int f_read2(uint8_t reg, uint8_t *hi, uint8_t *lo)
{
	uint32_t t;
	uint8_t v1, v2;

	/* 等总线空闲 */
	t = FTMO;
	while(I2C1->SR2 & (1u<<1)) if(--t == 0){ f_abort(); return 1; }

	I2C1->CR1 |= CR1_ACK;
	I2C1->CR1 |= CR1_START;
	t = FTMO;
	while(!(I2C1->SR1 & SR1_SB)) if(--t == 0){ f_abort(); return 1; }

	I2C1->DR = (AS5600_ADDR << 1);              /* 写 */
	t = FTMO;
	while(!(I2C1->SR1 & SR1_ADDR)){
		if(I2C1->SR1 & SR1_AF){ f_abort(); return 2; }
		if(--t == 0){ f_abort(); return 1; }
	}
	(void)I2C1->SR2;

	/* 寄存器号（AS5600 读角度时地址自增） */
	I2C1->DR = reg;
	t = FTMO;
	while(!(I2C1->SR1 & SR1_BTF)){
		if(I2C1->SR1 & SR1_AF){ f_abort(); return 2; }
		if(--t == 0){ f_abort(); return 1; }
	}

	/* 重复起始 + 读地址 */
	I2C1->CR1 |= CR1_START;
	t = FTMO;
	while(!(I2C1->SR1 & SR1_SB)) if(--t == 0){ f_abort(); return 1; }

	I2C1->DR = (AS5600_ADDR << 1) | 1;          /* 读 */
	t = FTMO;
	while(!(I2C1->SR1 & SR1_ADDR)){
		if(I2C1->SR1 & SR1_AF){ f_abort(); return 2; }
		if(--t == 0){ f_abort(); return 1; }
	}

	/* ★ 关键顺序：清 ADDR 前先置 ACK，让第 1 字节被应答 */
	I2C1->CR1 |=  CR1_ACK;
	I2C1->CR1 &= ~CR1_POS;                       /* 不用 POS 位，逻辑更直观 */
	(void)I2C1->SR2;

	/* 等第 1 字节 */
	t = FTMO;
	while(!(I2C1->SR1 & SR1_RXNE)) if(--t == 0){ f_abort(); return 1; }
	v1 = (uint8_t)I2C1->DR;

	/* 最后一个字节：清 ACK（NACK）+ 置 STOP */
	I2C1->CR1 &= ~CR1_ACK;
	I2C1->CR1 |=  CR1_STOP;

	/* 等第 2 字节（用 BTF 更稳） */
	t = FTMO;
	while(!(I2C1->SR1 & SR1_BTF)) if(--t == 0){ f_abort(); return 1; }
	v2 = (uint8_t)I2C1->DR;

	t = FTMO;
	while((I2C1->CR1 & CR1_STOP) && --t);

	I2C1->CR1 |= CR1_ACK;
	*hi = v1; *lo = v2;
	return 0;
}

/* 读 12 位原始角度 */
int encoder_fast_read_raw(uint16_t *out)
{
	uint8_t hi, lo;
	int r = f_read2(AS5600_REG_RAW_ANGLE_H, &hi, &lo);
	if(r){ encf_stat.fail++; return r; }
	*out = (uint16_t)(((uint16_t)(hi & 0x0F) << 8) | lo);
	encf_stat.ok++;
	return 0;
}

/* 带双读校验 */
int encoder_fast_read_checked(uint16_t *raw, uint16_t max_delta, uint8_t retry)
{
	uint16_t a, b;
	int16_t d;
	uint8_t i;

	for(i = 0; i < retry; i++)
	{
		if(encoder_fast_read_raw(&a)) continue;
		if(encoder_fast_read_raw(&b)) continue;
		d = (int16_t)a - (int16_t)b;
		if(d < 0) d = -d;
		if(d <= (int16_t)max_delta || d >= (int16_t)(4096 - max_delta))
		{
			*raw = b;
			encf_stat.checked_ok++;
			return 0;
		}
		encf_stat.outlier++;
	}
	return 1;
}

void encoder_fast_reset_stats(void)
{
	uint32_t sp = encf_stat.speed_hz;
	encf_stat.ok = 0; encf_stat.fail = 0; encf_stat.checked_ok = 0;
	encf_stat.outlier = 0; encf_stat.abort = 0; encf_stat.recover = 0;
	encf_stat.speed_hz = sp;
}
