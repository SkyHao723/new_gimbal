/* ============================================================================
 * encoder.c —— AS5600 角度读取（健壮版）
 *
 * 与参考工程原版的区别（原版在控制循环里会读失败 → 返回 0 → NaN）：
 *   1. 速率可配，默认 100kHz（400kHz 在排线上信号完整性差）
 *   2. 每个等待点都有有限超时，超时后【发 STOP 释放总线】
 *   3. 失败时返回明确的错误码，绝不返回 0 冒充有效角度
 *   4. 提供总线恢复：卡死时用 SCL 脉冲把从机踢出来
 *   5. 提供数据有效性校验：双读一致 + 跳变丢弃
 *   6. 统计：成功/失败/重同步次数
 *
 * 硬件：SCL=PB6  SDA=PB7（开漏，需外部上拉 2.2~4.7k 到 3.3V）
 * ==========================================================================*/
#include "encoder.h"

/* ---------------- 统计 ---------------- */
EncoderStats_t enc_stat;

/* ---------------- 内部寄存器操作 ---------------- */
#define I2C_CR1_PE        (1u<<0)
#define I2C_CR1_START     (1u<<8)
#define I2C_CR1_STOP      (1u<<9)
#define I2C_CR1_ACK       (1u<<10)
#define I2C_CR1_POS       (1u<<11)
#define I2C_SR1_SB        (1u<<0)
#define I2C_SR1_ADDR      (1u<<1)
#define I2C_SR1_BTF       (1u<<2)
#define I2C_SR1_RXNE      (1u<<6)
#define I2C_SR1_TXE       (1u<<7)
#define I2C_SR1_AF        (1u<<10)
#define I2C_SR1_BERR      (1u<<8)
#define I2C_SR1_ARLO      (1u<<9)

/* 每个等待点的最大轮询次数；72MHz 下 20000 次约几毫秒，足够慢速 I2C 完成 */
#define I2C_TMO   20000

static void i2c_bus_recover(void);

/* 清所有错误标志 */
static void i2c_clear_err(void)
{
	I2C1->SR1 &= ~(I2C_SR1_AF|I2C_SR1_BERR|I2C_SR1_ARLO);
}

/* 释放总线：发 STOP + 清 POS + 恢复 ACK */
static void i2c_abort(void)
{
	I2C1->CR1 |= I2C_CR1_STOP;
	{
		volatile uint32_t t = I2C_TMO;
		while((I2C1->CR1 & I2C_CR1_STOP) && --t);
	}
	I2C1->CR1 &= ~I2C_CR1_POS;
	I2C1->CR1 |=  I2C_CR1_ACK;
	i2c_clear_err();
	enc_stat.abort++;
}

/* ---------------- 总线恢复：SCL 打脉冲 ---------------- */
/* 当从机把 SDA 拉死时，用 9 个 SCL 脉冲把它踢出来，再补一个 STOP */
static void i2c_bus_recover(void)
{
	GPIO_InitTypeDef gi;
	uint8_t i;
	volatile uint32_t d;

	enc_stat.recover++;

	/* 1) 关掉 I2C 外设，把引脚切成开漏 GPIO */
	I2C1->CR1 &= ~I2C_CR1_PE;
	gi.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	gi.GPIO_Mode = GPIO_Mode_Out_OD;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gi);

	/* 2) SDA 释放高；SCL 打 9 个脉冲 */
	GPIO_SetBits(GPIOB, GPIO_Pin_7 | GPIO_Pin_6);
	for(d=0; d<200; d++);
	for(i=0; i<9; i++)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
		for(d=0; d<300; d++);
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		for(d=0; d<300; d++);
	}

	/* 3) 产生 STOP：SCL 高时 SDA 由低变高 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	for(d=0; d<300; d++);
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	for(d=0; d<300; d++);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
	for(d=0; d<300; d++);

	/* 4) 恢复 I2C 外设 */
	encoder_i2c_init(enc_stat.speed_hz);
}

/* ---------------- 初始化 ---------------- */
void encoder_i2c_init(uint32_t speed_hz)
{
	GPIO_InitTypeDef gi;
	I2C_InitTypeDef ii;

	enc_stat.speed_hz = speed_hz;
	if(enc_stat.speed_hz < 10000)  enc_stat.speed_hz = 10000;
	if(enc_stat.speed_hz > 400000) enc_stat.speed_hz = 400000;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* 开漏复用 */
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
	ii.I2C_ClockSpeed = enc_stat.speed_hz;
	I2C_Init(I2C1, &ii);
	I2C_Cmd(I2C1, ENABLE);
}

void encoder_bus_recover(void) { i2c_bus_recover(); }

/* ---------------- 读一个寄存器（8 位） ---------------- */
/* 返回 0 成功，非 0 失败 */
static int as5600_read_reg8(uint8_t reg, uint8_t *out)
{
	uint32_t t;
	uint8_t v;

	/* --- 阶段 1: START + 写地址(W) + 写寄存器号 --- */
	t = I2C_TMO;
	while(I2C1->SR2 & (1u<<1)) if(--t == 0){ i2c_abort(); return 1; }   /* 等 BUSY 清 */

	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	t = I2C_TMO;
	while(!(I2C1->SR1 & I2C_SR1_SB)) if(--t == 0){ i2c_abort(); return 1; }

	I2C1->DR = (AS5600_ADDR << 1);                       /* 写 */
	t = I2C_TMO;
	while(!(I2C1->SR1 & I2C_SR1_ADDR))
	{
		if(I2C1->SR1 & I2C_SR1_AF){ i2c_abort(); return 2; }   /* 从机不应答 */
		if(--t == 0){ i2c_abort(); return 1; }
	}
	(void)I2C1->SR2;                                    /* 清 ADDR */

	I2C1->DR = reg;
	t = I2C_TMO;
	while(!(I2C1->SR1 & I2C_SR1_BTF))
	{
		if(I2C1->SR1 & I2C_SR1_AF){ i2c_abort(); return 2; }
		if(--t == 0){ i2c_abort(); return 1; }
	}

	/* --- 阶段 2: 重复 START + 写地址(R) + 读一字节 --- */
	I2C1->CR1 |= I2C_CR1_START;
	t = I2C_TMO;
	while(!(I2C1->SR1 & I2C_SR1_SB)) if(--t == 0){ i2c_abort(); return 1; }

	I2C1->DR = (AS5600_ADDR << 1) | 1;                  /* 读 */
	t = I2C_TMO;
	while(!(I2C1->SR1 & I2C_SR1_ADDR))
	{
		if(I2C1->SR1 & I2C_SR1_AF){ i2c_abort(); return 2; }
		if(--t == 0){ i2c_abort(); return 1; }
	}
	/* 只读 1 字节 -> 先关 ACK 再清 ADDR */
	I2C1->CR1 &= ~I2C_CR1_ACK;
	(void)I2C1->SR2;
	I2C1->CR1 |= I2C_CR1_STOP;

	t = I2C_TMO;
	while(!(I2C1->SR1 & I2C_SR1_RXNE)) if(--t == 0){ i2c_abort(); return 1; }
	v = (uint8_t)I2C1->DR;

	t = I2C_TMO;
	while((I2C1->CR1 & I2C_CR1_STOP) && --t);           /* 等 STOP 完成 */

	I2C1->CR1 |= I2C_CR1_ACK;
	*out = v;
	return 0;
}

/* ---------------- 读 12 位角度（AS5600 raw angle, 0x0C/0x0D） ---------------- */
/* 返回 0 成功，非 0 失败 */
static int as5600_read_raw(uint16_t *out)
{
	uint8_t dh, dl;
	int r;

	r = as5600_read_reg8(AS5600_REG_RAW_ANGLE_H, &dh);
	if(r) return r;
	r = as5600_read_reg8(AS5600_REG_RAW_ANGLE_L, &dl);
	if(r) return r;

	*out = (uint16_t)(((uint16_t)(dh & 0x0F) << 8) | dl);
	return 0;
}

/* ---------------- 高层接口：带校验的连续读 ---------------- */
/* 失败会重试，最多 retry 次；成功后做跳变校验 */
int encoder_read(uint16_t *raw, uint8_t retry)
{
	uint16_t v;
	int r;
	uint8_t i;

	for(i = 0; i < retry; i++)
	{
		r = as5600_read_raw(&v);
		if(r == 0)
		{
			enc_stat.ok++;
			*raw = v;
			return 0;
		}
		enc_stat.fail++;
		if(r == 2){
			/* 从机不应答：很可能是总线卡住，做一次恢复 */
			i2c_bus_recover();
		}
	}
	return 1;
}

/* 双读一致校验：连续两次读到接近的值才认为有效 */
int encoder_read_checked(uint16_t *raw, uint16_t max_delta, uint8_t retry)
{
	uint16_t a, b;
	int16_t d;
	uint8_t i;

	for(i = 0; i < retry; i++)
	{
		if(encoder_read(&a, 1)) continue;
		if(encoder_read(&b, 1)) continue;

		d = (int16_t)a - (int16_t)b;
		if(d < 0) d = -d;
		if(d <= (int16_t)max_delta || d >= (int16_t)(4096 - max_delta))
		{
			*raw = b;
			enc_stat.checked_ok++;
			return 0;
		}
		enc_stat.outlier++;          /* 两次差太多 -> 丢弃重读 */
	}
	return 1;
}

/* ---------------- 角度换算 ---------------- */
float encoder_raw_to_rad(uint16_t raw)
{
	return ((float)raw / 4096.0f) * 6.28318531f;
}

/* ---------------- 自检信息 ---------------- */
void encoder_reset_stats(void)
{
	uint32_t sp = enc_stat.speed_hz;
	enc_stat.ok = 0;
	enc_stat.fail = 0;
	enc_stat.checked_ok = 0;
	enc_stat.outlier = 0;
	enc_stat.abort = 0;
	enc_stat.recover = 0;
	enc_stat.speed_hz = sp;
}

/* 静态抖动测试：连续读 n 次，返回最大-最小（用环形统计避免存数组） */
int encoder_jitter_test(uint16_t n, uint16_t *min_p, uint16_t *max_p, uint16_t *fail_p)
{
	uint16_t v, mn = 4095, mx = 0, fails = 0;
	uint16_t i;

	for(i = 0; i < n; i++)
	{
		if(encoder_read_checked(&v, 20, 3)){ fails++; continue; }
		if(v < mn) mn = v;
		if(v > mx) mx = v;
	}
	*min_p = mn; *max_p = mx; *fail_p = fails;
	return 0;
}

/* 检查 SCL/SDA 空闲电平（用于诊断上拉/接线）
 * 返回：bit0=SCL 高, bit1=SDA 高 */
uint8_t encoder_check_lines(void)
{
	GPIO_InitTypeDef gi;
	uint8_t r = 0;

	/* 暂时切成上拉输入来读电平 */
	I2C1->CR1 &= ~I2C_CR1_PE;
	gi.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	gi.GPIO_Mode = GPIO_Mode_IPU;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gi);
	{
		volatile uint32_t d; for(d=0; d<2000; d++);
	}
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)) r |= 0x01;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)) r |= 0x02;

	/* 恢复 I2C */
	encoder_i2c_init(enc_stat.speed_hz);
	return r;
}
