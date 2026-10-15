/* ============================================================================
 * encoder_test.c —— 阶段 1：AS5600 编码器可靠性测试固件
 *
 * 目的：把"读角度"这一件事做到可信，**不接电机、不产生任何 PWM 输出**。
 *
 * 硬件：SCL=PB6  SDA=PB7（需 2.2~4.7k 上拉到 3.3V）
 *       串口 PA2/PA3，115200
 *       电机驱动 EN(PB0) 始终保持失能，绝不输出
 *
 * 命令：
 *   ?        状态（统计 + 当前读数）
 *   A        读一次角度（显示 raw / 角度 / 状态）
 *   L        检查 SCL/SDA 空闲电平（诊断上拉和接线）
 *   R        总线恢复（卡死时用）
 *   S        统计信息
 *   Z        清零统计
 *   J<n>     抖动测试：连读 n 次，报告 min/max/失败数（默认 1000）
 *   K<x>     设置 I2C 速率，如 K100000 / K400000
 *   D<addr>  读 AS5600 指定寄存器（十六进制），如 D0B 读状态寄存器
 *   M        磁铁状态（读 STATUS + AGC + MAGNITUDE）
 *   C        连续读取模式：每秒报告一次成功率（再按 C 停止）
 * ==========================================================================*/
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include "encoder.h"

extern void RCC_Configuration(void);

/* ---------------- 自包含串口 USART2 (PA2/PA3) ---------------- */
#define RX_BUF_LEN 128
static volatile unsigned char rx_buf[RX_BUF_LEN];
static volatile unsigned short rx_sta = 0;      /* bit15=完成 bit14=收到CR */

void _write_impl(int fd, const char *buf, int len)
{
	int i; (void)fd;
	for(i=0;i<len;i++){
		while((USART2->SR & 0x40) == 0);
		USART2->DR = (unsigned char)buf[i];
	}
	while((USART2->SR & 0x40) == 0);
}

static void uart_init(unsigned long baud)
{
	GPIO_InitTypeDef gi;
	NVIC_InitTypeDef ni;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_2; gi.GPIO_Mode = GPIO_Mode_AF_PP;
	gi.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOA, &gi);
	gi.GPIO_Pin = GPIO_Pin_3; gi.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gi);
	USART2->BRR = (SystemCoreClock/2 + baud/2) / baud;   /* PCLK1 = SysClk/2 */
	USART2->CR1 = 0x200C;                                 /* UE|TE|RE */
	ni.NVIC_IRQChannel = USART2_IRQn;
	ni.NVIC_IRQChannelPreemptionPriority = 3;
	ni.NVIC_IRQChannelSubPriority = 3;
	ni.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&ni);
	USART2->CR1 |= 0x0020;                                /* RXNEIE */
}

void USART2_IRQHandler(void)
{
	if(USART2->SR & 0x20)
	{
		unsigned char r = (unsigned char)(USART2->DR & 0xFF);
		if((rx_sta & 0x8000) == 0)
		{
			if(rx_sta & 0x4000){
				if(r == 0x0A){ rx_sta |= 0x8000; rx_buf[rx_sta & 0x3FFF] = 0; }
				else rx_sta = 0;
			} else {
				if(r == 0x0D) rx_sta |= 0x4000;
				else {
					rx_buf[rx_sta & 0x3FFF] = r;
					rx_sta++;
					if(rx_sta > RX_BUF_LEN-1) rx_sta = 0;
				}
			}
		}
	}
}

static uint32_t clk_hz = 72000000;
static void dly_ms(uint32_t ms)
{
	volatile uint32_t n = (clk_hz / 4000) * ms;
	while(n--) __asm volatile("nop");
}

static void gpio_init(void)
{
	GPIO_InitTypeDef gi;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|
	                       RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	/* PC13 LED */
	gi.GPIO_Pin = GPIO_Pin_13; gi.GPIO_Mode = GPIO_Mode_Out_PP;
	gi.GPIO_Speed = GPIO_Speed_10MHz; GPIO_Init(GPIOC, &gi);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

	/* ★ PB0 = 电机 EN，本固件始终保持失能，绝不动电机 */
	gi.GPIO_Pin = GPIO_Pin_0; gi.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &gi);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/* 用 encoder.c 里的寄存器读（这里只做一个简单的“读任意寄存器”命令用） */
static uint8_t dbg_reg;
static uint8_t dbg_val;
static uint8_t dbg_ok;

/* 需要一个能读任意寄存器的入口——直接在测试固件里用 I2C1 重复 encoder 的方式过于繁琐，
 * 这里改用 encoder.c 未导出的能力：临时把寄存器地址写到 AS5600_REG_RAW_ANGLE_H 无效，
 * 所以单独实现一个最简读（复用同样的时序） */
static int read_reg_generic(uint8_t reg, uint8_t *out)
{
	uint32_t t;
	/* 借用 encoder 的读法：调用两次 encoder_read 只能读角度。
	 * 这里用最小实现直接读任意寄存器。 */
	#define TMO 20000
	uint8_t v;

	t = TMO;
	while(I2C1->SR2 & (1u<<1)) if(--t == 0) goto err;

	I2C1->CR1 |= (1u<<10);            /* ACK */
	I2C1->CR1 |= (1u<<8);             /* START */
	t = TMO;
	while(!(I2C1->SR1 & (1u<<0))) if(--t == 0) goto err;

	I2C1->DR = (AS5600_ADDR << 1);
	t = TMO;
	while(!(I2C1->SR1 & (1u<<1))){
		if(I2C1->SR1 & (1u<<10)) goto err2;
		if(--t == 0) goto err;
	}
	(void)I2C1->SR2;
	I2C1->DR = reg;
	t = TMO;
	while(!(I2C1->SR1 & (1u<<2))){
		if(I2C1->SR1 & (1u<<10)) goto err2;
		if(--t == 0) goto err;
	}

	I2C1->CR1 |= (1u<<8);             /* 重复 START */
	t = TMO;
	while(!(I2C1->SR1 & (1u<<0))) if(--t == 0) goto err;

	I2C1->DR = (AS5600_ADDR << 1) | 1;
	t = TMO;
	while(!(I2C1->SR1 & (1u<<1))){
		if(I2C1->SR1 & (1u<<10)) goto err2;
		if(--t == 0) goto err;
	}
	I2C1->CR1 &= ~(1u<<10);           /* NACK */
	(void)I2C1->SR2;
	I2C1->CR1 |= (1u<<9);             /* STOP */

	t = TMO;
	while(!(I2C1->SR1 & (1u<<6))) if(--t == 0) goto err;
	v = (uint8_t)I2C1->DR;
	t = TMO;
	while((I2C1->CR1 & (1u<<9)) && --t);
	I2C1->CR1 |= (1u<<10);
	*out = v;
	return 0;
err2:
	I2C1->SR1 &= ~(1u<<10);
err:
	I2C1->CR1 |= (1u<<9);
	{
		volatile uint32_t x = TMO;
		while((I2C1->CR1 & (1u<<9)) && --x);
	}
	I2C1->CR1 &= ~(1u<<11);
	I2C1->CR1 |=  (1u<<10);
	return 1;
	#undef TMO
}

static void cmd_status(void)
{
	uint16_t raw = 0;
	int r = encoder_read_checked(&raw, 20, 3);
	printf("read=%s raw=%u angle=%.4f rad\r\n",
		r ? "FAIL" : "OK", (unsigned)raw, encoder_raw_to_rad(raw));
	printf("stat: ok=%lu fail=%lu checked=%lu outlier=%lu abort=%lu recover=%lu  speed=%luHz\r\n",
		(unsigned long)enc_stat.ok, (unsigned long)enc_stat.fail,
		(unsigned long)enc_stat.checked_ok, (unsigned long)enc_stat.outlier,
		(unsigned long)enc_stat.abort, (unsigned long)enc_stat.recover,
		(unsigned long)enc_stat.speed_hz);
	{
		uint32_t tot = enc_stat.ok + enc_stat.fail;
		if(tot)
			printf("error rate = %.3f%%\r\n",
				(double)enc_stat.fail * 100.0 / (double)tot);
	}
}

static void cmd_lines(void)
{
	uint8_t b = encoder_check_lines();
	printf("SCL(PB6)=%s  SDA(PB7)=%s\r\n",
		(b & 0x01) ? "HIGH" : "LOW ", (b & 0x02) ? "HIGH" : "LOW ");
	if((b & 0x03) != 0x03)
		printf("!! 两根都应为 HIGH。有 LOW 说明上拉缺失/接线错/被拉死\r\n");
	else
		printf("   OK: 空闲电平正常（上拉有效）\r\n");
}

static void cmd_magnet(void)
{
	uint8_t st=0, agc=0, mh=0, ml=0;
	int e = 0;
	e += read_reg_generic(AS5600_REG_STATUS, &st);
	e += read_reg_generic(AS5600_REG_AGC, &agc);
	e += read_reg_generic(AS5600_REG_MAGNITUDE_H, &mh);
	e += read_reg_generic(AS5600_REG_MAGNITUDE_L, &ml);
	if(e){ printf("读寄存器失败\r\n"); return; }
	{
		uint16_t mag = (uint16_t)(((uint16_t)(mh & 0x0F) << 8) | ml);
		printf("STATUS=0x%02X  AGC=%u  MAGNITUDE=%u\r\n", st, agc, mag);
		printf("  MD(检测到磁铁)=%d  ML(太弱)=%d  MH(太强)=%d\r\n",
			(st>>5)&1, (st>>4)&1, (st>>3)&1);
		if(!((st>>5)&1)) printf("  !! 没检测到磁铁（MD=0）\r\n");
		else if((st>>4)&1) printf("  !! 磁场太弱（ML=1），磁铁太远\r\n");
		else if((st>>3)&1) printf("  !! 磁场太强（MH=1），磁铁太近\r\n");
		else printf("  磁铁状态正常\r\n");
		printf("  AGC 参考: 正常约 0~100, 接近 255 表示信号弱\r\n");
	}
}

static void cmd_jitter(uint16_t n)
{
	uint16_t mn, mx, fl;
	printf("jitter test: %u 次连续读...\r\n", (unsigned)n);
	encoder_reset_stats();
	encoder_jitter_test(n, &mn, &mx, &fl);
	{
		uint32_t tot = enc_stat.ok + enc_stat.fail;
		printf("  结果: min=%u max=%u 跨度=%u LSB  失败=%u/%lu\r\n",
			(unsigned)mn, (unsigned)mx, (unsigned)(mx-mn),
			(unsigned)fl, (unsigned long)tot);
		if(tot) printf("  错误率=%.3f%%  outlier=%lu abort=%lu recover=%lu\r\n",
			(double)enc_stat.fail*100.0/(double)tot,
			(unsigned long)enc_stat.outlier,(unsigned long)enc_stat.abort,
			(unsigned long)enc_stat.recover);
		printf("  判据: 静止时跨度应 < 5 LSB, 错误率 < 0.1%%\r\n");
	}
}

/* 定长压力测试：读 n 次，期间持续响应中断命令 */
static void cmd_stress(uint32_t n)
{
	uint32_t i;
	uint32_t report = n / 10;
	uint16_t raw;

	if(report == 0) report = 1;
	rx_sta = 0;                 /* 清掉命令回显残留 */
	encoder_reset_stats();
	printf("压力测试: %lu 次读取...\r\n", (unsigned long)n);

	for(i = 0; i < n; i++)
	{
		encoder_read_checked(&raw, 20, 2);
		if((i % report) == 0 && i)
		{
			uint32_t tot = enc_stat.ok + enc_stat.fail;
			printf("  %3lu%%  ok=%lu fail=%lu rate=%.3f%% abort=%lu recover=%lu\r\n",
				(unsigned long)(i*100/n), (unsigned long)enc_stat.ok,
				(unsigned long)enc_stat.fail,
				tot?(double)enc_stat.fail*100.0/(double)tot:0.0,
				(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover);
		}
		if(rx_sta & 0x8000){ rx_sta = 0; printf("  (被用户中断)\r\n"); break; }
	}
	{
		uint32_t tot = enc_stat.ok + enc_stat.fail;
		printf("完成: 共 %lu 次, ok=%lu fail=%lu\r\n",
			(unsigned long)tot,(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail);
		printf("  错误率 = %.4f%%   abort=%lu  recover=%lu  outlier=%lu\r\n",
			tot?(double)enc_stat.fail*100.0/(double)tot:0.0,
			(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover,
			(unsigned long)enc_stat.outlier);
		if(tot && (double)enc_stat.fail*100.0/(double)tot < 0.1)
			printf("  >>> PASS: 错误率 < 0.1%%\r\n");
		else
			printf("  >>> FAIL: 错误率过高\r\n");
	}
}

static void cmd_continuous(void)
{
	uint32_t sec = 0;
	uint32_t last_ok, last_fail;
	encoder_reset_stats();
	printf("连续模式已启动，再发 C 停止\r\n");
	last_ok = last_fail = 0;
	for(;;)
	{
		uint16_t raw;
		uint32_t i;
		for(i = 0; i < 200; i++) encoder_read_checked(&raw, 20, 3);
		dly_ms(200);
		sec++;
		{
			uint32_t d_ok = enc_stat.ok - last_ok;
			uint32_t d_fail = enc_stat.fail - last_fail;
			uint32_t tot = d_ok + d_fail;
			last_ok = enc_stat.ok; last_fail = enc_stat.fail;
			printf("[%4lu s] ok=%lu fail=%lu rate=%.3f%% abort=%lu recover=%lu\r\n",
				(unsigned long)sec,(unsigned long)d_ok,(unsigned long)d_fail,
				tot? (double)d_fail*100.0/(double)tot : 0.0,
				(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover);
		}
		{
			uint32_t w;
			for(w = 0; w < 20; w++)
			{
				dly_ms(50);
				if(rx_sta & 0x8000){
					if(rx_buf[0]=='C' || rx_buf[0]=='c'){
						rx_sta = 0; printf("已停止\r\n"); return;
					}
					rx_sta = 0;
				}
			}
		}
	}
}

int main(void)
{
	RCC_Configuration();
	clk_hz = SystemCoreClock;

	gpio_init();
	uart_init(115200);
	dly_ms(200);

	printf("\r\n=== ENCODER TEST (阶段1: 编码器可靠性) ===\r\n");
	printf("SysClk=%lu Hz\r\n", (unsigned long)clk_hz);
	printf("电机驱动 EN 保持失能，本固件不输出任何 PWM\r\n\r\n");

	encoder_reset_stats();
	encoder_i2c_init(100000);          /* ★ 默认 100kHz */
	dly_ms(10);

	printf("I2C1 已初始化 @ %lu Hz\r\n", (unsigned long)enc_stat.speed_hz);
	cmd_lines();
	printf("\r\n");
	cmd_status();
	printf("\r\n命令: ? A L R S Z J<n> K<Hz> D<hex> M C\r\n");
	printf("建议顺序: M(磁铁) -> J1000(抖动) -> T5000(压力) -> C(长跑)\r\n\r\n");

	for(;;)
	{
		if((rx_sta & 0x8000) != 0)
		{
			switch(rx_buf[0])
			{
				case '?': cmd_status(); break;
				case 'A': { uint16_t r=0; int e=encoder_read_checked(&r,20,3);
					printf("raw=%u angle=%.4f %s\r\n",(unsigned)r,
						encoder_raw_to_rad(r), e?"FAIL":"OK"); } break;
				case 'L': cmd_lines(); break;
				case 'R': encoder_bus_recover(); printf("已执行总线恢复\r\n"); break;
				case 'S':
					printf("ok=%lu fail=%lu checked=%lu outlier=%lu abort=%lu recover=%lu\r\n",
						(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail,
						(unsigned long)enc_stat.checked_ok,(unsigned long)enc_stat.outlier,
						(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover);
					break;
				case 'Z': encoder_reset_stats(); printf("统计已清零\r\n"); break;
				case 'J': {
					uint16_t n = (uint16_t)atoi((const char *)(rx_buf+1));
					if(n == 0) n = 1000;
					cmd_jitter(n); } break;
				case 'K': {
					uint32_t s = (uint32_t)atoi((const char *)(rx_buf+1));
					if(s == 0) s = 100000;
					encoder_i2c_init(s);
					printf("I2C speed = %lu Hz\r\n",(unsigned long)enc_stat.speed_hz); } break;
				case 'D': {
					uint32_t a = (uint32_t)strtol((const char *)(rx_buf+1),0,16);
					dbg_ok = (uint8_t)(read_reg_generic((uint8_t)a,&dbg_val)==0);
					printf("REG[0x%02lX] = 0x%02X %s\r\n",(unsigned long)a,
						dbg_val, dbg_ok?"":"(读失败)");
					(void)dbg_reg; } break;
				case 'M': cmd_magnet(); break;
				case 'T': {                  /* T<n> 压力测试，默认 5000 次 */
					uint32_t n = (uint32_t)atoi((const char *)(rx_buf+1));
					if(n == 0) n = 5000;
					cmd_stress(n); } break;
				case 'C': cmd_continuous(); break;
				default: break;
			}
			rx_sta = 0;
		}
		/* 心跳：LED 慢闪，证明主循环在跑 */
		{
			static uint32_t c = 0;
			if(++c > 2000000){ c = 0; GPIOC->ODR ^= (1<<13); }
		}
	}
}



