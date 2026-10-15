/* ============================================================================
 * enc_probe.c —— 最小编码器探测（在 mini_test 的骨架上加编码器读取）
 * 目的：确认"encoder_i2c_init + encoder_read_checked"本身是否会卡死
 *       不包含总线恢复、不包含复杂诊断
 * ==========================================================================*/
#include "stm32f10x.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>

extern void RCC_Configuration(void);

static volatile unsigned char rx_buf[64];
static volatile unsigned short rx_sta = 0;

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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_2; gi.GPIO_Mode = GPIO_Mode_AF_PP;
	gi.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOA, &gi);
	gi.GPIO_Pin = GPIO_Pin_3; gi.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gi);
	USART2->BRR = (SystemCoreClock/2 + baud/2) / baud;
	USART2->CR1 = 0x200C;
}

static uint32_t clk_hz = 72000000;
static void dly_ms(uint32_t ms){ volatile uint32_t n=(clk_hz/4000)*ms; while(n--) __asm volatile("nop"); }

int main(void)
{
	uint16_t raw;
	int r;

	RCC_Configuration();
	clk_hz = SystemCoreClock;

	{
		GPIO_InitTypeDef gi;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		gi.GPIO_Pin = GPIO_Pin_13; gi.GPIO_Mode = GPIO_Mode_Out_PP;
		gi.GPIO_Speed = GPIO_Speed_10MHz; GPIO_Init(GPIOC, &gi);
		gi.GPIO_Pin = GPIO_Pin_0; gi.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &gi);
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	}
	uart_init(115200);
	dly_ms(200);

	printf("\r\n=== ENC PROBE ===\r\n");
	printf("[1] clock ok, SysClk=%lu\r\n", (unsigned long)clk_hz);

	printf("[2] calling encoder_i2c_init...\r\n");
	encoder_i2c_init(100000);
	printf("[3] init returned, speed=%lu\r\n", (unsigned long)enc_stat.speed_hz);

	printf("[4] first read...\r\n");
	r = encoder_read_checked(&raw, 20, 2);
	printf("[5] read returned: %s raw=%u\r\n", r?"FAIL":"OK", (unsigned)raw);

	printf("[6] second read...\r\n");
	r = encoder_read_checked(&raw, 20, 2);
	printf("[7] read2: %s raw=%u\r\n", r?"FAIL":"OK", (unsigned)raw);

	printf("\r\nstats: ok=%lu fail=%lu abort=%lu recover=%lu\r\n",
		(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail,
		(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover);

	printf("\r\ncommands: A=read ?=status\r\n");

	for(;;)
	{
		if(rx_sta & 0x8000)
		{
			if(rx_buf[0]=='A'){
				r = encoder_read_checked(&raw, 20, 2);
				printf("raw=%u %s\r\n",(unsigned)raw, r?"FAIL":"OK");
			} else if(rx_buf[0]=='?'){
				printf("ok=%lu fail=%lu abort=%lu recover=%lu\r\n",
					(unsigned long)enc_stat.ok,(unsigned long)enc_stat.fail,
					(unsigned long)enc_stat.abort,(unsigned long)enc_stat.recover);
			}
			rx_sta = 0;
		}
	}
}
