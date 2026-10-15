/* ============================================================================
 * uart_dbg.c —— 串口收发诊断
 * 主循环用阻塞方式读，收到就回显，并打印寄存器
 * ==========================================================================*/
#include "stm32f10x.h"
#include <stdio.h>

extern void RCC_Configuration(void);

void _write_impl(int fd, const char *buf, int len)
{
	int i; (void)fd;
	for(i=0;i<len;i++){
		while((USART2->SR & 0x40) == 0);
		USART2->DR = (unsigned char)buf[i];
	}
	while((USART2->SR & 0x40) == 0);
}

static uint32_t clk_hz = 72000000;
static void dly_ms(uint32_t ms){ volatile uint32_t n=(clk_hz/4000)*ms; while(n--) __asm volatile("nop"); }

int main(void)
{
	GPIO_InitTypeDef gi;
	uint32_t n = 0;

	RCC_Configuration();
	clk_hz = SystemCoreClock;

	/* LED */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_13; gi.GPIO_Mode = GPIO_Mode_Out_PP;
	gi.GPIO_Speed = GPIO_Speed_10MHz; GPIO_Init(GPIOC, &gi);

	/* USART2 PA2/PA3 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_2; gi.GPIO_Mode = GPIO_Mode_AF_PP;
	gi.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOA, &gi);
	gi.GPIO_Pin = GPIO_Pin_3; gi.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gi);
	USART2->BRR = (SystemCoreClock/2 + 115200/2) / 115200;
	USART2->CR1 = 0x200C;          /* UE|TE|RE */
	USART2->CR2 = 0;
	USART2->CR3 = 0;

	dly_ms(200);

	printf("\r\n=== UART DEBUG ===\r\n");
	printf("SysClk=%lu BRR=%lu (expect 312)\r\n",
		(unsigned long)SystemCoreClock,(unsigned long)USART2->BRR);
	printf("CR1=0x%04lX CR2=0x%04lX CR3=0x%04lX SR=0x%04lX\r\n",
		(unsigned long)USART2->CR1,(unsigned long)USART2->CR2,
		(unsigned long)USART2->CR3,(unsigned long)USART2->SR);
	printf("GPIOA_CRL=0x%08lX (PA2=0xB, PA3=0x4 expected)\r\n",
		(unsigned long)GPIOA->CRL);
	printf("RX 用【阻塞读】，收到什么就回显什么。发字符试试。\r\n\r\n");

	for(;;)
	{
		/* 阻塞等待一个字节 */
		if(USART2->SR & 0x20)
		{
			uint8_t ch = (uint8_t)(USART2->DR & 0xFF);
			n++;
			printf("[%lu] got 0x%02X '%c'  SR=0x%04lX\r\n",
				(unsigned long)n, ch,
				(ch>=32 && ch<127) ? ch : '.',
				(unsigned long)USART2->SR);
			GPIOC->ODR ^= (1<<13);
		}
	}
}
