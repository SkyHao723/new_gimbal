/* ============================================================================
 * peri_test.c —— 逐个初始化外设，定位哪个会让系统卡死
 * 每步都打印，卡在哪一步就一目了然
 * ==========================================================================*/
#include "stm32f10x.h"
#include <stdio.h>

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
	gi.GPIO_Pin = GPIO_Pin_13; gi.GPIO_Mode = GPIO_Mode_Out_PP;
	gi.GPIO_Speed = GPIO_Speed_10MHz; GPIO_Init(GPIOC, &gi);
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	gi.GPIO_Pin = GPIO_Pin_0; gi.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &gi);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/* --- 各外设初始化，均带步骤打印 --- */
static void step_i2c(void)
{
	GPIO_InitTypeDef gi;
	I2C_InitTypeDef ii;
	printf("[A] I2C clock enable... ");
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	printf("done\r\n");

	printf("[B] PB6/PB7 AF_OD... ");
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	gi.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	gi.GPIO_Mode = GPIO_Mode_AF_OD;
	gi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gi);
	printf("done\r\n");

	printf("[C] I2C_DeInit... ");
	I2C_DeInit(I2C1);
	printf("done\r\n");

	printf("[D] I2C_Init... ");
	ii.I2C_Mode = I2C_Mode_I2C;
	ii.I2C_DutyCycle = I2C_DutyCycle_2;
	ii.I2C_OwnAddress1 = 0;
	ii.I2C_Ack = I2C_Ack_Enable;
	ii.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	ii.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1, &ii);
	printf("done\r\n");

	printf("[E] I2C_Cmd... ");
	I2C_Cmd(I2C1, ENABLE);
	printf("done\r\n");

	printf("[F] CR1=0x%04lX CR2=0x%04lX OAR1=0x%04lX\r\n",
		(unsigned long)I2C1->CR1,(unsigned long)I2C1->CR2,(unsigned long)I2C1->OAR1);
}

static void step_adc(void)
{
	ADC_InitTypeDef ai;
	DMA_InitTypeDef di;
	static volatile uint16_t buf[3];

	printf("[G] ADC+DMA clock... ");
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	printf("done\r\n");

	printf("[H] PWM pins / GPIO... ");
	{
		GPIO_InitTypeDef gi;
		gi.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
		gi.GPIO_Mode = GPIO_Mode_AF_PP;
		gi.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gi);
	}
	printf("done\r\n");

	printf("[I] DMA_DeInit... ");
	DMA_DeInit(DMA1_Channel1);
	printf("done\r\n");

	printf("[J] DMA_Init... ");
	di.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;
	di.DMA_MemoryBaseAddr=(uint32_t)buf;
	di.DMA_DIR=DMA_DIR_PeripheralSRC; di.DMA_BufferSize=3;
	di.DMA_PeripheralInc=DMA_PeripheralInc_Disable; di.DMA_MemoryInc=DMA_MemoryInc_Enable;
	di.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	di.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	di.DMA_Mode=DMA_Mode_Circular; di.DMA_Priority=DMA_Priority_High; di.DMA_M2M=DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1,&di);
	printf("done\r\n");

	printf("[K] DMA_Cmd... ");
	DMA_Cmd(DMA1_Channel1,ENABLE);
	printf("done\r\n");

	printf("[L] ADC_Init... ");
	ai.ADC_Mode=ADC_Mode_Independent; ai.ADC_ScanConvMode=ENABLE;
	ai.ADC_ContinuousConvMode=ENABLE; ai.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ai.ADC_DataAlign=ADC_DataAlign_Right; ai.ADC_NbrOfChannel=3;
	ADC_Init(ADC1,&ai);
	printf("done\r\n");

	printf("[M] channel cfg... ");
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_9,3,ADC_SampleTime_55Cycles5);
	printf("done\r\n");

	printf("[N] ADC_Cmd... ");
	ADC_DMACmd(ADC1,ENABLE); ADC_Cmd(ADC1,ENABLE);
	printf("done\r\n");

	printf("[O] reset calib... ");
	ADC_ResetCalibration(ADC1);
	printf("done; wait... ");
	while(ADC_GetResetCalibrationStatus(ADC1));
	printf("done\r\n");

	printf("[P] start calib... ");
	ADC_StartCalibration(ADC1);
	printf("done; wait... ");
	while(ADC_GetCalibrationStatus(ADC1));
	printf("done\r\n");

	printf("[Q] start conv... ");
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	printf("done\r\n");
	dly_ms(100);
	printf("[R] adc raw = %u/%u/%u\r\n",(unsigned)buf[0],(unsigned)buf[1],(unsigned)buf[2]);
}

int main(void)
{
	RCC_Configuration();
	clk_hz = SystemCoreClock;

	gpio_init();
	uart_init(115200);
	dly_ms(200);

	printf("\r\n=== PERIPHERAL TEST ===\r\n");
	printf("SysClk=%lu\r\n\r\n", (unsigned long)clk_hz);

	printf(">>> STEP 1: I2C\r\n");
	step_i2c();
	dly_ms(100);

	printf(">>> STEP 2: ADC/DMA\r\n");
	step_adc();
	dly_ms(100);

	printf(">>> STEP 3: IWDG\r\n");
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);
	IWDG_SetReload(2500);
	/* feed disabled */;
	/* IWDG disabled for debugging */;
	printf("IWDG enabled\r\n\r\n");

	printf(">>> ALL PERIPHERALS OK. Heartbeat follows.\r\n\r\n");

	for(;;)
	{
		volatile uint32_t i;
		/* feed disabled */;
		for(i=0;i<600000;i++);
		GPIOC->ODR ^= (1<<13);
		printf("beat\r\n");
	}
}

