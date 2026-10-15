

#include "usart.h"
#include "MyProject.h"

/* ============================================================================
 * usart.c —— 蓝板适配版
 *   USE_USART1 == 1 : USART1  PA9(TX) / PA10(RX)   <-- 默认
 *   USE_USART1 == 0 : USART2  PA2(TX) / PA3(RX)
 *
 * 默认用 USART1：因为本工程 PWM 占用 PA0/PA1/PA2，
 * 而 PA2 同时也是 USART2_TX，两者不能共存。
 * 只有当把 PWM 换到 TIM1(PA8/PA9/PA10) 时，才把 USE_USART1 改成 0 用 USART2。
 * ==========================================================================*/

#if USE_USART1
#define DEBUG_USART        USART1
#define DEBUG_USART_IRQn   USART1_IRQn
#else
#define DEBUG_USART        USART2
#define DEBUG_USART_IRQn   USART2_IRQn
#endif

//本段代码参考了正点原子，微调
/******************************************************************************/
/* printf 重定向底层实现：两种工具链共用
 *   ARMCC(Keil) : #pragma import(__use_no_semihosting) + fputc
 *   GCC         : syscalls.c 的 _write -> _write_impl
 * 注意：两份实现都直接操作 DEBUG_USART 寄存器，行为完全一致。 */
void _write_impl(int fd, const char *buf, int len)
{
	int i;
	(void)fd;
	for(i = 0; i < len; i++)
	{
		while((DEBUG_USART->SR & 0x40) == 0);   /* 等 TXE */
		DEBUG_USART->DR = (unsigned char)buf[i];
	}
	while((DEBUG_USART->SR & 0x40) == 0);
}

#ifdef __CC_ARM
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
_sys_exit(int x) { x = x; }
int fputc(int ch, FILE *f)
{
	(void)f;
	while((DEBUG_USART->SR&0X40)==0);
	DEBUG_USART->DR = (u8) ch;
	return ch;
}
#endif
/******************************************************************************/
/******************************************************************************/
unsigned char USART_RX_BUF[USART_REC_LEN];     //接收缓冲，usart.h中定义长度
//接收状态
//bit15  接收完成标志
//bit14  接收到0x0D
//bit13~0  接收的字节数
unsigned short USART_RX_STA=0;       //接收状态标志
/******************************************************************************/
void uart_init(unsigned long bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
#if USE_USART1
	/* ---- USART1: PA9=TX, PA10=RX ---- */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;              //PA9  TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;             //PA10 RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#else
	/* ---- USART2: PA2=TX, PA3=RX ---- */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;              //PA2  TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;              //PA3  RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;    //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			       //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
  
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(DEBUG_USART, &USART_InitStructure);
  USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);  //使能接收中断
  USART_Cmd(DEBUG_USART, ENABLE);                      //使能串口
}
/******************************************************************************/
/* 接收中断服务函数：注意函数名必须与实际使用的串口一致 */
#if USE_USART1
void USART1_IRQHandler(void)
#else
void USART2_IRQHandler(void)
#endif
{
	unsigned char Res;
	
	if(USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)  //
	{
		Res = USART_ReceiveData(DEBUG_USART);	//读取接收到的字节
		
		if((USART_RX_STA&0x8000)==0)    //接收未完成
		{
			if(USART_RX_STA&0x4000)       //接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;   //接收错误，重新开始
				else 
				{
					USART_RX_STA|=0x8000;	       //接收完成
					USART_RX_BUF[USART_RX_STA&0X3FFF]='\0';   //最后一个字节放'0'，方便判断
				}
			}
			else //还没收到0x0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;  //接收错误，重新开始  
				}		 
			}
		}
  }
}
/******************************************************************************/
