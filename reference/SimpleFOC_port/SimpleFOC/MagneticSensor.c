

#include "MyProject.h"


/************************************************
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
创建日期：20210925
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/******************************************************************************/

/******************************************************************************/
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C   //V2.1.1 bugfix
//#define  RAW_Angle_Lo    0x0D
#define  AS5600_CPR      4096
/******************************************************************************/
/* 读失败计数：用于评估 I2C 通信可靠性（配合 R 命令观察误码率） */
unsigned long I2C_FailCount = 0;
/******************************************************************************/
/******************************************************************************/
/* 读失败时的清理：产生 STOP 释放总线，并清掉 POS 位、恢复 ACK。
 * ★ 原工程在超时时直接 return 0，没有产生 STOP ——
 *   会导致 I2C 主机把 SCL 拉死、总线永久卡住，
 *   表现就是"角度稳定为 0，再也不动"。这里必须补上清理动作。 */
static void I2C_ReadAbort(I2C_TypeDef* I2Cx)
{
	volatile uint32_t t;

	I2C_FailCount++;       /* 统计读失败次数 */

	I2C_GenerateSTOP(I2Cx, ENABLE);

	/* 等 STOP 真正发出（CR1_STOP 位由硬件清除） */
	t = 0xFFFF;
	while((I2Cx->CR1 & 0x0200) && (--t));

	/* 清 POS 位、恢复 ACK，保证下一次读是干净状态 */
	I2Cx->CR1 &= 0xF7FF;   //CR1_POS_Reset
	I2Cx->CR1 |= 0x0400;   //CR1_ACK_Set
}
/******************************************************************************/
unsigned short I2C_getRawCount(I2C_TypeDef* I2Cx)
{
	uint32_t Timeout;
	unsigned short temp;
	unsigned char dh,dl;	
	
	Timeout = 0xFFFF;
	/* Send START condition */
	I2Cx->CR1 |= 0x0100;//CR1_START_Set;
	/* Wait until SB flag is set: EV5 */
	while ((I2Cx->SR1&0x0001) != 0x0001)
	{
		if (Timeout-- == 0){ I2C_ReadAbort(I2Cx); return 0; }
	}
	/* Send the slave address, Reset the address bit0 for write*/
	I2Cx->DR = AS5600_Address<<1;
	Timeout = 0xFFFF;
	/* Wait until ADDR is set: EV6 */
	while ((I2Cx->SR1 &0x0002) != 0x0002)
	{
		if (Timeout-- == 0){ I2C_ReadAbort(I2Cx); return 0; }
	}
	/* Clear ADDR flag by reading SR2 register */
	temp = I2Cx->SR2;
	/* Write the first data in DR register (EV8_1) */
	I2Cx->DR = RAW_Angle_Hi;
	/* EV8_2: Wait until BTF is set before programming the STOP
	 * ★ 原工程这里没有超时保护，从机会死等 —— 补上 */
	Timeout = 0xFFFF;
	while ((I2Cx->SR1 & 0x00004) != 0x000004)
	{
		if (Timeout-- == 0){ I2C_ReadAbort(I2Cx); return 0; }
	}
	
	/////////////////////////////////////////////////////////////////////////
	/* Set POS bit */
	I2Cx->CR1 |= 0x0800;//CR1_POS_Set;
	Timeout = 0xFFFF;
	/* Send START condition */
	I2Cx->CR1 |= 0x0100;//CR1_START_Set;
	/* Wait until SB flag is set: EV5 */
	while ((I2Cx->SR1&0x0001) != 0x0001)
	{
		if (Timeout-- == 0){ I2C_ReadAbort(I2Cx); return 0; }
	}
	Timeout = 0xFFFF;
	/* Send slave address */
	I2Cx->DR = (AS5600_Address<<1)+1;

	/* Wait until ADDR is set: EV6 */
	while ((I2Cx->SR1&0x0002) != 0x0002)
	{
		if (Timeout-- == 0){ I2C_ReadAbort(I2Cx); return 0; }
	}
	/* EV6_1: The acknowledge disable should be done just after EV6,
	that is after ADDR is cleared, so disable all active IRQs around ADDR clearing and 
	ACK clearing */
	__disable_irq();
	/* Clear ADDR by reading SR2 register  */
	temp = I2Cx->SR2;
	/* Clear ACK */
	I2Cx->CR1 &= 0xFBFF;//CR1_ACK_Reset;
	/*Re-enable IRQs */
	__enable_irq();
	/* Wait until BTF is set
	 * ★ 同样补上超时保护，否则从机会永远卡在这里 */
	Timeout = 0xFFFF;
	while ((I2Cx->SR1 & 0x00004) != 0x000004)
	{
		if (Timeout-- == 0){ I2C_ReadAbort(I2Cx); return 0; }
	}
	/* Disable IRQs around STOP programming and data reading because of the limitation ?*/
	__disable_irq();
	/* Program the STOP */
	I2C_GenerateSTOP(I2Cx, ENABLE);
	/* Read first data */
	dh = I2Cx->DR;
	/* Re-enable IRQs */
	__enable_irq();
	/**/
	/* Read second data */
	dl = I2Cx->DR;
	/* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
	Timeout = 0xFFFF;
	while (((I2Cx->CR1&0x200) == 0x200) && (--Timeout));
	/* Enable Acknowledgement to be ready for another reception */
	I2Cx->CR1  |= 0x0400;//CR1_ACK_Set;
	/* Clear POS bit */
	I2Cx->CR1  &= 0xF7FF;//CR1_POS_Reset;
	
	temp++;  //useless,otherwise warning
	return ((dh<<8)+dl);
}
/******************************************************************************/



/******************************************************************************/
//TLE5012B
#define READ_ANGLE_VALUE  0x8020
#define TLE5012B_CPR      32768
/******************************************************************************/
#define SPI2_CS1_L  GPIO_ResetBits(GPIOB, GPIO_Pin_8)      //CS1_L
#define SPI2_CS1_H  GPIO_SetBits(GPIOB, GPIO_Pin_8)        //CS1_H

/* 空操作占位（Twr_delay 时序等待用）
 * ARMCC(Keil) 用内建 __nop()；GCC 用内联汇编，保证换工具链也能编译 */
#ifdef __CC_ARM
#define SPI2_NOP()  __nop()
#else
#define SPI2_NOP()  __asm volatile("nop")
#endif

#define SPI2_TX_OFF {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0x40000000;}  //把PB15(MOSI)配置为浮空输入模式
#define SPI2_TX_ON  {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0xB0000000;}  //把PB15(MOSI)复用推挽输出(50MHz)
/******************************************************************************/
unsigned short SPIx_ReadWriteByte(unsigned short byte)
{
	unsigned short retry = 0;
	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
		if(++retry>200)return 0;
	}
	SPI_I2S_SendData(SPI2, byte);
	retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) 
	{
		if(++retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI2);
}
/******************************************************************************/
unsigned short ReadTLE5012B_1(unsigned short Comm)
{
	unsigned short u16Data;
	
	SPI2_CS1_L;
	SPIx_ReadWriteByte(Comm);
	SPI2_TX_OFF;
	SPI2_NOP();SPI2_NOP();SPI2_NOP();SPI2_NOP();SPI2_NOP();SPI2_NOP();SPI2_NOP();SPI2_NOP();  //Twr_delay=130ns min
	u16Data = SPIx_ReadWriteByte(0xffff);
	
	SPI2_CS1_H;
	SPI2_TX_ON;
	return(u16Data);
}
/******************************************************************************/
//#endif

/******************************************************************************/
/* I2C 设备探测：只发地址不发数据，看从机是否应答（ACK）
 * 返回 0 = 有应答(在线)，1 = 无应答(不在线 / 接线问题)
 * 用于排查编码器：AS5600 的 7 位地址是 0x36
 * 注意：addr7 传 7 位地址，函数内部左移成 8 位 */
#define I2C_DETECT_TIMEOUT   20000

int I2C_CheckDevice(I2C_TypeDef* I2Cx, unsigned char addr7)
{
	unsigned long t;
	int ret = 0;

	/* 确保总线空闲 */
	t = 0;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
		if(++t > I2C_DETECT_TIMEOUT) return 1;
	}

	I2C_GenerateSTART(I2Cx, ENABLE);

	t = 0;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(++t > I2C_DETECT_TIMEOUT){ I2C_GenerateSTOP(I2Cx, ENABLE); return 1; }
	}

	I2C_Send7bitAddress(I2Cx, addr7<<1, I2C_Direction_Transmitter);

	t = 0;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(++t > I2C_DETECT_TIMEOUT)
		{
			/* 超时：可能是 NACK(无设备)，也可能是总线被拉死，统一按"无应答"处理 */
			ret = 1;
			break;
		}
	}

	I2C_GenerateSTOP(I2Cx, ENABLE);
	return ret;
}
/******************************************************************************/

/******************************************************************************/
void MagneticSensor_Init(void)
{
#if M1_AS5600
	cpr=AS5600_CPR;
	angle_data_prev = I2C_getRawCount(I2C1);  
#elif M1_TLE5012B
	cpr=TLE5012B_CPR;
	angle_data_prev = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
#endif
	
	full_rotation_offset = 0;
	velocity_calc_timestamp=0;
}
/******************************************************************************/
float getAngle(void)
{
	float angle_data,d_angle;
	
#if M1_AS5600
	angle_data = I2C_getRawCount(I2C1);
#elif M1_TLE5012B
	angle_data = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
#endif
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - angle_data_prev;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) > (0.8*cpr) ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
	// save the current angle value for the next steps
	// in order to know if overflow happened
	angle_data_prev = angle_data;
	// return the full angle 
	// (number of full rotations)*2PI + current sensor angle 
	return  (full_rotation_offset + ( angle_data / (float)cpr) * _2PI) ;
}
/******************************************************************************/
// Shaft velocity calculation
float getVelocity(void)
{
	unsigned long now_us;
	float Ts, angle_c, vel;

	// calculate sample time
	now_us = SysTick->VAL; //_micros();
	if(now_us<velocity_calc_timestamp)Ts = (float)(velocity_calc_timestamp - now_us)/9*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/9*1e-6;
	// quick fix for strange cases (micros overflow)
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

	// current angle
	angle_c = getAngle();
	// velocity calculation
	vel = (angle_c - angle_prev)/Ts;

	// save variables for future pass
	angle_prev = angle_c;
	velocity_calc_timestamp = now_us;
	return vel;
}
/******************************************************************************/



