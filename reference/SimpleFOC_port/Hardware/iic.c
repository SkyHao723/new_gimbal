

#include "iic.h"

/* I2C1 通信速率，默认 400kHz。
 * 如果出现"时好时坏 / 角度偶尔为 0"，把它降到 100000 试试 ——
 * 400kHz 跑在排线上容易出信号完整性问题。 */
unsigned long I2C_Speed = 400000;

/******************************************************************************/
void I2C_Init_(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//GPIOB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_DeInit(I2C1);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;     //I2C_OAR1
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;    //默认 400KHz，可运行时修改
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}
/******************************************************************************/
/* 运行时修改 I2C 速率（用于排查信号完整性问题）
 * 例如 400000 -> 100000 */
void I2C_SetSpeed(unsigned long speed)
{
	if(speed < 10000)   speed = 10000;
	if(speed > 400000)  speed = 400000;
	I2C_Speed = speed;
	I2C_Init_();        // 重新初始化即可生效
}
/******************************************************************************/
