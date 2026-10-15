#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H

/******************************************************************************/
extern long  cpr;
extern float full_rotation_offset;
extern long  angle_data_prev;
extern unsigned long velocity_calc_timestamp;
extern float angle_prev;
/******************************************************************************/
void MagneticSensor_Init(void);
float getAngle(void);
float getVelocity(void);
/* I2C 设备探测：返回 0 表示应答，1 表示无应答（用于排查编码器接线） */
int I2C_CheckDevice(I2C_TypeDef* I2Cx, unsigned char addr7);
/* 读取 AS5600 原始角度计数（0~4095），失败返回 0 */
unsigned short I2C_getRawCount(I2C_TypeDef* I2Cx);
/* I2C 读失败累计次数，用于评估通信可靠性 */
extern unsigned long I2C_FailCount;
/******************************************************************************/

#endif
