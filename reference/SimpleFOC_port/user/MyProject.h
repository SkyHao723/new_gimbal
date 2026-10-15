

#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_it.h" 
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "iic.h"
#include "spi2.h"

#include "MagneticSensor.h" 
#include "foc_utils.h" 
#include "FOCMotor.h" 
#include "BLDCmotor.h" 
#include "lowpass_filter.h" 
#include "pid.h"

/************************************************
 * 硬件适配：STM32F103C8T6 最小系统板(蓝板) + 厂家驱动小板
 * ---------------------------------------------------------------
 * 实际接线（与你的实物一致）
 *
 *   PWM1  PA8   TIM1_CH1  ── 驱动小板 PWM1
 *   PWM2  PA9   TIM1_CH2  ── 驱动小板 PWM2
 *   PWM3  PA10  TIM1_CH3  ── 驱动小板 PWM3
 *   EN    PB0   GPIO 输出  ── 驱动小板 使能（MCU 控制，可用 D/E 命令急停）
 *   GND   GND             ── 驱动小板 GND（必须共地）
 *
 *   TTL   PA2(TX) / PA3(RX)  USART2
 *         ★ 因为 PA9/PA10 已作 PWM，串口只能用 USART2
 *   I2C   PB6(SCL) / PB7(SDA)  AS5600
 *   SWD   PA13 / PA14
 *   LED   PC13
 ************************************************/

/* ===== 电机使能 =====
 * EN 接在 PB0，高电平使能。
 * 若实测厂家小板是低有效，把这两行的高低电平对调即可。 */
#define M1_Enable    GPIO_SetBits(GPIOB,GPIO_Pin_0);          //高电平使能
#define M1_Disable   GPIO_ResetBits(GPIOB,GPIO_Pin_0);        //低电平失能

/* ===== 串口选择 =====
 * 1 = USART1 (PA9/PA10)
 * 0 = USART2 (PA2/PA3)   ← 当前使用：PA9/PA10 已被 PWM 占用
 * 二者不能同时用：PA9/PA10 与 TIM1_CH2/CH3 是同一对引脚。 */
#define USE_USART1   0

/* ===== 编码器类型，二者只能选一。设置使用的编码器为1，不使用的为0 ===== */
#define M1_AS5600    1
#define M1_TLE5012B  0
/* 本节代码只支持2种编码器 */

/* ===== 电机参数（在 main.c 中赋值，此处备忘）=====
 *   voltage_power_supply  驱动小板电源电压(V) —— 必须填对，决定占空比换算
 *   pole_pairs            电机极对数          —— 填错会抖动/反转/不转
 *   voltage_limit         最大相电压(V) ≤ Vbus/1.732
 *   voltage_sensor_align  对齐电压(V)，航模电机 0.5~1，云台电机 2~3
 */

#endif
