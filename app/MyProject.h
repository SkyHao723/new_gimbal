/* ============================================================================
 * MyProject.h —— 精简版（配套 app/encoder.c 使用的 FOC 库集成）
 *
 * 与参考工程 user/MyProject.h 的区别：
 *   1. 不包含 iic.h / spi2.h —— 库不再直接碰 I2C，
 *      而是通过 getAngle()/getVelocity() 调用我们验证过的 encoder.c
 *   2. 只保留库真正需要的类型、宏和全局变量声明
 *
 * ★ 这样做的原因：参考工程的 I2C 驱动在控制循环里会读失败（超时不发 STOP，
 *   总线卡死 → 返回 0 → angle 变 NaN）。阶段 1 已把它换成 encoder.c，
 *   这里把库的入口接到新驱动上。
 * ==========================================================================*/
#ifndef MYPROJECT_H
#define MYPROJECT_H

#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "delay.h"
#include "timer.h"

#include "foc_utils.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"

/* ===== 电机使能：EN 在 PB0，高电平使能 =====
 * 若实测厂家小板是低有效，把这两行电平对调 */
#define M1_Enable     do{ GPIO_SetBits(GPIOB, GPIO_Pin_0);   }while(0)
#define M1_Disable    do{ GPIO_ResetBits(GPIOB, GPIO_Pin_0); }while(0)

/* ===== 编码器：由 app/encoder.c 实现，见下面的桥接函数 ===== */
float getAngle(void);        /* rad, 已含符号 */
float getVelocity(void);     /* rad/s */

/* ===== 原 MagneticSensor.c 里定义的全局变量 =====
 * 本集成不使用 MagneticSensor.c（改用 encoder.c），
 * 但库的 BLDCMotor.c 需要 angle_prev，所以在 app/foc_idle_test.c 里定义。 */
extern long  cpr;
extern float full_rotation_offset;
extern long  angle_data_prev;
extern float angle_prev;
extern unsigned long velocity_calc_timestamp;

#endif /* MYPROJECT_H */
