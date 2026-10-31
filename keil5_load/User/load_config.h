#ifndef LOAD_CONFIG_H
#define LOAD_CONFIG_H

/* 负载板（旧 host 板）固件配置。
 *
 * 硬件和上位机板完全一致：
 *   PA9  / PA10 : USART1 <-> CH343P <-> USB（调试串口，115200 8N1）
 *   PB8  / PB9  : FDCAN1 <-> TJA1042（挂在上位机板 CAN 总线的末端）
 *   PB6  / PB7  : GPIO bit-bang I2C <-> ICM-42670-P（0x68）
 *   PA5         : 指示灯
 *
 * 唯一功能：按固定频率把自身 IMU 数据实时上报到 CAN，供上位机板读取显示。
 * 不驱动电机、不接受运动命令。
 */

/* ---- CAN 报文 ID ------------------------------------------------------
 * 上位机板占用：0x200~0x207（给驱动板的命令）、0x280~0x287（驱动板回包）。
 * 负载板用 0x2A0 ~ 0x2A2，互不冲突。
 */
#define LOAD_CAN_ID_ACC          0x2A0u   /* 加速度帧 */
#define LOAD_CAN_ID_GYRO         0x2A1u   /* 角速度 + 温度帧 */
#define LOAD_CAN_ID_STATUS       0x2A2u   /* 状态帧（1 Hz） */

/* ---- 上报频率 --------------------------------------------------------- */
#define LOAD_RATE_HZ_DEFAULT     100u     /* 默认 100 Hz；可用串口命令 r<Hz> 改 */
#define LOAD_RATE_HZ_MIN         10u
#define LOAD_RATE_HZ_MAX         500u

/* ---- IMU 量程（与 icm42670.c 里的配置一致）---------------------------- */
#define LOAD_ACC_LSB_PER_G       2048.0f  /* ±16 g  -> 2048 LSB/g  */
#define LOAD_GYRO_LSB_PER_DPS    16.4f    /* ±2000 dps -> 16.4 LSB/dps */

#endif /* LOAD_CONFIG_H */

/* 1 = do not touch CAN at all (diagnostic: proves whether the bus is
 * dragged down by this firmware or by the hardware). Default 0. */
#ifndef LOAD_DISABLE_CAN
#define LOAD_DISABLE_CAN 0
#endif
