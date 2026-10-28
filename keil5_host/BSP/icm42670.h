#ifndef HOST_ICM42670_H
#define HOST_ICM42670_H

#include <stdint.h>

/* ICM-42670-P（六轴 IMU）走 I2C1：PB6=SCL / PB7=SDA，AF4，开漏。
   寄存器按 bank0 直接读，没做 FIFO / 中断，够点板和后续姿态解算用。 */

typedef struct {
    uint8_t address;        /* 0x68 或 0x69 */
    uint8_t who_am_i;
    uint8_t speed_code;     /* 0=100k 1=50k 2=10k，实际能通上的那一档 */
} ImuInfo;

/* 初始化 I2C1（硬件 I2C）。返回 1 成功，0 失败（失败信息用 imu_last_error()）。 */
int imu_i2c_init(void);

/* 扫描 0x08..0x77，把有应答的地址写进 found[]，返回个数（最多 max）。 */
int imu_scan(uint8_t *found, int max);

/* 探测一块 IMU：核对 WHO_AM_I(0x75)==0x67，成功后配置电源模式。
   返回 1 成功；0 没找到 / 型号不符。 */
int imu_init(ImuInfo *imu);

/* 读一次原始数据：加速度（LSB/g = 2048）与角速度（LSB/dps = 16.4）。
   返回 1 成功。 */
int imu_read_raw(const ImuInfo *imu, int16_t accel[3], int16_t gyro[3], int16_t *temp_raw);

const char *imu_last_error(void);

#endif
