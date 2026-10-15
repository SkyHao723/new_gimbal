/* ============================================================================
 * encoder.h —— AS5600 角度读取（健壮版）
 * ==========================================================================*/
#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f10x.h"

/* AS5600 7 位地址与寄存器 */
#define AS5600_ADDR               0x36
#define AS5600_REG_RAW_ANGLE_H    0x0C
#define AS5600_REG_RAW_ANGLE_L    0x0D
#define AS5600_REG_STATUS         0x0B
#define AS5600_REG_AGC            0x1A
#define AS5600_REG_MAGNITUDE_H    0x1B
#define AS5600_REG_MAGNITUDE_L    0x1C

/* 统计信息 */
typedef struct {
	uint32_t ok;            /* 单次读成功次数 */
	uint32_t fail;          /* 单次读失败次数 */
	uint32_t checked_ok;    /* 双读校验通过次数 */
	uint32_t outlier;       /* 双读不一致被丢弃次数 */
	uint32_t abort;         /* 超时清理次数 */
	uint32_t recover;       /* 总线恢复次数 */
	uint32_t speed_hz;      /* 当前 I2C 速率 */
} EncoderStats_t;

extern EncoderStats_t enc_stat;

/* 初始化 / 速率设置 */
void encoder_i2c_init(uint32_t speed_hz);

/* 底层读（返回 0 成功） */
int  encoder_read(uint16_t *raw, uint8_t retry);
int  encoder_read_checked(uint16_t *raw, uint16_t max_delta, uint8_t retry);

/* 换算 */
float encoder_raw_to_rad(uint16_t raw);

/* 工具 */
void    encoder_reset_stats(void);
void    encoder_bus_recover(void);
uint8_t encoder_check_lines(void);
int     encoder_jitter_test(uint16_t n, uint16_t *min_p, uint16_t *max_p, uint16_t *fail_p);

#endif
