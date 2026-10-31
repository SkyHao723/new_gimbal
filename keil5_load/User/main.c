/* Load board firmware: stream this board's own IMU over CAN.
 *
 * Hardware = the old host board (G431 + CH343P on USART1 + TJA1042 on FDCAN1
 * + ICM-42670-P on PB6/PB7 bit-bang I2C + LED on PA5). It sits at the far end
 * of the CAN bus and only reports attitude data so the host can show what the
 * payload is doing.
 *
 * CAN frames (all standard ID, classic frames, 500 kbit/s):
 *   0x2A0  ACC    : [0]=seq [1..2]=ax [3..4]=ay [5..6]=az (int16 LE, mg) [7]=imu_ok
 *   0x2A1  GYRO   : [0]=seq [1..2]=gx [3..4]=gy [5..6]=gz (int16 LE, 0.1 dps)
 *                   [7]=temperature (int8, degC)
 *   0x2A2  STATUS : [0..1]=rate_hz (u16 LE) [2]=telemetry_on [3]=imu_ok
 *                   [4..7]=samples (u32 LE)
 *
 * Serial console (CH343P, 115200 8N1):
 *   h / ?     help
 *   s         status (rate, sample count, tx counters, IMU error count)
 *   m         read the IMU once and print it
 *   r<Hz>     set report rate (10..500 Hz)
 *   o<0|1>    telemetry off / on
 *   l         toggle the LED
 *
 * The board is intentionally passive: it never drives motors and ignores every
 * CAN command aimed at the driver boards.
 */

#include "stm32g4xx_hal.h"
#include "clock.h"
#include "console.h"
#include "host_board.h"
#include "host_config.h"
#include "icm42670.h"
#include "can_host.h"
#include "load_config.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static ImuInfo imu;
volatile uint8_t imu_ok;                    /* 0 = IMU not usable (visible on SWD) */
static uint8_t telemetry_on = 1u;
static uint16_t rate_hz = LOAD_RATE_HZ_DEFAULT;
static uint8_t seq;

volatile uint32_t load_samples;        /* IMU samples converted and sent      */
volatile uint32_t load_tx_acc;         /* accel frames queued                 */
volatile uint32_t load_tx_gyro;        /* gyro frames queued                  */
volatile uint32_t load_tx_fail;        /* frames rejected by the CAN driver   */
volatile uint32_t load_imu_errors;     /* failed IMU reads                    */
volatile uint32_t load_status_frames;  /* status frames queued                */
volatile uint32_t can_selftest[12];    /* internal-loopback self test result  */

static void put_i16(uint8_t *dst, int32_t value)
{
    if (value > 32767) {
        value = 32767;
    } else if (value < -32768) {
        value = -32768;
    }
    dst[0] = (uint8_t)((uint16_t)value & 0xFFu);
    dst[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFFu);
}

static int32_t accel_to_mg(int16_t raw)
{
    /* LSB/g = 2048  ->  mg = raw * 1000 / 2048 */
    return ((int32_t)raw * 1000) / 2048;
}

static int32_t gyro_to_tenth_dps(int16_t raw)
{
    /* LSB/dps = 16.4  ->  0.1 dps = raw * 10 / 16.4 = raw * 25 / 41 */
    return ((int32_t)raw * 25) / 41;
}

static int32_t temp_to_degc(int16_t raw)
{
    /* ICM-42670-P: temperature = raw / 132.48 + 25 degC */
    return ((int32_t)raw * 100) / 13248 + 25;
}

static void load_send_telemetry(void)
{
    int16_t acc[3];
    int16_t gyro[3];
    int16_t traw = 0;
    uint8_t a[8];
    uint8_t g[8];
    int i;

    if (!imu_read_raw(&imu, acc, gyro, &traw)) {
        ++load_imu_errors;
        imu_ok = 0u;
        return;
    }

    a[0] = seq;
    g[0] = seq;
    for (i = 0; i < 3; ++i) {
        put_i16(&a[1 + 2 * i], accel_to_mg(acc[i]));
        put_i16(&g[1 + 2 * i], gyro_to_tenth_dps(gyro[i]));
    }
    a[7] = imu_ok;
    {
        int32_t t = temp_to_degc(traw);
        if (t > 127) {
            t = 127;
        } else if (t < -128) {
            t = -128;
        }
        g[7] = (uint8_t)((uint8_t)(int8_t)t);
    }

    if (can_host_send(LOAD_CAN_ID_ACC, a, 8u)) {
        ++load_tx_acc;
    } else {
        ++load_tx_fail;
    }
    if (can_host_send(LOAD_CAN_ID_GYRO, g, 8u)) {
        ++load_tx_gyro;
    } else {
        ++load_tx_fail;
    }

    ++load_samples;
    ++seq;
}

static void load_send_status(void)
{
    uint8_t s[8];

    s[0] = (uint8_t)(rate_hz & 0xFFu);
    s[1] = (uint8_t)((rate_hz >> 8) & 0xFFu);
    s[2] = telemetry_on;
    s[3] = imu_ok;
    s[4] = (uint8_t)(load_samples & 0xFFu);
    s[5] = (uint8_t)((load_samples >> 8) & 0xFFu);
    s[6] = (uint8_t)((load_samples >> 16) & 0xFFu);
    s[7] = (uint8_t)((load_samples >> 24) & 0xFFu);
    if (can_host_send(LOAD_CAN_ID_STATUS, s, 8u)) {
        ++load_status_frames;
    }
}

static void print_imu_once(void)
{
    int16_t acc[3];
    int16_t gyro[3];
    int16_t traw = 0;

    if (!imu_ok) {
        printf("[IMU] not initialised (%s)\r\n", imu_last_error());
        return;
    }
    if (!imu_read_raw(&imu, acc, gyro, &traw)) {
        printf("[IMU] read failed (%s)\r\n", imu_last_error());
        return;
    }
    printf("[IMU] acc[g] %.3f %.3f %.3f  gyro[dps] %.1f %.1f %.1f  temp %ld C\r\n",
           (double)acc[0] / 2048.0, (double)acc[1] / 2048.0, (double)acc[2] / 2048.0,
           (double)gyro[0] / 16.4, (double)gyro[1] / 16.4, (double)gyro[2] / 16.4,
           (long)temp_to_degc(traw));
}

static void print_status(void)
{
    printf("[LOAD] rate=%u Hz  telemetry=%s  imu=%s addr=0x%02X\r\n",
           (unsigned)rate_hz, telemetry_on ? "on" : "off",
           imu_ok ? "ok" : "fail", (unsigned)imu.address);
    printf("[LOAD] samples=%lu tx_acc=%lu tx_gyro=%lu tx_fail=%lu imu_err=%lu\r\n",
           (unsigned long)load_samples, (unsigned long)load_tx_acc,
           (unsigned long)load_tx_gyro, (unsigned long)load_tx_fail,
           (unsigned long)load_imu_errors);
    printf("[CAN]  psr=0x%08lX ecr=0x%08lX txdrop=%lu busoff=%lu\r\n",
           (unsigned long)can_host_psr(), (unsigned long)can_host_error_count(),
           (unsigned long)can_host_tx_dropped(), (unsigned long)can_host_bus_off_count());
}

static void print_help(void)
{
    printf("commands:\r\n");
    printf("  h / ?   this help\r\n");
    printf("  s       status\r\n");
    printf("  m       read IMU once\r\n");
    printf("  r<Hz>   report rate, %u..%u\r\n",
           (unsigned)LOAD_RATE_HZ_MIN, (unsigned)LOAD_RATE_HZ_MAX);
    printf("  o<0|1>  telemetry off/on\r\n");
    printf("  l       toggle LED\r\n");
    printf("CAN out: 0x%03X acc / 0x%03X gyro / 0x%03X status\r\n",
           (unsigned)LOAD_CAN_ID_ACC, (unsigned)LOAD_CAN_ID_GYRO,
           (unsigned)LOAD_CAN_ID_STATUS);
}

static int parse_u32(const char *text, uint32_t *value)
{
    uint32_t v = 0u;
    int digits = 0;

    while (*text == ' ') {
        ++text;
    }
    while (*text >= '0' && *text <= '9') {
        v = v * 10u + (uint32_t)(*text - '0');
        ++text;
        ++digits;
        if (digits > 6) {
            return 0;
        }
    }
    if (digits == 0) {
        return 0;
    }
    *value = v;
    return 1;
}

static void handle_command(char *line)
{
    char op = line[0];
    uint32_t value;

    switch (op) {
    case 'h':
    case '?':
    case '\0':
        print_help();
        break;
    case 's':
        print_status();
        break;
    case 'm':
        print_imu_once();
        break;
    case 'l':
        host_led_toggle();
        printf("[LED] toggled\r\n");
        break;
    case 'r':
        if (!parse_u32(&line[1], &value)) {
            printf("[ERR] usage: r<Hz>\r\n");
            break;
        }
        if (value < LOAD_RATE_HZ_MIN || value > LOAD_RATE_HZ_MAX) {
            printf("[ERR] rate out of range %u..%u\r\n",
                   (unsigned)LOAD_RATE_HZ_MIN, (unsigned)LOAD_RATE_HZ_MAX);
            break;
        }
        rate_hz = (uint16_t)value;
        printf("[LOAD] rate=%u Hz\r\n", (unsigned)rate_hz);
        break;
    case 'o':
        if (!parse_u32(&line[1], &value)) {
            printf("[ERR] usage: o<0|1>\r\n");
            break;
        }
        telemetry_on = (value != 0u) ? 1u : 0u;
        printf("[LOAD] telemetry %s\r\n", telemetry_on ? "on" : "off");
        break;
    default:
        printf("[ERR] unknown command '%c' (h for help)\r\n", op);
        break;
    }
}

int main(void)
{
    char line[96];
    uint32_t last_sample_ms;
    uint32_t next_status_ms;
    uint32_t next_led_ms;

    if (!clock_init()) {
        for (;;) {
        }
    }

    console_init(HOST_CONSOLE_BAUD);
    host_board_init();

    printf("\r\n=== LOAD BOARD : IMU -> CAN ===\r\n");
    printf("SYSCLK=%lu Hz  CAN 500 kbit/s  LED=PA5  IMU=PB6/PB7 bit-bang\r\n",
           (unsigned long)HAL_RCC_GetHCLKFreq());

    console_flush_input();

#if !LOAD_DISABLE_CAN
    /* The old host board leaves the TJA1042 STB pin floating (PB5 reads high =
       standby, transmitter disabled -> every frame ends in bit0 error/bus-off).
       Drive PB5 low to put the transceiver into normal mode. Board specific to
       the load board, so it lives here instead of in the shared can_host.c. */
    {
        GPIO_InitTypeDef stb = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE();
        stb.Pin = GPIO_PIN_5;
        stb.Mode = GPIO_MODE_OUTPUT_PP;
        stb.Pull = GPIO_NOPULL;
        stb.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &stb);
        GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16;   /* low = normal mode */
    }

    if (!can_host_init()) {
        printf("[CAN] init FAILED\r\n");
    } else {
        uint32_t self[12];
        int ok = can_host_loopback_test(self);
        (void)can_host_external_loopback_test(self);
        int i;
        for (i = 0; i < 12; ++i) {
            can_selftest[i] = self[i];
        }
        if (ok) {
            printf("[CAN] loopback OK (id=0x%03lX dlc=%lu)\r\n",
                   (unsigned long)self[2], (unsigned long)self[3]);
        } else {
            printf("[CAN] loopback FAILED (sent=%lu got=%lu psr=0x%08lX)\r\n",
                   (unsigned long)self[0], (unsigned long)self[1],
                   (unsigned long)self[5]);
            printf("[CAN]   -> MCU side FDCAN problem, not the bus\r\n");
        }
    }
#endif

    imu_ok = 0u;
    if (imu_i2c_init()) {
        if (imu_init(&imu)) {
            imu_ok = 1u;
            printf("[IMU] OK addr=0x%02X WHO_AM_I=0x%02X\r\n",
                   (unsigned)imu.address, (unsigned)imu.who_am_i);
        } else {
            printf("[IMU] init failed (%s)\r\n", imu_last_error());
        }
    } else {
        printf("[IMU] bus init failed (%s)\r\n", imu_last_error());
    }

    printf("[LOAD] reporting at %u Hz, CAN 0x%03X/0x%03X/0x%03X\r\n",
           (unsigned)rate_hz, (unsigned)LOAD_CAN_ID_ACC,
           (unsigned)LOAD_CAN_ID_GYRO, (unsigned)LOAD_CAN_ID_STATUS);
    printf("Ready. Type h for help.\r\n> ");

    last_sample_ms = HAL_GetTick();
    next_status_ms = last_sample_ms + 1000u;
    next_led_ms = last_sample_ms + 500u;

    for (;;) {
        uint32_t now = HAL_GetTick();

        if (telemetry_on && imu_ok) {
            uint32_t period = 1000u / (uint32_t)rate_hz;
            if (period == 0u) {
                period = 1u;
            }
            if ((uint32_t)(now - last_sample_ms) >= period) {
                last_sample_ms = now;
                load_send_telemetry();
            }
        }

        if ((int32_t)(now - next_status_ms) >= 0) {
            next_status_ms = now + 1000u;
            load_send_status();
        }

        if ((int32_t)(now - next_led_ms) >= 0) {
            next_led_ms = now + 500u;
            host_led_toggle();
        }

        if (console_read_line(line, sizeof(line))) {
            handle_command(line);
            console_prompt();
        }

        can_host_service();
    }
}
