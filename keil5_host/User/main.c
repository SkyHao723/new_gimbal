/* Host board (G431 CAN master) bring-up firmware
 *
 * Goal: verify the four peripherals on a fresh board with no external code:
 *   1) clock + console UART (CH343P)   2) LED PA5
 *   3) I2C1 + ICM-42670-P (IMU)        4) FDCAN1 + the three driver boards
 *   5) USART2 towards ESP32-S3 (init only, protocol deferred)
 *
 * Console commands (115200 8N1):
 *   h / ?     help
 *   i         rescan I2C bus
 *   m         read IMU once
 *   n         poll the three driver boards (broadcast '?' and collect 2 s)
 *   c<N> <cmd>  send command to driver node N:  c1 T0.5   c2 M1   c0 V2.5 (c0 = broadcast)
 *   b         board status (clock, CAN error counters)
 *   l         toggle LED
 *
 * NOTE: kept ASCII-only because Keil ARMCC5 rejects UTF-8 multibyte literals.
 */

#include "stm32g4xx_hal.h"
#include "clock.h"
#include "console.h"
#include "host_board.h"
#include "host_config.h"
#include "can_host.h"
#include "icm42670.h"
#include <stdio.h>
#include <string.h>

static ImuInfo imu;

static void print_banner(void)
{
    printf("\r\n===== G431 host board bring-up =====\r\n");
    printf("SYSCLK=%lu Hz  HCLK=%lu  PCLK1=%lu  PCLK2=%lu\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(), (unsigned long)HAL_RCC_GetHCLKFreq(),
           (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)HAL_RCC_GetPCLK2Freq());
    printf("UART1 PA9/PA10 <-> CH343P @ %lu 8N1\r\n", (unsigned long)HOST_CONSOLE_BAUD);
    printf("I2C1 PB6/PB7 -> ICM-42670-P   FDCAN1 PB8/PB9 @ 500k   LED PA5\r\n");
    printf("Type h for the command list\r\n");
}

static void report_imu(void)
{
    int16_t acc[3], gyr[3], temp = 0;
    if (!imu_read_raw(&imu, acc, gyr, &temp)) {
        printf("IMU: read failed (%s)\r\n", imu_last_error());
        return;
    }
    printf("IMU acc[g] %.3f %.3f %.3f   gyro[dps] %.1f %.1f %.1f   temp %.1f C\r\n",
           acc[0] / 2048.0f, acc[1] / 2048.0f, acc[2] / 2048.0f,
           gyr[0] / 16.4f, gyr[1] / 16.4f, gyr[2] / 16.4f,
           25.0f + temp / 128.0f);
}

static void imu_bringup(void)
{
    uint8_t found[16];
    int n;

    /* 先初始化 I2C1（PB6/PB7 引脚 + 外设 + 100k 时序），否则后面的扫描/读寄存器全是空转 */
    imu_i2c_init();

    /* 先做引脚级总线自检（结果存 RAM，串口打不开也能用 SWD 读 bus_diag[8]），
       内部会顺带做 9 个时钟的总线解锁并初始化 I2C1 */
    imu_bus_diag();

    printf("\r\n[I2C] scanning 0x08..0x77 ...\r\n");
    printf("[I2C] bus levels  pu=%02lX pd=%02lX clear=%02lX\r\n",
           (unsigned long)bus_diag[0], (unsigned long)bus_diag[1], (unsigned long)bus_diag[3]);
    n = imu_scan(found, 16);
    imu_bus_diag_scan_result(n, found);
    if (n <= 0) {
        printf("[I2C] no device found - check IMU soldering, PB6/PB7 pull-ups, 3V3\r\n");
        return;
    }
    printf("[I2C] %d address(es):", n);
    for (int i = 0; i < n && i < 16; ++i) printf(" 0x%02X", found[i]);
    printf("\r\n");
    if (!imu_init(&imu)) {
        printf("[IMU] init failed: %s\r\n", imu_last_error());
        return;
    }
    printf("[IMU] OK  addr=0x%02X  WHO_AM_I=0x%02X  I2C clock=%s\r\n",
           imu.address, imu.who_am_i,
           imu.speed_code == 0 ? "100k" : (imu.speed_code == 1 ? "50k" : "10k"));
    report_imu();
}

/* Broadcast '?' and print whatever the driver boards answer, grouped per CAN ID. */
static void can_scan_drivers(void)
{
    uint32_t t0, last_ms;
    uint32_t cur_id = 0xFFFFFFFFu;
    char buf[256];
    unsigned blen = 0;
    int printed = 0;
    CanFrame f;

    if (!can_host_send_command(0u, "?")) {
        printf("[CAN] send failed (bus not ready)\r\n");
        return;
    }
    printf("\r\n[CAN] broadcast '?', collecting replies for 2 s (PSR=0x%08lX ECR=0x%08lX)\r\n",
           (unsigned long)can_host_psr(), (unsigned long)can_host_error_count());

    t0 = HAL_GetTick();
    last_ms = t0;
    for (;;) {
        uint32_t now = HAL_GetTick();
        can_host_service();
        while (can_host_recv(&f)) {
            if (f.id != cur_id || blen + f.dlc > sizeof(buf) - 1u) {
                if (blen > 0) {
                    buf[blen] = 0;
                    printf("  [0x%03lX] %s\r\n", (unsigned long)cur_id, buf);
                    ++printed;
                }
                cur_id = f.id;
                blen = 0;
            }
            memcpy(&buf[blen], f.data, f.dlc);
            blen += f.dlc;
            last_ms = now;
        }
        /* 一个节点的回复是一串 8 字节帧，静默 30 ms 视为该节点说完 */
        if (blen > 0 && (now - last_ms) > 30u) {
            buf[blen] = 0;
            printf("  [0x%03lX] %s\r\n", (unsigned long)cur_id, buf);
            ++printed;
            blen = 0;
            cur_id = 0xFFFFFFFFu;
        }
        if ((now - t0) > 2000u) break;
    }
    if (blen > 0) {
        buf[blen] = 0;
        printf("  [0x%03lX] %s\r\n", (unsigned long)cur_id, buf);
        ++printed;
    }
    if (printed == 0) {
        printf("  no reply: check CANH/CANL wiring, 120 ohm termination, driver board power\r\n");
    }
}

static void can_bringup(void)
{
    printf("\r\n[CAN] init FDCAN1 500k ... ");
    if (!can_host_init()) {
        printf("FAILED (check PB8/PB9 and TJA1042 supply)\r\n");
        return;
    }
    printf("OK  PSR=0x%08lX\r\n", (unsigned long)can_host_psr());
    can_scan_drivers();
}

static void print_status(void)
{
    printf("SYSCLK=%lu  CAN PSR=0x%08lX ECR=0x%08lX rx=%lu busoff=%lu txdrop=%lu\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(), (unsigned long)can_host_psr(),
           (unsigned long)can_host_error_count(), (unsigned long)can_host_rx_count(),
           (unsigned long)can_host_bus_off_count(), (unsigned long)can_host_tx_dropped());
    printf("UART1 dropped bytes=%lu  IMU addr=0x%02X\r\n",
           (unsigned long)console_dropped_bytes(), imu.address ? imu.address : 0u);
}

static void handle_command(char *line)
{
    if (!strcmp(line, "h") || !strcmp(line, "?")) {
        printf("h/?: help   i: I2C scan   m: read IMU   n: poll drivers   b: status   l: LED\r\n");
        printf("c<N> <cmd>: send to node N, e.g. c1 T0.5 / c2 M1 / c0 V2.5 (c0 = broadcast)\r\n");
        return;
    }
    if (!strcmp(line, "i")) { imu_bringup(); return; }
    if (!strcmp(line, "m")) { report_imu(); return; }
    if (!strcmp(line, "n")) { can_scan_drivers(); return; }
    if (!strcmp(line, "b")) { print_status(); return; }
    if (!strcmp(line, "l")) { host_led_toggle(); printf("LED toggled\r\n"); return; }
    if (line[0] == 'c' && line[1] >= '0' && line[1] <= '7' && line[2] == ' ') {
        uint8_t node = (uint8_t)(line[1] - '0');
        if (can_host_send_command(node, &line[3])) printf("TX node%u -> '%s'\r\n", node, &line[3]);
        else printf("bad command (E/D/?/R/H = 1 byte; T/V/L/M need a value)\r\n");
        return;
    }
    printf("unknown command, type h\r\n");
}

int main(void)
{
    char line[64];
    uint32_t next_blink, next_service;

    if (!clock_init()) {
        for (;;) { }                       /* clock failed: stay here for the debugger */
    }
    console_init(HOST_CONSOLE_BAUD);
    host_board_init();
    host_led(1);
    print_banner();

    host_esp_uart_init(HOST_ESP_BAUD);
    printf("[UART2] PA2/PA3 initialised @ %lu (ESP32 protocol deferred)\r\n",
           (unsigned long)HOST_ESP_BAUD);

    imu_bringup();
    can_bringup();
    printf("\r\nBring-up checklist done. Compare each line with README; fix the pin for any failure.\r\n");

    next_blink = HAL_GetTick() + 500u;
    next_service = HAL_GetTick();
    for (;;) {
        uint32_t now = HAL_GetTick();
        if (console_read_line(line, sizeof(line))) handle_command(line);
        if (now >= next_service) { can_host_service(); next_service = now + 20u; }
        if (now >= next_blink) {
            host_led_toggle();
            next_blink = now + 500u;
        }
    }
}
