#include "stm32g4xx_hal.h"
#include "can_host.h"
#include "host_config.h"
#include <stdlib.h>
#include <string.h>

#define RX_RING 64u

static FDCAN_HandleTypeDef can;
static CanFrame rx_ring[RX_RING];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_count, tx_dropped, bus_off_count;
static volatile int started;
static volatile int bus_off;
static uint32_t restart_at_ms;

static int config_filters(void)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID2 = 0x7ffu;
    filter.FilterIndex = 0u;
    filter.FilterID1 = 0x000u;      /* 全收：主机要看所有节点 */
    return HAL_FDCAN_ConfigFilter(&can, &filter) == HAL_OK;
}

static int can_start(void)
{
    if (HAL_FDCAN_Stop(&can) != HAL_OK) { /* 首次调用未启动，忽略 */ }
    if (!config_filters()) return 0;
    if (HAL_FDCAN_ConfigGlobalFilter(&can, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) return 0;
    if (HAL_FDCAN_ActivateNotification(&can,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
            FDCAN_IT_BUS_OFF, 0u) != HAL_OK) return 0;
    if (HAL_FDCAN_Start(&can) != HAL_OK) return 0;
    started = 1;
    bus_off = 0;
    return 1;
}

int can_host_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clocks = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
#if HOST_CAN_DRIVE_STB_ON_PB5
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16;   /* STB 拉低 = 正常模式 */
#endif
    clocks.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clocks.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&clocks) != HAL_OK) return 0;
    __HAL_RCC_FDCAN_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;         /* PB8 = FDCAN1_RX, PB9 = FDCAN1_TX */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &gpio);

    can.Instance = FDCAN1;
    can.Init.ClockDivider = FDCAN_CLOCK_DIV2;
    can.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    can.Init.Mode = FDCAN_MODE_NORMAL;
    can.Init.AutoRetransmission = DISABLE;
    can.Init.TransmitPause = ENABLE;
    can.Init.ProtocolException = DISABLE;
    /* PLLQ=85 MHz /2 /5 /(1+13+3) = 500 kbit/s，与驱动板相同 */
    can.Init.NominalPrescaler = 5;
    can.Init.NominalSyncJumpWidth = 3;
    can.Init.NominalTimeSeg1 = 13;
    can.Init.NominalTimeSeg2 = 3;
    can.Init.DataPrescaler = 1;
    can.Init.DataSyncJumpWidth = 1;
    can.Init.DataTimeSeg1 = 1;
    can.Init.DataTimeSeg2 = 1;
    can.Init.StdFiltersNbr = 1;
    can.Init.ExtFiltersNbr = 0;
    can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&can) != HAL_OK) return 0;
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    return can_start();
}

int can_host_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef header = {0};
    if (!started) return 0;
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = (uint32_t)len;          /* 本 HAL 的 DLC 用字节数 */
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&can, &header, (uint8_t *)data) != HAL_OK) {
        ++tx_dropped;
        return 0;
    }
    return 1;
}

int can_host_recv(CanFrame *frame)
{
    if (rx_tail == rx_head) return 0;
    *frame = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING);
    return 1;
}

uint32_t can_host_rx_count(void) { return rx_count; }uint32_t can_host_tx_dropped(void) { return tx_dropped; }
uint32_t can_host_bus_off_count(void) { return bus_off_count; }
uint32_t can_host_error_count(void) { return can.Instance->ECR; }
uint32_t can_host_psr(void) { return can.Instance->PSR; }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)hfdcan;
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];
        while (HAL_FDCAN_GetRxMessage(&can, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
            if (header.IdType != FDCAN_STANDARD_ID) continue;
            uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING);
            if (next == rx_tail) { ++tx_dropped; continue; }
            rx_ring[rx_head].id = header.Identifier;
            rx_ring[rx_head].dlc = (uint8_t)header.DataLength;
            memcpy(rx_ring[rx_head].data, data, sizeof(data));
            rx_head = next;
            ++rx_count;
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    (void)hfdcan;
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
        ++bus_off_count;
        started = 0;
        bus_off = 1;
        restart_at_ms = HAL_GetTick() + 500u;
    }
}

/* 总线掉线要能自己恢复：驱动板固件里已经踩过这个坑（M_CAN 进 bus-off 会锁死）。 */
void can_host_service(void)
{
    if (!started && bus_off && (HAL_GetTick() >= restart_at_ms)) {
        (void)can_start();
    }
}

/* ---- 中断入口 ----------------------------------------------------------
   本工程没有 stm32g4xx_it.c，中断入口必须自己提供。之前只在 can_host_init()
   里 HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn) 却没有定义 handler，结果第一个 CAN
   中断就跳进启动文件里的 weak 默认死循环（实测 PC=0x08000206、IPSR=0x25，
   即 IRQ21=FDCAN1_IT0），表现为"整板失联、串口无输出、uwTick 不再增加"。
   教训：EnableIRQ 之前必须确认同一工程里有同名 handler。 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&can);
}

/* 只用中断线 0；万一硬件把线 1 的标志置起来，也在这里清掉，
   避免再落进启动文件的 weak 死循环。 */
void FDCAN1_IT1_IRQHandler(void)
{
    can.Instance->IR = 0xFFFFFFFFu;
}

/* ---- 命令编码（与驱动板 can_commands.c 的线格式一致） ---- */
static const char *single_byte_ops = "ED?RH";

int can_host_send_command(uint8_t node, const char *text)
{
    uint8_t data[5];
    uint32_t id;
    char op;
    if (!text || !text[0]) return 0;
    op = text[0];
    id = (node == 0u) ? HOST_CAN_BROADCAST_ID : (HOST_CAN_BROADCAST_ID + node);
    if (strchr(single_byte_ops, op) != NULL && text[1] == 0) {
        data[0] = (uint8_t)op;
        return can_host_send(id, data, 1u);
    }
    if ((op == 'T' || op == 'V' || op == 'L' || op == 'M') && text[1] != 0) {
        float value = strtof(&text[1], NULL);
        union { float f; uint32_t u; } conv;
        conv.f = value;
        if (conv.f != conv.f) return 0;                       /* 拒绝 NaN */
        data[0] = (uint8_t)op;
        data[1] = (uint8_t)(conv.u & 0xFFu);
        data[2] = (uint8_t)((conv.u >> 8) & 0xFFu);
        data[3] = (uint8_t)((conv.u >> 16) & 0xFFu);
        data[4] = (uint8_t)((conv.u >> 24) & 0xFFu);
        return can_host_send(id, data, 5u);
    }
    return 0;
}

/* ---- 内部回环自测（判断"MCU 侧 FDCAN 是否正常"用）---------------------
 * 用 HAL 完整切换模式：DeInit -> Init(mode) -> 过滤器 -> Start。
 * 早先只在寄存器上点 TEST/LBCK，实测切不过去（PSR 仍报 LEC=5 bit0 错误、
 * 自己也收不到帧），所以改成 HAL 正规路径。
 * out[0]=模式切换是否成功 out[1]=发送是否入队 out[2]=是否收到
 * out[3..5]=收到的 id/dlc/首字节 out[6]=测试前 PSR out[7]=测试后 PSR
 * out[8]=测试后 ECR out[9]=回环时 CCCR out[10]=回环时 TEST
 * 返回 1 表示回环成功（说明 FDCAN 内核与引脚配置没问题）。 */
static int can_set_mode(uint32_t mode)
{
    started = 0;
    (void)HAL_FDCAN_DeInit(&can);
    can.Init.Mode = mode;
    if (HAL_FDCAN_Init(&can) != HAL_OK) return 0;
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    return can_start();
}

int can_host_loopback_test(uint32_t *out)
{
    const uint8_t payload[8] = {0xA5u, 0x5Au, 1u, 2u, 3u, 4u, 5u, 6u};
    CanFrame f;
    uint32_t psr_before;
    uint32_t guard;
    int mode_ok;
    int sent = 0;
    int got = 0;

    if (out == NULL) return 0;
    psr_before = can.Instance->PSR;

    mode_ok = can_set_mode(FDCAN_MODE_INTERNAL_LOOPBACK);
    out[9] = can.Instance->CCCR;
    out[10] = can.Instance->TEST;

    if (mode_ok) {
        sent = can_host_send(0x321u, payload, 8u);
        for (guard = 0u; guard < 200000u; guard++) {
            can_host_service();
            if (can_host_recv(&f)) {
                got = 1;
                break;
            }
        }
    }

    out[0] = (uint32_t)mode_ok;
    out[1] = (uint32_t)sent;
    out[2] = (uint32_t)got;
    out[3] = got ? f.id : 0u;
    out[4] = got ? (uint32_t)f.dlc : 0u;
    out[5] = got ? (uint32_t)f.data[0] : 0u;
    out[6] = psr_before;
    out[7] = can.Instance->PSR;
    out[8] = can.Instance->ECR;

    (void)can_set_mode(FDCAN_MODE_NORMAL);
    return got;
}

/* 外部回环：核心真的驱动总线（经收发器），但 RX 内部自环，不需要别的节点 ACK。
 * 用来区分"发送通路/收发器坏"与"接收通路（RXD）坏"：
 *   external OK  -> 收发器能驱动总线，问题在 RXD 回读（收发器 RX 脚/焊接）
 *   external 也失败 -> 收发器没电/损坏或 TXD 走线断
 * 结果写 out[11]。 */
int can_host_external_loopback_test(uint32_t *out)
{
    const uint8_t payload[8] = {0x5Au, 0xA5u, 7u, 8u, 9u, 10u, 11u, 12u};
    CanFrame f;
    uint32_t guard;
    int mode_ok;
    int sent = 0;
    int got = 0;

    if (out == NULL) return 0;
    mode_ok = can_set_mode(FDCAN_MODE_EXTERNAL_LOOPBACK);
    if (mode_ok) {
        sent = can_host_send(0x322u, payload, 8u);
        for (guard = 0u; guard < 200000u; guard++) {
            can_host_service();
            if (can_host_recv(&f)) {
                got = 1;
                break;
            }
        }
    }
    out[11] = ((uint32_t)mode_ok << 2) | ((uint32_t)sent << 1) | (uint32_t)got;
    (void)can_set_mode(FDCAN_MODE_NORMAL);
    return got;
}
