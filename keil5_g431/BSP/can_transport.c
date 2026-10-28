#include "board_config.h"
#include "board.h"
#include "can_transport.h"
#include "can_commands.h"
#include <stdio.h>

#define TX_SIZE 1024u
/* This bundled HAL uses an unshifted DLC code, unlike older Cube releases. */
typedef char ClassicDlcMustUseByteCount[(FDCAN_DLC_BYTES_8 == 8u) ? 1 : -1];
static FDCAN_HandleTypeDef can;
static CanCommands commands;
static uint8_t tx_buffer[TX_SIZE];
static uint16_t tx_head, tx_tail;
static unsigned long dropped_bytes;
static volatile int started;
/* A lone node (transceiver unpowered, bus unwired) fails every ACK and the
   controller latches bus-off with CCCR.INIT set. Retry the bus periodically so
   the link heals by itself once the wiring/power is fixed. */
static volatile int bus_off;
static uint32_t restart_at_ms;

/* Two exact-match filters: this node's command ID plus the broadcast ID.
   Both go to RX FIFO0; can_commands() decides what each one means. */
static int config_filters(void)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID2 = 0x7ffu;
    filter.FilterIndex = 0u;
    filter.FilterID1 = BOARD_CAN_COMMAND_ID;
    if (HAL_FDCAN_ConfigFilter(&can, &filter) != HAL_OK) return 0;
    filter.FilterIndex = 1u;
    filter.FilterID1 = BOARD_CAN_BROADCAST_ID;
    if (HAL_FDCAN_ConfigFilter(&can, &filter) != HAL_OK) return 0;
    return 1;
}

static int can_reopen(void)
{
    (void)HAL_FDCAN_Stop(&can);
    if (!config_filters()) return 0;
    if (HAL_FDCAN_ConfigGlobalFilter(&can, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) return 0;
    if (HAL_FDCAN_ActivateNotification(&can,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                       FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                       FDCAN_IT_BUS_OFF, 0u) != HAL_OK) return 0;
    if (HAL_FDCAN_Start(&can) != HAL_OK) return 0;
    GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16;
    return 1;
}

int can_transport_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clocks = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->BSRR = GPIO_PIN_5; /* Standby until FDCAN is configured. */
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOB, &gpio);
    clocks.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clocks.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&clocks) != HAL_OK) return 0;
    __HAL_RCC_FDCAN_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
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
    /* PLLQ=85MHz /2 /5 /(1+13+3) = 500kbit/s, sample point=82.35%. */
    can.Init.NominalPrescaler = 5;
    can.Init.NominalSyncJumpWidth = 3;
    can.Init.NominalTimeSeg1 = 13;
    can.Init.NominalTimeSeg2 = 3;
    can.Init.DataPrescaler = 1;
    can.Init.DataSyncJumpWidth = 1;
    can.Init.DataTimeSeg1 = 1;
    can.Init.DataTimeSeg2 = 1;
    can.Init.StdFiltersNbr = 2;
    can.Init.ExtFiltersNbr = 0;
    can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&can) != HAL_OK) return 0;
    if (!config_filters()) return 0;
    if (HAL_FDCAN_ConfigGlobalFilter(&can, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) return 0;
    if (HAL_FDCAN_ActivateNotification(&can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_BUS_OFF, 0u) != HAL_OK) return 0;
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    if (HAL_FDCAN_Start(&can) != HAL_OK) return 0;
    GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16;
    started = 1;
    return 1;
}

/* Never block FOC when the bus is disconnected, unacknowledged or congested.
   Failed transmissions are not replayed; the host requests fresh status. */
int fputc(int ch, FILE *stream)
{
    uint16_t next = (uint16_t)((tx_head + 1u) % TX_SIZE);
    (void)stream;
    if (next == tx_tail) ++dropped_bytes;
    else {
        tx_buffer[tx_head] = (uint8_t)ch;
        tx_head = next;
    }
    return ch;
}

void can_transport_service(void)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};
    unsigned int frames;
    if (!started) {
        if (bus_off && (int32_t)(board_millis() - restart_at_ms) >= 0) {
            if (can_reopen()) {
                started = 1;
                bus_off = 0;
                tx_tail = tx_head; /* stale bytes cannot be replayed */
            } else {
                restart_at_ms = board_millis() + 500u;
            }
        }
        return;
    }
    header.Identifier = BOARD_CAN_REPLY_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    for (frames = 0u; frames < 3u && tx_tail != tx_head; ++frames) {
        uint16_t tail = tx_tail;
        unsigned int length = 0u;
        if (!HAL_FDCAN_GetTxFifoFreeLevel(&can)) break;
        while (length < 8u && tail != tx_head) {
            data[length++] = tx_buffer[tail];
            tail = (uint16_t)((tail + 1u) % TX_SIZE);
        }
        header.DataLength = length;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&can, &header, data) != HAL_OK) break;
        tx_tail = tail;
    }
}

unsigned long can_transport_dropped_bytes(void) { return dropped_bytes; }

int board_get_command(char *dst)
{
    uint32_t irq = __get_PRIMASK();
    int result;
    CanCommand command;
    __disable_irq();
    result = can_commands_take(&commands, &command);
    __set_PRIMASK(irq);
    if (result) can_command_format(&command, dst);
    return result;
}

void board_discard_commands(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    can_commands_discard(&commands);
    __set_PRIMASK(irq);
}

void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&can); }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t flags)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64]; /* Also safe if an erroneous CAN FD frame reaches FIFO. */
    unsigned int frames;
    if (flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
        board_request_stop(BOARD_STOP_CAN);
        can_commands_discard(&commands);
    }
    for (frames = 0u; frames < 3u && HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0); ++frames) {
        unsigned int stop, length;
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            board_request_stop(BOARD_STOP_CAN);
            break;
        }
        length = header.DataLength;
        /* Accept this node's own command ID and the shared broadcast ID; the
         * filters already limit us to exactly those two. */
        if ((header.Identifier != BOARD_CAN_COMMAND_ID &&
             header.Identifier != BOARD_CAN_BROADCAST_ID) ||
            header.IdType != FDCAN_STANDARD_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME) continue;
        if (header.FDFormat != FDCAN_CLASSIC_CAN || length > 8u) {
            board_request_stop(BOARD_STOP_CAN);
            can_commands_discard(&commands);
            continue;
        }
        stop = can_commands_receive(&commands, data, length);
        if (stop) {
            board_request_stop(stop);
            can_commands_discard(&commands);
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *handle, uint32_t flags)
{
    (void)handle;
    if (flags & FDCAN_IT_BUS_OFF) {
        board_request_stop(BOARD_STOP_CAN);
        can_commands_discard(&commands);
        /* Hardware latched INIT; can_transport_service() retries the bus. */
        started = 0;
        bus_off = 1;
        restart_at_ms = board_millis() + 500u;
        GPIOB->BSRR = GPIO_PIN_5;
    }
}
