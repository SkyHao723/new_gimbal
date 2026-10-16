#include "stm32f10x.h"
#include "board.h"
#include "motor_config.h"
#include <stdio.h>

#define RX_DEPTH 16u
#define TX_SIZE 1024u
static volatile uint32_t time_high, time_ms;
static volatile uint8_t stop_requested;
static volatile uint8_t rx_head, rx_tail;
static char rx_queue[RX_DEPTH][BOARD_COMMAND_LENGTH];
static char rx_line[BOARD_COMMAND_LENGTH];
static uint8_t rx_length, rx_discard;
static volatile uint16_t tx_head, tx_tail;
static uint8_t tx_buffer[TX_SIZE];
static uint16_t pwm_period;

void board_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef channel;
    USART_InitTypeDef uart;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART2, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    /* Set EN low before switching the pin to output. Keep SWD available. */
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    RCC_GetClocksFreq(&clocks);
    TIM_TimeBaseStructInit(&timer);
    timer_clock = clocks.PCLK2_Frequency;
    if (clocks.PCLK2_Frequency != clocks.HCLK_Frequency) timer_clock *= 2u;
    pwm_period = (uint16_t)(timer_clock / (2u*MOTOR_PWM_HZ));
    timer.TIM_Period = pwm_period-1u;
    timer.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInit(TIM1, &timer);
    TIM_OCStructInit(&channel);
    channel.TIM_OCMode = TIM_OCMode_PWM1;
    channel.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM1, &channel);
    TIM_OC2Init(TIM1, &channel);
    TIM_OC3Init(TIM1, &channel);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    timer_clock = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) timer_clock *= 2u;
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(timer_clock/1000000u-1u);
    timer.TIM_Period = 0xffffu;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_SetPriority(TIM2_IRQn, 0u);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM_Cmd(TIM2, ENABLE);
    SysTick_Config(SystemCoreClock/1000u);

    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200u;
    USART_Init(USART2, &uart);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(USART2_IRQn, 1u);
    NVIC_EnableIRQ(USART2_IRQn);
    USART_Cmd(USART2, ENABLE);
}

uint32_t board_micros(void)
{
    uint32_t irq = __get_PRIMASK();
    uint32_t high, low;
    __disable_irq();
    high = time_high;
    low = TIM2->CNT;
    if (TIM2->SR & TIM_SR_UIF) { high += 65536u; low = TIM2->CNT; }
    __set_PRIMASK(irq);
    return high+low;
}

uint32_t board_millis(void) { return time_ms; }
void board_led(int on) { GPIO_WriteBit(GPIOC, GPIO_Pin_13, on ? Bit_RESET : Bit_SET); }

int board_driver_set(int enabled)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (!enabled || stop_requested) {
        GPIOB->BRR = GPIO_Pin_0;
        TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0u;
        enabled = 0;
    } else GPIOB->BSRR = GPIO_Pin_0;
    __set_PRIMASK(irq);
    return enabled;
}

void board_pwm(float a, float b, float c)
{
    TIM1->CCR1 = (uint16_t)(a*pwm_period);
    TIM1->CCR2 = (uint16_t)(b*pwm_period);
    TIM1->CCR3 = (uint16_t)(c*pwm_period);
}

int board_take_stop(void)
{
    uint32_t irq = __get_PRIMASK();
    int stop;
    __disable_irq();
    stop = stop_requested;
    stop_requested = 0u;
    __set_PRIMASK(irq);
    return stop;
}

void board_discard_commands(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    rx_tail = rx_head;
    __set_PRIMASK(irq);
}

int board_get_command(char *dst)
{
    unsigned int i;
    uint8_t tail = rx_tail;
    if (tail == rx_head) return 0;
    for (i=0u; i<BOARD_COMMAND_LENGTH; ++i) dst[i] = rx_queue[tail][i];
    __DMB();
    rx_tail = (uint8_t)((tail+1u) % RX_DEPTH);
    return 1;
}

/* Interrupt-driven TX keeps normal status replies out of the FOC timing path. */
int fputc(int ch, FILE *stream)
{
    uint16_t next = (uint16_t)((tx_head+1u) % TX_SIZE);
    (void)stream;
    while (next == tx_tail) {}
    tx_buffer[tx_head] = (uint8_t)ch;
    __DMB();
    tx_head = next;
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    return ch;
}

void USART2_IRQHandler(void)
{
    uint16_t status = USART2->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        uint8_t c = (uint8_t)USART2->DR;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) rx_discard = 1u;
        if (c == 'D' && rx_length == 0u) {
            /* Stop on receipt, even when the normal command queue is full. */
            GPIOB->BRR = GPIO_Pin_0;
            stop_requested = 1u;
            rx_discard = 1u;
        }
        if (c == '\r' || c == '\n') {
            if (rx_length && !rx_discard) {
                uint8_t next = (uint8_t)((rx_head+1u) % RX_DEPTH);
                unsigned int i;
                if (next != rx_tail) {
                    for (i=0u; i<rx_length; ++i) rx_queue[rx_head][i] = rx_line[i];
                    rx_queue[rx_head][rx_length] = '\0';
                    __DMB();
                    rx_head = next;
                }
            }
            rx_length = rx_discard = 0u;
        } else if (!rx_discard) {
            if (rx_length < BOARD_COMMAND_LENGTH-1u) rx_line[rx_length++] = (char)c;
            else rx_discard = 1u;
        }
    }
    if ((status & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE)) {
        if (tx_tail != tx_head) {
            USART2->DR = tx_buffer[tx_tail];
            tx_tail = (uint16_t)((tx_tail+1u) % TX_SIZE);
        } else USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = (uint16_t)~TIM_SR_UIF;
        time_high += 65536u;
    }
}

void SysTick_Handler(void) { ++time_ms; }
static void fault_stop(void)
{
    GPIOB->BRR = GPIO_Pin_0;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0u;
    while (1) {}
}
void NMI_Handler(void) { fault_stop(); }
void HardFault_Handler(void) { fault_stop(); }
void MemManage_Handler(void) { fault_stop(); }
void BusFault_Handler(void) { fault_stop(); }
void UsageFault_Handler(void) { fault_stop(); }
