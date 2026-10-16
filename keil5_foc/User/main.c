#include "stm32f10x.h"
#include "board.h"
#include "as5600.h"
#include "motor.h"
#include <stdio.h>

static Motor motor;

int main(void)
{
    char command[BOARD_COMMAND_LENGTH];
    board_init();
    printf("\r\nKeil5 / SPL / F103C8 + DRV8316C + AS5600\r\n");
    printf("SysClk=%lu  PWM=TIM1 PA8/9/10  EN=PB0\r\n", (unsigned long)SystemCoreClock);
    printf("UART=USART2 PA2/3 115200  I2C=PB6/7\r\n");
    as5600_init();
    motor_init(&motor);
    printf("commands: E D T<n> V<n> L<n> R ? H\r\n");
    while (1) {
        if (board_take_stop()) {
            motor_disable(&motor);
            board_discard_commands();
            printf("EN OFF (stop)\r\n");
        }
        motor_step(&motor);
        if (board_get_command(command)) motor_command(&motor, command);
        board_led((board_millis()/400u) & 1u);
    }
}
