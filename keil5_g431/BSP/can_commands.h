#ifndef CAN_COMMANDS_H
#define CAN_COMMANDS_H
#include <stdint.h>
#include <stddef.h>
#include "board.h"

#define COMMAND_DEPTH 16u
typedef struct { uint32_t bits; uint8_t opcode; } CanCommand;
typedef struct {
    CanCommand queue[COMMAND_DEPTH];
    uint8_t head, tail;
} CanCommands;

/* One atomic classic CAN frame: E/D/?/R/H (1 byte), T/V/L + float32 LE (5).
   Callers serialize access between ISR and foreground. */
unsigned int can_commands_receive(CanCommands *commands, const uint8_t *data, size_t count);
int can_commands_take(CanCommands *commands, CanCommand *command);
void can_command_format(const CanCommand *command, char *dst);
void can_commands_discard(CanCommands *commands);
#endif
