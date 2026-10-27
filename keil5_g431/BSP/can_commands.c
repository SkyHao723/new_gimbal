#include "can_commands.h"
#include <stdio.h>
#include <string.h>

typedef char FloatMustBe32Bits[(sizeof(float) == 4u) ? 1 : -1];

unsigned int can_commands_receive(CanCommands *s, const uint8_t *data, size_t count)
{
    CanCommand command = {0};
    uint8_t next;
    if (!count) return BOARD_STOP_CAN;
    if (data[0] == 'D') {
        can_commands_discard(s);
        return BOARD_STOP_COMMAND;
    }
    command.opcode = data[0];
    switch (command.opcode) {
    case 'E': case '?': case 'R': case 'H':
        if (count != 1u) return BOARD_STOP_CAN;
        break;
    case 'T': case 'V': case 'L':
        if (count != 5u) return BOARD_STOP_CAN;
        command.bits = (uint32_t)data[1] | (uint32_t)data[2] << 8 |
                       (uint32_t)data[3] << 16 | (uint32_t)data[4] << 24;
        /* Reject NaN/Inf in the ISR without using the FPU. */
        if ((command.bits & 0x7f800000u) == 0x7f800000u) return BOARD_STOP_CAN;
        break;
    default: return BOARD_STOP_CAN;
    }
    next = (uint8_t)((s->head + 1u) % COMMAND_DEPTH);
    if (next == s->tail) return BOARD_STOP_CAN;
    s->queue[s->head] = command;
    s->head = next;
    return 0u;
}

int can_commands_take(CanCommands *s, CanCommand *command)
{
    if (s->tail == s->head) return 0;
    *command = s->queue[s->tail];
    s->tail = (uint8_t)((s->tail + 1u) % COMMAND_DEPTH);
    return 1;
}

void can_command_format(const CanCommand *command, char *dst)
{
    float value;
    if (command->opcode == 'T' || command->opcode == 'V' || command->opcode == 'L') {
        memcpy(&value, &command->bits, sizeof(value));
        /* Nine significant digits round-trip a float32 into the shared parser. */
        snprintf(dst, BOARD_COMMAND_LENGTH, "%c%.9g", (int)command->opcode, (double)value);
    } else {
        dst[0] = (char)command->opcode;
        dst[1] = '\0';
    }
}

void can_commands_discard(CanCommands *s) { s->tail = s->head; }
