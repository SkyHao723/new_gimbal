#include "can_commands.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int pop(CanCommands *commands, char *dst)
{
    CanCommand command;
    if (!can_commands_take(commands, &command)) return 0;
    can_command_format(&command, dst);
    return 1;
}

int main(void)
{
    static const float values[] = {-1000000.0f, -100.0f, -0.12345678f, -0.0f,
                                  0.000000001f, 0.2f, 3.14159265f, 20.0f, 1000000.0f};
    static const char ops[] = {'T', 'V', 'L', 'M'};
    CanCommands queue = {0};
    char result[BOARD_COMMAND_LENGTH];
    unsigned int i, j;
    for (i = 0u; i < sizeof(values) / sizeof(values[0]); ++i) {
        for (j = 0u; j < sizeof(ops); ++j) {
            uint32_t bits;
            uint8_t data[5];
            memcpy(&bits, &values[i], 4u);
            data[0] = (uint8_t)ops[j];
            data[1] = (uint8_t)bits; data[2] = (uint8_t)(bits >> 8);
            data[3] = (uint8_t)(bits >> 16); data[4] = (uint8_t)(bits >> 24);
            assert(!can_commands_receive(&queue, data, 5u));
            assert(pop(&queue, result));
            assert(result[0] == ops[j] && (float)strtod(result + 1, NULL) == values[i]);
        }
    }
    for (i = 0u; i < 1000u; ++i) {
        assert(!can_commands_receive(&queue, (const uint8_t *)"E", 1u));
        assert(pop(&queue, result) && !strcmp(result, "E"));
    }
    for (i = 0u; i < COMMAND_DEPTH - 1u; ++i)
        assert(!can_commands_receive(&queue, (const uint8_t *)"E", 1u));
    assert(can_commands_receive(&queue, (const uint8_t *)"?", 1u) == BOARD_STOP_CAN);
    assert(can_commands_receive(&queue, (const uint8_t *)"D", 1u) == BOARD_STOP_COMMAND);
    assert(!pop(&queue, result));
    assert(can_commands_receive(&queue, (const uint8_t *)"T123", 4u) == BOARD_STOP_CAN);
    assert(can_commands_receive(&queue, (const uint8_t *)"E\n", 2u) == BOARD_STOP_CAN);
    {
        const uint8_t nan[] = {'T', 0, 0, 0xc0, 0x7f};
        const uint8_t inf[] = {'V', 0, 0, 0x80, 0xff};
        assert(can_commands_receive(&queue, nan, 5u) == BOARD_STOP_CAN);
        assert(can_commands_receive(&queue, inf, 5u) == BOARD_STOP_CAN);
    }
    assert(can_commands_receive(&queue, (const uint8_t *)"", 0u) == BOARD_STOP_CAN);
    assert(!can_commands_receive(&queue, (const uint8_t *)"?", 1u));
    assert(pop(&queue, result) && !strcmp(result, "?"));
    puts("CAN commands: atomic float32 round-trip, queue wrap/full, priority D, malformed/NaN/Inf PASS");
    return 0;
}
