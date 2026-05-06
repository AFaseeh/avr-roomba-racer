#include "bluetooth.h"
#include "uart.h"

#include <util/delay.h>

static char turn_buffer[COMM_MAX_TURNS];
static uint8_t turn_count = 0;

static uint8_t comm_queue_uint8(uint8_t value)
{
    if (value >= 100U) {
        uart_send_char((char)('0' + (value / 100U)));
        value %= 100U;
        uart_send_char((char)('0' + (value / 10U)));
        uart_send_char((char)('0' + (value % 10U)));
        return 1U;
    }

    if (value >= 10U) {
        uart_send_char((char)('0' + (value / 10U)));
        uart_send_char((char)('0' + (value % 10U)));
        return 1U;
    }

    uart_send_char((char)('0' + value));
    return 1U;
}

void COMM_Init(void)
{
    turn_count = 0;
}

uint8_t COMM_LogTurn(char turn_direction)
{
    if ((turn_direction != 'L') && (turn_direction != 'R')) {
        return 0U;
    }

    if (turn_count < COMM_MAX_TURNS) {
        turn_buffer[turn_count] = turn_direction;
        turn_count++;
        return 1U;
    }

    return 0U;
}

uint8_t COMM_TransmitFinalData(void)
{
    uint8_t i;

    uart_send_string("Turns: ");
    comm_queue_uint8(turn_count);
    uart_send_string("\r\nSequence: ");

    for (i = 0; i < turn_count; i++) {
        if (i > 0U) {
            uart_send_char(',');
        }

        uart_send_char(turn_buffer[i]);
    }

    uart_send_string("\r\n");
    return 1U;
}

void COMM_ResetTurns(void)
{
    turn_count = 0;
}

uint8_t COMM_IsTransmitBusy(void)
{
    return 0U;
}

uint8_t COMM_GetTurnCount(void)
{
    return turn_count;
}

void COMM_Test(void)
{
    COMM_Init();

    while (1) {
        COMM_LogTurn('L');
        COMM_LogTurn('R');
        COMM_LogTurn('L');
        COMM_LogTurn('L');

        COMM_TransmitFinalData();
        COMM_ResetTurns();

        _delay_ms(3000);
    }
}
