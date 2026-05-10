#include "bluetooth.h"
#include "uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

static char turn_buffer[COMM_MAX_TURNS];
static volatile uint8_t turn_count = 0;

static uint8_t comm_decimal_length(uint8_t value)
{
    if (value >= 100U) {
        return 3U;
    }

    if (value >= 10U) {
        return 2U;
    }

    return 1U;
}

static uint16_t comm_sequence_length(uint8_t count)
{
    if (count == 0U) {
        return 0U;
    }

    return (uint16_t)((uint16_t)count * 2U - 1U);
}

/*
 * Calculates the exact string length of the final report.
 * Returns uint16_t instead of uint8_t because a max payload
 * easily exceeds 255 bytes. Avoiding 8-bit overflow prevents false positives 
 * in the free space check, guaranteeing we never send partial truncated messages.
 */
static uint16_t comm_final_message_length(uint8_t count)
{
    return (uint16_t)(7U + comm_decimal_length(count) + 2U + 10U +
                      comm_sequence_length(count) + 2U);
}

static uint8_t comm_queue_uint8(uint8_t value)
{
    if (value >= 100U) {
        if (uart_queue_char((char)('0' + (value / 100U))) == 0U) {
            return 0U;
        }

        value %= 100U;
        if (uart_queue_char((char)('0' + (value / 10U))) == 0U) {
            return 0U;
        }

        return uart_queue_char((char)('0' + (value % 10U)));
    }

    if (value >= 10U) {
        if (uart_queue_char((char)('0' + (value / 10U))) == 0U) {
            return 0U;
        }

        return uart_queue_char((char)('0' + (value % 10U)));
    }

    return uart_queue_char((char)('0' + value));
}

/*
 * Initialize the Bluetooth communications.
 * All USART configuration and ISR logic has been stripped from this file and 
 * delegated to uart.c. This makes bluetooth.c a pure protocol layer, removing 
 * the hardware conflicts where both files competed for USART0 registers.
 */
void COMM_Init(void)
{
    uint8_t sreg_backup = SREG;

    uart_init(COMM_BAUD_RATE);

    cli();
    turn_count = 0;
    SREG = sreg_backup;
}

uint8_t COMM_LogTurn(char turn_direction)
{
    uint8_t logged = 0U;
    uint8_t sreg_backup = SREG;

    if ((turn_direction != 'L') && (turn_direction != 'R')) {
        return 0U;
    }

    cli();

    if (turn_count < COMM_MAX_TURNS) {
        turn_buffer[turn_count] = turn_direction;
        turn_count++;
        logged = 1U;
    }

    SREG = sreg_backup;

    return logged;
}

/*
 * Transmits the final sequence of turns when the run finishes.
 * 
 * We create a local snapshot array (turns_snapshot) inside an atomic block
 * (ISRs disabled) before serializing it. This ensures that if COMM_LogTurn 
 * gets triggered by an interrupt mid-transmission, it won't corrupt the buffer
 * while we are actively iterating through it.
 */
uint8_t COMM_TransmitFinalData(void)
{
    char turns_snapshot[COMM_MAX_TURNS];
    uint8_t count;
    uint8_t i;
    uint8_t sreg_backup = SREG;

    cli();
    count = turn_count;
    for (i = 0U; i < count; i++) {
        turns_snapshot[i] = turn_buffer[i];
    }
    SREG = sreg_backup;

    if (uart_tx_free_space() < comm_final_message_length(count)) {
        return 0U;
    }

    if (uart_queue_string("Turns: ") == 0U) {
        return 0U;
    }

    if (comm_queue_uint8(count) == 0U) {
        return 0U;
    }

    if (uart_queue_string("\r\nSequence: ") == 0U) {
        return 0U;
    }

    for (i = 0U; i < count; i++) {
        if (i > 0U) {
            if (uart_queue_char(',') == 0U) {
                return 0U;
            }
        }

        if (uart_queue_char(turns_snapshot[i]) == 0U) {
            return 0U;
        }
    }

    return uart_queue_string("\r\n");
}

void COMM_ResetTurns(void)
{
    uint8_t sreg_backup = SREG;

    cli();
    turn_count = 0;
    SREG = sreg_backup;
}

uint8_t COMM_IsTransmitBusy(void)
{
    return uart_tx_is_busy();
}

uint8_t COMM_GetTurnCount(void)
{
    uint8_t count;
    uint8_t sreg_backup = SREG;

    cli();
    count = turn_count;
    SREG = sreg_backup;

    return count;
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
