#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "helpers.h"

#define UART_TX_BUFFER_SIZE 192U

#if defined(USART0_UDRE_vect)
#define UART_UDRE_VECTOR USART0_UDRE_vect
#elif defined(USART_UDRE_vect)
#define UART_UDRE_VECTOR USART_UDRE_vect
#else
#error "USART data-register-empty interrupt vector is not available."
#endif

/*
 * UART Module - Interrupt-Driven Ring Buffer Implementation
 * 
 * Exclusive USART0 Ownership:
 * This module owns USART0 exclusively. The ring buffer (tx_buffer, tx_head,
 * tx_tail, tx_count) and the UDRE ISR live here. UDR0 is only written
 * inside the ISR. This prevents race conditions where both blocking polling 
 * code and an ISR try to claim the same hardware simultaneously.
 */

static volatile char tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;
static volatile uint8_t tx_count = 0;

static uint8_t uart_next_index(uint8_t index)
{
    index++;
    if (index >= UART_TX_BUFFER_SIZE) {
        index = 0;
    }

    return index;
}

static uint16_t uart_string_length(const char *str)
{
    uint16_t length = 0;

    while (str[length] != '\0') {
        length++;
    }

    return length;
}

void uart_init(uint32_t baud_rate)
{
    uint16_t ubrr_value = (uint16_t)((F_CPU / (16UL * baud_rate)) - 1UL);
    uint8_t sreg_backup = SREG;

    cli();

    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;

    UBRR0H = (uint8_t)(ubrr_value >> 8);
    UBRR0L = (uint8_t)ubrr_value;

    UCSR0B = (1U << RXEN0) | (1U << TXEN0);
    UCSR0C = (1U << UCSZ01) | (1U << UCSZ00);

    SREG = sreg_backup;
}

uint16_t uart_tx_free_space(void)
{
    uint16_t free_space;
    uint8_t sreg_backup = SREG;

    cli();
    free_space = (uint16_t)(UART_TX_BUFFER_SIZE - tx_count);
    SREG = sreg_backup;

    return free_space;
}

uint8_t uart_queue_char(char data)
{
    uint8_t queued = 0;
    uint8_t sreg_backup = SREG;

    cli();

    if (tx_count < UART_TX_BUFFER_SIZE) {
        tx_buffer[tx_head] = data;
        tx_head = uart_next_index(tx_head);
        tx_count++;
        SET_BIT(UCSR0B, UDRIE0);
        queued = 1U;
    }

    SREG = sreg_backup;

    return queued;
}

/*
 * Queue a full string.
 * The free-space check is done upfront to ensure that if there isn't room
 * for the whole string, the entire string is rejected. This prevents ending
 * up with partially transmitted, unparseable messages on the receiver side.
 */
uint8_t uart_queue_string(const char *str)
{
    if (uart_tx_free_space() < uart_string_length(str)) {
        return 0U;
    }

    while (*str != '\0') {
        if (uart_queue_char(*str) == 0U) {
            return 0U;
        }

        str++;
    }

    return 1U;
}

uint8_t uart_tx_is_busy(void)
{
    return (uart_tx_free_space() != UART_TX_BUFFER_SIZE) ? 1U : 0U;
}

/*
 * Non-blocking wrappers for compatibility with older code.
 * All writes are now routed through the interrupt-driven ring buffer.
 */
void uart_send_char(char data)
{
    (void)uart_queue_char(data);
}

void uart_send_string(const char *str)
{
    (void)uart_queue_string(str);
}

/*
 * Single producer of UDR0 values:
 * Because UDR0 is only written inside this ISR, it can never pre-empt itself
 * and silently drop a byte, safely draining the buffer in the background.
 */
ISR(UART_UDRE_VECTOR)
{
    if (tx_count > 0U) {
        UDR0 = tx_buffer[tx_tail];
        tx_tail = uart_next_index(tx_tail);
        tx_count--;
    } else {
        CLEAR_BIT(UCSR0B, UDRIE0);
    }
}
