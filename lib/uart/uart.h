#ifndef UART_H
#define UART_H

#include <stdint.h>

// Initialize the hardware UART (e.g., baud_rate = 9600)
void uart_init(uint32_t baud_rate);

// Queue a single character for interrupt-driven transmission.
// Returns 1 if queued, 0 if the TX buffer is full.
uint8_t uart_queue_char(char data);

// Queue a full string. Returns 1 if the whole string was queued.
uint8_t uart_queue_string(const char *str);

// Returns the current free space in the TX queue.
uint16_t uart_tx_free_space(void);

// Returns 1 while queued UART bytes are still pending.
uint8_t uart_tx_is_busy(void);

// Compatibility wrappers for debug output. These do not block.
void uart_send_char(char data);

void uart_send_string(const char *str);

#endif /* UART_H */
