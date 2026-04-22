#ifndef UART_H
#define UART_H

#include <stdint.h>

// Initialize the hardware UART (e.g., baud_rate = 9600)
void uart_init(uint32_t baud_rate);

// Send a single character
void uart_send_char(char data);

// Send a full string (like "Robot moving left!\r\n")
void uart_send_string(const char *str);

#endif /* UART_H */