#include "uart.h"
#include <avr/io.h>
#include "helpers.h"

void uart_init(uint32_t baud_rate) {
    // ubbr
    uint16_t ubrr_value = (F_CPU / (16UL * baud_rate)) - 1;

    // baud rate
    UBRR0H = (uint8_t)(ubrr_value >> 8);
    UBRR0L = (uint8_t)ubrr_value;

    // enable receiver and transmitter
    SET_BIT(UCSR0B, RXEN0);
    SET_BIT(UCSR0B, TXEN0);

    // frame format: 8data, 1stop bit, no parity
    SET_BIT(UCSR0C, UCSZ01);
    SET_BIT(UCSR0C, UCSZ00);
}

void uart_send_char(char data) {
    while (READ_BIT(UCSR0A, UDRE0) == 0) {
    }
    
    UDR0 = data;
}

void uart_send_string(const char *str) {
    while (*str != '\0') {
        uart_send_char(*str);
        str++;
    }
}