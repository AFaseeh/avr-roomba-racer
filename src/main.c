#include <util/delay.h>
#include <stdio.h>
#include "motor.h"
#include "uart.h"
#include "encoder.h"

#define TICKS_FOR_90_DEG 10

void init_system() {
    motor_init();
    uart_init(9600);
    encoder_init();
}

int main()
{
    init_system();

    char print_buffer[64];
    while (1) {
        encoder_reset();
        uart_send_string("Starting 90-degree right turn\r\n");

        motor_set_speed(50, -50);

        uint32_t current_left = 0;
        uint32_t current_right = 0;

        while (1) {
            encoder_get_both_ticks(&current_left, &current_right);
            sprintf(print_buffer, "Left: %lu | Right: %lu\r\n\n", current_left, current_right);
            uart_send_string(print_buffer);

            if (current_left >= TICKS_FOR_90_DEG) {
                break; 
            }
        }

        motor_stop();

        sprintf(print_buffer, "Turn Complete! Left: %lu | Right: %lu\r\n\n", current_left, current_right);
        uart_send_string(print_buffer);

        _delay_ms(3000);
    }

    return 0;
}