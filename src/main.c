#include <util/delay.h>
#include <stdio.h>
#include "motor.h"
#include "uart.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "bluetooth.h"

#define TICKS_FOR_90_DEG 10

void init_system() {
    motor_init();
    uart_init(9600);
    encoder_init();
    ultrasonic_init();
}

int main()
{
    init_system();

    char print_buffer[128];
    triple_ultrasonic_test(print_buffer);

    return 0;
}

void triple_ultrasonic_test(char* print_buffer) {
    while (1) {
        ultrasonic_trigger(US_FRONT);
        _delay_ms(40);

        ultrasonic_trigger(US_LEFT);
        _delay_ms(40);

        ultrasonic_trigger(US_RIGHT);
        _delay_ms(40);

        uint16_t front_distance = ultrasonic_get_distance(US_FRONT);
        uint16_t left_distance = ultrasonic_get_distance(US_LEFT);
        uint16_t right_distance = ultrasonic_get_distance(US_RIGHT);

        sprintf(print_buffer, "F: %3u cm | L: %3u cm | R: %3u cm\r\n", front_distance, left_distance, right_distance);
        uart_send_string(print_buffer);

        _delay_ms(100);
    }
}

void ultrasonic_test(char* print_buffer) {
    while (1) {
        ultrasonic_trigger(US_FRONT);
        uint16_t distance = ultrasonic_get_distance(US_FRONT);
        sprintf(print_buffer, "Distance: %u cm\r\n", distance);
        uart_send_string(print_buffer);
        _delay_ms(50);
    }
}

void encoder_test(char* print_buffer) {
    while (1) {
        encoder_reset();
        uart_send_string("Starting 90-degree right turn\r\n");

        motor_set_speed(50, 50);

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
}

