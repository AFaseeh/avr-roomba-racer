#include <util/delay.h>
#include <stdio.h>
#include "motor.h"
#include "uart.h"
#include "encoder.h"
#include "ultrasonic.h"
#include "bluetooth.h"
#include "timer2.h"
#include "fsm/fsm.h"

#define TICKS_FOR_90_DEG 10
#define PING_INTERVAL_MS 15

void init_system() {
    motor_init();
    uart_init(9600);
    encoder_init();
    ultrasonic_init();
    COMM_Init();
    timer2_init_millis();
    fsm_init();
}


void main_loop(char* print_buffer)
{
    for (int i = 0; i < 5; i++) {
        ultrasonic_full_sweep();
    }
    uint32_t last_ping_time = 0;
    while(1)
    {
        uint32_t current_time = get_millis();

        // Ultrasonic RR sensor polling
        if (current_time - last_ping_time >= PING_INTERVAL_MS) {
            ultrasonic_next();
            last_ping_time = current_time;
        }

        uint16_t front_dist = ultrasonic_get_distance(US_FRONT);
        uint16_t left_dist  = ultrasonic_get_distance(US_LEFT);
        uint16_t right_dist = ultrasonic_get_distance(US_RIGHT);
        
        fsm_update(left_dist, right_dist, front_dist, print_buffer);

        //sprintf(print_buffer, "time: %lu ms | F: %3u mm | State: %d\r\n", current_time, front_dist, current_state);
        //sprintf(print_buffer, "time: %lu | F: %3u mm | L: %3u mm | R: %3u mm\r\n", current_time, front_dist, left_dist, right_dist);
        uart_send_string(print_buffer);
    }
}
int main()
{
    init_system();

    char print_buffer[128];
    main_loop(print_buffer);
    //testing();

    return 0;
}


void triple_ultrasonic_test(char* print_buffer) {
    for (int i = 0; i < 5; i++) {
        ultrasonic_full_sweep();
    }
    uint32_t last_ping_time = 0;

    while (1) {
        uint32_t current_time = get_millis();
        uint16_t front_distance = ultrasonic_get_distance(US_FRONT);
        uint16_t left_distance = ultrasonic_get_distance(US_LEFT);
        uint16_t right_distance = ultrasonic_get_distance(US_RIGHT);
        if (current_time - last_ping_time >= PING_INTERVAL_MS) {
            sprintf(print_buffer, "%3u %lu ms\r\n", front_distance,current_time - last_ping_time);
            uart_send_string(print_buffer);
            ultrasonic_next();
            last_ping_time = current_time;
        }


        //sprintf(print_buffer, "time: %lu | F: %3u mm | L: %3u mm | R: %3u mm | Time: %lu ms\r\n", get_millis(), front_distance, left_distance, right_distance, get_time());
        // uart_send_string(print_buffer);
    }
}

void ultrasonic_test(char* print_buffer, UltrasonicID_t id) {
    while (1) {
        ultrasonic_trigger(id);
        uint16_t distance = ultrasonic_get_distance(id);
        sprintf(print_buffer, "Distance: %u mm\r\n", distance);
        uart_send_string(print_buffer);
        _delay_ms(200);
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

void motor_test() {
    while(1)
        motor_set_speed(100, 100);
}