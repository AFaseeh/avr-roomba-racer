#include "motor.h"
#include "uart.h"
#include <util/delay.h>

void init_system() {
    motor_init();
    uart_init(9600);
}

int main()
{
    init_system();

    int i = 1;
    while (1) {
        i *= -1;
        // Example: Set left motor to 50% forward and right motor to 50% reverse
        motor_set_speed(i*50, -50*i);
        uart_send_string("Motors set to 50% forward and 50% reverse\r\n\0");
        _delay_ms(2000);

        // Stop the motors
        motor_stop();
        uart_send_string("Motors stopped\r\n");
        _delay_ms(1000);

        // Coast the motors
        motor_coast();
        uart_send_string("Motors coasting\r\n\0");
        _delay_ms(1000);
    }

    return 0;
}