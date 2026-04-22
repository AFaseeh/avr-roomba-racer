#include "motor.h"
#include <util/delay.h>

int main()
{
    motor_init();

    int i = 1;
    while (1) {
        i *= -1;
        // Example: Set left motor to 50% forward and right motor to 50% reverse
        motor_set_speed(i*50, -50*i);
        _delay_ms(2000);

        // Stop the motors
        motor_stop();
        _delay_ms(1000);

        // Coast the motors
        motor_coast();
        _delay_ms(1000);
    }

    return 0;
}