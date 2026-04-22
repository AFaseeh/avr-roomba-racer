#include "timer.h"
#include "gpio.h"
#include "helpers.h"
#include <avr/io.h>

void timer0_init_fast_pwm(void)
{
    gpio_set_pin_direction(PORT_D, 6, GPIO_DIR_OUTPUT); // PD6 as output (OC0A)
    gpio_set_pin_direction(PORT_D, 5, GPIO_DIR_OUTPUT); // PD5 as output (OC0B)

    // fast pwm (WGM01, WGM00)
    SET_BIT(TCCR0A, WGM01);
    SET_BIT(TCCR0A, WGM00);

    // both non inverting(COM0A1, COM0B1)
    SET_BIT(TCCR0A, COM0A1);
    SET_BIT(TCCR0A, COM0B1);

    // prescaler = 64
    SET_BIT(TCCR0B, CS01);
    SET_BIT(TCCR0B, CS00);

    OCR0A = 0;
    OCR0B = 0;
}

void timer0_set_pwm_a(uint8_t duty) {
    OCR0A = duty;
}

void timer0_set_pwm_b(uint8_t duty) {
    OCR0B = duty;
}