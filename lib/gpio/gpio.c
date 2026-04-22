#include "gpio.h"
#include "helpers.h"
#include <avr/io.h>

void gpio_set_pin_direction(GPIO_Port_t port, uint8_t pin, GPIO_Dir_t direction) {
    if (direction == GPIO_DIR_OUTPUT) {
        switch (port) {
            case PORT_B: SET_BIT(DDRB, pin); break;
            case PORT_C: SET_BIT(DDRC, pin); break;
            case PORT_D: SET_BIT(DDRD, pin); break;
        }
    } else { // INPUT
        switch (port) {
            case PORT_B: CLEAR_BIT(DDRB, pin); break;
            case PORT_C: CLEAR_BIT(DDRC, pin); break;
            case PORT_D: CLEAR_BIT(DDRD, pin); break;
        }
    }
}

void gpio_write_pin(GPIO_Port_t port, uint8_t pin, GPIO_State_t state) {
    if (state == GPIO_PIN_HIGH) {
        switch (port) {
            case PORT_B: SET_BIT(PORTB, pin); break;
            case PORT_C: SET_BIT(PORTC, pin); break;
            case PORT_D: SET_BIT(PORTD, pin); break;
        }
    } else { // LOW
        switch (port) {
            case PORT_B: CLEAR_BIT(PORTB, pin); break;
            case PORT_C: CLEAR_BIT(PORTC, pin); break;
            case PORT_D: CLEAR_BIT(PORTD, pin); break;
        }
    }
}

GPIO_State_t gpio_read_pin(GPIO_Port_t port, uint8_t pin)
{
    switch (port) {
        case PORT_B: return READ_BIT(PINB, pin) ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
        case PORT_C: return READ_BIT(PINC, pin) ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
        case PORT_D: return READ_BIT(PIND, pin) ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
    }
    return GPIO_PIN_LOW;
}
