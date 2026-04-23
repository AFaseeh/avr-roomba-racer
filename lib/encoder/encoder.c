#include "encoder.h"
#include "gpio.h"
#include "helpers.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint32_t left_ticks = 0;
volatile uint32_t right_ticks = 0;

void encoder_init(void) {
    // Set (PD2 / INT0) and (PD3 / INT1) as inputs
    gpio_set_pin_direction(PORT_D, 2, GPIO_DIR_INPUT);
    gpio_set_pin_direction(PORT_D, 3, GPIO_DIR_INPUT);

    // rising edge interrupts on both pins (11)
    SET_BIT(EICRA, ISC01);
    SET_BIT(EICRA, ISC00);
    SET_BIT(EICRA, ISC11);
    SET_BIT(EICRA, ISC10);

    // enable INT0 and INT1
    SET_BIT(EIMSK, INT0);
    SET_BIT(EIMSK, INT1);

    sei(); 
}


ISR(INT0_vect) {
    left_ticks++;
}

ISR(INT1_vect) {
    right_ticks++;
}

void encoder_get_both_ticks(uint32_t *out_left, uint32_t *out_right)
{
    uint8_t sreg_backup = SREG;
    cli();
    *out_left = left_ticks;
    *out_right = right_ticks;
    SREG = sreg_backup;
}

void encoder_reset(void) {
    uint8_t sreg_backup = SREG;
    cli();
    left_ticks = 0;
    right_ticks = 0;
    SREG = sreg_backup;
}