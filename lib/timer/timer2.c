#include "timer2.h"
#include "gpio.h"
#include "helpers.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint32_t millis = 0;

void timer2_init_millis(void)
{
    // CTC mode
    SET_BIT(TCCR2B, WGM22);
    
    // Prescaler 64
    SET_BIT(TCCR2B, CS22);
     
    OCR2A = 249;
    TCNT2 = 0;

    SET_BIT(TIMSK2, OCIE2A);
}

uint32_t get_millis(void)
{
    uint32_t current_millis;
    cli();
    current_millis = millis;
    sei();
    return current_millis;
}

ISR(TIMER2_COMPA_vect) {
    millis++;
}