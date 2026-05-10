#include "timer1.h"
#include <avr/io.h>
#include "helpers.h"

void timer1_init_normal_mode(void) {
    // normal mode
    TCCR1A = 0;
    TCCR1B = 0;

    // prescaler 8 -> 0.5us per tick (16MHz / 8 = 2MHz)
    TCCR1B = (1U << CS11);

    // Start at 0
    TCNT1 = 0;
}
