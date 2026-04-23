#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

void timer0_init_fast_pwm(void);
void timer0_set_pwm_a(uint8_t duty); // For PD6
void timer0_set_pwm_b(uint8_t duty); // For PD5

#endif /* TIMER_H */