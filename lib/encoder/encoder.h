#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void); // INT 0 and INT 1 on PD2 and PD3

void encoder_get_both_ticks(uint32_t *left_ticks, uint32_t *right_ticks);

// reset the counts (to use before a 90-degree turn)
void encoder_reset(void);

#endif /* ENCODER_H */