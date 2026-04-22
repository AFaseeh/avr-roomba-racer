/* Motor.h */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);

void motor_set_speed(int8_t left_speed, int8_t right_speed);

void motor_stop(void);

void motor_coast(void);

#endif /* MOTOR_H */