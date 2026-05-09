#include "fsm.h"
#include "motor.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "pid/pid.h"
#include "bluetooth.h"
#include "uart.h"
#include <stdio.h>

// Wall following
#define KP 1.0f
#define KD 0.5f
#define CRITICAL_FRONT_DIST 150
#define SLOWDOWN_FRONT_DIST 350
#define TARGET_WALL_DIST 140
#define WALL_FOLLOW_SPEED 100
#define TURN_SPEED 80
#define ALIGN_SPEED 60
#define TICKS_FOR_90_DEG 90

volatile RobotState_t current_state = STATE_WALL_FOLLOW;
PD_Controller_t wall_pd;

void fsm_init(void)
{
    pd_init(&wall_pd, KP, KD);
}

void fsm_update(uint16_t dist_L, uint16_t dist_R, uint16_t dist_F, char* printf_buffer)
{
    uint32_t enc_left = 0, enc_right = 0;
    float error;
    int16_t correction, left_speed, right_speed;
    static int16_t turn_direction = 1; // -1 for left, 1 for right
    switch (current_state) {
        case STATE_WALL_FOLLOW:
            //sprintf(printf_buffer, "State: WALL_FOLLOW | F: %3u mm | L: %3u mm | R: %3u mm\r\n", dist_F, dist_L, dist_R);
            if (dist_F < CRITICAL_FRONT_DIST) {
                motor_stop();
                current_state = STATE_DECISION;
                break;
            } 
            error = (float)dist_L - TARGET_WALL_DIST;
            correction = pd_compute(&wall_pd, error);
            left_speed = WALL_FOLLOW_SPEED;// + correction;
            right_speed = WALL_FOLLOW_SPEED;// - correction;
            sprintf(printf_buffer, "State: WALL_FOLLOW | F: %3u mm | L: %3u mm | R: %3u mm | Error: %d | Correction: %d\r\n", dist_F, dist_L, dist_R, (int)error, correction);
            uart_send_string(printf_buffer);
            motor_set_speed(left_speed, right_speed);
            break;
        case STATE_DECISION:
            // takes around ~150 ms BLOCKING (before decision we get the newest distances)
            // TODO: full sweep only on left and right and do it like 3 times and take the average to reduce noise effect
            // and then decide based on the average distances
            sprintf(printf_buffer, "State: DECISION | F: %3u mm | L: %3u mm | R: %3u mm\r\n", dist_F, dist_L, dist_R);
            // ultrasonic_full_sweep();    
            encoder_reset();
            if (dist_L > dist_R) {
                // More space on the left
                COMM_LogTurn('L');
                current_state = STATE_TURN_LEFT;
                turn_direction = -1;
            } else {
                // More space on the right
                COMM_LogTurn('R');
                current_state = STATE_TURN_RIGHT;
                turn_direction = 1;
            }
            break;
        case STATE_TURN_LEFT:
        case STATE_TURN_RIGHT:
            encoder_get_both_ticks(&enc_left, &enc_right);
            sprintf(printf_buffer, "State: TURNING %d | L: %lu | R: %lu\r\n", turn_direction, enc_left, enc_right);
            if (((enc_left + enc_right) / 2) >= TICKS_FOR_90_DEG) {
                motor_stop();
                current_state = STATE_ALIGN;
            } else {
                motor_set_speed(turn_direction * TURN_SPEED, -turn_direction * TURN_SPEED);
            }
            break;
        case STATE_ALIGN:
            sprintf(printf_buffer, "State: ALIGN | F: %3u mm\r\n", dist_F);
            current_state = STATE_WALL_FOLLOW;
            break;
        case STATE_LOST_WALL:
            // TODO: after 1 sec of lost wall go to STATE_FINISH
            break;
        case STATE_FINISH:
            motor_stop();
            COMM_TransmitFinalData();
            break;
    }
}
