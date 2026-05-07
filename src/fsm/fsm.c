#include "fsm.h"
#include "motor.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "pid/pid.h"
#include "bluetooth.h"
#include "timer2.h"
#include <stdio.h>

// Wall following
#define KP 1.0f
#define KD 0.5f
#define ALIGN_KP 0.8f
#define ALIGN_KD 0.3f
#define CRITICAL_FRONT_DIST 150
#define SLOWDOWN_FRONT_DIST 350
#define TARGET_WALL_DIST 140
#define WALL_FOLLOW_SPEED 100
#define LOST_WALL_SPEED 45
#define TURN_SPEED 80
#define ALIGN_SPEED 60
#define ALIGN_FRONT_CLEAR_DIST 250
#define ALIGN_TOLERANCE_MM 20
#define ALIGN_STABLE_CYCLES 3
#define LOST_WALL_TIMEOUT_MS 1000UL
#define LOST_WALL_RECOVER_DIST 250
#define FINISH_OPEN_SIDE_DIST 300
#define FINISH_OPEN_FRONT_DIST 400
#define TICKS_FOR_90_DEG 10
#define INVALID_DISTANCE_MM 0xFFFFU

volatile RobotState_t current_state = STATE_WALL_FOLLOW;
PD_Controller_t wall_pd;
PD_Controller_t align_pd;

void fsm_init(void)
{
    pd_init(&wall_pd, KP, KD);
    pd_init(&align_pd, ALIGN_KP, ALIGN_KD);
}

void fsm_update(uint16_t dist_L, uint16_t dist_R, uint16_t dist_F, char* printf_buffer)
{
    uint32_t enc_left = 0, enc_right = 0;
    float error;
    int16_t correction, left_speed, right_speed;
    static int16_t turn_direction = 1; // -1 for left, 1 for right
    static uint8_t align_stable_cycles = 0;
    static uint8_t finish_data_sent = 0;
    static uint32_t lost_wall_start_ms = 0;
    static uint16_t decision_front_reference = INVALID_DISTANCE_MM;
    switch (current_state) {
        case STATE_WALL_FOLLOW:
            sprintf(printf_buffer, "State: WALL_FOLLOW | F: %3u mm | L: %3u mm | R: %3u mm\r\n", dist_F, dist_L, dist_R);
            if (dist_F < CRITICAL_FRONT_DIST) {
                motor_stop();
                current_state = STATE_DECISION;
                break;
            }
            if ((dist_L == INVALID_DISTANCE_MM) || (dist_L > LOST_WALL_RECOVER_DIST)) {
                lost_wall_start_ms = get_millis();
                current_state = STATE_LOST_WALL;
                break;
            }
            error = (float)dist_L - TARGET_WALL_DIST;
            correction = pd_compute(&wall_pd, error);
            left_speed = WALL_FOLLOW_SPEED + correction;
            right_speed = WALL_FOLLOW_SPEED - correction;
            motor_set_speed(left_speed, right_speed);
            break;
        case STATE_DECISION:
        {
            decision_front_reference = dist_F;
            sprintf(printf_buffer, "State: DECISION | F: %3u mm | L: %3u mm | R: %3u mm\r\n", dist_F, dist_L, dist_R);
            encoder_reset();

            if ((dist_L == INVALID_DISTANCE_MM) && (dist_R == INVALID_DISTANCE_MM)) {
                current_state = STATE_LOST_WALL;
                lost_wall_start_ms = get_millis();
                break;
            }

            if (dist_R == INVALID_DISTANCE_MM) {
                COMM_LogTurn('L');
                current_state = STATE_TURN_LEFT;
                turn_direction = -1;
                break;
            }

            if ((dist_L != INVALID_DISTANCE_MM) && (dist_L > dist_R)) {
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
        }
        case STATE_TURN_LEFT:
        case STATE_TURN_RIGHT:
            encoder_get_both_ticks(&enc_left, &enc_right);
            sprintf(printf_buffer, "State: TURNING %d | L: %lu | R: %lu\r\n", turn_direction, enc_left, enc_right);
            if (((enc_left + enc_right) / 2) >= TICKS_FOR_90_DEG) {
                motor_stop();
                align_stable_cycles = 0;
                current_state = STATE_ALIGN;
            } else {
                motor_set_speed(turn_direction * TURN_SPEED, -turn_direction * TURN_SPEED);
            }
            break;
        case STATE_ALIGN:
        {
            uint16_t align_distance = (turn_direction < 0) ? dist_R : dist_L;
            int16_t rotation_direction = 0;

            sprintf(printf_buffer,
                    "State: ALIGN | RefF: %3u mm | Align: %3u mm | stable: %u\r\n",
                    decision_front_reference,
                    align_distance,
                    align_stable_cycles);

            if ((decision_front_reference == INVALID_DISTANCE_MM) ||
                (align_distance == INVALID_DISTANCE_MM)) {
                align_stable_cycles = 0;
                motor_stop();
                break;
            }

            error = (float)align_distance - (float)decision_front_reference;
            correction = pd_compute(&align_pd, error);

            if ((error <= ALIGN_TOLERANCE_MM) && (error >= -ALIGN_TOLERANCE_MM)) {
                align_stable_cycles++;
            } else {
                align_stable_cycles = 0;
            }

            if (align_stable_cycles >= ALIGN_STABLE_CYCLES) {
                motor_stop();
                current_state = STATE_WALL_FOLLOW;
                break;
            }

            if (correction > 0) {
                rotation_direction = (int16_t)(-turn_direction);
            } else if (correction < 0) {
                rotation_direction = turn_direction;
            }

            if (rotation_direction == 0) {
                motor_stop();
            } else {
                left_speed = rotation_direction * ALIGN_SPEED;
                right_speed = -rotation_direction * ALIGN_SPEED;
                motor_set_speed(left_speed, right_speed);
            }
            break;
        }
        case STATE_LOST_WALL:
        {
            uint8_t left_open = ((dist_L == INVALID_DISTANCE_MM) || (dist_L > FINISH_OPEN_SIDE_DIST)) ? 1U : 0U;
            uint8_t right_open = ((dist_R == INVALID_DISTANCE_MM) || (dist_R > FINISH_OPEN_SIDE_DIST)) ? 1U : 0U;
            uint8_t front_open = ((dist_F == INVALID_DISTANCE_MM) || (dist_F > FINISH_OPEN_FRONT_DIST)) ? 1U : 0U;

            sprintf(printf_buffer, "State: LOST_WALL | F: %3u mm | L: %3u mm | R: %3u mm | elapsed: %lu\r\n",
                    dist_F,
                    dist_L,
                    dist_R,
                    (unsigned long)(get_millis() - lost_wall_start_ms));

            if ((dist_L != INVALID_DISTANCE_MM) && (dist_L <= LOST_WALL_RECOVER_DIST)) {
                current_state = STATE_WALL_FOLLOW;
                break;
            }

            if (left_open && right_open && front_open &&
                ((get_millis() - lost_wall_start_ms) >= LOST_WALL_TIMEOUT_MS)) {
                motor_stop();
                finish_data_sent = 0;
                current_state = STATE_FINISH;
                break;
            }

            motor_set_speed(LOST_WALL_SPEED, LOST_WALL_SPEED);
            break;
        }
        case STATE_FINISH:
            motor_stop();
            if (finish_data_sent == 0U) {
                COMM_TransmitFinalData();
                finish_data_sent = 1U;
            }
            break;
    }
}
