#include "fsm.h"
#include "motor.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "pid/pid.h"
#include "bluetooth.h"
#include "timer2.h"
#include "uart.h"
#include <stdio.h>
#include "helpers.h"

// Wall following
//#define KP 0.1f
#define KD 0.05f
#define MAX_CORRECTION 10
#define TARGET_WALL_DIST    150
#define MIN_WALL_DIST       100


//#define BREAK_KP 0.33       // KP = Speed / Breaking dist (error), at 350 mm we want it to be 100% then (350 - 150 (which is the Critical Dist)) = 
#define BREAK_KD 0.05f

#define ALIGN_KP 0.8f
#define ALIGN_KD 0.3f
#define CRITICAL_FRONT_DIST 200     // Front
#define SLOWDOWN_FRONT_DIST 500
#define WALL_FOLLOW_SPEED 80
#define LOST_WALL_SPEED 70
#define TURN_SPEED 80
#define ALIGN_SPEED 60

#define ALIGN_FRONT_CLEAR_DIST 250
#define ALIGN_TOLERANCE_MM 20
#define LOST_WALL_TIMEOUT_MS 2500UL // 2.5 secs
#define LOST_WALL_RECOVER_DIST 250
#define FINISH_OPEN_SIDE_DIST 400
#define FINISH_OPEN_FRONT_DIST 400
#define INVALID_DISTANCE_MM 0xFFFFU

#define MIN_MOTOR_SPEED 30
#define DECISION_TIME_MS 250
#define TICKS_FOR_90_DEG 310

volatile RobotState_t current_state = STATE_WALL_FOLLOW;
PD_Controller_t wall_pd;
PD_Controller_t align_pd;
PD_Controller_t brake_pd;

int16_t get_correction(uint16_t dist_L);
int16_t get_braking_speed(uint16_t dist_F, int16_t max_speed);

void fsm_init(void)
{
    float steer_zone = (float)(TARGET_WALL_DIST - MIN_WALL_DIST);
    float dynamic_steer_kp = (float)MAX_CORRECTION / steer_zone;
    pd_init(&wall_pd, dynamic_steer_kp, KD);

    float brake_zone_width = (float)(SLOWDOWN_FRONT_DIST - CRITICAL_FRONT_DIST);
    float speed_to_drop = (float)(WALL_FOLLOW_SPEED - MIN_MOTOR_SPEED);
    float dynamic_brake_kp = speed_to_drop / brake_zone_width;
    pd_init(&brake_pd, dynamic_brake_kp, BREAK_KD);
    
    pd_init(&align_pd, ALIGN_KP, ALIGN_KD);
}

void fsm_update(uint16_t dist_L, uint16_t dist_R, uint16_t dist_F, char* printf_buffer)
{
    uint32_t enc_left = 0, enc_right = 0;
    float error;
    int16_t correction, left_speed, right_speed;
    static int16_t turn_direction = 1; // -1 for left, 1 for right
    static uint8_t finish_data_sent = 0;
    static uint32_t lost_wall_start_ms = 0;
    static uint16_t decision_front_reference = INVALID_DISTANCE_MM;
    static uint32_t decision_start_ms = 0;
    static uint32_t post_turn_ms = 0;

    //sprintf(printf_buffer, "State: %d | F: %3u mm | L: %3u mm | R: %3u mm\r\n", current_state, dist_F, dist_L, dist_R);
    //uart_send_string(printf_buffer);
    switch (current_state) {
        case STATE_WALL_FOLLOW:
            if (dist_F < CRITICAL_FRONT_DIST) {
                motor_stop();
                current_state = STATE_DECISION;
                decision_start_ms = get_millis();
                break;
            }
            if ((dist_L == INVALID_DISTANCE_MM) || (dist_L > LOST_WALL_RECOVER_DIST)) {
                lost_wall_start_ms = get_millis();
                current_state = STATE_LOST_WALL;
                break;
            }
            correction = get_correction(dist_L);
            
            int16_t base_speed = get_braking_speed(dist_F, WALL_FOLLOW_SPEED);
            
            left_speed = base_speed - correction;
            right_speed = base_speed + correction;

            left_speed = CLAMP(left_speed, MIN_MOTOR_SPEED, WALL_FOLLOW_SPEED);
            right_speed = CLAMP(right_speed, MIN_MOTOR_SPEED, WALL_FOLLOW_SPEED);
            sprintf(printf_buffer, "State: WALL_FOLLOW | F: %3u mm | L: %3u mm | R: %3u mm, speeds L: %d R: %d\r\n", dist_F, dist_L, dist_R, left_speed, right_speed);
            // sprintf(printf_buffer, "State: WALL_FOLLOW | F: %3u mm | L: %3u mm | R: %3u mm | Error: %d | Correction: %d\r\n", dist_F, dist_L, dist_R, (int)error, correction);
            motor_set_speed(left_speed, right_speed);
            break;
        case STATE_DECISION:
        {
            if ((get_millis() - decision_start_ms) < DECISION_TIME_MS){
                break;
            }
            
            decision_front_reference = dist_F;
            encoder_reset();
            if ((dist_L == INVALID_DISTANCE_MM) && (dist_R == INVALID_DISTANCE_MM)) {
                current_state = STATE_LOST_WALL;
                lost_wall_start_ms = get_millis();
                break;
            }

            // Left open -> turn left
            if (dist_L == INVALID_DISTANCE_MM) {
                COMM_LogTurn('L');
                current_state = STATE_TURN_LEFT;
                turn_direction = -1;
            }
            // Right open -> turn right
            else if (dist_R == INVALID_DISTANCE_MM) {
                COMM_LogTurn('R');
                current_state = STATE_TURN_RIGHT;
                turn_direction = 1;
            }
            // turn towards more open side
            else if (dist_L > dist_R) {
                COMM_LogTurn('L');
                current_state = STATE_TURN_LEFT;
                turn_direction = -1;
            } 
            else {
                COMM_LogTurn('R');
                current_state = STATE_TURN_RIGHT;
                turn_direction = 1;
            }
            break;
        }
        case STATE_POST_TURN:
            if ((get_millis() - post_turn_ms) >= DECISION_TIME_MS) {
                current_state = STATE_WALL_FOLLOW;
            }
            break;
        case STATE_TURN_LEFT:
        case STATE_TURN_RIGHT:
            encoder_get_both_ticks(&enc_left, &enc_right);
            sprintf(printf_buffer, "State: TURNING %d | L: %lu | R: %lu\r\n", turn_direction, enc_left, enc_right);
            if (((enc_left + enc_right) / 2) >= TICKS_FOR_90_DEG) {
                motor_stop();
                current_state = STATE_POST_TURN;
                post_turn_ms = get_millis();
            } else {
                motor_set_speed(turn_direction * TURN_SPEED, -turn_direction * TURN_SPEED);
            }
            break;
        case STATE_ALIGN:
        {
            uint16_t align_distance = (turn_direction < 0) ? dist_R : dist_L;
            int16_t rotation_direction = 0;

            sprintf(printf_buffer,
                    "State: ALIGN | RefF: %3u mm | Align: %3u mm\r\n",
                    decision_front_reference,
                    align_distance);

            if ((decision_front_reference == INVALID_DISTANCE_MM) ||
                (align_distance == INVALID_DISTANCE_MM)) {
                current_state = STATE_FINISH;
                motor_stop();
                break;
            }

            error = (float)align_distance - (float)decision_front_reference;
            correction = pd_compute(&align_pd, error);

            if ((error <= ALIGN_TOLERANCE_MM) && (error >= -ALIGN_TOLERANCE_MM)) {
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

            if (dist_F < CRITICAL_FRONT_DIST) {
                motor_stop();
                current_state = STATE_DECISION;
                decision_start_ms = get_millis();
                break;
            }

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

            uint16_t speed = get_braking_speed(dist_F, LOST_WALL_SPEED);
            motor_set_speed(speed, speed);
            break;
        }
        case STATE_FINISH:
            motor_stop();
            if (finish_data_sent == 0U) {
                COMM_TransmitFinalData();
                finish_data_sent = 1U;
            }
            // else
            // {
            //     sprintf(printf_buffer, "State: FINISH | F: %3u mm | L: %3u mm | R: %3u mm\r\n", dist_F, dist_L, dist_R);
            // }
            break;
    }
}


int16_t get_braking_speed(uint16_t dist_F, int16_t max_speed)
{    
    if (dist_F < SLOWDOWN_FRONT_DIST) {
        float brake_error = (float)dist_F - CRITICAL_FRONT_DIST;
        int16_t pd = pd_compute(&brake_pd, brake_error);
        
        return CLAMP(pd + MIN_MOTOR_SPEED, MIN_MOTOR_SPEED, max_speed);
    }
    return max_speed;
}

int16_t get_correction(uint16_t dist_L)
{
    if (dist_L == INVALID_DISTANCE_MM || dist_L > 500) {
        return 0;
    }

    float error = (float)dist_L - TARGET_WALL_DIST;
    int16_t correction = pd_compute(&wall_pd, error);
    return CLAMP(correction, -MAX_CORRECTION, MAX_CORRECTION);
}