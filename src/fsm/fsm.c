#include "fsm.h"
#include "motor.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "pid/pid.h"
#include "bluetooth.h"
#include "timer2.h"
#include "uart.h"
#include <stdio.h>

// Fixed-point PD gains are scaled by 100.
#define KP_X100 100
#define KD_X100 50
#define ALIGN_KP_X100 80
#define ALIGN_KD_X100 30

#define CRITICAL_FRONT_DIST 150U
#define SLOWDOWN_FRONT_DIST 350U
#define TARGET_WALL_DIST 140U
#define WALL_FOLLOW_SPEED 100
#define WALL_APPROACH_SPEED 65
#define LOST_WALL_SPEED 45
#define TURN_SPEED 80
#define TURN_MIN_SPEED 50
#define TURN_MAX_SPEED 90
#define TURN_SKEW_DIVISOR 5L
#define TURN_TIMEOUT_MS 2000UL
#define ALIGN_SPEED 60
#define TICKS_FOR_90_DEG 90UL

#define ALIGN_TOLERANCE_MM 20
#define ALIGN_STABLE_CYCLES 3U
#define LOST_WALL_TIMEOUT_MS 1000UL
#define LOST_WALL_RECOVER_DIST 250U
#define FINISH_OPEN_SIDE_DIST 400U
#define FINISH_OPEN_FRONT_DIST 400U
#define INVALID_DISTANCE_MM ULTRASONIC_INVALID_DISTANCE_MM

#define MIN_DRIVE_SPEED 0
#define MAX_DRIVE_SPEED 100

#ifndef FSM_DEBUG_TELEMETRY
// what is this? will I used the same logic I made in the bluetooth where I made the car print  with an of condition when we want to debug
// not always as I think print is blocking I will fix that in the next commit 
// bluetooth and UART are weird now 
#define FSM_DEBUG_TELEMETRY 0
#endif

#define FSM_DEBUG_INTERVAL_MS 100UL

volatile RobotState_t current_state = STATE_WALL_FOLLOW;
PD_Controller_t wall_pd;
PD_Controller_t align_pd;

static int16_t turn_direction = 1; // -1 for left, 1 for right
static uint8_t align_stable_cycles = 0U;
static uint8_t finish_data_sent = 0U;
static uint32_t lost_wall_start_ms = 0UL;
static uint32_t turn_start_ms = 0UL;

static uint8_t distance_is_valid(uint16_t distance)
{
    return (distance != INVALID_DISTANCE_MM) ? 1U : 0U;
}

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}
/// @brief Clamps the motor speed to the specified range to prevent overflow and ensure safe operation.
/// @param speed 
/// @param min_value 
/// @param max_value 
/// @return 
static int8_t clamp_motor_speed(int16_t speed, int16_t min_value, int16_t max_value)
{
    return (int8_t)clamp_i16(speed, min_value, max_value);
}

static uint8_t distance_is_open(uint16_t distance, uint16_t threshold)
{
    return ((distance == INVALID_DISTANCE_MM) || (distance > threshold)) ? 1U : 0U;
}

/// @brief why do I need to seed the PD controller? well, if we start the wall follow state
/// when we are already at a certain distance from the wall, 
/// the initial error will be large and cause a big correction that can lead to instability.
/// By seeding the PD controller with the current error, 
/// we can ensure that it starts with a more accurate state and provides smoother control from the beginning. 
/// This is especially important when transitioning into the wall follow state from other states where the distance to the wall might have changed significantly.
/// @param dist_L 
static void seed_wall_pd(uint16_t dist_L)
{
    if (distance_is_valid(dist_L)) {
        pd_seed(&wall_pd, (int16_t)((int16_t)dist_L - (int16_t)TARGET_WALL_DIST));
    } else {
        pd_seed(&wall_pd, 0);
    }
}
// I made the states centeralized before there  was alot of repeated code 
// and I didn't understand alot of things lol
static void fsm_enter_state(RobotState_t next_state, uint16_t dist_L)
{
    if (current_state == next_state) {
        return;
    }

    current_state = next_state;

    switch (next_state) {
        case STATE_WALL_FOLLOW:
            align_stable_cycles = 0U;
            seed_wall_pd(dist_L);
            break;

        case STATE_DECISION:
            motor_stop();
            break;

        case STATE_TURN_LEFT:
            turn_direction = -1;
            encoder_reset();
            turn_start_ms = get_millis();
            break;

        case STATE_TURN_RIGHT:
            turn_direction = 1;
            encoder_reset();
            turn_start_ms = get_millis();
            break;

        case STATE_ALIGN:
            pd_init(&align_pd, ALIGN_KP_X100, ALIGN_KD_X100);
            align_stable_cycles = 0U;
            break;

        case STATE_LOST_WALL:
            lost_wall_start_ms = get_millis();
            break;

        case STATE_FINISH:
            motor_stop();
            finish_data_sent = 0U;
            break;
    }
}
/// @brief Commands the robot to turn based on encoder readings.
/// This function calculates the skew between the left and right encoders and adjusts the motor speeds accordingly to achieve a more accurate turn.
/// The turn continues until the robot has turned approximately 90 degrees (as determined by encoder ticks) or until a timeout occurs to prevent getting stuck in the turn state.
/// using min not average of the encoders to determine turn progress to avoid issues where one encoder might miss ticks and cause the robot to underturn.
/// @param enc_left 
/// @param enc_right 
static void command_turn(uint32_t enc_left, uint32_t enc_right)
{
    int32_t skew = (int32_t)enc_left - (int32_t)enc_right;
    int16_t left_mag = TURN_SPEED;
    int16_t right_mag = TURN_SPEED;
    int16_t correction;

    if (skew > 0L) {
        correction = clamp_i16((int16_t)(skew / TURN_SKEW_DIVISOR), 0, TURN_SPEED - TURN_MIN_SPEED);
        left_mag = (int16_t)(left_mag - correction);
        right_mag = (int16_t)(right_mag + correction);
    } else if (skew < 0L) {
        correction = clamp_i16((int16_t)((-skew) / TURN_SKEW_DIVISOR), 0, TURN_SPEED - TURN_MIN_SPEED);
        left_mag = (int16_t)(left_mag + correction);
        right_mag = (int16_t)(right_mag - correction);
    }

    left_mag = clamp_i16(left_mag, TURN_MIN_SPEED, TURN_MAX_SPEED);
    right_mag = clamp_i16(right_mag, TURN_MIN_SPEED, TURN_MAX_SPEED);

    motor_set_speed((int8_t)(turn_direction * left_mag),
                    (int8_t)(-turn_direction * right_mag));
}

static void fsm_debug(uint16_t dist_L, uint16_t dist_R, uint16_t dist_F,
                      char *printf_buffer, size_t printf_buffer_size)
{
#if FSM_DEBUG_TELEMETRY
    static uint32_t last_debug_ms = 0UL;
    uint32_t now = get_millis();

    if ((printf_buffer == NULL) || (printf_buffer_size == 0U) ||
        (current_state == STATE_FINISH) ||
        ((now - last_debug_ms) < FSM_DEBUG_INTERVAL_MS)) {
        return;
    }

    last_debug_ms = now;
    snprintf(printf_buffer, printf_buffer_size,
             "State:%u F:%u L:%u R:%u\r\n",
             (unsigned)current_state,
             (unsigned)dist_F,
             (unsigned)dist_L,
             (unsigned)dist_R);
    uart_send_string(printf_buffer);
#else
    (void)dist_L;
    (void)dist_R;
    (void)dist_F;
    (void)printf_buffer;
    (void)printf_buffer_size;
#endif
}

void fsm_init(void)
{
    pd_init(&wall_pd, KP_X100, KD_X100);
    pd_init(&align_pd, ALIGN_KP_X100, ALIGN_KD_X100);
    turn_direction = 1;
    align_stable_cycles = 0U;
    finish_data_sent = 0U;
    lost_wall_start_ms = 0UL;
    turn_start_ms = 0UL;
    current_state = STATE_WALL_FOLLOW;
}

void fsm_update(uint16_t dist_L, uint16_t dist_R, uint16_t dist_F,
                char *printf_buffer, size_t printf_buffer_size)
{
    uint32_t enc_left = 0UL;
    uint32_t enc_right = 0UL;
    int16_t error = 0;
    int16_t correction = 0;
    int16_t base_speed = WALL_FOLLOW_SPEED;
    int16_t left_speed = 0;
    int16_t right_speed = 0;

    switch (current_state) {
        case STATE_WALL_FOLLOW:
            if (distance_is_valid(dist_F) && (dist_F < CRITICAL_FRONT_DIST)) {
                fsm_enter_state(STATE_DECISION, dist_L);
                break;
            }

            if (!distance_is_valid(dist_L) || (dist_L > LOST_WALL_RECOVER_DIST)) {
                fsm_enter_state(STATE_LOST_WALL, dist_L);
                break;
            }
            // so here I made a decision to slow down the motor while turning 
            // idk if this is good or bad needs testing
            if (distance_is_valid(dist_F) && (dist_F < SLOWDOWN_FRONT_DIST)) {
                base_speed = WALL_APPROACH_SPEED;
            }

            error = (int16_t)((int16_t)dist_L - (int16_t)TARGET_WALL_DIST);
            correction = pd_compute(&wall_pd, error);

            // here I added the correction from PID and made anew function to avoid overflow
            left_speed = clamp_motor_speed((int16_t)(base_speed + correction),
                                           MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
            right_speed = clamp_motor_speed((int16_t)(base_speed - correction),
                                            MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
            motor_set_speed((int8_t)left_speed, (int8_t)right_speed);
            break;

        case STATE_DECISION:
            motor_stop();

            if (!distance_is_valid(dist_L) && !distance_is_valid(dist_R)) {
                fsm_enter_state(STATE_LOST_WALL, dist_L);
                break;
            }

            if (!distance_is_valid(dist_R)) {
                COMM_LogTurn('L');
                fsm_enter_state(STATE_TURN_LEFT, dist_L);
                break;
            }

            if (!distance_is_valid(dist_L)) {
                COMM_LogTurn('R');
                fsm_enter_state(STATE_TURN_RIGHT, dist_L);
                break;
            }

            if (dist_L > dist_R) {
                COMM_LogTurn('L');
                fsm_enter_state(STATE_TURN_LEFT, dist_L);
            } else {
                COMM_LogTurn('R');
                fsm_enter_state(STATE_TURN_RIGHT, dist_L);
            }
            break;

        case STATE_TURN_LEFT:
        case STATE_TURN_RIGHT:
        {
            uint32_t min_ticks;
            uint8_t turn_timed_out;

            encoder_get_both_ticks(&enc_left, &enc_right);
            min_ticks = (enc_left < enc_right) ? enc_left : enc_right;
            turn_timed_out = ((get_millis() - turn_start_ms) >= TURN_TIMEOUT_MS) ? 1U : 0U;

            if ((min_ticks >= TICKS_FOR_90_DEG) || turn_timed_out) {
                motor_stop();
                fsm_enter_state(STATE_ALIGN, dist_L);
                break;
            }

            command_turn(enc_left, enc_right);
            break;
        }

        case STATE_ALIGN:
        {
            uint16_t align_distance = (turn_direction < 0) ? dist_R : dist_L;
            int16_t rotation_direction;

            if (!distance_is_valid(align_distance)) {
                align_stable_cycles = 0U;
                motor_stop();
                fsm_enter_state(STATE_WALL_FOLLOW, dist_L);
                break;
            }

            error = (int16_t)((int16_t)align_distance - (int16_t)TARGET_WALL_DIST);
            correction = pd_compute(&align_pd, error);

            if ((error <= ALIGN_TOLERANCE_MM) && (error >= -ALIGN_TOLERANCE_MM)) { // requires 3 consecutive samples not relying on one sample that could overshoot
                align_stable_cycles++;
                motor_stop();

                if (align_stable_cycles >= ALIGN_STABLE_CYCLES) {
                    align_stable_cycles = 0U;
                    fsm_enter_state(STATE_WALL_FOLLOW, dist_L);
                }
                break;
            }

            align_stable_cycles = 0U;

            if (correction == 0) {
                motor_stop();
                break;
            }

            rotation_direction = (correction > 0) ? (int16_t)(-turn_direction) : turn_direction;
            motor_set_speed((int8_t)(rotation_direction * ALIGN_SPEED),
                            (int8_t)(-rotation_direction * ALIGN_SPEED));
            break;
        }

        case STATE_LOST_WALL: // so here there was a weird break and deadcode I removed it
        // when  wall appears the follow will continue normally
        {
            uint8_t left_open = distance_is_open(dist_L, FINISH_OPEN_SIDE_DIST);
            uint8_t right_open = distance_is_open(dist_R, FINISH_OPEN_SIDE_DIST);
            uint8_t front_open = distance_is_open(dist_F, FINISH_OPEN_FRONT_DIST);

            if (distance_is_valid(dist_L) && (dist_L <= LOST_WALL_RECOVER_DIST)) {
                fsm_enter_state(STATE_WALL_FOLLOW, dist_L);
                break;
            }

            if (left_open && right_open && front_open &&
                ((get_millis() - lost_wall_start_ms) >= LOST_WALL_TIMEOUT_MS)) {
                fsm_enter_state(STATE_FINISH, dist_L);
                break;
            }

            motor_set_speed(LOST_WALL_SPEED, LOST_WALL_SPEED);
            break;
        }

        case STATE_FINISH:
            motor_stop();
            if (finish_data_sent == 0U) {
                if (COMM_TransmitFinalData() != 0U) {
                    finish_data_sent = 1U;
                }
            }
            break;
    }

    fsm_debug(dist_L, dist_R, dist_F, printf_buffer, printf_buffer_size);
}
