#include "motor.h"
#include "gpio.h"
#include "timer0.h"
#include "helpers.h"

#define MOTOR_PORT          PORT_C
#define IN1_LEFT_FWD        0  // PC0
#define IN2_LEFT_REV        1  // PC1
#define IN3_RIGHT_FWD       2  // PC2
#define IN4_RIGHT_REV       3  // PC3

void motor_init(void) {
    // set direction pins as outputs
    gpio_set_pin_direction(MOTOR_PORT, IN1_LEFT_FWD, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(MOTOR_PORT, IN2_LEFT_REV, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(MOTOR_PORT, IN3_RIGHT_FWD, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(MOTOR_PORT, IN4_RIGHT_REV, GPIO_DIR_OUTPUT);
    
    timer0_init_fast_pwm();

    motor_coast();
}

void motor_set_speed(int8_t left_speed, int8_t right_speed) {
    left_speed = MAX(-100, MIN(100, left_speed));
    right_speed = MAX(-100, MIN(100, right_speed));

    if (left_speed > 0) {
        // Forward
        gpio_write_pin(MOTOR_PORT, IN1_LEFT_FWD, GPIO_PIN_HIGH);
        gpio_write_pin(MOTOR_PORT, IN2_LEFT_REV, GPIO_PIN_LOW);
        timer0_set_pwm_left((left_speed * 255) / 100);
    } else if (left_speed < 0) {
        // Reverse
        gpio_write_pin(MOTOR_PORT, IN1_LEFT_FWD, GPIO_PIN_LOW);
        gpio_write_pin(MOTOR_PORT, IN2_LEFT_REV, GPIO_PIN_HIGH);
        timer0_set_pwm_left((-left_speed * 255) / 100);
    } else {
        // Coast
        gpio_write_pin(MOTOR_PORT, IN1_LEFT_FWD, GPIO_PIN_LOW);
        gpio_write_pin(MOTOR_PORT, IN2_LEFT_REV, GPIO_PIN_LOW);
        timer0_set_pwm_left(0);
    }

    if (right_speed > 0) {
        // Forward
        gpio_write_pin(MOTOR_PORT, IN3_RIGHT_FWD, GPIO_PIN_HIGH);
        gpio_write_pin(MOTOR_PORT, IN4_RIGHT_REV, GPIO_PIN_LOW);
        timer0_set_pwm_right((right_speed * 255) / 100);
    } else if (right_speed < 0) {
        // Reverse
        gpio_write_pin(MOTOR_PORT, IN3_RIGHT_FWD, GPIO_PIN_LOW);
        gpio_write_pin(MOTOR_PORT, IN4_RIGHT_REV, GPIO_PIN_HIGH);
        timer0_set_pwm_right((-right_speed * 255) / 100);
    } else {
        // Coast
        gpio_write_pin(MOTOR_PORT, IN3_RIGHT_FWD, GPIO_PIN_LOW);
        gpio_write_pin(MOTOR_PORT, IN4_RIGHT_REV, GPIO_PIN_LOW);
        timer0_set_pwm_right(0);
    }
}

void motor_stop(void) {
    // all = 0 or 1 & PWM = enable (255) -> stop
    // maybe try all = 1, might be a stronger breaking force (should be the same)
    gpio_write_pin(MOTOR_PORT, IN1_LEFT_FWD, GPIO_PIN_LOW);
    gpio_write_pin(MOTOR_PORT, IN2_LEFT_REV, GPIO_PIN_LOW);
    gpio_write_pin(MOTOR_PORT, IN3_RIGHT_FWD, GPIO_PIN_LOW);
    gpio_write_pin(MOTOR_PORT, IN4_RIGHT_REV, GPIO_PIN_LOW);

    timer0_set_pwm_left(255);
    timer0_set_pwm_right(255);
}

void motor_coast(void) {
    // all = 0 or 1 & PWM = disable (0) -> coast
    gpio_write_pin(MOTOR_PORT, IN1_LEFT_FWD, GPIO_PIN_LOW);
    gpio_write_pin(MOTOR_PORT, IN2_LEFT_REV, GPIO_PIN_LOW);
    gpio_write_pin(MOTOR_PORT, IN3_RIGHT_FWD, GPIO_PIN_LOW);
    gpio_write_pin(MOTOR_PORT, IN4_RIGHT_REV, GPIO_PIN_LOW);

    // Drop PWM to 0
    timer0_set_pwm_left(0);
    timer0_set_pwm_right(0);
}