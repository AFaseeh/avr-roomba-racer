#include "motor.h"
#include "gpio.h"
#include "timer0.h"
#include "helpers.h"

#define IN1_PORT        PORT_D
#define IN1_LEFT_FWD         4  // D4
#define IN2_PORT        PORT_D
#define IN2_LEFT_REV         7  // D7
#define IN3_PORT        PORT_B
#define IN3_RIGHT_FWD         4  // D12
#define IN4_PORT        PORT_B
#define IN4_RIGHT_REV         5  // D13

void motor_init(void) {
    // set direction pins as outputs
    gpio_set_pin_direction(IN1_PORT, IN1_LEFT_FWD, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(IN2_PORT, IN2_LEFT_REV, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(IN3_PORT, IN3_RIGHT_FWD, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(IN4_PORT, IN4_RIGHT_REV, GPIO_DIR_OUTPUT);
    
    timer0_init_fast_pwm();

    motor_coast();
}

void motor_set_speed(int8_t left_speed, int8_t right_speed) {
    // Motors are swapped and oriented in opposite directions, so we need to swap and invert one side
    int8_t temp = -left_speed;
    left_speed = right_speed;
    right_speed = temp;

    left_speed = CLAMP(left_speed, -100, 100);
    right_speed = CLAMP(right_speed, -100, 100);

    if (left_speed > 0) {
        // Forward
        gpio_write_pin(IN1_PORT, IN1_LEFT_FWD, GPIO_PIN_HIGH);
        gpio_write_pin(IN2_PORT, IN2_LEFT_REV, GPIO_PIN_LOW);
        timer0_set_pwm_left( (uint8_t)( ((int16_t)left_speed * 255) / 100 ) );
    } else if (left_speed < 0) {
        // Reverse
        gpio_write_pin(IN1_PORT, IN1_LEFT_FWD, GPIO_PIN_LOW);
        gpio_write_pin(IN2_PORT, IN2_LEFT_REV, GPIO_PIN_HIGH);
        timer0_set_pwm_left( (uint8_t)( ((int16_t)(-left_speed) * 255) / 100 ) );
    } else {
        // Coast
        gpio_write_pin(IN1_PORT, IN1_LEFT_FWD, GPIO_PIN_LOW);
        gpio_write_pin(IN2_PORT, IN2_LEFT_REV, GPIO_PIN_LOW);
        timer0_set_pwm_left(0);
    }

    if (right_speed > 0) {
        // Forward
        gpio_write_pin(IN3_PORT, IN3_RIGHT_FWD, GPIO_PIN_HIGH);
        gpio_write_pin(IN4_PORT, IN4_RIGHT_REV, GPIO_PIN_LOW);
        timer0_set_pwm_right( (uint8_t)( ((int16_t)right_speed * 255) / 100 ) );
    } else if (right_speed < 0) {
        // Reverse
        gpio_write_pin(IN3_PORT, IN3_RIGHT_FWD, GPIO_PIN_LOW);
        gpio_write_pin(IN4_PORT, IN4_RIGHT_REV, GPIO_PIN_HIGH);
        timer0_set_pwm_right( (uint8_t)( ((int16_t)(-right_speed) * 255) / 100 ) );
    } else {
        // Coast
        gpio_write_pin(IN3_PORT, IN3_RIGHT_FWD, GPIO_PIN_LOW);
        gpio_write_pin(IN4_PORT, IN4_RIGHT_REV, GPIO_PIN_LOW);
        timer0_set_pwm_right(0);
    }
}

void motor_stop(void) {
    // all = 0 or 1 & PWM = enable (255) -> stop
    // maybe try all = 1, might be a stronger breaking force (should be the same)
    gpio_write_pin(IN1_PORT, IN1_LEFT_FWD, GPIO_PIN_LOW);
    gpio_write_pin(IN2_PORT, IN2_LEFT_REV, GPIO_PIN_LOW);
    gpio_write_pin(IN3_PORT, IN3_RIGHT_FWD, GPIO_PIN_LOW);
    gpio_write_pin(IN4_PORT, IN4_RIGHT_REV, GPIO_PIN_LOW);

    timer0_set_pwm_left(255);
    timer0_set_pwm_right(255);
}

void motor_coast(void) {
    // all = 0 or 1 & PWM = disable (0) -> coast
    gpio_write_pin(IN1_PORT, IN1_LEFT_FWD, GPIO_PIN_LOW);
    gpio_write_pin(IN2_PORT, IN2_LEFT_REV, GPIO_PIN_LOW);
    gpio_write_pin(IN3_PORT, IN3_RIGHT_FWD, GPIO_PIN_LOW);
    gpio_write_pin(IN4_PORT, IN4_RIGHT_REV, GPIO_PIN_LOW);

    // Drop PWM to 0
    timer0_set_pwm_left(0);
    timer0_set_pwm_right(0);
}