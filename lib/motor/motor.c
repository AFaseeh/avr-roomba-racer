#include "motor.h"
#include <avr/io.h>

// Timer Selection
#define TCCR_A_REG          TCCR0A
#define TCCR_B_REG          TCCR0B
#define PWM_LEFT_REG        OCR0A
#define PWM_RIGHT_REG       OCR0B

// Timer Mode Bits (Fast PWM)
#define WGM_BIT_0           WGM00
#define WGM_BIT_1           WGM01

// 3. Compare Output Mode Bits (Non-Inverting)
#define COM_LEFT_BIT        COM0A1
#define COM_RIGHT_BIT       COM0B1

// 4. Prescaler Bits (Prescaler = 64)
#define CS_BIT_0            CS00
#define CS_BIT_1            CS01

// 5. Physical PWM Pins (Timer0 uses PD6 and PD5)
#define PWM_DDR             DDRD
#define PWM_LEFT_PIN        PD6 
#define PWM_RIGHT_PIN       PD5


// Direction Pins (L298N)
#define DIR_DDR             DDRB
#define DIR_PORT            PORTB
#define IN1_LEFT_FWD        PB0  
#define IN2_LEFT_REV        PB1  
#define IN3_RIGHT_FWD       PB2 
#define IN4_RIGHT_REV       PB3 

/* Helper macros */
#define SET_BIT(REG, BIT)   ((REG) |=  (1U << (BIT)))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(1U << (BIT)))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

void motor_init(void) {
    // set direction pins as outputs
    DIR_DDR |= (1 << IN1_LEFT_FWD) | (1 << IN2_LEFT_REV) | (1 << IN3_RIGHT_FWD) | (1 << IN4_RIGHT_REV);
    
    // set PWM pins as outputs
    PWM_DDR |= (1 << PWM_LEFT_PIN) | (1 << PWM_RIGHT_PIN);

    // Timer: Fast PWM Mode (WGM_BIT_1, WGM_BIT_0) & Non-Inverting Output (COM_LEFT_BIT, COM_RIGHT_BIT)
    TCCR_A_REG = (1 << COM_LEFT_BIT) | (1 << COM_RIGHT_BIT) | (1 << WGM_BIT_1) | (1 << WGM_BIT_0);

    // Prescaler = 64
    TCCR_B_REG = (1 << CS_BIT_1) | (1 << CS_BIT_0);

    motor_coast();
}

void motor_set_speed(int8_t left_speed, int8_t right_speed) {
    left_speed = MAX(-100, MIN(100, left_speed));
    right_speed = MAX(-100, MIN(100, right_speed));

    if (left_speed > 0) {
        // Forward
        SET_BIT(DIR_PORT, IN1_LEFT_FWD);
        CLEAR_BIT(DIR_PORT, IN2_LEFT_REV);
        PWM_LEFT_REG = (left_speed * 255) / 100;
    } else if (left_speed < 0) {
        // Reverse
        CLEAR_BIT(DIR_PORT, IN1_LEFT_FWD);
        SET_BIT(DIR_PORT, IN2_LEFT_REV);
        PWM_LEFT_REG = (-left_speed * 255) / 100; 
    } else {
        // Coast
        CLEAR_BIT(DIR_PORT, IN1_LEFT_FWD);
        CLEAR_BIT(DIR_PORT, IN2_LEFT_REV);
        PWM_LEFT_REG = 0;
    }

    if (right_speed > 0) {
        // Forward
        SET_BIT(DIR_PORT, IN3_RIGHT_FWD);
        CLEAR_BIT(DIR_PORT, IN4_RIGHT_REV);
        PWM_RIGHT_REG = (right_speed * 255) / 100;
    } else if (right_speed < 0) {
        // Reverse
        CLEAR_BIT(DIR_PORT, IN3_RIGHT_FWD);
        SET_BIT(DIR_PORT, IN4_RIGHT_REV);
        PWM_RIGHT_REG = (-right_speed * 255) / 100;
    } else {
        // Coast
        CLEAR_BIT(DIR_PORT, IN3_RIGHT_FWD);
        CLEAR_BIT(DIR_PORT, IN4_RIGHT_REV);
        PWM_RIGHT_REG = 0;
    }
}

void motor_stop(void) {
    // all = 0 or 1 & PWM = enable (255) -> stop
    // maybe try all = 1, might be a stronger breaking force (should be the same)
    CLEAR_BIT(DIR_PORT, IN1_LEFT_FWD);
    CLEAR_BIT(DIR_PORT, IN2_LEFT_REV);
    CLEAR_BIT(DIR_PORT, IN3_RIGHT_FWD);
    CLEAR_BIT(DIR_PORT, IN4_RIGHT_REV);

    PWM_LEFT_REG = 255;
    PWM_RIGHT_REG = 255;
}

void motor_coast(void) {
    // all = 0 or 1 & PWM = disable (0) -> coast
    CLEAR_BIT(DIR_PORT, IN1_LEFT_FWD);
    CLEAR_BIT(DIR_PORT, IN2_LEFT_REV);
    CLEAR_BIT(DIR_PORT, IN3_RIGHT_FWD);
    CLEAR_BIT(DIR_PORT, IN4_RIGHT_REV);

    PWM_LEFT_REG = 0;
    PWM_RIGHT_REG = 0;
}
