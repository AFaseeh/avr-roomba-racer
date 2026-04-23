#include "ultrasonic.h"
#include "gpio.h"
#include "helpers.h"
#include "timer1.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define FRONT_TRIG_PORT PORT_B
#define FRONT_TRIG_PIN  1  // D9
#define FRONT_ECHO_PORT PORT_B
#define FRONT_ECHO_PIN  0  // D8 (ICP1)

#define LEFT_TRIG_PORT  PORT_C
#define LEFT_TRIG_PIN   0  // A0
#define LEFT_ECHO_PORT  PORT_C
#define LEFT_ECHO_PIN   1  // A1 (PCINT9)

#define RIGHT_TRIG_PORT PORT_C
#define RIGHT_TRIG_PIN  2  // A2
#define RIGHT_ECHO_PORT PORT_C
#define RIGHT_ECHO_PIN  3  // A3 (PCINT11)

// The 4 states of our Ultrasonic State Machine
typedef enum {
    US_IDLE,
    US_WAITING_RISING,
    US_WAITING_FALLING
} UltrasonicState_t;

//  shared with the ISR and the main loop
volatile UltrasonicState_t us_state[3] = {US_IDLE, US_IDLE, US_IDLE};
volatile uint16_t final_distance[3] = {0, 0, 0};
volatile uint16_t start_time[3] = {0, 0, 0};
volatile uint16_t end_time[3] = {0, 0, 0};

void ultrasonic_init(void) {
    timer1_init_normal_mode();

    // GPIO directions: triggers are outputs, echoes are inputs
    gpio_set_pin_direction(FRONT_TRIG_PORT, FRONT_TRIG_PIN, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(FRONT_ECHO_PORT, FRONT_ECHO_PIN, GPIO_DIR_INPUT);
    gpio_set_pin_direction(LEFT_TRIG_PORT, LEFT_TRIG_PIN, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(LEFT_ECHO_PORT, LEFT_ECHO_PIN, GPIO_DIR_INPUT);
    gpio_set_pin_direction(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, GPIO_DIR_OUTPUT);
    gpio_set_pin_direction(RIGHT_ECHO_PORT, RIGHT_ECHO_PIN, GPIO_DIR_INPUT);

    // triggers should start low
    gpio_write_pin(FRONT_TRIG_PORT, FRONT_TRIG_PIN, GPIO_PIN_LOW);
    gpio_write_pin(LEFT_TRIG_PORT, LEFT_TRIG_PIN, GPIO_PIN_LOW);
    gpio_write_pin(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, GPIO_PIN_LOW);

    // enable ICU interrupt for the front sensor
    SET_BIT(TIMSK1, ICIE1);

    // enable PCINT for left and right
    SET_BIT(PCICR, PCIE1); 
    SET_BIT(PCMSK1, PCINT9);  // PC1 (A1)
    SET_BIT(PCMSK1, PCINT11); // PC3 (A3)
}

void ultrasonic_trigger(UltrasonicID_t id) {
    if (us_state[id] != US_IDLE) {
        return; 
    }

    if (id == US_FRONT) {
        gpio_write_pin(FRONT_TRIG_PORT, FRONT_TRIG_PIN, GPIO_PIN_HIGH);
        _delay_us(10);
        gpio_write_pin(FRONT_TRIG_PORT, FRONT_TRIG_PIN, GPIO_PIN_LOW);
        
        SET_BIT(TCCR1B, ICES1); // Look for rising edge
        SET_BIT(TIFR1, ICF1);   // Clear ICU flag
    } 
    else if (id == US_LEFT) {
        gpio_write_pin(LEFT_TRIG_PORT, LEFT_TRIG_PIN, GPIO_PIN_HIGH);
        _delay_us(10);
        gpio_write_pin(LEFT_TRIG_PORT, LEFT_TRIG_PIN, GPIO_PIN_LOW);
        
        SET_BIT(PCIFR, PCIF1);  // Clear PCINT flag
    } 
    else if (id == US_RIGHT) {
        gpio_write_pin(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, GPIO_PIN_HIGH);
        _delay_us(10);
        gpio_write_pin(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, GPIO_PIN_LOW);
        
        SET_BIT(PCIFR, PCIF1);  // Clear PCINT flag
    }

    us_state[id] = US_WAITING_RISING;
}

uint16_t ultrasonic_get_distance(UltrasonicID_t id) {
    return final_distance[id];
}

ISR(TIMER1_CAPT_vect) {
    if (us_state[US_FRONT] == US_WAITING_RISING) {
        start_time[US_FRONT] = ICR1;
        
        // look for falling edge
        CLEAR_BIT(TCCR1B, ICES1);
        SET_BIT(TIFR1, ICF1); // clear flag
        
        us_state[US_FRONT] = US_WAITING_FALLING;
    } 
    else if (us_state[US_FRONT] == US_WAITING_FALLING) {
        end_time[US_FRONT] = ICR1;
        
        uint16_t total_ticks = end_time[US_FRONT] - start_time[US_FRONT];
        
        // Prescaler 8 -> 0.5us per tick
        uint32_t time_us = total_ticks / 2;
        final_distance[US_FRONT] = time_us / 58;

        us_state[US_FRONT] = US_IDLE;
    }
}

// --- The Left & Right Sensors (Pin Change Interrupts) ---
ISR(PCINT1_vect) {
    // 1. GRAB THE TIME IMMEDIATELY! 
    // We do this first so the CPU delay doesn't affect our math.
    uint16_t current_time = TCNT1; 
    
    // 2. Read the actual physical pins using your HAL
    uint8_t left_pin_state = gpio_read_pin(LEFT_ECHO_PORT, LEFT_ECHO_PIN);
    uint8_t right_pin_state = gpio_read_pin(RIGHT_ECHO_PORT, RIGHT_ECHO_PIN);

    // --- Process Left Sensor ---
    if (us_state[US_LEFT] == US_WAITING_RISING) {
        if (left_pin_state == GPIO_PIN_HIGH) { // It went high!
            start_time[US_LEFT] = current_time;
            us_state[US_LEFT] = US_WAITING_FALLING;
        }
    } 
    else if (us_state[US_LEFT] == US_WAITING_FALLING) {
        if (left_pin_state == GPIO_PIN_LOW) { // It went low!
            end_time[US_LEFT] = current_time;
            uint32_t time_us = (end_time[US_LEFT] - start_time[US_LEFT]) / 2;
            final_distance[US_LEFT] = time_us / 58;
            us_state[US_LEFT] = US_IDLE;
        }
    }

    // --- Process Right Sensor ---
    if (us_state[US_RIGHT] == US_WAITING_RISING) {
        if (right_pin_state == GPIO_PIN_HIGH) {
            start_time[US_RIGHT] = current_time;
            us_state[US_RIGHT] = US_WAITING_FALLING;
        }
    } 
    else if (us_state[US_RIGHT] == US_WAITING_FALLING) {
        if (right_pin_state == GPIO_PIN_LOW) {
            end_time[US_RIGHT] = current_time;
            uint32_t time_us = (end_time[US_RIGHT] - start_time[US_RIGHT]) / 2;
            final_distance[US_RIGHT] = time_us / 58;
            us_state[US_RIGHT] = US_IDLE;
        }
    }
}