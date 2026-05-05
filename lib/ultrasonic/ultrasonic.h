#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

typedef enum {
    US_FRONT = 0,
    US_LEFT  = 1,
    US_RIGHT = 2
} UltrasonicID_t;

// Initialize the sensor pins and enable the ICU interrupt
void ultrasonic_init(void);

// Send a 10 us trigger pulse to start a measurement.
// This only runs when the sensor is idle, it does not wait for the echo.
void ultrasonic_trigger(UltrasonicID_t id);

// get the latest measured distance in cm (saves the results until the next measurement is taken)
uint16_t ultrasonic_get_distance(UltrasonicID_t id);

// Trigger the next sensor in the sequence (F -> L -> R -> F -> ...)
void ultrasonic_next(void);

// Blocks until all 3 sensors have completed a measurement
// Used in initializing the system
void ultrasonic_full_sweep(void);
#endif /* ULTRASONIC_H */