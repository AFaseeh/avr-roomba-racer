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

// 10us pulse to trigger the sensor (non-blocking, returns immediately)
// only triggers if the sensor is not currently measuring a distance (if it is idle)
void ultrasonic_trigger(UltrasonicID_t id);

// get the latest measured distance in cm (caches results until the next measurement is taken)
uint16_t ultrasonic_get_distance(UltrasonicID_t id);

#endif /* ULTRASONIC_H */