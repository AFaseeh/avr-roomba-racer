#ifndef PID_H
#define PID_H

#include <stdint.h>

// We dont need last time in calculating the differential term
// Since time intervals are almost the same ~150 ms
typedef struct {
    float Kp;
    float Kd;
    float prev_error;
    uint32_t prev_time;
    float last_d;
} PD_Controller_t;

void pd_init(PD_Controller_t *pd, float kp, float kd);

int16_t pd_compute(PD_Controller_t *pd, float current_error);

#endif /* PID_H */