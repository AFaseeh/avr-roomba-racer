#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    int16_t kp_x100;
    int16_t kd_x100;
    int16_t prev_error;
} PD_Controller_t;

void pd_init(PD_Controller_t *pd, int16_t kp_x100, int16_t kd_x100);

void pd_seed(PD_Controller_t *pd, int16_t current_error);

int16_t pd_compute(PD_Controller_t *pd, int16_t current_error);

#endif /* PID_H */
