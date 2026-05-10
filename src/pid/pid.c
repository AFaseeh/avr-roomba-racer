#include "pid.h"

void pd_init(PD_Controller_t *pd, int16_t kp_x100, int16_t kd_x100)
{
    pd->kp_x100 = kp_x100; // to make calculations faster for atmega as it doesn't have floating point unit so we use fixed point math with a scale factor of 100
    pd->kd_x100 = kd_x100;
    pd->prev_error = 0;
}

void pd_seed(PD_Controller_t *pd, int16_t current_error)
{
    pd->prev_error = current_error;
}

int16_t pd_compute(PD_Controller_t *pd, int16_t current_error)
{
    int16_t delta = (int16_t)(current_error - pd->prev_error);
    int16_t p = (int16_t)(((int32_t)pd->kp_x100 * current_error) / 100L);
    int16_t d = (int16_t)(((int32_t)pd->kd_x100 * delta) / 100L);

    pd->prev_error = current_error;

    return (int16_t)(p + d);
}
