#include "pid.h"

void pd_init(PD_Controller_t *pd, float kp, float kd)
{
    pd->Kp = kp;
    pd->Kd = kd;
    pd->prev_error = 0.0f;
}

int16_t pd_compute(PD_Controller_t *pd, float current_error)
{
    float p = pd->Kp * current_error;
    float d = pd->Kd * (current_error - pd->prev_error);
    pd->prev_error = current_error;
    return (int16_t)(p + d);
}
