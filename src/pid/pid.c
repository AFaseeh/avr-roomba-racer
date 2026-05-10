#include "pid.h"
#include "timer2.h"

void pd_init(PD_Controller_t *pd, float kp, float kd)
{
    pd->Kp = kp;
    pd->Kd = kd;
    pd->prev_error = 0.0f;
    pd->prev_time = get_millis();
    pd->last_d = 0.0f;
}

int16_t pd_compute(PD_Controller_t *pd, float current_error)
{
    uint32_t current_time = get_millis();
    float dt_ms = (float)(current_time - pd->prev_time); // Convert ms to seconds
    float p = pd->Kp * current_error;
    if (dt_ms > 0.0f) {        
        pd->last_d = pd->Kd * (current_error - pd->prev_error) / dt_ms * 1000.0f;
        
        pd->prev_error = current_error;
        pd->prev_time = current_time;
    }
    return (int16_t)(p + pd->last_d);
}
