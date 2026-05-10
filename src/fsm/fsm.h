#ifndef FSM_H
#define FSM_H

#include <stddef.h>
#include <stdint.h>



// Define the States
typedef enum { // TODO: maybe add a backward state to get away from the wall if we are too close
    //STATE_INIT,
    STATE_WALL_FOLLOW,
    STATE_DECISION,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_ALIGN,        // TODO: use Ultrasonic to align with the wall after turning, maybe use PD control here as well? or just turn until we see the wall at the right distance?
    STATE_LOST_WALL,    // TODO: if we lost the wall for more than 1 sec, we can assume we are in the finish line (since there is no wall at the finish line) and then we can stop and transmit the data
    STATE_FINISH        
} RobotState_t;

extern volatile RobotState_t current_state;

void fsm_init(void);

void fsm_update(uint16_t dist_L, uint16_t dist_R, uint16_t dist_F,
                char *printf_buffer, size_t printf_buffer_size);

#endif /* FSM_H */
