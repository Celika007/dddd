#ifndef __UNICOMM_H
#define __UNICOMM_H

#include <stdint.h>

extern float current_x;
extern float current_y;
extern float target_x;
extern float target_y;
extern float upstream_current_yaw;
extern float upstream_goal_yaw;
extern uint8_t nav_current_valid;
extern uint8_t nav_goal_valid;

void UniComm_UART8_Init(void);
void UniComm_SendArrival(void);

#endif
