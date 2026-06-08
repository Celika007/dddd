#ifndef POSTURECONTROL_TASK__H
#define POSTURECONTROL_TASK__H

#include "action.h"
#include <stm32f4xx_hal.h>
#include "stdbool.h"
#include "math.h"

#define YES 1
#define NO 0
#define PI 3.14159

#define BLL 11 //大腿长度
#define SLL 22 //小腿长度

void PostureControl(void);
void gait_detached(DetachedParam d_params, float leg0_offset, float leg1_offset, float leg2_offset, float leg3_offset);
void CoupledMoveLeg(float t, GaitParams params, float gait_offset, int LegId);
void CycloidTrajectory(float t, GaitParams params, float gait_offset);
void CartesianToTheta(void);
void SetCoupledPosition(int LegId);

#endif
