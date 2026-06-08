#ifndef MOTORCONTROL_TASK__H
#define MOTORCONTROL_TASK__H

#include "motor_def.h"
#include "can.h"
#include "stdbool.h"
#include "bsp_can.h"
#include "dji_motor.h"

#define ReductionAndAngleRatio 19.2

#define LEFT_MOTOR_COUNT 4
#define RIGHT_MOTOR_COUNT 4

#define IsReady 1
#define IsNotReady 0

extern bool IsMotoReadyOrNot;


void LeftBuildDefaultMotorConfig(Motor_Init_Config_s config[LEFT_MOTOR_COUNT]);
void RightBuildDefaultMotorConfig(Motor_Init_Config_s config[RIGHT_MOTOR_COUNT]);
bool LeftMotorInit(void);
bool RightMotorInit(void);
DJIMotorInstance *LeftGetMotor(uint8_t index);
DJIMotorInstance *RightGetMotor(uint8_t index);
#endif 
