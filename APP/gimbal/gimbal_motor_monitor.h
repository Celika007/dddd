#ifndef GIMBAL_MOTOR_MONITOR_H
#define GIMBAL_MOTOR_MONITOR_H

#include <stdint.h>

void GimbalMotorMonitor_Init(void);
void GimbalMotorMonitor_Task(void);
float GimbalMotorMonitor_GetAngleDeg(void);
uint16_t GimbalMotorMonitor_GetEcd(void);
uint8_t GimbalMotorMonitor_HasFeedback(void);

#endif
