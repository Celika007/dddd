#ifndef ACTION_H
#define ACTION_H

#include <stdint.h>

#include "stdlib.h"
#include "MotorControl_task.h"

#define START_POS1 -18
#define START_POS2 140
#define START_POS3 -140
#define START_POS4 18
#define START_POS5 -140
#define START_POS6 18
#define START_POS7 -18
#define START_POS8 140

#define NAV_ARRIVAL_THRESHOLD_CM 8.0f
#define NAV_YAW_FINISH_THRESHOLD_DEG 2.0f
#define TURN_GAIN 1.0f
#define RUN_YAW_CORRECTION_GAIN 0.0f
#define ACTION_RECORD_UART8_ENABLE 1U
#define ACTION_RECORD_RATE_MULTIPLIER 2.0f
#define ACTION_RECORD_TX_BUF_SIZE 384U

extern float  temp_theta1;
extern float  temp_theta2;
extern float  temp_theta3;
extern float  temp_theta4;
extern float  temp_theta5;
extern float  temp_theta6;
extern float  temp_theta7;
extern float  temp_theta8;

enum States
{
    RUN=0,
    STAND=1,
    /* Task race state. AKANE obstacle-course debug logic does not use GRAB. */
    GRAB=2,
    LEFT_TURN=3,
    RIGHT_TURN=4,
    IN_PLACE=5,
    STOP=6,
    REALSE=7,
};

typedef struct  {		// 腿部参数结构体
    float stance_height ; // 狗身到地面的距离 (cm)
    float step_length ; // 步长的距离 (cm)
    float up_amp ; // 上部振幅y (cm)
    float down_amp ; // 下部振幅 (cm)
    float flight_percent ; // 摆动相百分比 (cm)
    float freq ; // 步频 (Hz)
    float stance_height_offset; //狗身到地面的距离的偏移量 (cm)
    float step_length_offset ; // 步长的距离的偏移量 (cm)
    float up_amp_offset ; // 上部振幅y偏移量 (cm)
    float down_amp_offset ; // 下部振幅偏移量 (cm)
    float flight_percent_offset ; // 摆动相百分比偏移量 (cm)
    float freq_offset ; // 步频偏移量 (Hz)
} GaitParams;

typedef struct
{
    GaitParams detached_params_0;//右前
    GaitParams detached_params_1;//左前
    GaitParams detached_params_2;//左后
    GaitParams detached_params_3;//右后
}DetachedParam;

extern DetachedParam detached_params;
extern DetachedParam state_detached_params[];
extern enum States state;
extern int start_flag;
extern float target_yaw;

void DOWN(void);
void Action_UART8DebugInit(void);
void ActionProcessNavigation(void);
void ActionProcessSerialState(void);
void ActionProcessDebugControl(void);
void Action_ApplyRunYawCorrection(DetachedParam *run_params);
void ActionDebug_UpdateWalkPhase(float base_phase);
void ActionDebug_ResetWalkRecord(void);
uint8_t ActionDebug_ShouldHoldRealse(void);

#endif 
