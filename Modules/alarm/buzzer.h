
/*******************************************************************************
#
#                      版权所有 (C), 2025-2026, NCU_Roboteam
#
# *******************************************************************************
#  文 件 名: buzzer.h
#  版 本 号: V20251120.1
#  作    者: 肖景煊
#  生成日期: 2025年11月20日
#  功能描述: 蜂鸣器告警模块头文件，定义告警实例、枚举与接口
#  补    充: 无
#
#*******************************************************************************/

/****************************头文件开头必备内容********************************/

#ifndef BUZZER_H

#define BUZZER_H

/********************************包含头文件************************************/

#include "bsp_pwm.h"

/*******************************文件内宏定义***********************************/

#define BUZZER_DEVICE_CNT 5

#define  DoFreq  523

#define  ReFreq  587

#define  MiFreq  659

#define  FaFreq  698

#define  SoFreq  784

#define  LaFreq  880

#define  SiFreq  988

/*******************************文件内结构体***********************************/

typedef enum
{
  
  OCTAVE_1 = 0,
  
  OCTAVE_2,
  
  OCTAVE_3,
  
  OCTAVE_4,
  
  OCTAVE_5,
  
  OCTAVE_6,
  
  OCTAVE_7,
  
  OCTAVE_8,
}octave_e;


typedef enum
{
  
  ALARM_LEVEL_HIGH = 0,
  
  ALARM_LEVEL_ABOVE_MEDIUM,
  
  ALARM_LEVEL_MEDIUM,
  
  ALARM_LEVEL_BELOW_MEDIUM,
  
  ALARM_LEVEL_LOW,
}AlarmLevel_e;


typedef enum
{
  
  ALARM_OFF = 0,
  
  ALARM_ON,
}AlarmState_e;


typedef struct
{
  
  AlarmLevel_e alarm_level;
  
  octave_e octave;
  
  float loudness;
}Buzzer_config_s;


typedef struct
{
  
  float loudness;
  
  octave_e octave;
  
  AlarmLevel_e alarm_level;
  
  AlarmState_e alarm_state;
}BuzzzerInstance;

/*******************************全局变量声明***********************************/

/********************************自定义函数************************************/

void BuzzerInit();

void BuzzerTask();

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state);

/*代码注释：*/

/******************************** 文件结束 ************************************/
#endif // !BUZZER_H
