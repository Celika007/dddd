#/*******************************************************************************
#
#                      版权所有 (C), 2025-2026, NCU_Roboteam
#
# *******************************************************************************
#  文 件 名: buzzer.c
#  版 本 号: V20251120.1
#  作    者: 肖景煊
#  生成日期: 2025年11月20日
#  功能描述: 蜂鸣器告警模块源文件，基于PWM实现音阶与响度控制
#  补    充: 无
#
#*******************************************************************************/

#/********************************包含头文件************************************/

#include "bsp_pwm.h"

#include "buzzer.h"

#include "bsp_dwt.h"

#include "string.h"

#/*******************************全局变量定义***********************************/

static PWMInstance *buzzer;

// static uint8_t idx;

static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

#/********************************文件内变量************************************/

#/********************************自定义函数************************************/
/**********************************************************************
  * @ 函数名  ： BuzzerInit
  * @ 功能说明： 蜂鸣器PWM通道初始化并注册为BSP层PWM实例
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： PWMInstance *buzzer：蜂鸣器PWM实例指针
  ********************************************************************/

void BuzzerInit()
{
    
    PWM_Init_Config_s buzzer_config = {
        
        .htim = &htim12,
        
        .channel = TIM_CHANNEL_2,
        
        .dutyratio = 0,
        
        .period = 0.001,
    };
    
    buzzer = PWMRegister(&buzzer_config);
}

/**********************************************************************
  * @ 函数名  ： BuzzerRegister
  * @ 功能说明： 注册一个蜂鸣器告警实例，并保存入实例列表
  * @ 参数    ： Buzzer_config_s *config：告警配置（级别/音阶/响度）
  * @ 返回值  ： BuzzzerInstance* 新创建的实例指针
  * @ 涉及变量： static BuzzzerInstance *buzzer_list[]：实例数组
  ********************************************************************/

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    
    if (config->alarm_level > BUZZER_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        
        while (1)
            ;
    
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    
    buzzer_temp->alarm_level = config->alarm_level;
    
    buzzer_temp->loudness = config->loudness;
    
    buzzer_temp->octave = config->octave;
    
    buzzer_temp->alarm_state = ALARM_OFF;
    
    buzzer_list[config->alarm_level] = buzzer_temp;
    
    return buzzer_temp;
}

/**********************************************************************
  * @ 函数名  ： AlarmSetStatus
  * @ 功能说明： 设置蜂鸣器告警实例的ON/OFF状态
  * @ 参数    ： BuzzzerInstance *buzzer：实例指针；AlarmState_e state：状态
  * @ 返回值  ： 无
  * @ 涉及变量： BuzzzerInstance::alarm_state
  ********************************************************************/

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    
    buzzer->alarm_state = state;
}

/**********************************************************************
  * @ 函数名  ： BuzzerTask
  * @ 功能说明： 蜂鸣器任务轮询，根据告警实例设置PWM响度与音阶
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： buzzer_list[]、PWMInstance *buzzer
  ********************************************************************/

void BuzzerTask()
{
    
    BuzzzerInstance *buzz;
    
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        
        buzz = buzzer_list[i];
        
        if (buzz->alarm_level > ALARM_LEVEL_LOW)
        {
            
            continue;
        }
        
        if (buzz->alarm_state == ALARM_OFF)
        {
            
            PWMSetDutyRatio(buzzer, 0);
        }
        else
        {
            
            PWMSetDutyRatio(buzzer, buzz->loudness);
            
            switch (buzz->octave)
            {
            
            case OCTAVE_1:
                
                PWMSetPeriod(buzzer, (float)1 / DoFreq);
                
                break;
            
            case OCTAVE_2:
                
                PWMSetPeriod(buzzer, (float)1 / ReFreq);
                
                break;
            
            case OCTAVE_3:
                
                PWMSetPeriod(buzzer, (float)1 / MiFreq);
                
                break;
            
            case OCTAVE_4:
                
                PWMSetPeriod(buzzer, (float)1 / FaFreq);
                
                break;
            
            case OCTAVE_5:
                
                PWMSetPeriod(buzzer, (float)1 / SoFreq);
                
                break;
            
            case OCTAVE_6:
                
                PWMSetPeriod(buzzer, (float)1 / LaFreq);
                
                break;
            
            case OCTAVE_7:
                
                PWMSetPeriod(buzzer, (float)1 / SiFreq);
                
                break;
            
            default:
                
                break;
            }
            
            break;
        }
    }
}

#/******************************** 文件结束 ************************************/
