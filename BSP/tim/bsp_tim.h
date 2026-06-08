/**
*******************************************************************************

                      版权所有 (C), 2025-2026, NCU_Roboteam

 *******************************************************************************
  文 件 名: bsp_tim.h
  版 本 号: 20251120+01
  作    者: 肖景煊
  生成日期: 2025年11月20日
  功能描述: 通用定时器(TIM) BSP 头文件，提供实例注册、启动/停止、周期设定与回调机制
  补    充: 大疆A板可用 TIM1-13；TIM14 作为 FreeRTOS 时间基准保留，不在此层使用

*******************************************************************************/

/****************************头文件开头必备内容********************************/

#ifndef BSP_TIM_H
#define BSP_TIM_H

/********************************包含头文件************************************/

#include "tim.h"
#include <stdint.h>

/*******************************文件内宏定义***********************************/

/* 最大支持的 TIM 实例数量（TIM1-13 可用，TIM14保留） */
#define TIM_DEVICE_CNT 13

/* 禁止注册的定时器（用于 FreeRTOS 时间基准） */
#define TIM_FORBIDDEN_INSTANCE TIM14

/* TIM 工作模式枚举（按需组合） */
typedef enum
{
    TIM_MODE_BASE            = 0x01, /* 基本定时（更新事件） */
    TIM_MODE_INPUT_CAPTURE   = 0x02, /* 输入捕获 */
    TIM_MODE_OUTPUT_COMPARE  = 0x04, /* 输出比较 */
} TIM_MODE_e;

/*******************************文件内结构体***********************************/

/* TIM 实例结构体定义 */
typedef struct tim_ins
{
    TIM_HandleTypeDef *htim;                               /* TIM 句柄 */
    TIM_MODE_e mode;                                       /* 工作模式 */
    uint32_t channels_mask;                                /* 通道掩码（如 TIM_CHANNEL_1 | TIM_CHANNEL_2） */
    void (*period_callback)(struct tim_ins *);             /* 周期更新事件回调 */
    void (*ic_callback)(struct tim_ins *, uint32_t ch);    /* 输入捕获回调（携带通道） */
    void (*oc_callback)(struct tim_ins *, uint32_t ch);    /* 输出比较回调（携带通道） */
    void *id;                                              /* 模块实例地址 */
} TIMInstance;

/* TIM 初始化配置结构体定义 */
typedef struct
{
    TIM_HandleTypeDef *htim;                               /* TIM 句柄 */
    TIM_MODE_e mode;                                       /* 工作模式 */
    uint32_t channels_mask;                                /* 通道掩码 */
    void (*period_callback)(TIMInstance *);                /* 周期更新事件回调 */
    void (*ic_callback)(TIMInstance *, uint32_t ch);       /* 输入捕获回调 */
    void (*oc_callback)(TIMInstance *, uint32_t ch);       /* 输出比较回调 */
    void *id;                                              /* 模块实例地址 */
} TIM_Init_Config_s;

/*******************************全局变量声明***********************************/

/********************************自定义函数************************************/

/* 注册并返回 TIM 实例指针 */
TIMInstance *TIMRegister(TIM_Init_Config_s *config);

/* 基本定时：启动/停止（更新事件中断） */
void TIMBaseStart_IT(TIMInstance *instance);
void TIMBaseStop_IT(TIMInstance *instance);

/* 输入捕获：按通道启动/停止（中断方式） */
void TIMICStart_IT(TIMInstance *instance, uint32_t channel);
void TIMICStop_IT(TIMInstance *instance, uint32_t channel);

/* 输出比较：按通道启动/停止（中断方式） */
void TIMOCStart_IT(TIMInstance *instance, uint32_t channel);
void TIMOCStop_IT(TIMInstance *instance, uint32_t channel);

/* 设置基本定时周期（微秒），根据当前 PSC 计算自动重装值 */
void TIMSetPeriodUs(TIMInstance *instance, uint32_t period_us);

/* 计数器读写接口 */
uint32_t TIMGetCounter(TIMInstance *instance);

void TIMSetCounter(TIMInstance *instance, uint32_t cnt);

void BSP_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* BSP_TIM_H */

/******************************** 文件结束 ************************************/