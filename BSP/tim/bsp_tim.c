/**
*******************************************************************************

                      版权所有 (C), 2025-2026, NCU_Roboteam

 *******************************************************************************
  文 件 名: bsp_tim.c
  版 本 号: 20251120+01
  作    者: 肖景煊
  生成日期: 2025年11月20日
  功能描述: 通用定时器(TIM) BSP，实现实例注册、基本定时/输入捕获/输出比较的启动与回调分发
  补    充: TIM1-13 可用；TIM14保留给FreeRTOS时间基准，不在此层使用

*******************************************************************************/

/********************************包含头文件************************************/

/* TIM BSP 头文件 */
#include "bsp_tim.h"
/* 动态内存分配与内存操作 */
#include "stdlib.h"
#include "memory.h"

/*******************************全局变量定义***********************************/

/* 所有 TIM 实例指针保存于此，用于回调分发与管理 */
static TIMInstance *tim_instances[TIM_DEVICE_CNT] = {NULL};

/* 已注册实例数量 */
static uint8_t tim_idx = 0;

/********************************文件内变量************************************/

/* 计算指定 TIM 的计时时钟频率（单位 Hz），用于周期参数换算 */
static uint32_t TIM_SelectClockHz(TIM_HandleTypeDef *htim)
{
    /* 若为 APB2 侧定时器：TIM1/TIM8/TIM9/TIM10/TIM11 */
    if (htim->Instance == TIM1 || htim->Instance == TIM8 ||
        htim->Instance == TIM9 || htim->Instance == TIM10 || htim->Instance == TIM11)
    {
        /* 读取 APB2 时钟频率（PCLK2） */
        uint32_t pclk = HAL_RCC_GetPCLK2Freq();
        /* 读取 APB2 预分频配置以判断是否存在倍频 */
        uint32_t ppre = (RCC->CFGR & RCC_CFGR_PPRE2);
        /* 当 APB2 分频不为 1 时，TIM 内核时钟为 APB2 时钟的两倍 */
        return (ppre == RCC_CFGR_PPRE2_DIV1) ? pclk : (pclk * 2U);
    }

    /* 其余默认属于 APB1：TIM2/TIM3/TIM4/TIM5/TIM6/TIM7/TIM12/TIM13 */
    else
    {
        /* 读取 APB1 时钟频率（PCLK1） */
        uint32_t pclk = HAL_RCC_GetPCLK1Freq();
        /* 读取 APB1 预分频配置以判断是否存在倍频 */
        uint32_t ppre = (RCC->CFGR & RCC_CFGR_PPRE1);
        /* 当 APB1 分频不为 1 时，TIM 内核时钟为 APB1 时钟的两倍 */
        return (ppre == RCC_CFGR_PPRE1_DIV1) ? pclk : (pclk * 2U);
    }
}

/********************************自定义函数************************************/

/* 注册并返回 TIM 实例指针 */
TIMInstance *TIMRegister(TIM_Init_Config_s *config)
{
    /* 超过可注册实例最大数量，进入保护（需调整宏或排查泄漏） */
    if (tim_idx >= TIM_DEVICE_CNT)
        while (1)
            ;

    /* 禁止使用 FreeRTOS 时间基准对应的 TIM14 */
    if (config->htim->Instance == TIM_FORBIDDEN_INSTANCE)
        while (1)
            ;

    /* 为 TIM 实例动态分配内存 */
    TIMInstance *inst = (TIMInstance *)malloc(sizeof(TIMInstance));
    /* 清零结构体，确保字段初始状态一致 */
    memset(inst, 0, sizeof(TIMInstance));

    /* 绑定硬件句柄 */
    inst->htim = config->htim;
    /* 配置工作模式（BASE/IC/OC 可组合） */
    inst->mode = config->mode;
    /* 记录通道掩码，用于多通道场景 */
    inst->channels_mask = config->channels_mask;
    /* 设置周期更新事件回调（可为空） */
    inst->period_callback = config->period_callback;
    /* 设置输入捕获回调（可为空） */
    inst->ic_callback = config->ic_callback;
    /* 设置输出比较回调（可为空） */
    inst->oc_callback = config->oc_callback;
    /* 记录模块实例地址（parent pointer） */
    inst->id = config->id;

    /* 写入全局实例数组并递增计数 */
    tim_instances[tim_idx++] = inst;
    /* 返回注册后的实例指针 */
    return inst;
}

/* 基本定时：启动更新事件中断 */
void TIMBaseStart_IT(TIMInstance *instance)
{
    /* 调用 HAL 启动基础计时中断 */
    HAL_TIM_Base_Start_IT(instance->htim);
}

/* 基本定时：停止更新事件中断 */
void TIMBaseStop_IT(TIMInstance *instance)
{
    /* 调用 HAL 停止基础计时中断 */
    HAL_TIM_Base_Stop_IT(instance->htim);
}

/* 启动指定通道的输入捕获中断，用于测频/测时等 */
void TIMICStart_IT(TIMInstance *instance, uint32_t channel)
{
    /* 调用 HAL 启动输入捕获中断 */
    HAL_TIM_IC_Start_IT(instance->htim, channel);
}

/* 停止指定通道的输入捕获中断 */
void TIMICStop_IT(TIMInstance *instance, uint32_t channel)
{
    /* 调用 HAL 停止输入捕获中断 */
    HAL_TIM_IC_Stop_IT(instance->htim, channel);
}

/* 启动指定通道的输出比较中断，用于事件对齐/定时输出 */
void TIMOCStart_IT(TIMInstance *instance, uint32_t channel)
{
    /* 调用 HAL 启动输出比较中断 */
    HAL_TIM_OC_Start_IT(instance->htim, channel);
}

/* 停止指定通道的输出比较中断 */
void TIMOCStop_IT(TIMInstance *instance, uint32_t channel)
{
    /* 调用 HAL 停止输出比较中断 */
    HAL_TIM_OC_Stop_IT(instance->htim, channel);
}

/* 设置基本定时周期（微秒），根据当前 PSC 计算 ARR */
void TIMSetPeriodUs(TIMInstance *instance, uint32_t period_us)
{
    /* 计算当前 TIM 的计时内核时钟频率 */
    uint32_t clk = TIM_SelectClockHz(instance->htim);
    /* 读取当前预分频值（实际分频为 PSC+1） */
    uint32_t psc = instance->htim->Init.Prescaler + 1U;
    /* 根据公式 arr = period_us * clk / ((psc) * 1e6) 计算自动重装值 */
    uint64_t arr = ((uint64_t)period_us * (uint64_t)clk) / ((uint64_t)psc * 1000000ULL);
    /* 设置自动重装寄存器以更新周期 */
    __HAL_TIM_SET_AUTORELOAD(instance->htim, (uint32_t)arr);
    /* 复位计数器，从零开始新周期 */
    __HAL_TIM_SET_COUNTER(instance->htim, 0U);
}

/* 计数器读写接口 */
uint32_t TIMGetCounter(TIMInstance *instance)
{
    /* 返回计数器当前值 */
    return __HAL_TIM_GET_COUNTER(instance->htim);
}

/* 设置计数器数值（用于对齐或复位） */
void TIMSetCounter(TIMInstance *instance, uint32_t cnt)
{
    /* 写入新的计数器值 */
    __HAL_TIM_SET_COUNTER(instance->htim, cnt);
}

/********************************HAL回调分发***********************************/

/* HAL 更新事件回调：将事件分发到已注册实例的 period_callback */
void BSP_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 遍历所有实例，匹配产生回调的硬件句柄 */
    for (uint8_t i = 0; i < tim_idx; i++)
    {
        /* 命中对应实例且存在回调时，调用并返回 */
        if (tim_instances[i]->htim == htim && tim_instances[i]->period_callback)
        {
            /* 调用周期更新事件回调（模块内实现具体逻辑） */
            tim_instances[i]->period_callback(tim_instances[i]);
            /* 一次仅分发到命中实例，返回避免重复处理 */
            return;
        }
    }
}

/* HAL 输入捕获回调：将事件分发到 ic_callback 并携带通道标识 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* 遍历所有实例，匹配产生回调的硬件句柄 */
    for (uint8_t i = 0; i < tim_idx; i++)
    {
        /* 命中对应实例且存在回调时，调用并返回 */
        if (tim_instances[i]->htim == htim && tim_instances[i]->ic_callback)
        {
            /* 调用输入捕获回调，传入被触发的通道 */
            tim_instances[i]->ic_callback(tim_instances[i], htim->Channel);
            /* 一次仅分发到命中实例，返回避免重复处理 */
            return;
        }
    }
}

/* HAL 输出比较回调：将事件分发到 oc_callback 并携带通道标识 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 遍历所有实例，匹配产生回调的硬件句柄 */
    for (uint8_t i = 0; i < tim_idx; i++)
    {
        /* 命中对应实例且存在回调时，调用并返回 */
        if (tim_instances[i]->htim == htim && tim_instances[i]->oc_callback)
        {
            /* 调用输出比较回调，传入被触发的通道 */
            tim_instances[i]->oc_callback(tim_instances[i], htim->Channel);
            /* 一次仅分发到命中实例，返回避免重复处理 */
            return;
        }
    }
}

/******************************** 文件结束 ************************************/