/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author modified by Neo with annotation
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 */

/* 包含 DWT BSP 头文件，声明类型与接口 */
#include "bsp_dwt.h"
/* 包含 CMSIS-OS 头文件，用于与 RTOS 协同工作 */
#include "cmsis_os.h"

/* 系统时间结构体，保存秒/ms/us 三段时间 */
static DWT_Time_t SysTime;
/* CPU 频率（Hz）及其 ms/us 级派生值 */
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
/* CYCCNT 溢出次数累计，用于扩展 64 位计数 */
static uint32_t CYCCNT_RountCount;
/* 上次读取的 CYCCNT 值，用于溢出判断与 dt 计算 */
static uint32_t CYCCNT_LAST;
/* 64 位扩展的累积周期计数，避免 32 位溢出影响时间轴 */
static uint64_t CYCCNT64;

/**
 * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 *
 * @todo 更好的方案是为dwt的时间更新单独设置一个任务?
 *       不过,使用dwt的初衷是定时不被中断/任务等因素影响,因此该实现仍然有其存在的意义
 *
 */
/*
**********************************************************************
  * @ 函数名  ： DWT_CNT_Update
  * @ 功能说明： 检查 DWT CYCCNT 溢出并更新轮次计数与最后值
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： CYCCNT_RountCount、CYCCNT_LAST
********************************************************************/
/*
***************************函数专属变量******************************
*/
static void DWT_CNT_Update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        bit_locker = 0;
    }
}

/*
**********************************************************************
  * @ 函数名  ： DWT_Init
  * @ 功能说明： 初始化 DWT 计数器，配置 CPU 频率参数与计数开关
  * @ 参数    ： CPU_Freq_mHz  CPU 主频（MHz）
  * @ 返回值  ： 无
  * @ 涉及变量： CPU_FREQ_Hz/ms/us、CYCCNT_RountCount、CYCCNT_LAST
********************************************************************/
/*
***************************函数专属变量******************************
*/
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

    DWT_CNT_Update();
}

/*
**********************************************************************
  * @ 函数名  ： DWT_GetDeltaT
  * @ 功能说明： 计算两次调用之间的时间间隔（秒，float）
  * @ 参数    ： cnt_last 上次计数（CYCCNT）
  * @ 返回值  ： float 间隔时间（s）
  * @ 涉及变量： CPU_FREQ_Hz、CYCCNT_LAST
********************************************************************/
/*
***************************函数专属变量******************************
*/
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/*
**********************************************************************
  * @ 函数名  ： DWT_GetDeltaT64
  * @ 功能说明： 计算两次调用之间的时间间隔（秒，double 高精度）
  * @ 参数    ： cnt_last 上次计数（CYCCNT）
  * @ 返回值  ： double 间隔时间（s）
  * @ 涉及变量： CPU_FREQ_Hz、CYCCNT_LAST
********************************************************************/
/*
***************************函数专属变量******************************
*/
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/*
**********************************************************************
  * @ 函数名  ： DWT_SysTimeUpdate
  * @ 功能说明： 刷新系统时间轴（秒/ms/us），处理 CYCCNT 溢出扩展
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： CYCCNT64、SysTime（s/ms/us）、CPU_FREQ 派生值
********************************************************************/
/*
***************************函数专属变量******************************
*/
void DWT_SysTimeUpdate(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

/*
**********************************************************************
  * @ 函数名  ： DWT_GetTimeline_s
  * @ 功能说明： 获取当前时间轴（秒，float）
  * @ 参数    ： 无
  * @ 返回值  ： float 时间轴（s）
  * @ 涉及变量： SysTime（s/ms/us）
********************************************************************/
/*
***************************函数专属变量******************************
*/
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

/*
**********************************************************************
  * @ 函数名  ： DWT_GetTimeline_ms
  * @ 功能说明： 获取当前时间轴（毫秒，float）
  * @ 参数    ： 无
  * @ 返回值  ： float 时间轴（ms）
  * @ 涉及变量： SysTime（s/ms/us）
********************************************************************/
/*
***************************函数专属变量******************************
*/
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

/*
**********************************************************************
  * @ 函数名  ： DWT_GetTimeline_us
  * @ 功能说明： 获取当前时间轴（微秒，uint64_t）
  * @ 参数    ： 无
  * @ 返回值  ： uint64_t 时间轴（us）
  * @ 涉及变量： SysTime（s/ms/us）、CYCCNT64
********************************************************************/
/*
***************************函数专属变量******************************
*/
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

/*
**********************************************************************
  * @ 函数名  ： DWT_Delay
  * @ 功能说明： 基于 CYCCNT 的高精度延时（秒，float）
  * @ 参数    ： Delay 延时时间（s）
  * @ 返回值  ： 无
  * @ 涉及变量： CPU_FREQ_Hz
********************************************************************/
/*
***************************函数专属变量******************************
*/
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}

