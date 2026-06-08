/**
*******************************************************************************

 *******************************************************************************
  文 件 名: daemon.h
  版 本 号: 20251120+01
  生成日期: 2025年11月20日
  功能描述: 守护模块对外接口定义，提供喂狗、在线检测与离线回调机制
  补    充: 无

*******************************************************************************/

/****************************头文件开头必备内容********************************/
/* 头文件防重复包含保护开始 */
#ifndef MONITOR_H
#define MONITOR_H
/* 标准整型类型定义 */
#include "stdint.h"
/* 字符串/内存操作接口 */
#include "string.h"
/* 守护实例最大数量定义 */
#define DAEMON_MX_CNT 64
/* 模块离线处理函数指针类型定义 */
/* 模块离线处理函数指针 */
typedef void (*offline_callback)(void *);
/* 守护实例结构体定义 */
/* daemon结构体定义 */
typedef struct daemon_ins
{
    /* 重载值：喂狗后计数器设置的目标值 */
    uint16_t reload_count;     // 重载值
    /* 异常/离线处理回调函数指针 */
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用
    /* 当前计数值，减为零说明模块离线或异常 */
    uint16_t temp_count; // 当前值,减为零说明模块离线或异常
    /* 守护实例的拥有者ID（通常为模块实例地址） */
    void *owner_id;      // daemon实例的地址,初始化的时候填入
} DaemonInstance;
/* 守护实例初始化配置结构体定义 */
/* daemon初始化配置 */
typedef struct
{
    /* 喂狗时的重载计数值 */
    uint16_t reload_count;     // 实际上这是app唯一需要设置的值?
    /* 初始上线等待计数，用于某些需要准备时间的模块 */
    uint16_t init_count;       // 上线等待时间,有些模块需要收到主控的指令才会反馈报文,或pc等需要开机时间
    /* 异常/离线处理回调函数指针 */
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用
    /* 拥有者ID（模块实例地址），用于在回调中识别具体实例 */
    void *owner_id;            // id取拥有daemon的实例的地址,如DJIMotorInstance*,cast成void*类型
} Daemon_Init_Config_s;

/**
 * @brief 注册一个daemon实例
 *
 * @param config 初始化配置
 * @return DaemonInstance* 返回实例指针
 */
/* 注册守护实例接口声明 */
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);

/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载temp_count,相当于"喂狗"
 *
 * @param instance daemon实例指针
 */
/* 喂狗接口声明：重载计数以表示模块在线 */
void DaemonReload(DaemonInstance *instance);

/**
 * @brief 确认模块是否离线
 *
 * @param instance daemon实例指针
 * @return uint8_t 若在线且工作正常,返回1;否则返回零. 后续根据异常类型和离线状态等进行优化.
 */
/* 在线状态检查接口声明 */
uint8_t DaemonIsOnline(DaemonInstance *instance);

/**
 * @brief 放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
/* 守护任务接口声明：周期性递减与超时回调触发 */
void DaemonTask();
/* 头文件防重复包含保护结束 */
#endif // !MONITOR_H