/*******************************************************************************

                      版权所有 (C), 2025-2026, NCU_Roboteam

 *******************************************************************************
  文 件 名: bsp_usart.h
  版 本 号: 20251103001
  作    者: 
  生成日期: 2025年11月03日
  功能描述: USART串口通信函数库头文件，提供串口通信的基本操作接口
  补    充: 支持STM32H723芯片的USART操作，包含串口实例管理、数据收发、工作模式设置等功能

*******************************************************************************/

/****************************头文件开头必备内容********************************/

#ifndef BSP_RC_H
#define BSP_RC_H

/********************************包含头文件************************************/

#include <stdint.h>
#include "main.h"

/*******************************文件内宏定义***********************************/

/*MC02串口数量定义，共8个串口*/
#define DEVICE_USART_CNT 8
/*串口接收缓冲区大小限制，如果协议需要更大的缓冲区，请修改这里*/
#define USART_RXBUFF_LIMIT 256

/*******************************文件内结构体***********************************/

/* 模块回调函数定义，用于解析协议 */
typedef void (*usart_module_callback)();

/* USART传输模式枚举定义 */
typedef enum
{
    USART_TRANSFER_NONE = 0, /* 无传输模式 */
    USART_TRANSFER_BLOCKING, /* 阻塞传输模式 */
    USART_TRANSFER_IT,       /* 中断传输模式 */
    USART_TRANSFER_DMA,      /* DMA传输模式 */
} USART_TRANSFER_MODE;

/* 串口实例结构体定义，每个module都要包含一个实例 */
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; /* 预先定义的最大缓冲区大小 */
    uint8_t recv_buff_size;                /* 模块接收一包数据的大小 */
    UART_HandleTypeDef *usart_handle;      /* 实例对应的USART句柄 */
    usart_module_callback module_callback; /* 解析收到的数据的回调函数 */
} USARTInstance;

/* USART初始化配置结构体定义 */
typedef struct
{
    uint8_t recv_buff_size;                /* 模块接收一包数据的大小 */
    UART_HandleTypeDef *usart_handle;      /* 实例对应的USART句柄 */
    usart_module_callback module_callback; /* 解析收到的数据的回调函数 */
} USART_Init_Config_s;

/*******************************全局变量声明***********************************/

/********************************自定义函数************************************/

/**********************************************************************
  * @ 函数名  ： USARTRegister
  * @ 功能说明： 注册一个串口实例，返回一个串口实例指针
  * @ 参数    ： USART_Init_Config_s *init_config：串口初始化配置结构体指针
  * @ 返回值  ： USARTInstance*：返回串口实例指针，之后通过该指针操作串口外设
  * @ 涉及变量： USART实例数组、实例索引
  ********************************************************************/
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**********************************************************************
  * @ 函数名  ： USARTServiceInit
  * @ 功能说明： 启动串口服务，需要传入一个USART实例，一般用于lost callback的情况
  * @ 参数    ： USARTInstance *_instance：串口实例指针
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void USARTServiceInit(USARTInstance *_instance);

/**********************************************************************
  * @ 函数名  ： USARTSend
  * @ 功能说明： 通过调用该函数可以发送一帧数据，需要传入串口实例、发送缓冲区以及数据长度
  * @ 参数    ： USARTInstance *_instance：串口实例指针
  *              uint8_t *send_buf：待发送数据的缓冲区
  *              uint16_t send_size：发送数据的字节数
  *              USART_TRANSFER_MODE mode：传输模式
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  * @ 注意事项： 在短时间内连续调用此接口，若采用IT/DMA会导致上一次的发送未完成而新的发送取消
  ********************************************************************/
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode);

/**********************************************************************
  * @ 函数名  ： USARTIsReady
  * @ 功能说明： 判断串口是否准备好，用于连续或异步的IT/DMA发送
  * @ 参数    ： USARTInstance *_instance：要判断的串口实例指针
  * @ 返回值  ： uint8_t：准备好返回1，忙碌返回0
  * @ 涉及变量： 无
  ********************************************************************/
uint8_t USARTIsReady(USARTInstance *_instance);

#endif /* BSP_RC_H */

/******************************** 文件结束 ************************************/
