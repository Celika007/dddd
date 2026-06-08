/*******************************************************************************

                      版权所有 (C), 2025-2026, NCU_Roboteam

 *******************************************************************************
  文 件 名: bsp_usart.c
  版 本 号: 20251103001
  作    者: 
  生成日期: 2025年11月03日
  功能描述: USART串口通信函数库，提供串口通信的基本操作实现
  补    充: 支持STM32H723芯片的USART操作，包含串口实例管理、数据收发、工作模式设置等功能

*******************************************************************************/

/********************************包含头文件************************************/

#include "bsp_usart.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"

/*******************************全局变量定义***********************************/

/********************************文件内变量************************************/

/* USART服务实例，所有注册了USART的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/********************************自定义函数************************************/

/**********************************************************************
  * @ 函数名  ： USARTServiceInit
  * @ 功能说明： 启动串口服务，会在每个实例注册之后自动启用接收，当前实现为DMA接收
  * @ 参数    ： USARTInstance *_instance：模块拥有的串口实例指针
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
/***************************函数专属变量******************************/

void USARTServiceInit(USARTInstance *_instance)
{
    /*启动DMA接收到空闲中断模式*/
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    /*关闭DMA半传输中断防止两次进入HAL_UARTEx_RxEventCallback()*/
    /*这是HAL库的一个设计失误，发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()*/
    /*我们只希望处理第一种和第三种情况，因此直接关闭DMA半传输中断*/
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

/**********************************************************************
  * @ 函数名  ： USARTRegister
  * @ 功能说明： 注册一个串口实例，返回一个串口实例指针
  * @ 参数    ： USART_Init_Config_s *init_config：串口初始化配置结构体指针
  * @ 返回值  ： USARTInstance*：返回串口实例指针，之后通过该指针操作串口外设
  * @ 涉及变量： usart_instance数组、idx索引
  ********************************************************************/
/***************************函数专属变量******************************/

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    /*检查是否超过最大实例数*/
    if (idx >= DEVICE_USART_CNT)
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    /*检查是否已经注册过相同的串口句柄*/
    for (uint8_t i = 0; i < idx; i++)
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    /*分配内存并初始化串口实例*/
    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    /*配置串口实例参数*/
    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    /*将实例添加到实例数组中并启动串口服务*/
    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/**********************************************************************
  * @ 函数名  ： USARTSend
  * @ 功能说明： 通过调用该函数可以发送一帧数据，需要传入串口实例、发送缓冲区以及数据长度
  * @ 参数    ： USARTInstance *_instance：串口实例指针
  *              uint8_t *send_buf：待发送数据的缓冲区
  *              uint16_t send_size：发送数据的字节数
  *              USART_TRANSFER_MODE mode：传输模式
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
/***************************函数专属变量******************************/

void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    /*根据传输模式选择相应的发送方式*/
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        /*阻塞模式发送，超时时间100ms*/
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        /*中断模式发送*/
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        /*DMA模式发送*/
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        /*非法模式，检查代码上下文，可能出现指针越界*/
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/**********************************************************************
  * @ 函数名  ： USARTIsReady
  * @ 功能说明： 判断串口是否准备好，用于连续或异步的IT/DMA发送
  * @ 参数    ： USARTInstance *_instance：要判断的串口实例指针
  * @ 返回值  ： uint8_t：准备好返回1，忙碌返回0
  * @ 涉及变量： 无
  ********************************************************************/
/***************************函数专属变量******************************/

uint8_t USARTIsReady(USARTInstance *_instance)
{
    /*串口发送时，gstate会被设为BUSY_TX*/
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

/**********************************************************************
  * @ 函数名  ： HAL_UARTEx_RxEventCallback
  * @ 功能说明： 每次DMA/IDLE中断发生时，都会调用此函数，对于每个UART实例会调用对应的回调进行进一步的处理
  * @ 参数    ： UART_HandleTypeDef *huart：发生中断的串口句柄
  *              uint16_t Size：此次接收到的总数据量
  * @ 返回值  ： 无
  * @ 涉及变量： usart_instance数组、idx索引
  ********************************************************************/
/***************************函数专属变量******************************/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /*遍历所有串口实例，找到对应的实例*/
    for (uint8_t i = 0; i < idx; ++i)
    {
        /*找到正在处理的实例*/
        if (huart == usart_instance[i]->usart_handle)
        {
            /*如果回调函数不为空，则调用回调函数*/
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                /*接收结束后清空缓冲区，对于变长数据是必要的*/
                memset(usart_instance[i]->recv_buff, 0, Size);
            }
            /*重新启动DMA接收*/
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            /*关闭DMA半传输中断*/
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
}

/**********************************************************************
  * @ 函数名  ： HAL_UART_ErrorCallback
  * @ 功能说明： 当串口发送/接收出现错误时，会调用此函数，此时这个函数要做的就是重新启动接收
  * @ 参数    ： UART_HandleTypeDef *huart：发生错误的串口句柄
  * @ 返回值  ： 无
  * @ 涉及变量： usart_instance数组、idx索引
  ********************************************************************/
/***************************函数专属变量******************************/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /*遍历所有串口实例，找到对应的实例*/
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            /*重新启动DMA接收*/
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            /*关闭DMA半传输中断*/
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            /*记录警告信息*/
            LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
}

/******************************** 文件结束 ************************************/