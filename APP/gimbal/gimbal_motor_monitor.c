#include "gimbal_motor_monitor.h"

#include <stdio.h>
#include <string.h>

#include "bsp_can.h"
#include "cmsis_os.h"
#include "usart.h"

#define GIMBAL_MOTOR_ID 5U
#define GIMBAL_M3508_RX_ID (0x200U + GIMBAL_MOTOR_ID)
#define GIMBAL_ECD_TO_DEG (360.0f / 8192.0f)
#define GIMBAL_UART_SEND_INTERVAL_MS 20U

typedef struct
{
    CANInstance *can_instance;
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t current;
    uint8_t temperature;
    float angle_deg;
    uint8_t has_feedback;
    volatile uint8_t new_feedback;
} GimbalMotorMonitor_s;

static GimbalMotorMonitor_s gimbal_motor_monitor = {0};
static char gimbal_uart_tx_buffer[96];
static uint32_t gimbal_last_tx_tick = 0U;

static void GimbalMotorMonitor_Decode(CANInstance *instance)
{
    const uint8_t *rx;

    if (instance == NULL || instance->rx_len < 7U)
    {
        return;
    }

    rx = instance->rx_buff;
    gimbal_motor_monitor.ecd = ((uint16_t)rx[0] << 8) | rx[1];
    gimbal_motor_monitor.speed_rpm = (int16_t)(((uint16_t)rx[2] << 8) | rx[3]);
    gimbal_motor_monitor.current = (int16_t)(((uint16_t)rx[4] << 8) | rx[5]);
    gimbal_motor_monitor.temperature = rx[6];
    gimbal_motor_monitor.angle_deg = (float)gimbal_motor_monitor.ecd * GIMBAL_ECD_TO_DEG;
    gimbal_motor_monitor.has_feedback = 1U;
    gimbal_motor_monitor.new_feedback = 1U;
}

void GimbalMotorMonitor_Init(void)
{
    CAN_Init_Config_s config;

    if (gimbal_motor_monitor.can_instance != NULL)
    {
        return;
    }

    memset(&config, 0, sizeof(config));
    config.can_handle = &hcan1;
    config.tx_id = GIMBAL_MOTOR_ID;
    config.rx_id = GIMBAL_M3508_RX_ID;
    config.can_module_callback = GimbalMotorMonitor_Decode;
    config.id = &gimbal_motor_monitor;

    gimbal_motor_monitor.can_instance = CANRegister(&config);
}

void GimbalMotorMonitor_Task(void)
{
    uint32_t now = osKernelSysTick();
    int len;
    uint16_t tx_len;

    if (!gimbal_motor_monitor.new_feedback)
    {
        return;
    }

    if ((now - gimbal_last_tx_tick) < GIMBAL_UART_SEND_INTERVAL_MS)
    {
        return;
    }

    if (huart8.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    gimbal_motor_monitor.new_feedback = 0U;
    len = snprintf(gimbal_uart_tx_buffer,
                   sizeof(gimbal_uart_tx_buffer),
                   "GIMBAL,id=%u,angle_deg=%.2f,ecd=%u\r\n",
                   (unsigned int)GIMBAL_MOTOR_ID,
                   (double)gimbal_motor_monitor.angle_deg,
                   (unsigned int)gimbal_motor_monitor.ecd);

    if (len <= 0)
    {
        return;
    }

    tx_len = (len < (int)sizeof(gimbal_uart_tx_buffer)) ?
             (uint16_t)len :
             (uint16_t)(sizeof(gimbal_uart_tx_buffer) - 1U);

    if (HAL_UART_Transmit_DMA(&huart8, (uint8_t *)gimbal_uart_tx_buffer, tx_len) == HAL_OK)
    {
        gimbal_last_tx_tick = now;
    }
}

float GimbalMotorMonitor_GetAngleDeg(void)
{
    return gimbal_motor_monitor.angle_deg;
}

uint16_t GimbalMotorMonitor_GetEcd(void)
{
    return gimbal_motor_monitor.ecd;
}

uint8_t GimbalMotorMonitor_HasFeedback(void)
{
    return gimbal_motor_monitor.has_feedback;
}
