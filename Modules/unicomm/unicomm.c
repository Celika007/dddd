#include "unicomm.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "action.h"
#include "bsp_usart.h"
#include "usart.h"

#define UNICOMM_RX_BUFFER_SIZE 255U
#define UNICOMM_ARRIVAL_FRAME "RCArrivalMX"

float current_x = 0.0f;
float current_y = 0.0f;
float target_x = 0.0f;
float target_y = 0.0f;
float upstream_current_yaw = 0.0f;
float upstream_goal_yaw = 0.0f;
uint8_t nav_current_valid = 0U;
uint8_t nav_goal_valid = 0U;

static USARTInstance *unicomm_usart_instance = NULL;
static char unicomm_line_buffer[UNICOMM_RX_BUFFER_SIZE + 1U];
static size_t unicomm_line_length = 0U;
static uint8_t unicomm_line_overflow = 0U;

static const char *UniComm_FindFieldValue(const char *line, const char *key)
{
    const size_t key_len = strlen(key);
    const char *cursor = line;

    while ((cursor = strstr(cursor, key)) != NULL)
    {
        if (cursor == line || cursor[-1] == ';')
        {
            return cursor + key_len;
        }

        cursor += key_len;
    }

    return NULL;
}

static int UniComm_ParseIntField(const char *line, const char *key, int *value)
{
    const char *start = UniComm_FindFieldValue(line, key);
    char *end = NULL;
    long parsed;

    if (start == NULL)
    {
        return 0;
    }

    parsed = strtol(start, &end, 10);
    if (end == start || parsed < INT_MIN || parsed > INT_MAX)
    {
        return 0;
    }

    if (*end != ';' && *end != '\0')
    {
        return 0;
    }

    *value = (int)parsed;
    return 1;
}

static int UniComm_ParseFloatField(const char *line, const char *key, float *value)
{
    const char *start = UniComm_FindFieldValue(line, key);
    char *end = NULL;
    float parsed;

    if (start == NULL)
    {
        return 0;
    }

    parsed = strtof(start, &end);
    if (end == start || !isfinite(parsed))
    {
        return 0;
    }

    if (*end != ';' && *end != '\0')
    {
        return 0;
    }

    *value = parsed;
    return 1;
}

static void UniComm_HandleNavFrame(const char *line)
{
    int current_valid_value;
    int goal_valid_value;
    float new_current_x;
    float new_current_y;
    float new_target_x;
    float new_target_y;
    float new_upstream_current_yaw;
    float new_upstream_goal_yaw;

    if (!UniComm_ParseIntField(line, "cur_valid=", &current_valid_value) ||
        !UniComm_ParseIntField(line, "goal_valid=", &goal_valid_value) ||
        !UniComm_ParseFloatField(line, "cur_x=", &new_current_x) ||
        !UniComm_ParseFloatField(line, "cur_y=", &new_current_y) ||
        !UniComm_ParseFloatField(line, "goal_x=", &new_target_x) ||
        !UniComm_ParseFloatField(line, "goal_y=", &new_target_y) ||
        !UniComm_ParseFloatField(line, "cur_yaw=", &new_upstream_current_yaw) ||
        !UniComm_ParseFloatField(line, "goal_yaw=", &new_upstream_goal_yaw))
    {
        return;
    }

    nav_current_valid = current_valid_value != 0 ? 1U : 0U;
    nav_goal_valid = goal_valid_value != 0 ? 1U : 0U;
    current_x = new_current_x;
    current_y = new_current_y;
    target_x = new_target_x;
    target_y = new_target_y;
    upstream_current_yaw = new_upstream_current_yaw;
    upstream_goal_yaw = new_upstream_goal_yaw;
}

static void UniComm_ProcessLine(const char *line)
{
    if (line[0] == '\0')
    {
        return;
    }

    if (strcmp(line, "RCPickUpBoxes") == 0)
    {
        state = GRAB;
        return;
    }

    if (strncmp(line, "RCplace=", 8) == 0)
    {
        state = STAND;
        return;
    }

    if (strncmp(line, "RCNAV;", 6) == 0)
    {
        UniComm_HandleNavFrame(line);
    }
}

static void UniComm_ProcessIncomingBytes(const uint8_t *buffer, size_t data_len)
{
    size_t i;

    for (i = 0U; i < data_len; ++i)
    {
        char current_char = (char)buffer[i];

        if (current_char == '\r')
        {
            continue;
        }

        if (current_char == '\n')
        {
            if (!unicomm_line_overflow)
            {
                unicomm_line_buffer[unicomm_line_length] = '\0';
                UniComm_ProcessLine(unicomm_line_buffer);
            }

            unicomm_line_length = 0U;
            unicomm_line_overflow = 0U;
            continue;
        }

        if (unicomm_line_overflow)
        {
            continue;
        }

        if (unicomm_line_length >= UNICOMM_RX_BUFFER_SIZE)
        {
            unicomm_line_length = 0U;
            unicomm_line_overflow = 1U;
            continue;
        }

        unicomm_line_buffer[unicomm_line_length++] = current_char;
    }
}

static void UniComm_UART8_Callback(void)
{
    if (unicomm_usart_instance == NULL)
    {
        return;
    }

    if (unicomm_usart_instance->recv_len > 0U)
    {
        HAL_UART_Transmit(&huart7,
                          unicomm_usart_instance->recv_buff,
                          unicomm_usart_instance->recv_len,
                          20);
    }

    UniComm_ProcessIncomingBytes(unicomm_usart_instance->recv_buff,
                                 unicomm_usart_instance->recv_len);
}

void UniComm_UART8_Init(void)
{
    USART_Init_Config_s unicomm_usart_conf = {
        .recv_buff_size = UNICOMM_RX_BUFFER_SIZE,
        .usart_handle = &huart8,
        .module_callback = UniComm_UART8_Callback,
    };

    unicomm_usart_instance = USARTRegister(&unicomm_usart_conf);
}

void UniComm_SendArrival(void)
{
    static uint8_t arrival_frame[] = UNICOMM_ARRIVAL_FRAME;

    if (unicomm_usart_instance == NULL)
    {
        return;
    }

    USARTSend(unicomm_usart_instance,
              arrival_frame,
              (uint16_t)(sizeof(arrival_frame) - 1U),
              USART_TRANSFER_BLOCKING);
}
