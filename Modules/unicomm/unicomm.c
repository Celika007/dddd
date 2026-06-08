#include "unicomm.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "action.h"
#include "bsp_usart.h"
#include "usart.h"

#define UNICOMM_RX_BUFFER_SIZE 64U
#define UNICOMM_ARRIVAL_FRAME "RCArrivalMX"

int32_t current_x = 0;
int32_t current_y = 0;
int32_t target_x = 0;
int32_t target_y = 0;

static USARTInstance *unicomm_usart_instance = NULL;

static size_t UniComm_BoundedStrnLen(const char *str, size_t max_len)
{
    size_t len = 0;

    while (len < max_len && str[len] != '\0')
    {
        ++len;
    }

    return len;
}

static int UniComm_ExtractFramePayload(const uint8_t *buffer,
                                       size_t buffer_size,
                                       const char *header,
                                       const char *tail,
                                       char *payload,
                                       size_t payload_size)
{
    size_t data_len = UniComm_BoundedStrnLen((const char *)buffer, buffer_size);
    size_t header_len = strlen(header);
    size_t tail_len = strlen(tail);
    size_t i = 0;

    if (payload_size == 0U || data_len < (header_len + tail_len))
    {
        return 0;
    }

    while ((i + header_len) <= data_len)
    {
        if (memcmp(&buffer[i], header, header_len) == 0)
        {
            size_t payload_start = i + header_len;
            size_t j = payload_start;

            while ((j + tail_len) <= data_len)
            {
                if (memcmp(&buffer[j], tail, tail_len) == 0)
                {
                    size_t payload_len = j - payload_start;

                    if ((payload_len + 1U) > payload_size)
                    {
                        return 0;
                    }

                    memcpy(payload, &buffer[payload_start], payload_len);
                    payload[payload_len] = '\0';
                    return 1;
                }

                ++j;
            }

            return 0;
        }

        ++i;
    }

    return 0;
}

static int UniComm_ParseInt32(const char **cursor, const char *prefix, int32_t *value)
{
    const size_t prefix_len = strlen(prefix);
    const char *start;
    char *end = NULL;
    long parsed;

    if (strncmp(*cursor, prefix, prefix_len) != 0)
    {
        return 0;
    }

    start = *cursor + prefix_len;
    parsed = strtol(start, &end, 10);
    if (end == start || parsed < INT32_MIN || parsed > INT32_MAX)
    {
        return 0;
    }

    *value = (int32_t)parsed;
    *cursor = end;
    return 1;
}

static int UniComm_ParsePositionPayload(const char *payload,
                                        int32_t *new_current_x,
                                        int32_t *new_current_y,
                                        int32_t *new_target_x,
                                        int32_t *new_target_y)
{
    const char *cursor = payload;

    if (!UniComm_ParseInt32(&cursor, "N{X:", new_current_x))
    {
        return 0;
    }

    if (!UniComm_ParseInt32(&cursor, ",Y:", new_current_y))
    {
        return 0;
    }

    if (!UniComm_ParseInt32(&cursor, "},T{X:", new_target_x))
    {
        return 0;
    }

    if (!UniComm_ParseInt32(&cursor, ",Y:", new_target_y))
    {
        return 0;
    }

    if (strcmp(cursor, "}") != 0)
    {
        return 0;
    }

    return 1;
}

static void UniComm_HandlePositionFrame(const char *payload)
{
    int32_t new_current_x;
    int32_t new_current_y;
    int32_t new_target_x;
    int32_t new_target_y;

    if (UniComm_ParsePositionPayload(payload,
                                     &new_current_x,
                                     &new_current_y,
                                     &new_target_x,
                                     &new_target_y))
    {
        current_x = new_current_x;
        current_y = new_current_y;
        target_x = new_target_x;
        target_y = new_target_y;
    }
}

static void UniComm_HandleActionFrame(const char *payload)
{
    if (strcmp(payload, "GRAB") == 0)
    {
        state = GRAB;
    }
    else if (strcmp(payload, "PUTDOWN") == 0)
    {
        state = STAND;
    }
}

static void UniComm_UART8_Callback(void)
{
    char payload[UNICOMM_RX_BUFFER_SIZE];

    if (unicomm_usart_instance == NULL)
    {
        return;
    }

    if (UniComm_ExtractFramePayload(unicomm_usart_instance->recv_buff,
                                    UNICOMM_RX_BUFFER_SIZE,
                                    "SI",
                                    "ZU",
                                    payload,
                                    sizeof(payload)))
    {
        UniComm_HandlePositionFrame(payload);
    }

    if (UniComm_ExtractFramePayload(unicomm_usart_instance->recv_buff,
                                    UNICOMM_RX_BUFFER_SIZE,
                                    "RC",
                                    "MX",
                                    payload,
                                    sizeof(payload)))
    {
        UniComm_HandleActionFrame(payload);
    }
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
