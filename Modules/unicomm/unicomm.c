#include "unicomm.h"

#include <string.h>

#include "bsp_usart.h"
#include "gpio.h"
#include "usart.h"

#define UNICOMM_RX_BUFFER_SIZE 255U
#define UNICOMM_FRAME_SIZE 32U
#define UNICOMM_STATE_SIZE 9U
#define UNICOMM_END_INDEX 31U
#define UNICOMM_END_FRAME 0xEDU
#define UNICOMM_MEASURING_STATE "MEASURING"
#define UNICOMM_ACTIONING_STATE "ACTIONING"
#define UNICOMM_ACTION_RESERVED_TAG 0x0FU
#define UNICOMM_TAG_COUNT 8U
#define UNICOMM_CM_PER_METER 100.0f

float current_x = 0.0f;
float current_y = 0.0f;
float target_x = 0.0f;
float target_y = 0.0f;
float upstream_current_yaw = 0.0f;
float upstream_goal_yaw = 0.0f;
uint8_t nav_current_valid = 0U;
uint8_t nav_goal_valid = 0U;

UniCommStage_e unicomm_latest_stage = UNICOMM_STAGE_NONE;
uint32_t unicomm_frame_seq = 0U;
uint8_t unicomm_current_tag_id = 0U;
uint8_t unicomm_has_current_tag = 0U;
float unicomm_x_m = 0.0f;
float unicomm_y_m = 0.0f;
float unicomm_bearing_deg = 0.0f;
float unicomm_yaw_deg = 0.0f;
float unicomm_margin = 0.0f;
char unicomm_action_code[UNICOMM_ACTION_CODE_MAX_LEN + 1U] = {0};

static USARTInstance *unicomm_usart_instance = NULL;
static uint8_t unicomm_frame_buffer[UNICOMM_FRAME_SIZE];
static size_t unicomm_frame_length = 0U;

static const uint16_t unicomm_pg_pins[UNICOMM_TAG_COUNT] = {
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
};

static void UniComm_SetAllStagePins(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOG,
                      GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                          GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8,
                      state);
}

static void UniComm_SetActionPin(uint8_t tag_id)
{
    UniComm_SetAllStagePins(GPIO_PIN_RESET);

    if (tag_id < UNICOMM_TAG_COUNT)
    {
        HAL_GPIO_WritePin(GPIOG, unicomm_pg_pins[tag_id], GPIO_PIN_SET);
    }
}

static void UniComm_SetMeasuringPin(uint8_t tag_id)
{
    UniComm_SetAllStagePins(GPIO_PIN_SET);

    if (tag_id < UNICOMM_TAG_COUNT)
    {
        HAL_GPIO_WritePin(GPIOG, unicomm_pg_pins[tag_id], GPIO_PIN_RESET);
    }
}

static uint8_t UniComm_IsKnownState(const uint8_t *frame)
{
    return memcmp(frame, UNICOMM_MEASURING_STATE, UNICOMM_STATE_SIZE) == 0 ||
           memcmp(frame, UNICOMM_ACTIONING_STATE, UNICOMM_STATE_SIZE) == 0;
}

static uint8_t UniComm_IsValidFrame(const uint8_t *frame)
{
    if (frame[UNICOMM_END_INDEX] != UNICOMM_END_FRAME)
    {
        return 0U;
    }

    return UniComm_IsKnownState(frame);
}

static float UniComm_ReadFloatLE(const uint8_t *data)
{
    float value;
    uint8_t raw[sizeof(float)];

    raw[0] = data[0];
    raw[1] = data[1];
    raw[2] = data[2];
    raw[3] = data[3];
    memcpy(&value, raw, sizeof(value));

    return value;
}

static void UniComm_HandleMeasuringFrame(const uint8_t *frame)
{
    uint8_t tag_id = frame[9];

    if (tag_id >= UNICOMM_TAG_COUNT)
    {
        return;
    }

    unicomm_latest_stage = UNICOMM_STAGE_MEASURING;
    unicomm_frame_seq++;
    unicomm_current_tag_id = tag_id;
    unicomm_has_current_tag = 1U;
    unicomm_x_m = UniComm_ReadFloatLE(&frame[10]);
    unicomm_y_m = UniComm_ReadFloatLE(&frame[14]);
    unicomm_bearing_deg = UniComm_ReadFloatLE(&frame[18]);
    unicomm_yaw_deg = UniComm_ReadFloatLE(&frame[22]);
    unicomm_margin = UniComm_ReadFloatLE(&frame[26]);

    current_x = unicomm_x_m * UNICOMM_CM_PER_METER;
    current_y = unicomm_y_m * UNICOMM_CM_PER_METER;
    target_x = 0.0f;
    target_y = 0.0f;
    upstream_current_yaw = unicomm_yaw_deg;
    upstream_goal_yaw = unicomm_bearing_deg;
    nav_current_valid = 1U;
    nav_goal_valid = 1U;

    UniComm_SetMeasuringPin(tag_id);
}

static void UniComm_HandleActioningFrame(const uint8_t *frame)
{
    size_t action_len = UNICOMM_ACTION_CODE_MAX_LEN;

    if (frame[9] != UNICOMM_ACTION_RESERVED_TAG)
    {
        return;
    }

    memcpy(unicomm_action_code, &frame[10], UNICOMM_ACTION_CODE_MAX_LEN);
    while (action_len > 0U && unicomm_action_code[action_len - 1U] == '-')
    {
        --action_len;
    }
    unicomm_action_code[action_len] = '\0';
    unicomm_latest_stage = UNICOMM_STAGE_ACTIONING;
    unicomm_frame_seq++;

    if (unicomm_has_current_tag)
    {
        UniComm_SetActionPin(unicomm_current_tag_id);
    }
    else
    {
        UniComm_SetAllStagePins(GPIO_PIN_RESET);
    }
}

static void UniComm_ProcessFrame(const uint8_t *frame)
{
    if (memcmp(frame, UNICOMM_MEASURING_STATE, UNICOMM_STATE_SIZE) == 0)
    {
        UniComm_HandleMeasuringFrame(frame);
        return;
    }

    if (memcmp(frame, UNICOMM_ACTIONING_STATE, UNICOMM_STATE_SIZE) == 0)
    {
        UniComm_HandleActioningFrame(frame);
    }
}

static void UniComm_ProcessIncomingBytes(const uint8_t *buffer, size_t data_len)
{
    size_t i;

    for (i = 0U; i < data_len; ++i)
    {
        if (unicomm_frame_length < UNICOMM_FRAME_SIZE)
        {
            unicomm_frame_buffer[unicomm_frame_length++] = buffer[i];
        }
        else
        {
            memmove(&unicomm_frame_buffer[0],
                    &unicomm_frame_buffer[1],
                    UNICOMM_FRAME_SIZE - 1U);
            unicomm_frame_buffer[UNICOMM_FRAME_SIZE - 1U] = buffer[i];
        }

        if (unicomm_frame_length < UNICOMM_FRAME_SIZE)
        {
            continue;
        }

        if (UniComm_IsValidFrame(unicomm_frame_buffer))
        {
            UniComm_ProcessFrame(unicomm_frame_buffer);
            unicomm_frame_length = 0U;
        }
    }
}

static void UniComm_UART7_Callback(void)
{
    if (unicomm_usart_instance == NULL)
    {
        return;
    }

    UniComm_ProcessIncomingBytes(unicomm_usart_instance->recv_buff,
                                 unicomm_usart_instance->recv_len);
}

void UniComm_UART7_Init(void)
{
    USART_Init_Config_s unicomm_usart_conf = {
        .recv_buff_size = UNICOMM_RX_BUFFER_SIZE,
        .usart_handle = &huart7,
        .module_callback = UniComm_UART7_Callback,
    };

    unicomm_usart_instance = USARTRegister(&unicomm_usart_conf);
}
