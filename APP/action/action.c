#include "action.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "IMU.h"
#include "cmsis_os.h"
#include "unicomm.h"
#include "usart.h"

#define ACTION_SETTLE_MS 500U
#define ACTION_WALK_STEP_MS 500U
#define ACTION_TURN_30_MS 300U
#define ACTION_TURN_60_MS 600U
#define ACTION_TURN_120_MS 1200U
#define ACTION_TURN_255_MS 2550U
#define ACTION_DEFAULT_RUN_MS 1500U
#define ACTION_DEBUG_RX_BUFFER_SIZE 128U
#define ACTION_DEBUG_LINE_BUFFER_SIZE 128U
#define ACTION_DEBUG_LOG_MIN_INTERVAL_MS 100U
#define ACTION_DEBUG_LOG_MAX_INTERVAL_MS 1000U
#define ACTION_DEBUG_STEP_LENGTH_DELTA 1.0f
#define ACTION_DEBUG_AMP_DELTA 0.1f
#define ACTION_DEBUG_STANCE_HEIGHT_DELTA 1.0f
#define ACTION_DEBUG_FREQ_DELTA 0.1f
#define ACTION_DEBUG_FLIGHT_PERCENT_DELTA 0.05f

typedef enum
{
    ACTION_SERIAL_BOOT_STAND = 0,
    ACTION_SERIAL_APPROACH,
    ACTION_SERIAL_SETTLE,
    ACTION_SERIAL_EXECUTE,
    ACTION_SERIAL_DONE,
} ActionSerialPhase_e;

typedef enum
{
    ACTION_SCRIPT_STAND = 0,
    ACTION_SCRIPT_RUN,
    ACTION_SCRIPT_LEFT_TURN,
    ACTION_SCRIPT_RIGHT_TURN,
} ActionScriptState_e;

typedef enum
{
    ACTION_DEBUG_PARAM_STEP_LENGTH = 0,
    ACTION_DEBUG_PARAM_UP_AMP,
    ACTION_DEBUG_PARAM_DOWN_AMP,
    ACTION_DEBUG_PARAM_STANCE_HEIGHT,
    ACTION_DEBUG_PARAM_FREQ,
    ACTION_DEBUG_PARAM_FLIGHT_PERCENT,
} ActionDebugParam_e;

typedef struct
{
    ActionScriptState_e state;
    uint32_t duration_ms;
} ActionScriptStep_s;

DetachedParam detached_params;
float target_yaw = 0.0f;

static ActionSerialPhase_e action_serial_phase = ACTION_SERIAL_BOOT_STAND;
static uint32_t action_seen_frame_seq = 0U;
static uint8_t action_expected_tag_id = 0U;
static uint8_t action_active_tag_id = 0U;
static TickType_t action_phase_start_tick = 0U;
static const ActionScriptStep_s *action_script = NULL;
static uint8_t action_script_len = 0U;
static uint8_t action_script_index = 0U;
static char action_active_code[UNICOMM_ACTION_CODE_MAX_LEN + 1U] = {0};

#if ACTION_RECORD_UART8_ENABLE
static USARTInstance *action_debug_uart8_instance = NULL;
static char action_debug_line[ACTION_DEBUG_LINE_BUFFER_SIZE];
static size_t action_debug_line_len = 0U;
static char action_record_tx_buffer[ACTION_RECORD_TX_BUF_SIZE];
static uint32_t action_walk_done = 0U;
static float action_last_base_phase = 0.0f;
static uint8_t action_walk_phase_valid = 0U;
static TickType_t action_record_last_tx_tick = 0U;
static uint8_t action_debug_hold_realse = 0U;
static float action_imu_base_roll = 0.0f;
static float action_imu_base_pitch = 0.0f;
static float action_imu_base_yaw = 0.0f;
#endif

static const ActionScriptStep_s action_script_weave[] = {
    {ACTION_SCRIPT_RIGHT_TURN, ACTION_TURN_60_MS},
    {ACTION_SCRIPT_RUN, ACTION_WALK_STEP_MS * 4U},
    {ACTION_SCRIPT_RIGHT_TURN, ACTION_TURN_120_MS},
    {ACTION_SCRIPT_RUN, ACTION_WALK_STEP_MS * 8U},
    {ACTION_SCRIPT_LEFT_TURN, ACTION_TURN_120_MS},
    {ACTION_SCRIPT_RUN, ACTION_WALK_STEP_MS * 8U},
    {ACTION_SCRIPT_RIGHT_TURN, ACTION_TURN_255_MS},
    {ACTION_SCRIPT_RUN, ACTION_WALK_STEP_MS * 3U},
    {ACTION_SCRIPT_RIGHT_TURN, ACTION_TURN_255_MS},
    {ACTION_SCRIPT_RUN, ACTION_WALK_STEP_MS * 8U},
    {ACTION_SCRIPT_LEFT_TURN, ACTION_TURN_120_MS},
    {ACTION_SCRIPT_RUN, ACTION_WALK_STEP_MS * 4U},
    {ACTION_SCRIPT_RIGHT_TURN, ACTION_TURN_30_MS},
    {ACTION_SCRIPT_STAND, ACTION_SETTLE_MS},
};

static const ActionScriptStep_s action_script_default[] = {
    {ACTION_SCRIPT_RUN, ACTION_DEFAULT_RUN_MS},
    {ACTION_SCRIPT_STAND, ACTION_SETTLE_MS},
};

static const ActionScriptStep_s action_script_stand_only[] = {
    {ACTION_SCRIPT_STAND, ACTION_SETTLE_MS},
};

DetachedParam state_detached_params[] =
{   // {stance_height, step_length, up_amp, down_amp, flight_percent, freq}
    { // RUN
        {18, 12, 6, 0.0, 0.4, 1.8},
        {18, 12, 6, 0.0, 0.4, 1.8},
        {18, 12, 6, 0.0, 0.4, 1.8},
        {18, 12, 6, 0.0, 0.4, 1.8}
    },
    { // STAND
        {18, 12, 6, 0.0, 0.5, 2.0},
        {18, 12, 6, 0.0, 0.5, 2.0},
        {18, 12, 6, 0.0, 0.5, 2.0},
        {18, 12, 6, 0.0, 0.5, 2.0}
    },
    { // GRAB
        {13, 12, 6, 0.0, 0.5, 2.0},
        {13, 12, 6, 0.0, 0.5, 2.0},
        {13, 12, 6, 0.0, 0.5, 2.0},
        {13, 12, 6, 0.0, 0.5, 2.0}
    },
    { // LEFT_TURN
        {17, 2.5, 5.5, 0.0, 0.4, 2.0},
        {17, 2.5, 5.5, 0.0, 0.4, 2.0},
        {17, 2.5, 5.5, 0.0, 0.4, 2.0},
        {17, 2.5, 5.5, 0.0, 0.4, 2.0}
    },
    { // RIGHT_TURN
        {17, 2.5, 5.5, 0.0, 0.4, 2.0},
        {17, 2.5, 5.5, 0.0, 0.4, 2.0},
        {17, 2.5, 5.5, 0.0, 0.4, 2.0},
        {17, 2.5, 5.5, 0.0, 0.4, 2.0}
    },
    { // IN_PLACE, AKANE obstacle-course debug state
        {18, 0, 6, 0.0, 0.5, 2.0},
        {18, 0, 6, 0.0, 0.5, 2.0},
        {18, 0, 6, 0.0, 0.5, 2.0},
        {18, 0, 6, 0.0, 0.5, 2.0}
    },
};

static int Action_IsWithinThreshold(float delta, float threshold)
{
    return fabsf(delta) < threshold;
}

static const char *Action_StateName(enum States action_state)
{
    switch (action_state)
    {
    case RUN:
        return "RUN";
    case STAND:
        return "STAND";
    case GRAB:
        return "GRAB";
    case LEFT_TURN:
        return "LEFT_TURN";
    case RIGHT_TURN:
        return "RIGHT_TURN";
    case IN_PLACE:
        return "IN_PLACE";
    case STOP:
        return "STOP";
    case REALSE:
        return "REALSE";
    default:
        return "UNKNOWN";
    }
}

static uint8_t Action_IsGaitParamState(enum States action_state)
{
    return action_state == RUN ||
           action_state == STAND ||
           action_state == GRAB ||
           action_state == LEFT_TURN ||
           action_state == RIGHT_TURN ||
           action_state == IN_PLACE;
}

static uint8_t Action_IsAkaneMotionState(enum States action_state)
{
    return action_state == RUN ||
           action_state == LEFT_TURN ||
           action_state == RIGHT_TURN ||
           action_state == IN_PLACE;
}

static enum States Action_GetParamState(void)
{
    if (Action_IsGaitParamState(state))
    {
        return state;
    }

    return STAND;
}

static GaitParams *Action_GetPrimaryGaitParams(enum States action_state)
{
    if (!Action_IsGaitParamState(action_state))
    {
        action_state = STAND;
    }

    return &state_detached_params[action_state].detached_params_0;
}

static uint8_t Action_StringEquals(const char *lhs, const char *rhs);

#if ACTION_RECORD_UART8_ENABLE
static void ActionDebug_TrySendRecord(uint8_t force_send);

static uint8_t Action_Uart8TxReady(void)
{
    return huart8.gState == HAL_UART_STATE_READY;
}

static void ActionDebug_FormatFloat(char *buffer, size_t buffer_size, float value, uint8_t decimals)
{
    int32_t scale = 1;
    int32_t scaled_value;
    int32_t integer_part;
    int32_t decimal_part;
    uint8_t i;

    if (buffer == NULL || buffer_size == 0U)
    {
        return;
    }

    for (i = 0U; i < decimals; ++i)
    {
        scale *= 10;
    }

    scaled_value = (int32_t)lroundf(value * (float)scale);
    integer_part = scaled_value / scale;
    decimal_part = scaled_value % scale;
    if (decimal_part < 0)
    {
        decimal_part = -decimal_part;
    }

    if (decimals == 0U)
    {
        (void)snprintf(buffer, buffer_size, "%ld", (long)integer_part);
    }
    else if (decimals == 1U)
    {
        (void)snprintf(buffer, buffer_size, "%ld.%01ld", (long)integer_part, (long)decimal_part);
    }
    else
    {
        (void)snprintf(buffer, buffer_size, "%ld.%02ld", (long)integer_part, (long)decimal_part);
    }
}

static void ActionDebug_CaptureImuBase(void)
{
    action_imu_base_roll = Saber_DATA.Saber_imu_eluer.ELUER_ROLL;
    action_imu_base_pitch = Saber_DATA.Saber_imu_eluer.ELUER_PITCH;
    action_imu_base_yaw = Saber_DATA.Saber_imu_eluer.ELUER_YAW_now;
}

static float ActionDebug_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static void ActionDebug_AdjustOneParam(GaitParams *params, ActionDebugParam_e param, float delta)
{
    if (params == NULL)
    {
        return;
    }

    switch (param)
    {
    case ACTION_DEBUG_PARAM_STEP_LENGTH:
        params->step_length = ActionDebug_ClampFloat(params->step_length + delta, 0.0f, 100.0f);
        break;
    case ACTION_DEBUG_PARAM_UP_AMP:
        params->up_amp = ActionDebug_ClampFloat(params->up_amp + delta, 0.0f, 100.0f);
        break;
    case ACTION_DEBUG_PARAM_DOWN_AMP:
        params->down_amp = ActionDebug_ClampFloat(params->down_amp + delta, 0.0f, 100.0f);
        break;
    case ACTION_DEBUG_PARAM_STANCE_HEIGHT:
        params->stance_height = ActionDebug_ClampFloat(params->stance_height + delta, 0.0f, 100.0f);
        break;
    case ACTION_DEBUG_PARAM_FREQ:
        params->freq = ActionDebug_ClampFloat(params->freq + delta, 0.1f, 10.0f);
        break;
    case ACTION_DEBUG_PARAM_FLIGHT_PERCENT:
        params->flight_percent = ActionDebug_ClampFloat(params->flight_percent + delta, 0.05f, 0.95f);
        break;
    default:
        break;
    }
}

static void ActionDebug_AdjustCurrentStateParam(ActionDebugParam_e param, float delta)
{
    enum States param_state = Action_GetParamState();
    DetachedParam *params = &state_detached_params[param_state];

    ActionDebug_AdjustOneParam(&params->detached_params_0, param, delta);
    ActionDebug_AdjustOneParam(&params->detached_params_1, param, delta);
    ActionDebug_AdjustOneParam(&params->detached_params_2, param, delta);
    ActionDebug_AdjustOneParam(&params->detached_params_3, param, delta);
}

static void ActionDebug_SetState(enum States next_state)
{
    uint8_t was_motion = Action_IsAkaneMotionState(state);
    uint8_t is_motion = Action_IsAkaneMotionState(next_state);

    state = next_state;
    action_debug_hold_realse = (next_state == REALSE) ? 1U : 0U;
    start_flag = (next_state == REALSE || next_state == STOP) ? 0 : 1;
    nav_current_valid = 0U;
    nav_goal_valid = 0U;
    target_yaw = 0.0f;

    if (is_motion && !was_motion)
    {
        ActionDebug_CaptureImuBase();
        action_walk_phase_valid = 0U;
    }
}

static uint8_t ActionDebug_ProcessParamCommand(const char *cmd,
                                               const char *name,
                                               ActionDebugParam_e param,
                                               float delta)
{
    size_t name_len = strlen(name);

    if (strncmp(cmd, name, name_len) != 0)
    {
        return 0U;
    }

    if (cmd[name_len] == '+')
    {
        ActionDebug_AdjustCurrentStateParam(param, delta);
        return 1U;
    }

    if (cmd[name_len] == '-')
    {
        ActionDebug_AdjustCurrentStateParam(param, -delta);
        return 1U;
    }

    return 0U;
}

static uint8_t ActionDebug_HandleCommand(const char *line)
{
    const char *cmd;

    if (strncmp(line, "debug:", 6U) != 0)
    {
        return 0U;
    }

    cmd = line + 6U;

    if (Action_StringEquals(cmd, "run"))
    {
        ActionDebug_SetState(RUN);
        return 1U;
    }

    if (Action_StringEquals(cmd, "left_turn"))
    {
        ActionDebug_SetState(LEFT_TURN);
        return 1U;
    }

    if (Action_StringEquals(cmd, "right_turn"))
    {
        ActionDebug_SetState(RIGHT_TURN);
        return 1U;
    }

    if (Action_StringEquals(cmd, "in_place"))
    {
        ActionDebug_SetState(IN_PLACE);
        return 1U;
    }

    if (Action_StringEquals(cmd, "stand"))
    {
        ActionDebug_SetState(STAND);
        return 1U;
    }

    if (Action_StringEquals(cmd, "realse"))
    {
        ActionDebug_SetState(REALSE);
        return 1U;
    }

    if (Action_StringEquals(cmd, "clear"))
    {
        ActionDebug_ResetWalkRecord();
        return 1U;
    }

    if (ActionDebug_ProcessParamCommand(cmd, "step_length", ACTION_DEBUG_PARAM_STEP_LENGTH, ACTION_DEBUG_STEP_LENGTH_DELTA) ||
        ActionDebug_ProcessParamCommand(cmd, "up_amp", ACTION_DEBUG_PARAM_UP_AMP, ACTION_DEBUG_AMP_DELTA) ||
        ActionDebug_ProcessParamCommand(cmd, "down_amp", ACTION_DEBUG_PARAM_DOWN_AMP, ACTION_DEBUG_AMP_DELTA) ||
        ActionDebug_ProcessParamCommand(cmd, "stance_height", ACTION_DEBUG_PARAM_STANCE_HEIGHT, ACTION_DEBUG_STANCE_HEIGHT_DELTA) ||
        ActionDebug_ProcessParamCommand(cmd, "freq", ACTION_DEBUG_PARAM_FREQ, ACTION_DEBUG_FREQ_DELTA) ||
        ActionDebug_ProcessParamCommand(cmd, "flight_percent", ACTION_DEBUG_PARAM_FLIGHT_PERCENT, ACTION_DEBUG_FLIGHT_PERCENT_DELTA))
    {
        return 1U;
    }

    return 0U;
}

static void ActionDebug_FeedByte(uint8_t data)
{
    if (data == '\n' || data == '\r')
    {
        if (action_debug_line_len > 0U)
        {
            action_debug_line[action_debug_line_len] = '\0';
            if (ActionDebug_HandleCommand(action_debug_line))
            {
                ActionDebug_TrySendRecord(1U);
            }
            action_debug_line_len = 0U;
        }
        return;
    }

    if (action_debug_line_len < (ACTION_DEBUG_LINE_BUFFER_SIZE - 1U))
    {
        action_debug_line[action_debug_line_len++] = (char)data;
    }
    else
    {
        action_debug_line_len = 0U;
    }
}

static void Action_UART8DebugCallback(void)
{
    uint16_t i;

    if (action_debug_uart8_instance == NULL)
    {
        return;
    }

    for (i = 0U; i < action_debug_uart8_instance->recv_len; ++i)
    {
        ActionDebug_FeedByte(action_debug_uart8_instance->recv_buff[i]);
    }
}
#endif

static uint8_t Action_StringEquals(const char *lhs, const char *rhs)
{
    return strcmp(lhs, rhs) == 0;
}

static void Action_SelectScript(const char *action_code)
{
    action_script = action_script_stand_only;
    action_script_len = (uint8_t)(sizeof(action_script_stand_only) / sizeof(action_script_stand_only[0]));

    if (Action_StringEquals(action_code, "WEAVE"))
    {
        action_script = action_script_weave;
        action_script_len = (uint8_t)(sizeof(action_script_weave) /  sizeof(action_script_weave[0]));
        return;
    }

    if (Action_StringEquals(action_code, "SAND_PIT") ||
        Action_StringEquals(action_code, "DUCK") ||
        Action_StringEquals(action_code, "CLIMB") ||
        Action_StringEquals(action_code, "CROSS_A") ||
        Action_StringEquals(action_code, "CROSS_B") ||
        Action_StringEquals(action_code, "RAMP") ||
        Action_StringEquals(action_code, "STEPS"))
    {
        action_script = action_script_default;
        action_script_len = (uint8_t)(sizeof(action_script_default) / sizeof(action_script_default[0]));
    }
}

static void Action_ApplyScriptState(ActionScriptState_e script_state)
{
    switch (script_state)
    {
    case ACTION_SCRIPT_RUN:
        state = RUN;
        break;
    case ACTION_SCRIPT_LEFT_TURN:
        state = LEFT_TURN;
        break;
    case ACTION_SCRIPT_RIGHT_TURN:
        state = RIGHT_TURN;
        break;
    case ACTION_SCRIPT_STAND:
    default:
        state = STAND;
        break;
    }
}

static void Action_StartScript(const char *action_code)
{
    strncpy(action_active_code, action_code, UNICOMM_ACTION_CODE_MAX_LEN);
    action_active_code[UNICOMM_ACTION_CODE_MAX_LEN] = '\0';
    Action_SelectScript(action_active_code);
    action_script_index = 0U;
    action_phase_start_tick = xTaskGetTickCount();
}

static uint8_t Action_RunScript(void)
{
    TickType_t now = xTaskGetTickCount();
    const ActionScriptStep_s *step;

    if (action_script == NULL || action_script_index >= action_script_len)
    {
        state = STAND;
        return 1U;
    }

    step = &action_script[action_script_index];
    Action_ApplyScriptState(step->state);

    if ((now - action_phase_start_tick) >= pdMS_TO_TICKS(step->duration_ms))
    {
        action_script_index++;
        action_phase_start_tick = now;
    }

    return action_script_index >= action_script_len;
}

void Action_UART8DebugInit(void)
{
#if ACTION_RECORD_UART8_ENABLE
    USART_Init_Config_s action_debug_usart_conf = {
        .recv_buff_size = ACTION_DEBUG_RX_BUFFER_SIZE,
        .usart_handle = &huart8,
        .module_callback = Action_UART8DebugCallback,
    };

    action_debug_uart8_instance = USARTRegister(&action_debug_usart_conf);
    ActionDebug_ResetWalkRecord();
#endif
}

void ActionDebug_ResetWalkRecord(void)
{
#if ACTION_RECORD_UART8_ENABLE
    action_walk_done = 0U;
    action_last_base_phase = 0.0f;
    action_walk_phase_valid = 0U;
    action_record_last_tx_tick = 0U;
    ActionDebug_CaptureImuBase();
#endif
}

uint8_t ActionDebug_ShouldHoldRealse(void)
{
#if ACTION_RECORD_UART8_ENABLE
    return action_debug_hold_realse;
#else
    return 0U;
#endif
}

void ActionDebug_UpdateWalkPhase(float base_phase)
{
#if ACTION_RECORD_UART8_ENABLE
    if (!Action_IsAkaneMotionState(state))
    {
        action_walk_phase_valid = 0U;
        return;
    }

    if (base_phase < 0.0f)
    {
        base_phase += 1.0f;
    }
    else if (base_phase >= 1.0f)
    {
        base_phase -= 1.0f;
    }

    if (action_walk_phase_valid && base_phase < action_last_base_phase)
    {
        action_walk_done++;
    }

    action_last_base_phase = base_phase;
    action_walk_phase_valid = 1U;
#else
    (void)base_phase;
#endif
}

#if ACTION_RECORD_UART8_ENABLE
static uint32_t ActionDebug_GetLogIntervalMs(void)
{
    GaitParams *params = Action_GetPrimaryGaitParams(Action_GetParamState());
    float freq = params->freq + params->freq_offset;
    float interval_ms;

    if (freq < 0.1f)
    {
        freq = 0.1f;
    }

    interval_ms = 1000.0f / (freq * ACTION_RECORD_RATE_MULTIPLIER);

    if (interval_ms < (float)ACTION_DEBUG_LOG_MIN_INTERVAL_MS)
    {
        return ACTION_DEBUG_LOG_MIN_INTERVAL_MS;
    }

    if (interval_ms > (float)ACTION_DEBUG_LOG_MAX_INTERVAL_MS)
    {
        return ACTION_DEBUG_LOG_MAX_INTERVAL_MS;
    }

    return (uint32_t)interval_ms;
}

static void ActionDebug_TrySendRecord(uint8_t force_send)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t interval_ms;
    GaitParams *params;
    float freq;
    char freq_text[16];
    char stance_height_text[16];
    char step_length_text[16];
    char up_amp_text[16];
    char down_amp_text[16];
    char flight_percent_text[16];
    char param_freq_text[16];
    char imu_roll_text[16];
    char imu_pitch_text[16];
    char imu_yaw_text[16];
    char delta_roll_text[16];
    char delta_pitch_text[16];
    char delta_yaw_text[16];
    int len;
    uint16_t tx_len;

    if (!force_send && !Action_IsAkaneMotionState(state))
    {
        return;
    }

    interval_ms = ActionDebug_GetLogIntervalMs();
    if (!force_send &&
        action_record_last_tx_tick != 0U &&
        (now - action_record_last_tx_tick) < pdMS_TO_TICKS(interval_ms))
    {
        return;
    }

    if (!Action_Uart8TxReady())
    {
        return;
    }

    params = Action_GetPrimaryGaitParams(Action_GetParamState());
    freq = params->freq + params->freq_offset;
    ActionDebug_FormatFloat(freq_text, sizeof(freq_text), freq, 2U);
    ActionDebug_FormatFloat(stance_height_text, sizeof(stance_height_text), params->stance_height, 1U);
    ActionDebug_FormatFloat(step_length_text, sizeof(step_length_text), params->step_length, 1U);
    ActionDebug_FormatFloat(up_amp_text, sizeof(up_amp_text), params->up_amp, 1U);
    ActionDebug_FormatFloat(down_amp_text, sizeof(down_amp_text), params->down_amp, 1U);
    ActionDebug_FormatFloat(flight_percent_text, sizeof(flight_percent_text), params->flight_percent, 2U);
    ActionDebug_FormatFloat(param_freq_text, sizeof(param_freq_text), params->freq, 2U);
    ActionDebug_FormatFloat(imu_roll_text, sizeof(imu_roll_text), Saber_DATA.Saber_imu_eluer.ELUER_ROLL, 2U);
    ActionDebug_FormatFloat(imu_pitch_text, sizeof(imu_pitch_text), Saber_DATA.Saber_imu_eluer.ELUER_PITCH, 2U);
    ActionDebug_FormatFloat(imu_yaw_text, sizeof(imu_yaw_text), Saber_DATA.Saber_imu_eluer.ELUER_YAW_now, 2U);
    ActionDebug_FormatFloat(delta_roll_text, sizeof(delta_roll_text), Saber_DATA.Saber_imu_eluer.ELUER_ROLL - action_imu_base_roll, 2U);
    ActionDebug_FormatFloat(delta_pitch_text, sizeof(delta_pitch_text), Saber_DATA.Saber_imu_eluer.ELUER_PITCH - action_imu_base_pitch, 2U);
    ActionDebug_FormatFloat(delta_yaw_text, sizeof(delta_yaw_text), Saber_DATA.Saber_imu_eluer.ELUER_YAW_now - action_imu_base_yaw, 2U);

    len = snprintf(action_record_tx_buffer,
                   sizeof(action_record_tx_buffer),
                   "ALOG,mode=AKANE_DEBUG,state=%s,walk_done=%lu,freq=%s,"
                   "sp=%s/%s/%s/%s/%s/%s,"
                   "imu_r=%s,imu_p=%s,imu_y=%s,"
                   "dr=%s,dp=%s,dy=%s\r\n",
                   Action_StateName(state),
                   (unsigned long)action_walk_done,
                   freq_text,
                   stance_height_text,
                   step_length_text,
                   up_amp_text,
                   down_amp_text,
                   flight_percent_text,
                   param_freq_text,
                   imu_roll_text,
                   imu_pitch_text,
                   imu_yaw_text,
                   delta_roll_text,
                   delta_pitch_text,
                   delta_yaw_text);

    if (len <= 0)
    {
        return;
    }

    tx_len = (len < (int)sizeof(action_record_tx_buffer)) ? (uint16_t)len : (uint16_t)(sizeof(action_record_tx_buffer) - 1U);
    if (HAL_UART_Transmit_DMA(&huart8, (uint8_t *)action_record_tx_buffer, tx_len) == HAL_OK)
    {
        action_record_last_tx_tick = now;
    }
}
#endif

void ActionProcessDebugControl(void)
{
#if ACTION_RECORD_UART8_ENABLE
    /* AKANE obstacle-course debug mode: UART8 owns motion commands; UART7 only parses image host frames. */
    nav_current_valid = 0U;
    nav_goal_valid = 0U;
    target_yaw = 0.0f;
    ActionDebug_TrySendRecord(0U);
#endif
}

static float Action_NormalizeYaw180(float yaw)
{
    while (yaw > 180.0f)
    {
        yaw -= 360.0f;
    }

    while (yaw < -180.0f)
    {
        yaw += 360.0f;
    }

    return yaw;
}

static float Action_AngularErrorDeg(float current_yaw, float expected_yaw)
{
    return Action_NormalizeYaw180(current_yaw - expected_yaw);
}

static float Action_DegToRad(float degree)
{
    return degree * (float)(3.14159265358979323846 / 180.0);
}

static float Action_RadToDeg(float radian)
{
    return radian * (float)(180.0 / 3.14159265358979323846);
}

static float Action_CircularMeanDeg(float yaw_a, float yaw_b)
{
    float vector_x = cosf(Action_DegToRad(yaw_a)) + cosf(Action_DegToRad(yaw_b));
    float vector_y = sinf(Action_DegToRad(yaw_a)) + sinf(Action_DegToRad(yaw_b));

    if (fabsf(vector_x) < 1e-6f && fabsf(vector_y) < 1e-6f)
    {
        return Action_NormalizeYaw180(yaw_a);
    }

    return Action_NormalizeYaw180(Action_RadToDeg(atan2f(vector_y, vector_x)));
}

static float Action_GetNavigationGoalYaw(void)
{
    return Action_NormalizeYaw180(upstream_goal_yaw);
}

static float Action_GetFusedCurrentYaw(void)
{
    float imu_yaw = Action_NormalizeYaw180(Saber_DATA.Saber_imu_eluer.ELUER_YAW_now);
    float upstream_yaw = Action_NormalizeYaw180(upstream_current_yaw);

    return Action_CircularMeanDeg(imu_yaw, upstream_yaw);
}

void Action_ApplyRunYawCorrection(DetachedParam *run_params)
{
    float current_yaw;
    float goal_yaw;
    float err_yaw;
    float correction_offset;

    if (run_params == NULL)
    {
        return;
    }

    run_params->detached_params_0.step_length_offset = 0.0f;
    run_params->detached_params_1.step_length_offset = 0.0f;
    run_params->detached_params_2.step_length_offset = 0.0f;
    run_params->detached_params_3.step_length_offset = 0.0f;

    if (!nav_current_valid || !nav_goal_valid)
    {
        return;
    }

    current_yaw = Action_NormalizeYaw180(upstream_current_yaw);
    goal_yaw = Action_GetNavigationGoalYaw();
    err_yaw = Action_AngularErrorDeg(current_yaw, goal_yaw);
    correction_offset = RUN_YAW_CORRECTION_GAIN * fabsf(err_yaw);

    if (err_yaw > 0.0f)
    {
        run_params->detached_params_0.step_length_offset = correction_offset;
        run_params->detached_params_3.step_length_offset = correction_offset;
    }
    else if (err_yaw < 0.0f)
    {
        run_params->detached_params_1.step_length_offset = correction_offset;
        run_params->detached_params_2.step_length_offset = correction_offset;
    }
}

void ActionProcessSerialState(void)
{
    /* AKANE obstacle-course auto mode: UART7 MEASURING/ACTIONING drives motion when UART8 debug is disabled. */
    uint32_t latest_frame_seq = unicomm_frame_seq;
    TickType_t now = xTaskGetTickCount();
    uint8_t has_new_frame = (latest_frame_seq != action_seen_frame_seq);

    if (state == REALSE || state == STOP)
    {
        return;
    }

    if (action_serial_phase == ACTION_SERIAL_BOOT_STAND)
    {
        action_serial_phase = ACTION_SERIAL_APPROACH;
        state = STAND;
    }

    if (has_new_frame)
    {
        action_seen_frame_seq = latest_frame_seq;

        if (unicomm_latest_stage == UNICOMM_STAGE_MEASURING)
        {
            if (action_serial_phase == ACTION_SERIAL_APPROACH &&
                unicomm_has_current_tag &&
                unicomm_current_tag_id == action_expected_tag_id)
            {
                action_active_tag_id = unicomm_current_tag_id;
                nav_current_valid = 1U;
                nav_goal_valid = 1U;
            }
        }
        else if (unicomm_latest_stage == UNICOMM_STAGE_ACTIONING)
        {
            if (action_serial_phase == ACTION_SERIAL_APPROACH)
            {
                action_serial_phase = ACTION_SERIAL_SETTLE;
                action_phase_start_tick = now;
                nav_current_valid = 0U;
                nav_goal_valid = 0U;
                state = STAND;
            }
        }
    }

    switch (action_serial_phase)
    {
    case ACTION_SERIAL_APPROACH:
        if (unicomm_has_current_tag && unicomm_current_tag_id == action_expected_tag_id)
        {
            ActionProcessNavigation();
        }
        else
        {
            state = STAND;
        }
        break;

    case ACTION_SERIAL_SETTLE:
        state = STAND;
        if ((now - action_phase_start_tick) >= pdMS_TO_TICKS(ACTION_SETTLE_MS))
        {
            Action_StartScript(unicomm_action_code);
            action_serial_phase = ACTION_SERIAL_EXECUTE;
        }
        break;

    case ACTION_SERIAL_EXECUTE:
        if (Action_RunScript())
        {
            action_serial_phase = ACTION_SERIAL_DONE;
            action_phase_start_tick = now;
            state = STAND;
        }
        break;

    case ACTION_SERIAL_DONE:
        state = STAND;
        if ((now - action_phase_start_tick) >= pdMS_TO_TICKS(ACTION_SETTLE_MS))
        {
            if (action_expected_tag_id == action_active_tag_id)
            {
                action_expected_tag_id++;
            }
            nav_current_valid = 0U;
            nav_goal_valid = 0U;
            action_script = NULL;
            action_script_len = 0U;
            action_script_index = 0U;
            action_serial_phase = ACTION_SERIAL_APPROACH;
        }
        break;

    case ACTION_SERIAL_BOOT_STAND:
    default:
        state = STAND;
        break;
    }
}

void ActionProcessNavigation(void)
{
    float dx;
    float dy;
    float current_yaw;
    float yaw_error;

    if (state == GRAB || state == STOP || state == REALSE)
    {
        return;
    }

    if (!nav_current_valid || !nav_goal_valid)
    {
        target_yaw = 0.0f;
        state = STAND;
        return;
    }

    current_yaw = Action_NormalizeYaw180(upstream_current_yaw);
    target_yaw = Action_GetNavigationGoalYaw();
    yaw_error = Action_AngularErrorDeg(current_yaw, target_yaw);

    if (yaw_error > NAV_YAW_FINISH_THRESHOLD_DEG)
    {
        state = RIGHT_TURN;
        return;
    }

    if (yaw_error < -NAV_YAW_FINISH_THRESHOLD_DEG)
    {
        state = LEFT_TURN;
        return;
    }

    dx = current_x - target_x;
    dy = current_y - target_y;

    if (Action_IsWithinThreshold(dx, NAV_ARRIVAL_THRESHOLD_CM) &&
        Action_IsWithinThreshold(dy, NAV_ARRIVAL_THRESHOLD_CM))
    {
        state = STAND;
        return;
    }

    state = RUN;
}

void DOWN(void)
{
    DJIMotorSetRef(RightGetMotor(0), 0);
    DJIMotorSetRef(RightGetMotor(1), 0);
    DJIMotorSetRef(LeftGetMotor(0), 0);
    DJIMotorSetRef(LeftGetMotor(1), 0);
    DJIMotorSetRef(LeftGetMotor(2), 0);
    DJIMotorSetRef(LeftGetMotor(3), 0);
    DJIMotorSetRef(RightGetMotor(2), 0);
    DJIMotorSetRef(RightGetMotor(3), 0);

    IsMotoReadyOrNot = IsReady;
}
