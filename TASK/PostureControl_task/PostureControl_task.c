#include "PostureControl_task.h"
#include "MotorControl_task.h"
#include "action.h"
#include "IMU.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define STARTUP_DELAY_MS 2000U

float  temp_theta1;
float  temp_theta2;
float  temp_theta3;
float  temp_theta4;
float  temp_theta5;
float  temp_theta6;
float  temp_theta7;
float  temp_theta8;

float  theta1;
float  theta2;
float  theta3;
float  theta4;
float  theta5;
float  theta6;
float  theta7;
float  theta8;

bool _leg_active[4]={1,1,1,1};

// 定义 stanceHeight 为支撑高度
float stanceHeight=0;
// 定义 downAMP 为下降幅度
float downAMP=0;
// 定义 upAMP 为上升幅度
float upAMP=0;
// 从参数结构体中获取摆动相百分比
float flightPercent=0;
// 定义步长
float stepLength=0;
// 从参数结构体中获取频率
float FREQ=0;

float foot_x;//足端到电机轴心的x距离
float foot_y;//足端到电机轴心的y距离

enum States state = REALSE;
int start_flag = 0;

static TickType_t startup_tick = 0;
static uint8_t startup_tick_initialized = 0;
static float gait_phase_accumulator = 0.0f;
static float gait_phase_prev_t = 0.0f;

DJIMotorInstance* _g_left_motor[4]={0};
DJIMotorInstance* _g_right_motor[4]={0};

static void StandByStartPosition(void);

static int IsMotionState(enum States current_state)
{
    return current_state == RUN ||
           current_state == GRAB ||
           current_state == LEFT_TURN ||
           current_state == RIGHT_TURN ||
           current_state == IN_PLACE;
}

static void UpdateStartupState(void)
{
    TickType_t current_tick = xTaskGetTickCount();

    if (!startup_tick_initialized)
    {
        startup_tick = current_tick;
        startup_tick_initialized = 1;
    }

    if ((current_tick - startup_tick) < pdMS_TO_TICKS(STARTUP_DELAY_MS))
    {
        state = REALSE;
        return;
    }

    if (state == REALSE && !ActionDebug_ShouldHoldRealse())
    {
        state = STAND;
    }
}

static void UpdateStartFlag(void)
{
    if (state == REALSE || state == STOP)
    {
        start_flag = 0;
    }
    else if (state == STAND)
    {
        start_flag = 1;
    }
}

static void ApplyTurnStepLength(DetachedParam *turn_params, int is_left_turn)
{
    float inner_step = TURN_GAIN * fabsf(turn_params->detached_params_0.step_length);
    float outer_step = TURN_GAIN * fabsf(turn_params->detached_params_1.step_length);

    if (is_left_turn)
    {
        turn_params->detached_params_0.step_length = inner_step*1.2;
        turn_params->detached_params_3.step_length = TURN_GAIN * fabsf(turn_params->detached_params_3.step_length)*1.2;
        turn_params->detached_params_1.step_length = -outer_step;
        turn_params->detached_params_2.step_length = -TURN_GAIN * fabsf(turn_params->detached_params_2.step_length);
    }
    else
    {
        turn_params->detached_params_0.step_length = -inner_step;
        turn_params->detached_params_3.step_length = -TURN_GAIN * fabsf(turn_params->detached_params_3.step_length);
        turn_params->detached_params_1.step_length = outer_step;
        turn_params->detached_params_2.step_length = TURN_GAIN * fabsf(turn_params->detached_params_2.step_length);
    }
}

void PostureControl_task(const void* argument)
{
    TickType_t LastWakeTime;
    LastWakeTime = xTaskGetTickCount();
#if !ACTION_RECORD_UART8_ENABLE
    static char buffer[100];
#endif
    for(;;)
    {

#if !ACTION_RECORD_UART8_ENABLE
        if (HAL_UART_GetState(&huart8) == HAL_UART_STATE_READY)
        {
            int length = snprintf(buffer, sizeof(buffer), "%d,%d,%d\r\n", state, (int)target_yaw, (int)Saber_DATA.Saber_imu_eluer.ELUER_YAW_now);
            if (length > 0)
            {
                HAL_UART_Transmit_DMA(&huart8, (uint8_t *)buffer, (uint16_t)length);
            }
        }
#endif

        PostureControl();
        osDelayUntil(&LastWakeTime,8);
    }
}

void PostureControl()
{
    DetachedParam turn_params;
    int motion_blocked = 0;

    IMU_ParseLatestFrame();

    UpdateStartupState();
    UpdateStartFlag();

    if (start_flag == 0 && IsMotionState(state))
    {
        state = STAND;
        UpdateStartFlag();
        motion_blocked = 1;
    }

    if (!motion_blocked)
    {
#if ACTION_RECORD_UART8_ENABLE
        ActionProcessDebugControl();
#else
        ActionProcessSerialState();
#endif
        UpdateStartFlag();
    }

    switch (state)
    {
    case RUN:
        turn_params = state_detached_params[state];
        Action_ApplyRunYawCorrection(&turn_params);
        detached_params = turn_params;
        gait_detached(detached_params,0.0,0.5,0.0,0.5);
        break;
    case STAND:
        StandByStartPosition();
        break;
    case GRAB:

        // Task-race behavior. AKANE obstacle-course debug/auto paths do not enter GRAB.
        detached_params=state_detached_params[state];
        gait_detached(detached_params,0.0,0.0,0.0,0.0);
        break;
    case LEFT_TURN:

        turn_params = state_detached_params[state];
        ApplyTurnStepLength(&turn_params, 1);
        detached_params = turn_params;
        gait_detached(detached_params,0.0,0.5,0.0,0.5);
        break;
    case RIGHT_TURN:

        turn_params = state_detached_params[state];
        ApplyTurnStepLength(&turn_params, 0);
        detached_params = turn_params;
        gait_detached(detached_params,0.0,0.5,0.0,0.5);
        break;
    case IN_PLACE:

        detached_params = state_detached_params[state];
        gait_detached(detached_params,0.0,0.5,0.0,0.5);
        break;
    case STOP:

        DOWN();
        break;
    case REALSE:

        osDelay(50);
        break;
    default:
        break;
    }
}

static void StandByStartPosition(void)
{
    DJIMotorSetRef(RightGetMotor(0), START_POS1 * ReductionAndAngleRatio);
    DJIMotorSetRef(RightGetMotor(1), START_POS2 * ReductionAndAngleRatio);
    DJIMotorSetRef(LeftGetMotor(0), START_POS3 * ReductionAndAngleRatio);
    DJIMotorSetRef(LeftGetMotor(1), START_POS4 * ReductionAndAngleRatio);
    DJIMotorSetRef(LeftGetMotor(2), START_POS5 * ReductionAndAngleRatio);
    DJIMotorSetRef(LeftGetMotor(3), START_POS6 * ReductionAndAngleRatio);
    DJIMotorSetRef(RightGetMotor(2), START_POS7 * ReductionAndAngleRatio);
    DJIMotorSetRef(RightGetMotor(3), START_POS8 * ReductionAndAngleRatio);

    IsMotoReadyOrNot = IsReady;
}

void gait_detached(DetachedParam d_params, float leg0_offset, float leg1_offset, float leg2_offset, float leg3_offset)
{
    float t = HAL_GetTick() / 1000.0;
    if (_leg_active[0] == YES)
        CoupledMoveLeg(t, d_params.detached_params_0, leg0_offset, 0);

    if (_leg_active[1] == YES)
        CoupledMoveLeg(t, d_params.detached_params_1, leg1_offset, 1);

    if (_leg_active[2] == YES)
        CoupledMoveLeg(t, d_params.detached_params_2, leg2_offset, 2);

    if (_leg_active[3] == YES)
        CoupledMoveLeg(t, d_params.detached_params_3, leg3_offset, 3);

    IsMotoReadyOrNot = IsReady; // 数据填充完毕
    //vTaskDelay(50);
}

/**
 * NAME: void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId)
 * FUNCTION : 驱动并联腿 传递参数
 */
void CoupledMoveLeg(float t, GaitParams params, float gait_offset, int LegId)
{
    int legId = LegId;
    if (legId == 0)
    {
        CycloidTrajectory(t, params, gait_offset);
    }
    if (legId == 1)
    {
        CycloidTrajectory(t, params, gait_offset);
    }
    if (legId == 2)
    {
        CycloidTrajectory(t, params, gait_offset);
    }
    if (legId == 3)
    {
        CycloidTrajectory(t, params, gait_offset);
    }

    CartesianToTheta();        // 笛卡尔坐标转换到伽马坐标
    SetCoupledPosition(LegId); // 发送数据给电机驱动函数
}

void CycloidTrajectory(float t, GaitParams params, float gait_offset)
{
    float dt;
    float base_phase;


    // 定义 stanceHeight 为支撑高度
    stanceHeight = params.stance_height + params.stance_height_offset;

    // 定义 downAMP 为下降幅度
    downAMP = params.down_amp + params.down_amp_offset;

    // 定义 upAMP 为上升幅度
    upAMP = params.up_amp + params.up_amp_offset;
    // 从参数结构体中获取摆动相百分比
    flightPercent = params.flight_percent + params.flight_percent_offset;

    // 定义步长
    stepLength = params.step_length + params.step_length_offset;

    // 从参数结构体中获取频率
    FREQ = params.freq + params.freq_offset;

    // 更新时间累积 p，并记录当前时间 t 为下一次的时间基准
    dt = t - gait_phase_prev_t;
    if (dt < 0.0f)
    {
        dt = 0.0f;
    }
    gait_phase_accumulator += FREQ * dt;
    gait_phase_prev_t = t;
    base_phase = fmod(gait_phase_accumulator, 1.0f);
    if (dt > 0.0f)
    {
        ActionDebug_UpdateWalkPhase(base_phase);
    }
    // 计算当前时间累积 p 加上步态偏移量后的模1值，用于周期性计算
    float gp = fmod((gait_phase_accumulator + gait_offset), 1.0);
    if (gp <= flightPercent) // 足端摆动相
    {

        foot_x = stepLength * ((2 * PI * gp / flightPercent) - sin((2 * PI * gp / flightPercent))) / (2 * PI);
        if ((gp >= 0) && (gp <= (flightPercent / 2)))
        {
            foot_y = -2 * upAMP * (gp / flightPercent - sin(4 * PI * gp / flightPercent) / (4 * PI)) + stanceHeight;
        }
        else if ((gp > (flightPercent / 2)) && (gp <= flightPercent))
        {
            foot_y = (2 * upAMP * (gp / flightPercent - sin(4 * PI * gp / flightPercent) / (4 * PI)) - 2 * upAMP) + stanceHeight;
        }
    }
    else // 足端支撑相
    {
        foot_x = stepLength - (stepLength * ((2 * PI * (gp - flightPercent) / (1 - flightPercent)) - sin((2 * PI * (gp - flightPercent) / (1 - flightPercent)))) / (2 * PI)) ;
        foot_y = 0 + stanceHeight + downAMP * sin(PI * (gp - flightPercent) / (1.0 - flightPercent));
    }

    if(state == STAND||state == GRAB)
    {
        foot_x=0;
        foot_y=stanceHeight;
    }

    else if(state==RUN || state==LEFT_TURN ||state ==RIGHT_TURN || state == IN_PLACE)
    {
        if (gp <= flightPercent) // 足端摆动相
        {

            foot_x = -stepLength/2.0f+ stepLength * ((2 * PI * gp / flightPercent) - sin((2 * PI * gp / flightPercent))) / (2 * PI);

            if ((gp >= 0) && (gp <= (flightPercent / 2)))
            {
                foot_y = -2 * upAMP * (gp / flightPercent - sin(4 * PI * gp / flightPercent) / (4 * PI)) + stanceHeight;
            }
            else if ((gp > (flightPercent / 2)) && (gp <= flightPercent))
            {
                foot_y = (2 * upAMP * (gp / flightPercent - sin(4 * PI * gp / flightPercent) / (4 * PI)) - 2 * upAMP) + stanceHeight;
            }
        }
        else // 足端支撑相
        {
            // 添加两个新参数控制支撑相范围
            float supportStart = -stepLength/2.0f;  // 支撑相起始位置(可设为负值)
            float supportEnd = stepLength/2.0f;    // 支撑相结束位置
        
            // 重新计算支撑相长度
            float supportLength = supportEnd - supportStart;
            
            // 计算支撑相内的相对相位
            float supportPhase = (gp - flightPercent) / (1.0 - flightPercent);
            
            // 使用摆线轨迹从supportEnd移动到supportStart
            foot_x = supportEnd - supportLength * ((2 * PI * supportPhase) - sin(2 * PI * supportPhase)) / (2 * PI);
            
            // Y坐标保持不变
            foot_y = stanceHeight + downAMP * sin(PI * supportPhase);
        }
    }
    
}

void CartesianToTheta(void)
{
    float leg_length = 0;//电机轴心到足端轴心的距离
    double r1 = 0;//左侧大腿与电机轴心与足端轴心连线的夹角
    double r2 = 0;//右侧大腿与电机轴心与足端轴心连线的夹角
    float n = 0;//电机轴心与足端轴心连线与竖直方向的夹角
    float a1 = 0;//左侧大腿与竖直方向的夹角
    float a2 = 0;//右侧大腿与竖直方向的夹角

    leg_length = sqrt(pow(foot_x, 2) + pow(foot_y, 2)); 

    r1 = acos((pow(leg_length, 2) + pow(BLL, 2) - pow(SLL, 2)) / (2 * BLL * leg_length)) * 180.0 / PI;
    r2 = acos((pow(leg_length, 2) + pow(BLL, 2) - pow(SLL, 2)) / (2 * BLL * leg_length)) * 180.0 / PI;

    n = asin(foot_x / leg_length) * 180.0 / PI;

    a1 = r1 - n;
    a2 = r2 + n;

    
    theta1 = 90 - a1 + START_POS1;
    theta2 = 90 - a2 + START_POS2;
    theta3 = -90 + a2 + START_POS3;
    theta4 = -90 + a1 + START_POS4;
    theta5 = -90 + a2 + START_POS5;
    theta6 = -90 + a1 + START_POS6;
    theta7 = 90 - a1 + START_POS7;
    theta8 = 90 - a2 + START_POS8;
    
}

void SetCoupledPosition(int LegId)
{
    if (LegId == 0)
    {
        temp_theta1=theta1 * ReductionAndAngleRatio;
        temp_theta2=theta2 * ReductionAndAngleRatio;
        DJIMotorSetRef(RightGetMotor(0),temp_theta1);
        DJIMotorSetRef(RightGetMotor(1),temp_theta2);
    }
    else if (LegId == 1)
    {
        temp_theta3=theta3 * ReductionAndAngleRatio;
        temp_theta4=theta4 * ReductionAndAngleRatio;
        DJIMotorSetRef(LeftGetMotor(0),temp_theta3);
        DJIMotorSetRef(LeftGetMotor(1),temp_theta4);
    }

    else if (LegId == 2)
    {
        temp_theta5=theta5 * ReductionAndAngleRatio;
        temp_theta6=theta6 * ReductionAndAngleRatio;
        DJIMotorSetRef(LeftGetMotor(2),temp_theta5);
        DJIMotorSetRef(LeftGetMotor(3),temp_theta6);
    }
    else if (LegId == 3)
    {
        temp_theta7=theta7 * ReductionAndAngleRatio;
        temp_theta8=theta8 * ReductionAndAngleRatio;
        DJIMotorSetRef(RightGetMotor(2),temp_theta7);
        DJIMotorSetRef(RightGetMotor(3),temp_theta8);
    }

    
}
