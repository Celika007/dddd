#include "MotorControl_task.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "PostureControl_task.h"


#define MOTOR_MAX_CONTINUOUS_CURRENT 12000   // 持续电流阈值
#define OVERCURRENT_TIME_THRESHOLD 2000       // 过流持续时间 
#define MOTOR_MAX_CONTINUOUS_TEMP 40   // 持续阈值
#define OVERTEMPERATURE_TIME_THRESHOLD 2000       // 持续时间 

static DJIMotorInstance *g_left_motor[LEFT_MOTOR_COUNT] = {NULL};
static bool g_left_motor_initialized = false;

static DJIMotorInstance *g_right_motor[RIGHT_MOTOR_COUNT] = {NULL};
static bool g_right_motor_initialized = false;

bool IsMotoReadyOrNot;


void MotorControl_task(const void* argument)
{
    TickType_t LastWakeTime;
    LastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        if(state==STAND)
        {
            for(int i=0;i<LEFT_MOTOR_COUNT;i++)
            {
                g_left_motor[i]->motor_controller.angle_PID.MaxOut=1000;
                g_left_motor[i]->motor_controller.speed_PID.MaxOut=16000;
            }
            for(int i=0;i<RIGHT_MOTOR_COUNT;i++)
            {
                g_right_motor[i]->motor_controller.angle_PID.MaxOut=1000;
                g_right_motor[i]->motor_controller.speed_PID.MaxOut=16000;
            }
        }

        if(state==RUN)
        {
            for(int i=0;i<LEFT_MOTOR_COUNT;i++)
            {
                g_left_motor[i]->motor_controller.angle_PID.MaxOut=5000;
                g_left_motor[i]->motor_controller.speed_PID.MaxOut=10000;
            }
            for(int i=0;i<RIGHT_MOTOR_COUNT;i++)
            {
                g_right_motor[i]->motor_controller.angle_PID.MaxOut=5000;
                g_right_motor[i]->motor_controller.speed_PID.MaxOut=10000;
            }
        }

        if(IsMotoReadyOrNot == IsReady)
        {
            DJIMotorControl();
        }
        osDelayUntil(&LastWakeTime,3);
    }
}

void LeftBuildDefaultMotorConfig(Motor_Init_Config_s config[LEFT_MOTOR_COUNT])
{
    for(int i=0;i<LEFT_MOTOR_COUNT;i++)
    {
        if (&config[i] == NULL)
        {
            return;
        }

        memset(&config[i], 0, sizeof(config[i]));

        config[i].motor_type = M3508;

        config[i].can_init_config.can_handle = &hcan1;
        config[i].can_init_config.tx_id = i+1; /* TODO: 后续根据实际电机ID修改 */

        config[i].controller_setting_init_config.outer_loop_type = ANGLE_LOOP; /* TODO: 后续根据实际控制需求修改 */
        config[i].controller_setting_init_config.close_loop_type = ANGLE_LOOP|SPEED_LOOP ; /* TODO: 后续根据实际控制链路修改 */
        config[i].controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL; /* TODO: 后续根据实际安装方向修改 */
        config[i].controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL; /* TODO: 后续根据实际反馈方向修改 */
        config[i].controller_setting_init_config.angle_feedback_source = MOTOR_FEED; /* TODO: 后续如需外部反馈请修改 */
        config[i].controller_setting_init_config.speed_feedback_source = MOTOR_FEED; /* TODO: 后续如需外部反馈请修改 */
        config[i].controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE; /* TODO: 后续如需前馈请修改 */

        config[i].controller_param_init_config.speed_PID.Kp = 15.5f; /* TODO: 后续如启用角度环请修改 */
        config[i].controller_param_init_config.speed_PID.Ki = 0.0001f;
        config[i].controller_param_init_config.speed_PID.Kd = 0.0001f;
        config[i].controller_param_init_config.speed_PID.MaxOut = 8000.0f;
        config[i].controller_param_init_config.speed_PID.IntegralLimit=2000.0f;
        config[i].controller_param_init_config.speed_PID.DeadBand = 1.0f;
        config[i].controller_param_init_config.speed_PID.Improve = PID_Integral_Limit;

        config[i].controller_param_init_config.angle_PID.Kp = 8.0f; /* TODO: 后续如启用角度环请修改 */
        config[i].controller_param_init_config.angle_PID.Ki = 0.008f;
        config[i].controller_param_init_config.angle_PID.Kd = 0.1f;
        config[i].controller_param_init_config.angle_PID.MaxOut = 8000.0f;
        config[i].controller_param_init_config.angle_PID.IntegralLimit=2000.0f;
        config[i].controller_param_init_config.angle_PID.DeadBand = 0.5f;
        config[i].controller_param_init_config.angle_PID.Improve = PID_Integral_Limit;

        for(int k=0;k<4;k++)
        {
            config[i].controller_param_init_config.base_ecd[k] = 0;
        }
        

        // if(state == STAND)
        // {
        //     config[i].controller_param_init_config.angle_PID.MaxOut = 2000.0f;

        //     config[2].controller_param_init_config.speed_PID.Kp = 3.5f;
        //     config[3].controller_param_init_config.speed_PID.Kp = 3.5f;
        // }
        
        // if(state == RUN)
        // {
        //     config[i].controller_param_init_config.angle_PID.MaxOut = 5000.0f;
        // }
    }
}

bool LeftMotorInit(void)
{
    Motor_Init_Config_s left_motor_config[LEFT_MOTOR_COUNT];
    bool all_ok = true;

    if (g_left_motor_initialized)
    {
        return true;
    }

    LeftBuildDefaultMotorConfig(left_motor_config);

    for(int i=0;i<LEFT_MOTOR_COUNT;i++)
    {
        g_left_motor[i]=DJIMotorInit(&left_motor_config[i]);
        if(g_left_motor[i]==NULL)
        {
            all_ok=false;
        }
    }

    g_left_motor_initialized = true;

    return (all_ok);
}

void RightBuildDefaultMotorConfig(Motor_Init_Config_s config[RIGHT_MOTOR_COUNT])
{
    for(int i=0;i<RIGHT_MOTOR_COUNT;i++)
    {
        if (&config[i] == NULL)
        {
            return;
        }

        memset(&config[i], 0, sizeof(config[i]));

        config[i].motor_type = M3508;

        config[i].can_init_config.can_handle = &hcan2;
        config[i].can_init_config.tx_id = i+1; /* TODO: 后续根据实际电机ID修改 */

        config[i].controller_setting_init_config.outer_loop_type = ANGLE_LOOP; /* TODO: 后续根据实际控制需求修改 */
        config[i].controller_setting_init_config.close_loop_type = ANGLE_LOOP|SPEED_LOOP ; /* TODO: 后续根据实际控制链路修改 */
        config[i].controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL; /* TODO: 后续根据实际安装方向修改 */
        config[i].controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL; /* TODO: 后续根据实际反馈方向修改 */
        config[i].controller_setting_init_config.angle_feedback_source = MOTOR_FEED; /* TODO: 后续如需外部反馈请修改 */
        config[i].controller_setting_init_config.speed_feedback_source = MOTOR_FEED; /* TODO: 后续如需外部反馈请修改 */
        config[i].controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE; /* TODO: 后续如需前馈请修改 */

        config[i].controller_param_init_config.speed_PID.Kp = 15.5f; /* TODO: 后续如启用角度环请修改 */
        config[i].controller_param_init_config.speed_PID.Ki = 0.0001f;
        config[i].controller_param_init_config.speed_PID.Kd = 0.0001f;
        config[i].controller_param_init_config.speed_PID.MaxOut = 8000.0f;
        config[i].controller_param_init_config.speed_PID.IntegralLimit=2000.0f;
        config[i].controller_param_init_config.speed_PID.DeadBand = 1.0f;
        config[i].controller_param_init_config.speed_PID.Improve = PID_Integral_Limit;

        config[i].controller_param_init_config.angle_PID.Kp = 8.0f; /* TODO: 后续如启用角度环请修改 */
        config[i].controller_param_init_config.angle_PID.Ki = 0.008f;
        config[i].controller_param_init_config.angle_PID.Kd = 0.1f;
        config[i].controller_param_init_config.angle_PID.MaxOut = 8000.0f;
        config[i].controller_param_init_config.angle_PID.IntegralLimit=2000.0f;
        config[i].controller_param_init_config.angle_PID.DeadBand = 0.5f;
        config[i].controller_param_init_config.angle_PID.Improve = PID_Integral_Limit;


        for(int k=0;k<4;k++)
        {
            config[i].controller_param_init_config.base_ecd[k] = 0;
        }

        // if(state == STAND)
        // {
        //     config[i].controller_param_init_config.angle_PID.MaxOut = 2000.0f;

        //     config[2].controller_param_init_config.speed_PID.Kp = 3.5f;
        //     config[3].controller_param_init_config.speed_PID.Kp = 3.5f;
        // }
        
        if(state == RUN)
        {
            config[i].controller_param_init_config.angle_PID.MaxOut = 5000.0f;
        }
    }
}

bool RightMotorInit(void)
{
    Motor_Init_Config_s right_motor_config[RIGHT_MOTOR_COUNT];
    bool all_ok = true;

    if (g_right_motor_initialized)
    {
        return true;
    }

    RightBuildDefaultMotorConfig(right_motor_config);

    for(int i=0;i<RIGHT_MOTOR_COUNT;i++)
    {
        g_right_motor[i]=DJIMotorInit(&right_motor_config[i]);
        if(g_right_motor[i]==NULL)
        {
            all_ok=false;
        }
    }

    g_right_motor_initialized = true;

    return (all_ok);
}

DJIMotorInstance *LeftGetMotor(uint8_t index)
{
    if(index>=LEFT_MOTOR_COUNT)
    {
        return NULL;
    }
    return g_left_motor[index];
}

DJIMotorInstance *RightGetMotor(uint8_t index)
{
    if(index>=RIGHT_MOTOR_COUNT)
    {
        return NULL;
    }
    return g_right_motor[index];
}

