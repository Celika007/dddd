#include "action.h"

#include <math.h>

#include "IMU.h"
#include "unicomm.h"

DetachedParam detached_params;
int left_count = 0;
int right_count = 0;
float target_yaw = 0.0f;

static int arrival_report_sent = 0;

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
        {1, 2, 6, 0.0, 0.5, 2.6}, /* TODO: fill left-turn detached params */
        {1, 2, 6, 0.0, 0.5, 2.6}, /* TODO: fill left-turn detached params */
        {1, 2, 6, 0.0, 0.5, 2.6}, /* TODO: fill left-turn detached params */
        {1, 2, 6, 0.0, 0.5, 2.6}  /* TODO: fill left-turn detached params */
    },
    { // RIGHT_TURN
        {1, 2, 6, 0.0, 0.5, 2.6}, /* TODO: fill right-turn detached params */
        {1, 2, 6, 0.0, 0.5, 2.6}, /* TODO: fill right-turn detached params */
        {1, 2, 6, 0.0, 0.5, 2.6}, /* TODO: fill right-turn detached params */
        {1, 2, 6, 0.0, 0.5, 2.6}  /* TODO: fill right-turn detached params */
    },
};

static int Action_IsWithinThreshold(int32_t delta, int32_t threshold)
{
    return abs(delta) < threshold;
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

static int Action_IsYawNear(float yaw, float center)
{
    return fabsf(Action_AngularErrorDeg(yaw, center)) <= NAV_YAW_FINISH_THRESHOLD_DEG;
}

static void Action_UpdateTurnTargetYaw(void)
{
    float raw_target_yaw = -90.0f * left_count + 90.0f * right_count;

    target_yaw = Action_NormalizeYaw180(raw_target_yaw);
}

static void Action_StartLeftTurn(void)
{
    left_count += 1;
    Action_UpdateTurnTargetYaw();
    state = LEFT_TURN;
}

static void Action_StartRightTurn(void)
{
    right_count += 1;
    Action_UpdateTurnTargetYaw();
    state = RIGHT_TURN;
}

void Action_ApplyRunYawCorrection(DetachedParam *run_params)
{
    float current_yaw;
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

    current_yaw = Saber_DATA.Saber_imu_eluer.ELUER_YAW_now;
    err_yaw = Action_AngularErrorDeg(current_yaw, target_yaw);
    correction_offset = RUN_YAW_CORRECTION_GAIN * fabsf(err_yaw);

    //fix_flag

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

void ActionProcessNavigation(void)
{
    int32_t dx = current_x - target_x;
    int32_t dy = current_y - target_y;
    float current_yaw = Saber_DATA.Saber_imu_eluer.ELUER_YAW_now;

    if (state == GRAB || state == STOP || state == REALSE)
    {
        return;
    }

    if (Action_IsWithinThreshold(dx, NAV_ARRIVAL_THRESHOLD_CM) &&
        Action_IsWithinThreshold(dy, NAV_ARRIVAL_THRESHOLD_CM))
    {
        state = STAND;
        if (!arrival_report_sent)
        {
            UniComm_SendArrival();
            arrival_report_sent = 1;
        }
        return;
    }

    arrival_report_sent = 0;

    if (state == LEFT_TURN || state == RIGHT_TURN)
    {
        if (fabsf(Action_AngularErrorDeg(current_yaw, target_yaw)) < NAV_YAW_FINISH_THRESHOLD_DEG)
        {
            state = RUN;
        }
        return;
    }

    state = RUN;

    if (Action_IsYawNear(current_yaw, 0.0f) &&
        Action_IsWithinThreshold(dx, NAV_ARRIVAL_THRESHOLD_CM))
    {
        if (dx > 0)
        {
            Action_StartLeftTurn();
        }
        else if (dx < 0)
        {
            Action_StartRightTurn();
        }
        return;
    }

    if (Action_IsYawNear(current_yaw, 180.0f) &&
        Action_IsWithinThreshold(dx, NAV_ARRIVAL_THRESHOLD_CM))
    {
        if (dx > 0)
        {
            Action_StartRightTurn();
        }
        else if (dx < 0)
        {
            Action_StartLeftTurn();
        }
        return;
    }

    if (Action_IsYawNear(current_yaw, 90.0f) &&
        Action_IsWithinThreshold(dy, NAV_ARRIVAL_THRESHOLD_CM))
    {
        if (dy > 0)
        {
            Action_StartRightTurn();
        }
        else if (dy < 0)
        {
            Action_StartLeftTurn();
        }
        return;
    }

    if (Action_IsYawNear(current_yaw, -90.0f) &&
        Action_IsWithinThreshold(dy, NAV_ARRIVAL_THRESHOLD_CM))
    {
        if (dy > 0)
        {
            Action_StartLeftTurn();
        }
        else if (dy < 0)
        {
            Action_StartRightTurn();
        }
    }
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
