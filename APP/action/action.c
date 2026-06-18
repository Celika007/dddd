#include "action.h"

#include <math.h>

#include "IMU.h"
#include "unicomm.h"

DetachedParam detached_params;
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
};

static int Action_IsWithinThreshold(float delta, float threshold)
{
    return fabsf(delta) < threshold;
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

    // current_yaw = Action_GetFusedCurrentYaw();
    current_yaw = Saber_DATA.Saber_imu_eluer.ELUER_YAW_now;
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
        arrival_report_sent = 0;
        target_yaw = 0.0f;
        state = STAND;
        return;
    }

    // current_yaw = Action_GetFusedCurrentYaw();
    current_yaw = Saber_DATA.Saber_imu_eluer.ELUER_YAW_now;
    target_yaw = Action_GetNavigationGoalYaw();
    yaw_error = Action_AngularErrorDeg(current_yaw, target_yaw);

    if (yaw_error > NAV_YAW_FINISH_THRESHOLD_DEG)
    {
        arrival_report_sent = 0;
        state = RIGHT_TURN;
        return;
    }

    if (yaw_error < -NAV_YAW_FINISH_THRESHOLD_DEG)
    {
        arrival_report_sent = 0;
        state = LEFT_TURN;
        return;
    }

    dx = current_x - target_x;
    dy = current_y - target_y;

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
