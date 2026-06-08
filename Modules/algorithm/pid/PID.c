/*******************************************************************************

                      版权所有 (C), 2023-2024, NCU_Roboteam

 *******************************************************************************
  文 件 名: PID.c
  版 本 号: V20240306.1
  作    者: 巢奖文
  生成日期: 2024年3月6日
  功能描述: PID控制函数库
  补    充: 无

*******************************************************************************/

/********************************包含头文件************************************/
#include "PID.h"

/*******************************全局变量定义***********************************/


/*导航预瞄点跟踪PID控制参数结构体x方向*/
PID PID_Preview_pid_x = {0};

/*导航预瞄点跟踪PID控制参数结构体y方向*/
PID PID_Preview_pid_y = {0};

/*导航映射点跟踪PID控制参数结构体x方向*/
PID PID_Reflection_pid_x = {0};

/*导航映射点跟踪PID控制参数结构体y方向*/
PID PID_Reflection_pid_y = {0};

/*导航末端速度环PID控制参数结构体x方向*/
PID PID_Speed_pid_x = {0};

/*导航末端速度环PID控制参数结构体y方向*/
PID PID_Speed_pid_y = {0};

/*视觉偏差模式x方向PID控制参数结构体*/
PID PID_OpenCV_relative_x = {0};

/*视觉偏差模式y方向PID控制参数结构体*/
PID PID_OpenCV_relative_y = {0};

/********************************文件内变量************************************/

/********************************自定义函数************************************/
/**********************************************************************
  * @ 函数名  ： PID_count
  * @ 功能说明： PID闭环计算
  * @ 参数    ： PID *pid：PID控制参数结构体
								 float now：反馈值
								 float set：目标值
  * @ 返回值  ： PID计算输出值
	* @ 涉及变量： 无
  ********************************************************************/
float PID_count(PID *pid, float now, float set)
{
	/*PID误差计算*/
	pid->error_this_time = set - now;

	/*PID死区*/
	if (pid->dead_zone_flag == 1)
	{
		/*误差处于死区内*/
		if (fabsf(pid->error_this_time) <= pid->dead_zone)
		{
			/*误差置0*/
			pid->error_this_time = 0;
		}
	}

	/*PID误差和计算*/
	pid->error_sum += pid->error_this_time;

	/*积分限幅*/
	if (pid->I_limit_flag == 1)
	{
		/*积分项超过积分限幅值*/
		if (pid->error_sum > pid->I_SUM_limit)
		{
			/*积分项赋值限幅值*/
			pid->error_sum = pid->I_SUM_limit;
		}
		/*积分项小于积分限幅负值*/
		else if (pid->error_sum < -pid->I_SUM_limit)
		{
			/*积分项赋值限幅负值*/
			pid->error_sum = -pid->I_SUM_limit;
		}
	}

	/*PID输出计算*/
	pid->output = pid->KP * pid->error_this_time + pid->KI * pid->error_sum + pid->KD * (pid->error_this_time - pid->error_last_time);

	/*输出限幅*/
	if (pid->OUTPUT_limit_flag == 1)
	{
		/*输出值超过输出限幅值*/
		if (pid->output > pid->OUTPUT_limit)
		{
			/*输出值赋值限幅值*/
			pid->output = pid->OUTPUT_limit;
		}
		/*输出值小于输出限幅值负值*/
		else if (pid->output < -pid->OUTPUT_limit)
		{
			/*输出值赋值限幅值负值*/
			pid->output = -pid->OUTPUT_limit;
		}
	}

	/*上一次误差赋值更新，用于下一次PID闭环计算*/
	pid->error_last_time = pid->error_this_time;

	/*返回PID计算输出值*/
	return pid->output;
}

/**********************************************************************
  * @ 函数名  ： PID_count_feedforward
  * @ 功能说明： 前馈PID计算函数
  * @ 参数    ： PID *pid：PID控制参数结构体
                 float now：当前值
                 float set：目标值
                 float set_last：上一次目标值
  * @ 返回值  ： PID计算输出值
  * @ 涉及函数： 无
  ********************************************************************/
float PID_count_feedforward(PID *pid, float now, float set, float set_last)
{
    /*PID误差项*/
    pid->error_this_time = set - now;

    /*死区处理*/
    if (pid->dead_zone_flag == 1)
    {
        /*误差在死区内*/
        if (fabsf(pid->error_this_time) <= pid->dead_zone)
        {
            /*误差置0*/
            pid->error_this_time = 0;
        }
    }

    /*PID积分计算*/
    pid->error_sum += pid->error_this_time;

    /*积分限幅*/
    if (pid->I_limit_flag == 1)
    {
        /*积分大于积分限幅值*/
        if (pid->error_sum > pid->I_SUM_limit)
        {
            /*积分值限幅值*/
            pid->error_sum = pid->I_SUM_limit;
        }
        /*积分小于积分限幅负值*/
        else if (pid->error_sum < -pid->I_SUM_limit)
        {
            /*积分值限幅负值*/
            pid->error_sum = -pid->I_SUM_limit;
        }
    }

    /*前馈项计算*/
    float feedforward = pid->KF * (set - set_last);

    /*PID输出计算*/
    pid->output = pid->KP * pid->error_this_time + 
                 pid->KI * pid->error_sum + 
                 pid->KD * (pid->error_this_time - pid->error_last_time) + 
                 feedforward;

    /*输出限幅*/
    if (pid->OUTPUT_limit_flag == 1)
    {
        /*输出值大于输出限幅值*/
        if (pid->output > pid->OUTPUT_limit)
        {
            /*输出值限幅值*/
            pid->output = pid->OUTPUT_limit;
        }
        /*输出值小于输出限幅值负值*/
        else if (pid->output < -pid->OUTPUT_limit)
        {
            /*输出值限幅值负值*/
            pid->output = -pid->OUTPUT_limit;
        }
    }

    /*上一次误差更新，用于下一次PID计算处理*/
    pid->error_last_time = pid->error_this_time;

    /*返回PID计算输出值*/
    return pid->output;
}


/**********************************************************************
  * @ 函数名  ： PID_reset
  * @ 功能说明： PID误差清空
  * @ 参数    ： PID *pid：PID控制参数结构体
  * @ 返回值  ： 无
	* @ 涉及变量： 无
  ********************************************************************/
void PID_reset(PID *pid)
{
	/*当前误差清零*/
	pid->error_this_time = 0.0f;

	/*上一次计算误差清零*/
	pid->error_last_time = 0.0f;

	/*误差和计算清零*/
	pid->error_sum = 0.0f;

	/*计算输出清零*/
	pid->output = 0.0f;
}

/******************************** 文件结束 ************************************/
