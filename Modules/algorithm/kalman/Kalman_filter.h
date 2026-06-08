/*******************************************************************************

					  版权所有 (C), 2023-2024, NCU_Roboteam

 *******************************************************************************
  文 件 名: Kalman_filter.h
  版 本 号: V20240330.1
  作    者: 巢奖文
  生成日期: 2024年3月30日
  功能描述: 卡尔曼滤波相关函数库
  补    充: 无

*******************************************************************************/

/****************************头文件开头必备内容********************************/
#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

/********************************包含头文件************************************/
#include "linear_algebra.h"

/*******************************文件内宏定义***********************************/
/*卡尔曼滤波阶数*/
#define order 3

/*时间步长*/
#define time_step 0.02f

/*******************************文件内结构体***********************************/
/*卡尔曼滤波器参数结构体*/
typedef struct
{
	/*先验估计*/
	double Prior_estimation[3][1];

	/*后验估计*/
	double Posteriori_estimate[3][1];

	/*控制输入*/
	double Control_input[3][1];


	/*状态转移矩阵*/
	double A[3][3];

	/*状态转移矩阵转置矩阵*/
	double A_T[3][3];

	/*控制矩阵*/
	double B[3][3];


	/*量纲转换矩阵*/
	double H[1][1];

	/*量纲转换矩阵转置矩阵*/
	double H_T[1][1];

	/*观测噪声*/
	double R[3][3];

	/*测量噪声*/
	double Q[3][3];


	/*先验误差协方差矩阵*/
	double P_Prior[3][3];

	/*误差协方差*/
	double P_error[3][3];


	/*卡尔曼增益*/
	double Kk[3][3];

	/*单位矩阵*/
	double Identity_matrix[3][3];


	/*观测量*/
	double Zk[3][1];

	/*临时结果存储矩阵*/
	double temp_matrix1[3][3];

	/*临时结果存储矩阵*/
	double temp_matrix2[3][3];

	/*临时结果存储矩阵*/
	double temp_matrix3[3][3];

} Parameter_init;


/*******************************全局变量声明***********************************/
/*卡尔曼估计位置x*/
extern float Kalman_chassis_position_x;

/*卡尔曼估计位置y*/
extern float Kalman_chassis_position_y;

/*y方向卡尔曼滤波器全局变量结构体*/
extern Parameter_init Kalman_filter_parameter_Y;

/*x方向卡尔曼滤波器全局变量结构体*/
extern Parameter_init Kalman_filter_parameter_X;

/********************************自定义函数************************************/
/**********************************************************************
  * @ 函数名  ： Kalman_filter_parameter_initialization
  * @ 功能说明： 卡尔曼滤波器参数初始化
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_parameter_initialization(Parameter_init *Kalman_filter_parameter);

/**********************************************************************
  * @ 函数名  ： Kalman_filter_Prior_estimation
  * @ 功能说明： 卡尔曼滤波先验估计计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_Prior_estimation(Parameter_init *Kalman_filter_parameter);

/**********************************************************************
  * @ 函数名  ： Kalman_filter_P_Prior
  * @ 功能说明： 卡尔曼滤波先验误差协方差计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_P_Prior(Parameter_init *Kalman_filter_parameter);

/**********************************************************************
  * @ 函数名  ： Kalman_filter_Kk
  * @ 功能说明： 卡尔曼滤波卡尔曼增益计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_Kk(Parameter_init *Kalman_filter_parameter);

/**********************************************************************
  * @ 函数名  ： Kalman_filter_Posteriori_estimate
  * @ 功能说明： 卡尔曼滤波后验估计计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
				 double position：位置观测值
				 double velocity：速度观测值
				 double acc：加速度观测值
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_Posteriori_estimate(Parameter_init *Kalman_filter_parameter);

/**********************************************************************
  * @ 函数名  ： Kalman_filter_P_error
  * @ 功能说明： 卡尔曼滤波误差协方差更新计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_P_error(Parameter_init *Kalman_filter_parameter);

/**********************************************************************
  * @ 函数名  ： Kalman_filter
  * @ 功能说明： x方向卡尔曼滤波器
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter(Parameter_init *Kalman_filter_parameter, double position, double velocity, double acc);

#endif

/******************************** 文件结束 ************************************/
