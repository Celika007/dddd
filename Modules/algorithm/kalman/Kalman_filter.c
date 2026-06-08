/*******************************************************************************

					  版权所有 (C), 2023-2024, NCU_Roboteam

 *******************************************************************************
  文 件 名: Kalman_filter.c
  版 本 号: V20240330.1
  作    者: 巢奖文
  生成日期: 2024年3月30日
  功能描述: 卡尔曼滤波相关函数库
  补    充: 无

*******************************************************************************/
#include "Kalman_filter.h"

/*******************************全局变量定义***********************************/
/*卡尔曼估计位置x*/
float Kalman_chassis_position_x = 0;

/*卡尔曼估计位置y*/
float Kalman_chassis_position_y = 0;

/*x方向卡尔曼滤波器全局变量结构体*/
Parameter_init Kalman_filter_parameter_X;

/*y方向卡尔曼滤波器全局变量结构体*/
Parameter_init Kalman_filter_parameter_Y;

/********************************文件内变量************************************/

/********************************自定义函数************************************/
/**********************************************************************
  * @ 函数名  ： Kalman_filter_parameter_initialization
  * @ 功能说明： 卡尔曼滤波器参数初始化
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_parameter_initialization(Parameter_init *Kalman_filter_parameter)
{
	/*先验估计矩阵初始化*/
	Kalman_filter_parameter->Prior_estimation[0][0] = 0.0f;
	Kalman_filter_parameter->Prior_estimation[1][0] = 0.0f;
	Kalman_filter_parameter->Prior_estimation[2][0] = 0.0f;

	/*后验估计矩阵初始化*/
	Kalman_filter_parameter->Posteriori_estimate[0][0] = 0.0f;
	Kalman_filter_parameter->Posteriori_estimate[1][0] = 0.0f;
	Kalman_filter_parameter->Posteriori_estimate[2][0] = 0.0f;

	/*控制输入矩阵初始化*/
	Kalman_filter_parameter->Control_input[0][0] = 0.0f;
	Kalman_filter_parameter->Control_input[1][0] = 0.0f;
	Kalman_filter_parameter->Control_input[2][0] = 0.0f;

	/*状态转移矩阵初始化*/
	Kalman_filter_parameter->A[0][0] = 1.0f;
	Kalman_filter_parameter->A[0][1] = time_step;
	Kalman_filter_parameter->A[0][2] = time_step * time_step * 0.5f;
	Kalman_filter_parameter->A[1][0] = 0.0f;
	Kalman_filter_parameter->A[1][1] = 1.0f;
	Kalman_filter_parameter->A[1][2] = time_step;
	Kalman_filter_parameter->A[2][0] = 0.0f;
	Kalman_filter_parameter->A[2][1] = 0.0f;
	Kalman_filter_parameter->A[2][2] = 1.0f;

	/*状态转移矩阵转置矩阵初始化*/
	matrix_transpose(&(Kalman_filter_parameter->A[0][0]), &(Kalman_filter_parameter->A_T[0][0]), 3, 3);

	/*控制矩阵初始化*/
	Kalman_filter_parameter->B[0][0] = 0.0f;
	Kalman_filter_parameter->B[0][1] = 0.0f;
	Kalman_filter_parameter->B[0][2] = 0.0f;
	Kalman_filter_parameter->B[1][0] = 0.0f;
	Kalman_filter_parameter->B[1][1] = 0.0f;
	Kalman_filter_parameter->B[1][2] = 0.0f;
	Kalman_filter_parameter->B[2][0] = 0.0f;
	Kalman_filter_parameter->B[2][1] = 0.0f;
	Kalman_filter_parameter->B[2][2] = 0.0f;

	/*量纲转换矩阵初始化*/
	Kalman_filter_parameter->H[0][0] = 1.0f;

	/*量纲转换矩阵转置矩阵初始化*/
	matrix_transpose(&(Kalman_filter_parameter->H[0][0]), &(Kalman_filter_parameter->H_T[0][0]), 1, 1);

	/*观测噪声矩阵初始化*/
	Kalman_filter_parameter->R[0][0] = 50.0f;
	Kalman_filter_parameter->R[0][1] = 0.0f;
	Kalman_filter_parameter->R[0][2] = 0.0f;

	Kalman_filter_parameter->R[1][0] = 0.0f;
	Kalman_filter_parameter->R[1][1] = 10.0f;
	Kalman_filter_parameter->R[1][2] = 0.0f;

	Kalman_filter_parameter->R[2][0] = 0.0f;
	Kalman_filter_parameter->R[2][1] = 0.0f;
	Kalman_filter_parameter->R[2][2] = 2.0;

	/*测量噪声矩阵初始化*/
	Kalman_filter_parameter->Q[0][0] = 0.5f;
	Kalman_filter_parameter->Q[0][1] = 0.0f;
	Kalman_filter_parameter->Q[0][2] = 0.0f;

	Kalman_filter_parameter->Q[1][0] = 0.0f;
	Kalman_filter_parameter->Q[1][1] = 0.5f;
	Kalman_filter_parameter->Q[1][2] = 0.0f;

	Kalman_filter_parameter->Q[2][0] = 0.0f;
	Kalman_filter_parameter->Q[2][1] = 0.0f;
	Kalman_filter_parameter->Q[2][2] = 1.0f;

	/*先验误差协方差矩阵初始化*/
	Kalman_filter_parameter->P_Prior[0][0] = 1.0f;
	Kalman_filter_parameter->P_Prior[0][1] = 0.0f;
	Kalman_filter_parameter->P_Prior[0][2] = 0.0f;
	Kalman_filter_parameter->P_Prior[1][0] = 0.0f;
	Kalman_filter_parameter->P_Prior[1][1] = 1.0f;
	Kalman_filter_parameter->P_Prior[1][2] = 0.0f;
	Kalman_filter_parameter->P_Prior[2][0] = 0.0f;
	Kalman_filter_parameter->P_Prior[2][1] = 0.0f;
	Kalman_filter_parameter->P_Prior[2][2] = 1.0f;

	/*误差协方差矩阵初始化*/
	Kalman_filter_parameter->P_error[0][0] = 1.0f;
	Kalman_filter_parameter->P_error[0][1] = 0.0f;
	Kalman_filter_parameter->P_error[0][2] = 0.0f;
	Kalman_filter_parameter->P_error[1][0] = 0.0f;
	Kalman_filter_parameter->P_error[1][1] = 1.0f;
	Kalman_filter_parameter->P_error[1][2] = 0.0f;
	Kalman_filter_parameter->P_error[2][0] = 0.0f;
	Kalman_filter_parameter->P_error[2][1] = 0.0f;
	Kalman_filter_parameter->P_error[2][2] = 1.0f;

	/*卡尔曼增益矩阵初始化*/
	Kalman_filter_parameter->Kk[0][0] = 1.0f;
	Kalman_filter_parameter->Kk[0][1] = 0.0f;
	Kalman_filter_parameter->Kk[0][2] = 0.0f;
	Kalman_filter_parameter->Kk[1][0] = 0.0f;
	Kalman_filter_parameter->Kk[1][1] = 1.0f;
	Kalman_filter_parameter->Kk[1][2] = 0.0f;
	Kalman_filter_parameter->Kk[2][0] = 0.0f;
	Kalman_filter_parameter->Kk[2][1] = 0.0f;
	Kalman_filter_parameter->Kk[2][2] = 1.0f;

	/*单位矩阵初始化*/
	Kalman_filter_parameter->Identity_matrix[0][0] = 1.0f;
	Kalman_filter_parameter->Identity_matrix[0][1] = 0.0f;
	Kalman_filter_parameter->Identity_matrix[0][2] = 0.0f;
	Kalman_filter_parameter->Identity_matrix[1][0] = 0.0f;
	Kalman_filter_parameter->Identity_matrix[1][1] = 1.0f;
	Kalman_filter_parameter->Identity_matrix[1][2] = 0.0f;
	Kalman_filter_parameter->Identity_matrix[2][0] = 0.0f;
	Kalman_filter_parameter->Identity_matrix[2][1] = 0.0f;
	Kalman_filter_parameter->Identity_matrix[2][2] = 1.0f;
}

/**********************************************************************
  * @ 函数名  ： Kalman_filter_Prior_estimation
  * @ 功能说明： 卡尔曼滤波先验估计计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_Prior_estimation(Parameter_init *Kalman_filter_parameter)
{
	/*状态转移矩阵乘上一时刻后验估计矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->A[0][0]), 3, 3, &(Kalman_filter_parameter->Posteriori_estimate[0][0]), 3, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]));

	/*控制矩阵乘控制输入矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->B[0][0]), 3, 3, &(Kalman_filter_parameter->Control_input[0][0]), 3, 1, &(Kalman_filter_parameter->temp_matrix2[0][0]));

	/*先验估计计算*/
	matrix_addition(&(Kalman_filter_parameter->temp_matrix1[0][0]), 3, 1, 1, &(Kalman_filter_parameter->temp_matrix2[0][0]), 3, 1, 1, &(Kalman_filter_parameter->Prior_estimation[0][0]));
}

/**********************************************************************
  * @ 函数名  ： Kalman_filter_P_Prior
  * @ 功能说明： 卡尔曼滤波先验误差协方差计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_P_Prior(Parameter_init *Kalman_filter_parameter)
{
	/*状态转移矩阵乘误差协方差矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->A[0][0]), 3, 3, &(Kalman_filter_parameter->P_error[0][0]), 3, 3, &(Kalman_filter_parameter->temp_matrix1[0][0]));

	/*状态转移矩阵乘误差协方差结果乘状态转移矩阵转置矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->temp_matrix1[0][0]), 3, 3, &(Kalman_filter_parameter->A_T[0][0]), 3, 3, &(Kalman_filter_parameter->temp_matrix2[0][0]));

	/*先验误差协方差计算*/
	matrix_addition(&(Kalman_filter_parameter->temp_matrix2[0][0]), 3, 3, 1, &(Kalman_filter_parameter->Q[0][0]), 3, 3, 1, &(Kalman_filter_parameter->P_Prior[0][0]));

}

/**********************************************************************
  * @ 函数名  ： Kalman_filter_Kk
  * @ 功能说明： 卡尔曼滤波卡尔曼增益计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_Kk(Parameter_init *Kalman_filter_parameter)
{
	/*量纲转换矩阵乘先验误差协方差矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->H[0][0]), 1, 1, &(Kalman_filter_parameter->P_Prior[0][0]), 3, 3, &(Kalman_filter_parameter->temp_matrix1[0][0]));

	/*量纲转换矩阵乘先验误差协方差矩阵结果乘量纲转换矩阵转置矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->temp_matrix1[0][0]), 3, 3, &(Kalman_filter_parameter->H_T[0][0]), 1, 1, &(Kalman_filter_parameter->temp_matrix2[0][0]));

	/*求逆前矩阵求和计算*/
	matrix_addition(&(Kalman_filter_parameter->temp_matrix2[0][0]), 3, 3, 1, &(Kalman_filter_parameter->R[0][0]), 3, 3, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]));

	/*矩阵求逆计算*/
	matrix_inversion(&(Kalman_filter_parameter->temp_matrix1[0][0]), &(Kalman_filter_parameter->temp_matrix2[0][0]), &(Kalman_filter_parameter->temp_matrix3[0][0]), 3);

	/*量纲转换矩阵乘先验误差协方差矩阵结果乘量纲转换矩阵转置矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->P_Prior[0][0]), 3, 3, &(Kalman_filter_parameter->H_T[0][0]), 1, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]));

	/*卡尔曼增益计算*/
	matrix_multiplication(&(Kalman_filter_parameter->temp_matrix1[0][0]), 3, 3, &(Kalman_filter_parameter->temp_matrix2[0][0]), 3, 3, &(Kalman_filter_parameter->Kk[0][0]));
}

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
void Kalman_filter_Posteriori_estimate(Parameter_init *Kalman_filter_parameter)
{
	/*量纲转换矩阵乘先验估计矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->H[0][0]), 1, 1, &(Kalman_filter_parameter->Prior_estimation[0][0]), 3, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]));

	/*观测矩阵同上一步计算结果做差*/
	matrix_addition(&(Kalman_filter_parameter->Zk[0][0]), 3, 1, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]), 3, 1, -1, &(Kalman_filter_parameter->temp_matrix2[0][0]));

	/*卡尔曼增益矩阵乘上一步计算结果*/
	matrix_multiplication(&(Kalman_filter_parameter->Kk[0][0]), 3, 3, &(Kalman_filter_parameter->temp_matrix2[0][0]), 3, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]));

	/*后验估计计算*/
	matrix_addition(&(Kalman_filter_parameter->Prior_estimation[0][0]), 3, 1, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]), 3, 1, 1, &(Kalman_filter_parameter->Posteriori_estimate[0][0]));
}

/**********************************************************************
  * @ 函数名  ： Kalman_filter_P_error
  * @ 功能说明： 卡尔曼滤波误差协方差更新计算
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter_P_error(Parameter_init *Kalman_filter_parameter)
{
	/*卡尔曼增益矩阵乘量纲转换矩阵*/
	matrix_multiplication(&(Kalman_filter_parameter->Kk[0][0]), 3, 3, &(Kalman_filter_parameter->H[0][0]), 1, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]));

	/*单位矩阵同上一步计算结果做差*/
	matrix_addition(&(Kalman_filter_parameter->Identity_matrix[0][0]), 3, 3, 1, &(Kalman_filter_parameter->temp_matrix1[0][0]), 3, 3, -1, &(Kalman_filter_parameter->temp_matrix2[0][0]));

	/*更新误差协方差计算*/
	matrix_multiplication(&(Kalman_filter_parameter->temp_matrix2[0][0]), 3, 3, &(Kalman_filter_parameter->P_Prior[0][0]), 3, 3, &(Kalman_filter_parameter->P_error[0][0]));

}

/**********************************************************************
  * @ 函数名  ： Kalman_filter
  * @ 功能说明： x方向卡尔曼滤波器
  * @ 参数    ： Parameter_init *Kalman_filter_parameter：卡尔曼滤波参数结构体
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void Kalman_filter(Parameter_init *Kalman_filter_parameter, double position, double velocity, double acc)
{
	/*先验估计计算*/
	Kalman_filter_Prior_estimation(Kalman_filter_parameter);

	if(isnan(Kalman_filter_parameter->Prior_estimation[0][0]) == 1)
	{
		Kalman_filter_parameter->Prior_estimation[0][0] = position;
	}
	
	if(isnan(Kalman_filter_parameter->Prior_estimation[1][0]) == 1)
	{
		Kalman_filter_parameter->Prior_estimation[1][0] = velocity;
	}
	
	if(isnan(Kalman_filter_parameter->Prior_estimation[2][0]) == 1)
	{
		Kalman_filter_parameter->Prior_estimation[2][0] = acc;
	}
	
	/*先验误差协方差计算*/
	Kalman_filter_P_Prior(Kalman_filter_parameter);

	/*卡尔曼增益计算*/
	Kalman_filter_Kk(Kalman_filter_parameter);

	/*位置观测量赋值*/
	Kalman_filter_parameter->Zk[0][0] = position;

	/*速度观测量赋值*/
	Kalman_filter_parameter->Zk[1][0] = velocity;

	/*加速度观测量赋值*/
	Kalman_filter_parameter->Zk[2][0] = acc;

	/*后验估计计算*/
	Kalman_filter_Posteriori_estimate(Kalman_filter_parameter);

	/*更新误差协方差计算*/
	Kalman_filter_P_error(Kalman_filter_parameter);
}

/******************************** 文件结束 ************************************/
