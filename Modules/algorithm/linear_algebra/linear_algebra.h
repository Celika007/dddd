/*******************************************************************************

					  版权所有 (C), 2023-2024, NCU_Roboteam

 *******************************************************************************
  文 件 名: linear_algebra.h
  版 本 号: V20240306.1
  作    者: 巢奖文
  生成日期: 2024年4月21日
  功能描述: 线性代数运算函数库
  补    充: 无

*******************************************************************************/

/****************************头文件开头必备内容********************************/
#ifndef _LINEAR_ALGEBRA_H
#define _LINEAR_ALGEBRA_H

/********************************包含头文件************************************/

#include <math.h>

/*******************************文件内宏定义***********************************/

/*******************************文件内结构体***********************************/

/*******************************全局变量声明***********************************/

/********************************自定义函数************************************/
/**********************************************************************
  * @ 函数名  ： matrix_transpose
  * @ 功能说明： 矩阵转置运算
  * @ 参数    ： double *matrix1：待转置矩阵
				 double *matrix2：转置结果存储矩阵
				 int matrix_row：待转置矩阵行数
				 int matrix_list：待转置矩阵列数
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void matrix_transpose(double *matrix1, double *matrix2, int matrix_row, int matrix_list);

/**********************************************************************
  * @ 函数名  ： matrix_addition
  * @ 功能说明： 矩阵加法运算
  * @ 参数    ： double *matrix1：左侧矩阵
				 int matrix1_row：左侧矩阵行数
				 int matrix1_list：左侧矩阵列数
				 int coefficient1：左侧矩阵符号系数
				 double *matrix2：右侧矩阵
				 int matrix2_row：右侧矩阵行数
				 int matrix2_list：右侧矩阵列数
				 int coefficient2：右侧矩阵符号系数
				 double *matrix3：运算结果存储矩阵
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void matrix_addition(double *matrix1, int matrix1_row, int matrix1_list, int coefficient1, double *matrix2, int matrix2_row, int matrix2_list, int coefficient2, double *matrix3);

/**********************************************************************
  * @ 函数名  ： matrix_multiplication
  * @ 功能说明： 矩阵相乘运算
  * @ 参数    ： double *matrix1：左乘矩阵
				 int matrix1_row：左乘矩阵行数
				 int matrix1_list：左乘矩阵列数
				 double *matrix2：右乘矩阵
				 int matrix2_row：右乘矩阵行数
				 int matrix2_list：右乘矩阵列数
				 double *matrix3：运算结果存储矩阵
  * @ 返回值  ： 无
  * @ 涉及变量： 无
  ********************************************************************/
void matrix_multiplication(double *matrix1, int matrix1_row, int matrix1_list, double *matrix2, int matrix2_row, int matrix2_list, double *matrix3);

/**********************************************************************
  * @ 函数名  ： matrix_inversion
  * @ 功能说明： 矩阵求逆运算
  * @ 参数    ： double *matrix1：求逆矩阵
				 int dimension：求逆矩阵维数
				 double *matrix2：运算结果存储矩阵
				 double *Identity_matrix：计算过程辅助单位矩阵
  * @ 返回值  ： 1：矩阵求逆成功
				-1：矩阵为奇异矩阵
  * @ 涉及变量： 无
  ********************************************************************/
int matrix_inversion(double *matrix1, double *matrix2, double *Identity_matrix, int dimension);

#endif

/******************************** 文件结束 ************************************/
