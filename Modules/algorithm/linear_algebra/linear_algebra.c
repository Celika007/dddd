/*******************************************************************************

					  版权所有 (C), 2023-2024, NCU_Roboteam

 *******************************************************************************
  文 件 名: linear_algebra.c
  版 本 号: V20240306.1
  作    者: 巢奖文
  生成日期: 2024年4月21日
  功能描述: 线性代数运算函数库
  补    充: 无

*******************************************************************************/

/********************************包含头文件************************************/
#include "linear_algebra.h"
/*******************************全局变量定义***********************************/

/********************************文件内变量************************************/

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
void matrix_transpose(double *matrix1, double *matrix2, int matrix_row, int matrix_list)
{
	/*函数内循环变量*/
	int i, j;

	/*遍历行*/
	for (i = 0; i < matrix_row; i++)
	{
		/*遍历列*/
		for (j = 0; j < matrix_list; j++)
		{
			/*行列转换*/
			*(matrix2 + j * matrix_row + i) = (*(matrix1 + i * matrix_list + j));
		}
	}
}

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
void matrix_addition(double *matrix1, int matrix1_row, int matrix1_list, int coefficient1, double *matrix2, int matrix2_row, int matrix2_list, int coefficient2, double *matrix3)
{
	/*函数内循环变量*/
	int i, j;

	/*侧矩阵为常数*/
	if (matrix1_row == 1 && matrix1_list == 1)
	{
		/*遍历行*/
		for (i = 0; i < matrix2_row; i++)
		{
			/*遍历列*/
			for (j = 0; j < matrix2_list; j++)
			{
				/*计算结果赋值结果存储矩阵*/
				*(matrix3 + i * matrix2_list + j) = (*matrix1) * coefficient1 + (*(matrix2 + i * matrix2_list + j)) * coefficient2;
			}
		}
	}
	/*右侧矩阵为常数*/
	else if (matrix2_row == 1 && matrix2_list == 1)
	{
		/*遍历行*/
		for (i = 0; i < matrix1_row; i++)
		{
			/*遍历列*/
			for (j = 0; j < matrix1_list; j++)
			{
				/*计算结果赋值结果存储矩阵*/
				*(matrix3 + i * matrix1_list + j) = (*(matrix1 + i * matrix1_list + j)) * coefficient1 + (*matrix2) * coefficient2;
			}
		}
	}
	/*两矩阵均不为常数*/
	else
	{
		/*遍历行*/
		for (i = 0; i < matrix1_row; i++)
		{
			/*遍历列*/
			for (j = 0; j < matrix1_list; j++)
			{
				/*计算结果赋值结果存储矩阵*/
				*(matrix3 + i * matrix1_list + j) = (*(matrix1 + i * matrix1_list + j)) * coefficient1 + (*(matrix2 + i * matrix2_list + j)) * coefficient2;
			}
		}
	}
}

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
void matrix_multiplication(double *matrix1, int matrix1_row, int matrix1_list, double *matrix2, int matrix2_row, int matrix2_list, double *matrix3)
{
	/*函数内循环变量*/
	int i, j, k;

	/*左乘矩阵为一维方阵*/
	if (matrix1_row == 1 && matrix1_list == 1)
	{
		/*遍历行*/
		for (i = 0; i < matrix2_row; i++)
		{
			/*遍历列*/
			for (j = 0; j < matrix2_list; j++)
			{
				/*对应元素相乘并存储结果*/
				*(matrix3 + i * matrix2_list + j) = (*matrix1) * (*(matrix2 + i * matrix2_list + j));
			}
		}
	}
	/*右侧矩阵为一维方阵*/
	else if (matrix2_row == 1 && matrix2_list == 1)
	{
		/*遍历行*/
		for (i = 0; i < matrix1_row; i++)
		{
			/*遍历列*/
			for (j = 0; j < matrix1_list; j++)
			{
				/*对应元素相乘并存储结果*/
				*(matrix3 + i * matrix1_list + j) = (*matrix2) * (*(matrix1 + i * matrix1_list + j));
			}
		}
	}
	/*两相乘矩阵均不为一维方阵*/
	else
	{
		/*根据矩阵乘法定义运算，遍历行*/
		for (i = 0; i < matrix1_row; i++)
		{
			/*遍历列*/
			for (j = 0; j < matrix2_list; j++)
			{
				/*当前元素计算结果存储位置置零*/
				*(matrix3 + i * matrix2_list + j) = 0;

				/*计算当前元素行乘列结果并存储于结果存储数组*/
				for (k = 0; k < matrix1_list; k++)
				{
					/*相应元素相乘并存储结果*/
					*(matrix3 + i * matrix2_list + j) += ((*(matrix1 + i * matrix1_list + k)) * (*(matrix2 + k * matrix2_list + j)));
				}
			}
		}
	}
}

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
int matrix_inversion(double *matrix1, double *matrix2, double *Identity_matrix, int dimension)
{
	/*函数内循环变量*/
	int i, j, k;

	/*函数内临时结果存储变量*/
	double temp;

	/*确保传入单位矩阵为单位矩阵，遍历行*/
	for (i = 0; i < dimension; i++)
	{
		/*遍历列*/
		for (j = 0; j < dimension; j++)
		{
			/*对角元素*/
			if (i == j)
			{
				/*对角元素置1*/
				*(Identity_matrix + i * dimension + j) = 1.0f;
			}
			/*非对角元素*/
			else
			{
				/*非对角元素置0*/
				*(Identity_matrix + i * dimension + j) = 0.0f;
			}
		}
	}

	/*伴随矩阵法求逆*/
	for (i = 0; i < dimension; i++)
	{
		/*当前主元数值为0*/
		if (*(matrix1 + i * dimension + i) == 0)
		{
			/*当前列向下寻找非零元素*/
			for (j = i; j < dimension; j++)
			{
				/*当前元素非零*/
				if (*(matrix1 + j * dimension + i) != 0)
				{
					/*两行元素交换*/
					for (k = 0; k < dimension; k++)
					{
						/*求逆矩阵操作*/
						/*临时存储被交换元素*/
						temp = *(matrix1 + i * dimension + k);

						/*被交换元素赋值*/
						*(matrix1 + i * dimension + k) = *(matrix1 + j * dimension + k);

						/*交换元素赋值*/
						*(matrix1 + j * dimension + k) = temp;


						/*辅助单位矩阵同步操作*/
						/*临时存储被交换元素*/
						temp = *(Identity_matrix + i * dimension + k);

						/*被交换元素赋值*/
						*(Identity_matrix + i * dimension + k) = *(Identity_matrix + j * dimension + k);

						/*交换元素赋值*/
						*(Identity_matrix + j * dimension + k) = temp;
					}
				}
				/*当前列所有元素均为0，矩阵为奇异矩阵*/
				else if (*(matrix1 + j * dimension + i) == 0 && j == dimension - 1)
				{
					/*返回-1*/
					return -1;
				}
			}
		}

		/*临时变量赋值主元素值*/
		temp = *(matrix1 + i * dimension + i);

		/*将当前处理行主元位置化为1*/
		for (j = 0; j < dimension; j++)
		{
			/*求逆矩阵操作*/
			*(matrix1 + i * dimension + j) /= temp;

			/*辅助单位矩阵同步操作*/
			*(Identity_matrix + i * dimension + j) /= temp;
		}

		/*将当前主元所在列其他元素置零*/
		for (j = 0; j < dimension; j++)
		{
			/*遍历至主元所在位置*/
			if (j == i)
			{
				/*跳过本次循环*/
				continue;
			}
			/*遍历至其他位置*/
			else
			{
				/*临时变量赋值主元值*/
				temp = *(matrix1 + j * dimension + i);

				/*遍历处理当前元素所在行元素*/
				for (k = 0; k < dimension; k++)
				{
					/*求逆矩阵操作*/
					*(matrix1 + j * dimension + k) = *(matrix1 + j * dimension + k) - temp * (*(matrix1 + i * dimension + k));

					/*辅助单位矩阵同步操作*/
					*(Identity_matrix + j * dimension + k) = *(Identity_matrix + j * dimension + k) - temp * (*(Identity_matrix + i * dimension + k));
				}
			}
		}
	}

	/*循环赋值求逆结果，遍历行*/
	for (i = 0; i < dimension; i++)
	{
		/*遍历列*/
		for (j = 0; j < dimension; j++)
		{
			/*结果存储矩阵赋值*/
			*(matrix2 + i * dimension + j) = *(Identity_matrix + i * dimension + j);
		}
	}

	/*矩阵求逆完成，返回1*/
	return 1;
}

/******************************** 文件结束 ************************************/
