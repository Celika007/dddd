/*******************************************************************************

                      版权所有 (C), 2023-2024, NCU_Roboteam

 *******************************************************************************
  文 件 名: coordinate_system.c
  版 本 号: V20240306.1
  作    者: 巢奖文
  生成日期: 2024年3月6日
  功能描述: 坐标系转换函数库
  补    充: 无

*******************************************************************************/

/********************************包含头文件************************************/
#include "coordinate_system.h"

/*******************************文件内变量***********************************/

/********************************宏定义************************************/

/********************************内部函数************************************/
/**********************************************************************
  * @ 函数名  ： rotation_of_coordinate_system
  * @ 功能说明： 坐标系旋转函数
  * @ 参数    ： float x：原始坐标x分量
	               float y：原始坐标y分量
								 float z：原始坐标z分量
								 float target_x：目标坐标x分量
								 float target_y：目标坐标y分量
								 float target_z：目标坐标z分量
								 int sign：旋转方向，1表示正向旋转，-1表示反向旋转
  * @ 返回值  ： 无
  * @ 涉及变量： IMU_DATA Saber_DATA：姿态数据存储结构体
  ********************************************************************/
void rotation_of_coordinate_system(float x, float y, float z, float *target_x, float *target_y, float *target_z, int sign)
{
	/*定义中间变量，用于存储坐标系转换过程中的中间值*/
	float x1, y1, z1;

	/*定义欧拉角变量，用于存储姿态角度*/
	float R, P, Y;


	/*获取横滚角并转换为弧度制*/
	R = sign * Saber_DATA.Saber_imu_eluer.ELUER_ROLL / 180.0f * PI;

	/*获取俯仰角并转换为弧度制*/
	P = sign * Saber_DATA.Saber_imu_eluer.ELUER_PITCH / 180.0f * PI;

	/*获取偏航角并转换为弧度制*/
	Y = sign * Saber_DATA.Saber_imu_eluer.ELUER_YAW / 180.0f * PI;


	/*绕z轴旋转坐标变换*/
	/*x轴坐标变换计算*/
	x1 = x * arm_cos_f32(Y) + y * arm_sin_f32(Y);

	/*y轴坐标变换计算*/
	y1 = -x * arm_sin_f32(Y) + y * arm_cos_f32(Y);

	/*z轴坐标变换计算*/
	z1 = z;


	/*绕x轴旋转坐标变换*/
	/*x轴坐标变换计算*/
	x = x1;

	/*y轴坐标变换计算*/
	y = y1 * arm_cos_f32(P) + z1 * arm_sin_f32(P);

	/*z轴坐标变换计算*/
	z = -y1 * arm_sin_f32(P) + z1 * arm_cos_f32(P);


	/*绕y轴旋转坐标变换*/
	/*x轴坐标变换计算*/
	x1 = x * arm_cos_f32(R) - z * arm_sin_f32(R);

	/*y轴坐标变换计算*/
	y1 = y;

	/*z轴坐标变换计算*/
	z1 = x * arm_sin_f32(R) + z * arm_cos_f32(R);


	/*x轴坐标变换结果*/
	*target_x = x1;

	/*y轴坐标变换结果*/
	*target_y = y1;

	/*z轴坐标变换结果*/
	*target_z = z1;
}

float convert_angle_system(float angle_3oclock) {
    // 将3点钟方向为0度的角度转换为12点钟方向为0度的角度
    float angle_12oclock = angle_3oclock - 90.0f;
    
    // 将角度限制在-180到180度的范围内
    while (angle_12oclock > 180.0f) {
        angle_12oclock -= 360.0f;
    }
    while (angle_12oclock <= -180.0f) {
        angle_12oclock += 360.0f;
    }
    
    return angle_12oclock;
}


/******************************** 文件结束 ************************************/
