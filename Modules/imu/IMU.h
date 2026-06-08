/*******************************************************************************

                      版权所有 (C), 2023-2024, NCU_Roboteam

 *******************************************************************************
  文 件 名: IMU.h
  版 本 号: V20240330.1
  作    者: 巢奖文
  生成日期: 2024年3月30日
  功能描述: IMU相关函数库
  补    充: 无

*******************************************************************************/

/****************************头文件开头必备内容********************************/
#ifndef __IMU_H
#define __IMU_H

/********************************包含头文件************************************/
#include "main.h"
#include "bsp_usart.h"
#include "usart.h"
#include <string.h> //临时引用，后续封装移除
#include "coordinate_system.h" //临时引用，后续封装移除
//#include "chassis.h" //临时引用，后续封装移除

/*******************************文件内宏定义***********************************/
/*惯导数据接收数组大小*/
#define Saber_Data_rxbuffer_size 57

/*重力加速度大小*/
#define Gravity 9.81f

/*******************************文件内结构体***********************************/
/*IMU加速度数据存储结构体*/
typedef struct
{
	/*x轴加速度*/
	float ACC_X;

	/*y轴加速度*/
	float ACC_Y;

	/*z轴加速度*/
	float ACC_Z;
} IMU_ACC;

/*IMU陀螺仪数据存储结构体*/
typedef struct
{
	/*x轴角速度*/
	float GYRO_X;

	/*y轴角速度*/
	float GYRO_Y;

	/*z轴角速度*/
	float GYRO_Z;
} IMU_GYRO;

/*IMU欧拉角数据存储结构体*/
typedef struct
{
	/*横滚角*/
	float ELUER_ROLL;

	/*俯仰角*/
	float ELUER_PITCH;

	/*偏航角*/
	float ELUER_YAW_now;

	/*满足通信协议格式多余变量,此处用来将单圈测量改为多圈数值*/
	float ELUER_YAW_last;
	
	/*多圈计算后偏航角*/
	float ELUER_YAW;
	
	/*旋转圈数*/
	int number_of_turns;
} IMU_ELUER;

/*IMU数据存储结构体*/
typedef struct
{
	/*IMU加速度数据*/
	IMU_ACC Saber_imu_acc;

	/*消除重力影响加速度数据*/
	IMU_ACC Normal_acc;

	/*世界坐标系加速度数据*/
	IMU_ACC World_ACC;

	/*IMU陀螺仪数据*/
	IMU_GYRO Saber_imu_gyro;

	/*IMU欧拉角数据*/
	IMU_ELUER Saber_imu_eluer;
} IMU_DATA;

/*******************************全局变量声明***********************************/
/*惯导数据接收数组*/
extern uint8_t Saber_Data_rxbuffer[Saber_Data_rxbuffer_size];

/*IMU数据存储结构体*/
extern IMU_DATA Saber_DATA;

/*IMU初始化成功标志*/
extern int IMU_init_flag;

/********************************自定义函数************************************/
/**********************************************************************
  * @ 函数名  ： Eliminating_the_influence_of_gravity
  * @ 功能说明： 加速度数据消除重力影响
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： IMU_DATA Saber_DATA：惯导数据存储结构体
  ********************************************************************/
void Eliminating_the_influence_of_gravity(void);

/**********************************************************************
  * @ 函数名  ： Rotation_processing
  * @ 功能说明： 加速度数据从机器坐标系转换为世界坐标系
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： IMU_DATA Saber_DATA：惯导数据存储结构体
  ********************************************************************/
void Rotation_processing(void);

/**********************************************************************
  * @ 函数名  ： Saber_imu_data_analysis
  * @ 功能说明： 惯导数据解析
  * @ 参数    ： uint8_t Data[57]：惯导数据存储数组
  * @ 返回值  ： 无
  * @ 涉及变量： IMU_DATA Saber_DATA：惯导数据存储结构体
  ********************************************************************/
void Saber_imu_data_analysis(uint8_t Data[57]);

/**********************************************************************
  * @ 函数名  ： IMU_USART6_Init
  * @ 功能说明： 使用 BSP USART 注册 USART6 并启动 DMA+空闲接收
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： Saber_Data_rxbuffer、Saber_DATA
  ********************************************************************/
void IMU_USART6_Init(void);

#endif

/******************************** 文件结束 ************************************/
