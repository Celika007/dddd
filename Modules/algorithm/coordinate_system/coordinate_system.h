/*
*******************************************************************************

                      版权所有 (C), 2025-2026, NCU_Roboteam

 *******************************************************************************
  文 件 名: coordinate_system.h
  版 本 号: V20251113.1
  作    者: 肖景煊
  生成日期: 2025年11月13日
  功能描述: 坐标系旋转与角度转换算法接口声明
  补    充: 无

*******************************************************************************/

/*头文件开头必备内容宏定义保护，防止重复包含*/
#ifndef _COORDINATE_SYSTEM_H
#define _COORDINATE_SYSTEM_H

#include "IMU.h"

#include "arm_math.h"

/*该模块仅提供函数声明，无需额外头文件依赖*/

/*暂无宏定义*/

/*暂无结构体*/

/*暂无全局变量声明*/

/*坐标系旋转，将(x, y, z)按IMU朝向旋转到目标坐标系；sign为方向符号（1正旋，-1逆旋）*/
void rotation_of_coordinate_system(float x, float y, float z, float *target_x, float *target_y, float *target_z, int sign);

/*角度转换，将“3点钟方向为零”的角度系转换为常规数学角度系*/
float convert_angle_system(float angle_3oclock);

/*文件结束*/
#endif

/*文件结束*/
