/*******************************************************************************

                      版权所有 (C), 2023-2024, NCU_Roboteam

 *******************************************************************************
  文 件 名: IMU.c
  版 本 号: V20240330.1
  作    者: 巢奖文
  生成日期: 2024年3月30日
  功能描述: IMU相关函数库
  补    充: 无

*******************************************************************************/

/********************************包含头文件************************************/
#include "IMU.h"

/*******************************文件内宏定义***********************************/

/*******************************文件内结构体***********************************/

/*******************************全局变量定义***********************************/
/*惯导数据接收数组*/
uint8_t Saber_Data_rxbuffer[Saber_Data_rxbuffer_size] = {0};

/*IMU数据存储结构体*/
IMU_DATA Saber_DATA = {0};

/*IMU初始化成功标志*/
int IMU_init_flag = 0;

/* USART6 实例指针，用于在回调中访问接收缓冲 */
static USARTInstance *imu_usart_instance = NULL;
static uint8_t imu_latest_frame[Saber_Data_rxbuffer_size] = {0};
static volatile uint8_t imu_latest_seq = 0;
static uint8_t imu_parsed_seq = 0;

/* 惯导串口接收回调：仅缓存最新完整帧 */
static void IMU_USART_Callback(void)
{
    if (imu_usart_instance == NULL)
    {
        return;
    }

    if (imu_usart_instance->recv_len != Saber_Data_rxbuffer_size)
    {
        return;
    }

    memcpy(imu_latest_frame, imu_usart_instance->recv_buff, Saber_Data_rxbuffer_size);
    imu_latest_seq++;
}

/********************************自定义函数************************************/
/**********************************************************************
  * @ 函数名  ： Eliminating_the_influence_of_gravity
  * @ 功能说明： 加速度数据消除重力影响
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： IMU_DATA Saber_DATA：惯导数据存储结构体
  ********************************************************************/
void Eliminating_the_influence_of_gravity(void)
{
	/*重力加速度由世界坐标系旋转至机器坐标系*/
	rotation_of_coordinate_system(0.0f, 0.0f, 1.0f, &Saber_DATA.Normal_acc.ACC_X, &Saber_DATA.Normal_acc.ACC_Y, &Saber_DATA.Normal_acc.ACC_Z, 1);


	/*计算x轴加速度大小*/
	Saber_DATA.Normal_acc.ACC_X = (Saber_DATA.Saber_imu_acc.ACC_X - Saber_DATA.Normal_acc.ACC_X) * Gravity;

	/*计算y轴加速度大小*/
	Saber_DATA.Normal_acc.ACC_Y = (Saber_DATA.Saber_imu_acc.ACC_Y - Saber_DATA.Normal_acc.ACC_Y) * Gravity;

	/*计算z轴加速度大小*/
	Saber_DATA.Normal_acc.ACC_Z = (Saber_DATA.Saber_imu_acc.ACC_Z - Saber_DATA.Normal_acc.ACC_Z) * Gravity;
}

/**********************************************************************
  * @ 函数名  ： Rotation_processing
  * @ 功能说明： 加速度数据从机器坐标系转换为世界坐标系
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： IMU_DATA Saber_DATA：惯导数据存储结构体
  ********************************************************************/
void Rotation_processing(void)
{
	/*各轴加速度由机器坐标系旋转至世界坐标系*/
	rotation_of_coordinate_system(Saber_DATA.Normal_acc.ACC_X, Saber_DATA.Normal_acc.ACC_Y, Saber_DATA.Normal_acc.ACC_Z, &Saber_DATA.World_ACC.ACC_X, &Saber_DATA.World_ACC.ACC_Y, &Saber_DATA.World_ACC.ACC_Z, -1);
}



/**********************************************************************
  * @ 函数名  ： Saber_imu_data_analysis
  * @ 功能说明： 惯导数据解析
  * @ 参数    ： uint8_t Data[57]：惯导数据存储数组
  * @ 返回值  ： 无
  * @ 涉及变量： IMU_DATA Saber_DATA：惯导数据存储结构体
  ********************************************************************/
void Saber_imu_data_analysis(uint8_t Data[57])
{
	/*函数内变量，索引值*/
	int index = 0;

	/*函数内变量，数据包标签*/
	uint16_t PID = 0;

	/*函数内变量，数据包长度*/
	uint8_t length = 0;

	/*索引值跳过数据包前部分，帧头已由接收中断进行校验*/
	index += 6;


	/*数据包循环解析*/
	while (index < 55)
	{
		/*数据包标签赋值*/
		PID = Data[index] + (Data[index + 1] << 8);

		/*数据包长度*/
		length = Data[index + 2];

		/*加速度数据包*/
		if (PID == 0x8801)
		{
			/*加速度数据包解析*/
			memcpy(&Saber_DATA.Saber_imu_acc, &Data[index + 3], length);

			/*检索下一数据包*/
			index = index + 3 + length;
		}

		/*陀螺仪数据包*/
		if (PID == 0x8C01)
		{
			/*陀螺仪数据包解析*/
			memcpy(&Saber_DATA.Saber_imu_gyro, &Data[index + 3], length);

			/*检索下一数据包*/
			index = index + 3 + length;
		}

		/*欧拉角数据包*/
		if (PID == 0xB001)
		{
			/*欧拉角数据包解析*/
			memcpy(&Saber_DATA.Saber_imu_eluer, &Data[index + 3], 12);
			
			/*底盘逆时针旋转超过一圈*/
			if(Saber_DATA.Saber_imu_eluer.ELUER_YAW_now - Saber_DATA.Saber_imu_eluer.ELUER_YAW_last < -300.0f)
			{
				/*偏航角圈数自增*/
				Saber_DATA.Saber_imu_eluer.number_of_turns ++; 
			}
			/*底盘顺时针旋转超过一圈*/
			else if(Saber_DATA.Saber_imu_eluer.ELUER_YAW_now - Saber_DATA.Saber_imu_eluer.ELUER_YAW_last > 300.0f)
			{
				/*偏航角圈数自减*/
				Saber_DATA.Saber_imu_eluer.number_of_turns --; 
			}
			
			/*数值更新，用于下一次计算*/
			Saber_DATA.Saber_imu_eluer.ELUER_YAW_last = Saber_DATA.Saber_imu_eluer.ELUER_YAW_now;
			
			/*偏航角数值计算*/
			Saber_DATA.Saber_imu_eluer.ELUER_YAW = Saber_DATA.Saber_imu_eluer.ELUER_YAW_now + 360.0f * Saber_DATA.Saber_imu_eluer.number_of_turns;
			
			/*赋值*/
			//chassis_parameter.IMU.yaw = Saber_DATA.Saber_imu_eluer.ELUER_YAW;

			/*检索下一数据包*/
			index = index + 3 + length;
		}else
		{
			/*未知数据包，跳过*/
			index = index + 3 + length;
		}
	}

	/*加速度数据消除重力影响*/
	Eliminating_the_influence_of_gravity();

	/*加速度数据从机器坐标系转换为世界坐标系*/
	Rotation_processing();
}

void IMU_ParseLatestFrame(void)
{
    uint8_t local_seq = imu_latest_seq;

    if (local_seq == imu_parsed_seq)
    {
        return;
    }

    __disable_irq();
    memcpy(Saber_Data_rxbuffer, imu_latest_frame, Saber_Data_rxbuffer_size);
    imu_parsed_seq = local_seq;
    __enable_irq();

    Saber_imu_data_analysis(Saber_Data_rxbuffer);
}

/**********************************************************************
  * @ 函数
  *  ： IMU_USART6_Init
  * @ 功能说明： 注册 USART6 为 IMU 数据输入，启用 DMA+空闲接收并设定解析回调
  * @ 参数    ： 无
  * @ 返回值  ： 无
  * @ 涉及变量： imu_usart_instance、Saber_Data_rxbuffer、Saber_DATA
  ********************************************************************/
void IMU_USART6_Init(void)
{
    USART_Init_Config_s imu_usart_conf = {
        .recv_buff_size = Saber_Data_rxbuffer_size,
        .usart_handle = &huart6,
        .module_callback = IMU_USART_Callback,
    };
    imu_usart_instance = USARTRegister(&imu_usart_conf);
    IMU_init_flag = 1;
}

/******************************** 文件结束 ************************************/
