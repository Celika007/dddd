# daemon

<p align='right'>neozng1@hnu.edu.cn</p>

用于监测模块和应用运行情况的module(和官方代码中的deteck task作用相同)

## 使用范例

要使用该module，则包含`daemon.h`的头文件，并在使用daemon的文件中保留一个daemon的指针。

初始化的时候，要传入以下参数：

```c
typedef struct
{
    uint16_t reload_count;     // 实际上这是app唯一需要设置的值?
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用
    void *owner_id;            // id取拥有daemon的实例的地址,如DJIMotorInstance*,cast成void*类型
} Daemon_Init_Config_s;
```

`reload_count`
是”喂狗“时的重载值，一般根据你希望的离线容许时间和模块接收数据/访问数据的频率确定。daemonTask递减计数器的频率是100hz(
在HAL_N_Middlewares/Src/freertos.c中查看任务),你可以据此以及模块收到数据/操作的频率设置reload_count。

`daemon_task()`
会在实时系统中以1kHz的频率运行，每次运行该任务，都会将所有daemon实例当前的count进行自减操作，当count减为零，则说明模块已经很久没有上线（处于deactivated状态，即没有收到数据，也没有进行其他读写操作）。

`offline_callback`是模块离线的回调函数，即当包含daemon的模块发生离线情况时，该函数会被调用以应对离线情况，如重启模块，重新初始化等。如果没有则传入
`NULL`即可。

`owner_id`即模块取自身地址并通过强制类型转换化为`void*`类型，用于拥有多个实例的模块在`offline_callback`
中区分自身。如多个电机都使用一个相同的`offline_callback`，那么在调用回调函数的时候就可以通过该指针来访问某个特定的电机。

> 这种方法也称作“parent pointer”，即**保存拥有指向自身的指针对象的地址**。这样就可以在特定的情况下通过自身来访问自己的父对象。

## 具体实现

即`DaemonTask()`，在操作系统中以1kHz运行。注释详细，请参见`daemon.c`。

## 详细使用教程

- 模块概述
    - 守护模块用于监测外设/任务是否持续在线，通过“喂狗”机制维护一个计数器，超时自动触发离线回调。
    - 关键接口：`DaemonRegister`、`DaemonReload`、`DaemonIsOnline`、`DaemonTask`。

- 快速上手
    - 在你的模块文件中包含头文件：`#include "daemon.h"`。
    - 在模块的实例结构体中保留一个指针：`DaemonInstance *xxx_daemon;`。
    - 初始化时调用注册函数：
      ```c
      Daemon_Init_Config_s conf = {
          .reload_count = 100,      // 喂狗后的计数目标值，结合模块数据频率设置
          .init_count   = 100,      // 上线等待计数（可选）
          .callback     = YourOfflineCallback,
          .owner_id     = (void*)your_module_instance,
      };
      your_module_instance->xxx_daemon = DaemonRegister(&conf);
      ```
    - 在每次成功接收到数据或成功完成一次操作时调用喂狗：
      ```c
      DaemonReload(your_module_instance->xxx_daemon);
      ```
    - 在RTOS中定时调用任务：
      ```c
      void DaemonTask(void); // 放入1kHz任务中
      ```

- 参数说明
    - `reload_count`：喂狗后计数器被设置到的值。若模块数据到达频率为`F`，RTOS守护任务频率为`f_task`，建议
      `reload_count >= 1.5 * f_task / F`，确保正常抖动下不误判离线。
    - `init_count`：系统启动后允许的等待计数。适用于需要一定准备时间的外设或任务（PC/相机/上位机）。若不需要可设为`0`。
    - `callback`：离线回调函数。建议在内部执行“通信重启/资源重置/重新初始化”等动作，避免阻塞。
    - `owner_id`：模块实例地址（如`DJIMotorInstance*`），用于在回调中识别具体实例。

- 典型用法示例
    - 电机模块（含守护指针）：`e:\RC2026\RSCF_2\Modules\motor\DMmotor\dmmotor.h:1` 中结构体包含
      `DaemonInstance* motor_daemon;`。
    - 离线回调示例（重启通信）：`e:\RC2026\RSCF_2\Modules\master_machine\master_process.c:44` 中的
      `VisionOfflineCallback(void *id)` 通过重启串口通信恢复在线状态。
    - 在接收数据处喂狗：在CAN/USART数据解析成功后调用 `DaemonReload(instance->motor_daemon);`。

- 与RTOS集成
    - 将 `DaemonTask()` 放入系统的1kHz周期任务中；每次调用会对所有注册实例执行：计数递减→超时判断→触发回调。
    - 接口参考：
        - 注册：`e:\RC2026\RSCF_2\Modules\daemon\daemon.c:11`
        - 喂狗：`e:\RC2026\RSCF_2\Modules\daemon\daemon.c:27`
        - 在线判断：`e:\RC2026\RSCF_2\Modules\daemon\daemon.c:32`
        - 守护任务：`e:\RC2026\RSCF_2\Modules\daemon\daemon.c:37`

- 常见问题
    - 计数器过小导致误判离线：提高`reload_count`或增加喂狗频率；确保`DaemonTask()`频率稳定。
    - 回调中执行阻塞操作：将重启/初始化动作拆分为异步任务或设置短时非阻塞操作，避免影响其他实例的处理。
    - 多实例识别错误：确保`owner_id`始终填入对应模块实例地址，并在回调中正确强制类型转换。