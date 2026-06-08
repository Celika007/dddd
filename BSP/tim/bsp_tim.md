# bsp_tim

通用定时器（TIM）BSP 层，面向模块提供统一的计时与事件回调接口。参考 `bsp_can`、`bsp_pwm`、`bsp_spi`、`bsp_usart` 的风格实现。

注意：大疆 A 板可用 TIM 为 TIM1–TIM13，`TIM14` 用于 FreeRTOS 时间基准，不在本 BSP 使用范围内。

## 类型与结构体

- `TIM_MODE_e`

    - `TIM_MODE_BASE` 基本定时（更新事件）
    - `TIM_MODE_INPUT_CAPTURE` 输入捕获
    - `TIM_MODE_OUTPUT_COMPARE` 输出比较
- `TIMInstance`

    - `TIM_HandleTypeDef *htim` TIM 句柄（CubeMX 生成）
    - `TIM_MODE_e mode` 工作模式（可组合）
    - `uint32_t channels_mask` 通道掩码（如 `TIM_CHANNEL_1 | TIM_CHANNEL_2`）
    - `period_callback(TIMInstance*)` 更新事件回调
    - `ic_callback(TIMInstance*, uint32_t)` 输入捕获回调，携带通道
    - `oc_callback(TIMInstance*, uint32_t)` 输出比较回调，携带通道
    - `void *id` 模块实例地址（parent pointer）
- `TIM_Init_Config_s`

    - 与 `TIMInstance` 对应的初始化配置，用于注册时填充上述字段

## 外部接口

```c
TIMInstance *TIMRegister(TIM_Init_Config_s *config);
void TIMBaseStart_IT(TIMInstance *instance);
void TIMBaseStop_IT(TIMInstance *instance);
void TIMICStart_IT(TIMInstance *instance, uint32_t channel);
void TIMICStop_IT(TIMInstance *instance, uint32_t channel);
void TIMOCStart_IT(TIMInstance *instance, uint32_t channel);
void TIMOCStop_IT(TIMInstance *instance, uint32_t channel);
void TIMSetPeriodUs(TIMInstance *instance, uint32_t period_us);
uint32_t TIMGetCounter(TIMInstance *instance);
void TIMSetCounter(TIMInstance *instance, uint32_t cnt);
```

## 使用说明

- 初始化与注册

    - 在 CubeMX 完成 TIM 外设的基本配置（时钟、PSC、ARR、模式），并生成 `tim.h/tim.c`
    - 在模块中持有 `TIMInstance *` 指针，注册时传入 `TIM_Init_Config_s`：
      ```c
      TIM_Init_Config_s conf = {
        .htim = &htim3,
        .mode = TIM_MODE_BASE,
        .channels_mask = 0,
        .period_callback = MyPeriodCallback,
        .ic_callback = NULL,
        .oc_callback = NULL,
        .id = (void*)my_module,
      };
      my_module->tim = TIMRegister(&conf);
      ```
- 基本定时（更新事件）

    - 调用 `TIMBaseStart_IT(instance)` 启动周期中断
    - 通过 `TIMSetPeriodUs(instance, us)` 更新周期；内部按 `PSC` 计算 `ARR`
    - 在 `MyPeriodCallback(TIMInstance*)` 中处理周期任务；回调通过 HAL 的 `HAL_TIM_PeriodElapsedCallback` 分发
- 输入捕获

    - 注册时提供 `ic_callback`，并在需要的通道上调用 `TIMICStart_IT(instance, TIM_CHANNEL_x)`
    - 回调通过 HAL 的 `HAL_TIM_IC_CaptureCallback` 分发，并携带对应通道标识 `htim->Channel`
- 输出比较

    - 注册时提供 `oc_callback`，并在需要的通道上调用 `TIMOCStart_IT(instance, TIM_CHANNEL_x)`
    - 回调通过 HAL 的 `HAL_TIM_OC_DelayElapsedCallback` 分发，并携带对应通道标识
- 计数器读写

    - `TIMGetCounter/TIMSetCounter` 用于读取与重置计数器（如在周期更新前清零）

## 设计注意事项

- TIM14用于 FreeRTOS 时间基准，不允许通过 `TIMRegister` 注册，若传入 `htim14` 会触发保护
- `TIMSetPeriodUs` 基于当前 `PSC` 重新计算 `ARR`；若需要更细粒度控制，请在 CubeMX 中设置 `PSC` 并保证溢出范围足够
- APB 时钟倍频规则：当 APB prescaler > 1 时，TIM 时钟为 APB 时钟的 2 倍；内部已按 STM32F4 的 PCLK1/PCLK2 规则计算
- 输入捕获/输出比较的 HAL 回调以 `htim->Channel` 标识当前通道；模块需据此区分不同通道的事件

## 典型示例：1kHz 周期任务

```c
static void MyPeriodCallback(TIMInstance *inst) {
  // 处理周期性任务，例如状态机驱动或传感器采样触发
}

void MyModuleInit(void) {
  TIM_Init_Config_s conf = {
    .htim = &htim3,
    .mode = TIM_MODE_BASE,
    .channels_mask = 0,
    .period_callback = MyPeriodCallback,
    .ic_callback = NULL,
    .oc_callback = NULL,
    .id = NULL,
  };
  TIMInstance *t = TIMRegister(&conf);
  TIMSetPeriodUs(t, 1000);     // 1kHz
  TIMBaseStart_IT(t);
}
```

## 常见问题

- 周期不稳定或不触发：检查 CubeMX 中 TIM 的时钟源与分频设置；确认已使能更新事件中断
- 输入捕获无效：确保通道模式设置为 `IC`，并正确配置 GPIO 复用与滤波；在需要的通道上调用 `TIMICStart_IT`
- 输出比较不触发：确认通道已配置为 `OC` 模式，并设置比较值；调用 `TIMOCStart_IT`
