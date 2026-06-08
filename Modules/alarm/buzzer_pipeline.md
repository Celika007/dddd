# Buzzer 运行说明（TIM PWM → BSP PWM → Buzzer）

## 概览

- 底层通过 HAL TIM 产生 PWM 波形作为蜂鸣器的驱动信号。
- BSP 层 `bsp_pwm` 对 HAL PWM 接口进行统一封装，提供注册、周期与占空比设置。
- 上层 `Modules/alarm/buzzer` 使用 BSP PWM 设置响度（占空比）与音阶频率（周期）。

## 关键路径

- 定时器 PWM 初始化与设置
    - 注册 PWM 实例：`PWMRegister` 在 `BSP/pwm/bsp_pwm.c:27`
    - 设置周期：`PWMSetPeriod` 在 `BSP/pwm/bsp_pwm.c:68`
    - 设置占空比：`PWMSetDutyRatio` 在 `BSP/pwm/bsp_pwm.c:78`

- 蜂鸣器模块逻辑
    - 初始化蜂鸣器：`BuzzerInit` 在 `Modules/alarm/buzzer.c:38`
    - 注册实例：`BuzzerRegister` 在 `Modules/alarm/buzzer.c:65`
    - 设置状态：`AlarmSetStatus` 在 `Modules/alarm/buzzer.c:93`
    - 任务轮询：`BuzzerTask` 在 `Modules/alarm/buzzer.c:105`

## 运行机理

1. PWM 实例注册
    - `BuzzerInit` 构造 `PWM_Init_Config_s`（定时器句柄 `htim12`、通道 `TIM_CHANNEL_2`、初始周期与占空比），调用
      `PWMRegister` 注册实例。
    - `PWMRegister` 内部流程：
        - 计算定时器时钟 `tclk`（`PWMSelectTclk`），启动 HAL PWM，调用 `PWMSetPeriod` 与 `PWMSetDutyRatio` 完成初值设置。

2. 周期与占空比换算
    - `PWMSetPeriod` 将周期（秒）换算为自动重装载寄存器 `ARR`：`ARR = period * tclk / (Prescaler + 1)`。
    - `PWMSetDutyRatio` 将 `dutyratio ∈ [0,1]` 换算为比较寄存器 `CCR = dutyratio * ARR`。

3. 蜂鸣器响度与音阶控制
    - `BuzzerTask` 按告警实例状态：
        - `ALARM_OFF`：占空比设为 0（关闭）。
        - `ALARM_ON`：占空比设为实例响度（0~1），按音阶枚举设置周期为 `1 / 频率`（如 Do=523Hz）。

## 使用范例

参考 `Modules/alarm/buzzer.md:7`：

```c
Buzzer_config_s buzzer_config ={
    .alarm_level = ALARM_LEVEL_HIGH,
    .loudness=  0.4,
    .octave=  OCTAVE_1,
};
BuzzzerInstance* robocmd_alarm = BuzzerRegister(&buzzer_config);
AlarmSetStatus(robocmd_alarm, ALARM_ON);
AlarmSetStatus(robocmd_alarm, ALARM_OFF);
```

## 代码参考

- `BSP/pwm/bsp_pwm.h:49` 注册接口声明
- `BSP/pwm/bsp_pwm.c:43` HAL PWM 启动
- `BSP/pwm/bsp_pwm.c:70` 周期换算与写入 ARR
- `BSP/pwm/bsp_pwm.c:80` 占空比换算与写入 CCR
- `Modules/alarm/buzzer.c:38` 蜂鸣器 PWM 初始化与注册
- `Modules/alarm/buzzer.c:105` 任务轮询与音阶/响度设置

## 注意事项

-- 在同一定时器下更改某通道周期可能影响其他通道，请参考 `BSP/pwm/bsp_pwm.md:3`。
-- 使用 DMA 中断时，占空比为 0 不会进入中断，参考 `BSP/pwm/bsp_pwm.md:4`。
-- `BSP/pwm/bsp_pwm.h` 针对 H7 的时钟选择示例，若在 F4 平台使用，请确保 `PWMSelectTclk` 时钟源逻辑与芯片一致。