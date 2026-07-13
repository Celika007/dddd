# AKANE UART8 调试指南

## 目的

本文档用于 AKANE 障碍赛调试。

- 图像主机：通过 `UART7` 向 MCU 发送 AprilTag `MEASURING` / `ACTIONING` 帧。
- 调试主机：通过 `UART8` 向 MCU 发送 `debug:...` 文本命令，并通过 `UART8` 接收 MCU 返回的 `ALOG,...` 记录。
- `GRAB` 和 `APP/action/navigation_logic.md` 等任务赛代码不属于此 AKANE 障碍赛调试流程。

当 `ACTION_RECORD_UART8_ENABLE` 为 `1` 时，`UART7` 仍会解析图像主机帧并更新 PG 引脚，但不会驱动电机动作。电机动作由 `UART8` 调试主机控制。

## 编译期开关

在 `APP/action/action.h` 中：

```c
#define ACTION_RECORD_UART8_ENABLE 1U
```

- `1U`：AKANE 调试模式。`UART8` 接收调试命令并发送低频日志。旧的 8 ms yaw 调试输出会被关闭。
- `0U`：自动模式。`UART7` 的 `MEASURING/ACTIONING` 驱动障碍赛动作状态机。

## UART8 命令格式

命令为 ASCII 文本，必须以 `\n` 或 `\r\n` 结尾。

运动命令：

```text
debug:run
debug:left_turn
debug:right_turn
debug:in_place
debug:jumpupstairs
debug:stand
debug:realse
```

参数命令会同时修改当前状态下四条腿的参数：

```text
debug:step_length+
debug:step_length-
debug:up_amp+
debug:up_amp-
debug:down_amp+
debug:down_amp-
debug:stance_height+
debug:stance_height-
debug:freq+
debug:freq-
debug:flight_percent+
debug:flight_percent-
```

STAND 姿态和 jumpupstairs 调试参数：

```text
debug:frontstretch+
debug:frontstretch-
debug:frontlean+
debug:frontlean-
debug:rightlean+
debug:rightlean-
debug:jump_length+
debug:jump_length-
debug:jump_ready+
debug:jump_ready-
debug:jump_land+
debug:jump_land-
debug:jt7+
debug:jt7-
debug:jt6+
debug:jt6-
```

记录重置：

```text
debug:clear
```

`debug:clear` 会重置 `walk_done`，并把 IMU 增量基准设置为当前 IMU 姿态。

GPIO 调试命令（`1` 输出 RESET，`0` 输出 SET）：

```text
debug:A1 / debug:A0    PI0
debug:B1 / debug:B0    PH12
debug:C1 / debug:C0    PH11
debug:D1 / debug:D0    PH10
```

PWM compare 调试命令：

```text
debug:angle000    compare = 500
debug:angle999    compare = 1499
```

`angle` 后必须为三位十进制数字 `000-999`，写入 `TIM2 CH1` 的 compare 为 `500 + 输入值`。

臂长和舵机标定命令：

```text
debug:arm_final22
debug:servo_max127
debug:servo_min038
```

`arm_final` 后必须为两位十进制整数。该命令根据臂长解算角度，并立即更新 `TIM2 CH1` 的 compare；compare 被限制在 `500-1667`。`servo_max` 和 `servo_min` 后必须为三位十进制角度，命令只更新全局标定值，并从下一条 `arm_final` 命令开始生效。

## 参数步进值

```text
step_length     +/- 1.0
up_amp          +/- 0.1
down_amp        +/- 0.1
stance_height   +/- 1.0
freq            +/- 0.1
flight_percent  +/- 0.05
frontstretch     +/- 1.0 deg
frontlean        +/- 1.0 deg
rightlean        +/- 1.0 deg
jump_length      +/- 1.0 cm
jump_ready       +/- 1.0 cm
jump_land        +/- 1.0 cm
jt7              +/- 1.0 deg
jt6              +/- 1.0 deg
```

命令作用于 `state_detached_params[state]`。由于 AKANE 调试命令会同时调节四条腿，返回日志中只包含一个 `sp` 字段，而不是 `sp0/sp1/sp2/sp3`。

## UART8 日志格式

`STAND` 状态以 2 Hz 发送日志，用于记录 `frontstretch/frontlean/rightlean` 三种姿态参数。

`RUN`、`LEFT_TURN`、`RIGHT_TURN`、`IN_PLACE`、`JUMP_UPSTAIRS` 等调试状态会持续发送日志。步态运动状态的发送频率约为 `2 * freq`，发送接口使用 `HAL_UART_Transmit_DMA()`。如果 UART8 TX 正忙，本条记录会被跳过，而不会阻塞控制流程。

示例：

```text
ALOG,mode=AKANE_DEBUG,state=RUN,walk_done=12,freq=1.80,sp=18.0/12.0/6.0/0.0/0.40/1.80,adj=0.0/2.0/-1.0,jp=28.0/12.0/25.0,imu_r=1.20,imu_p=-0.40,imu_y=35.60,dr=0.30,dp=-0.10,dy=12.50
```

字段：

```text
state      当前运动状态
walk_done  根据 CycloidTrajectory() 基准相位回绕统计的步态周期数
freq       当前 freq + freq_offset
sp         stance_height/step_length/up_amp/down_amp/flight_percent/freq
adj        frontstretch/frontlean/rightlean
jp         jump_length/jt7/jt6
compare    TIM2 CH1 当前 compare
abcd       A/B/C/D 当前逻辑状态；1 表示引脚 RESET，0 表示引脚 SET
imu_r/p/y  当前 IMU roll/pitch/yaw
dr/dp/dy   相对上一次 debug:clear 或进入运动状态时的 IMU 增量
```

`debug:jumpupstairs` 会进入 `JUMP_UPSTAIRS` 调试状态，按参考 `jumpupstairs()` 的阶段顺序执行一次约 1 秒的跳台阶调试动作，然后回到 `STAND`。这个状态只用于 UART8 物理验证，不会被 UART7 `ACTIONING` 自动启用。

`SAND_PIT` 和 `STEPS` 的 `ACTIONING` 第一段动作目前保留为代码注释占位：调试者记录 `frontstretch/frontlean/rightlean` 和 `jump_length/jt7/jt6` 后，再把验证过的参数填回对应 action_code 的正式脚本。

## 典型调试流程

1. 将 `ACTION_RECORD_UART8_ENABLE` 设置为 `1U` 后编译并烧录。
2. 将图像主机连接到 `UART7`；它可以继续发送 AprilTag 帧。
3. 将调试主机连接到 `UART8`。
4. 发送 `debug:clear`。
5. 发送运动命令，例如 `debug:run`。
6. 使用 `debug:step_length+` 或 `debug:freq-` 等命令调节参数。
7. 在调试主机上读取 `ALOG` 记录。
8. 断电后，根据 UART8 记录更新固定 `ACTIONING` 动作脚本参数。
