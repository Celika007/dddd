# 导航逻辑

> 说明：本文档描述任务赛导航和 `GRAB` 逻辑。AKANE 障碍赛逻辑使用 `UART7` 接收图像主机的 `MEASURING/ACTIONING` 帧，并使用 `UART8` 接收调试主机的 `debug:...` 命令。AKANE 障碍赛调试请阅读 `AKANE_DEBUG_UART8.md` 和 `AKANE_ACTION_FLOW.md`。

## 概览

- 上位机命令通过 `UART8` 接收，并在 `Modules/unicomm/unicomm.c` 中解析。
- 当前解析器接受基于行的 ASCII 帧：

```text
RCPickUpBoxes\n
RCplace=0,3,count=3\n
RCNAV;seq=1;stamp_ms=1710000000123;cur_valid=1;cur_frame=map;cur_x=123.000;cur_y=42.000;cur_z=0.000;cur_yaw=0.000;goal_valid=1;goal_frame=map;goal_x=300.000;goal_y=200.000;goal_z=0.000;goal_yaw=0.000\n
```

- `PostureControl()` 在每个控制周期调用一次 `ActionProcessNavigation()`。

## 状态映射

- `RCPickUpBoxes` 设置 `state = GRAB`
- `RCplace=...` 设置 `state = STAND`
- `RCNAV;...` 更新：
  - `current_x`
  - `current_y`
  - `target_x`
  - `target_y`
  - `upstream_current_yaw`
  - `upstream_goal_yaw`
  - `nav_current_valid`
  - `nav_goal_valid`

## 坐标与 yaw 规则

- 上位机导航坐标当前约定：
  - 前方为 `+y`
  - 左侧为 `+x`
- 上位机 yaw 当前约定：
  - `0°` -> 朝向 `+y`
  - `90°` -> 朝向 `+x`
  - `-90°` -> 朝向 `-x`
  - `180°/-180°` -> 朝向 `-y`
  - 逆时针为正

- 导航使用的目标 yaw：

```c
target_yaw = upstream_goal_yaw;
```

- 如果启用融合 yaw，上位机当前 yaw 使用同一套符号约定：

```c
up_yaw = upstream_current_yaw;
```

- 所有 yaw 比较都会归一化到 `[-180, 180]`。

## 导航决策顺序

当 `state` 为 `GRAB`、`STOP` 或 `REALSE` 时，导航不做处理。

决策优先级：

1. 如果 `nav_current_valid == 0` 或 `nav_goal_valid == 0`，保持 `STAND`
2. 先比较 yaw
3. 只有 yaw 对齐后，才比较位置

yaw 对齐阈值：

```c
abs(normalize(actual_yaw - target_yaw)) <= 2.0f
```

yaw 未对齐时：

- `actual_yaw > target_yaw` -> `RIGHT_TURN`
- `actual_yaw < target_yaw` -> `LEFT_TURN`

位置到达阈值：

```c
abs(current_x - target_x) < 8.0f &&
abs(current_y - target_y) < 8.0f
```

其中：

- `current_x - target_x`：左右误差
- `current_y - target_y`：前后误差

yaw 对齐时：

- 位置已到达 -> `STAND`
- 位置未到达 -> `RUN`

目标位置到达时：

- `UniComm_SendArrival()` 发送一次 `RCArrivalMX`

## RUN 状态 yaw 纠偏

- `RUN` 状态仍支持基于 yaw 的步长纠偏
- 纠偏使用融合后的当前 yaw 与当前导航目标 yaw 比较
- `RUN_YAW_CORRECTION_GAIN` 仍为 `0.0f`，因此该路径已接好但默认关闭

## 备注

- 旧的 `SIN...ZU`、`RCGRABMX` 和 `RCPUTDOWNMX` 解析已移除
- 旧的基于象限的转向触发逻辑已移除
- `target_yaw` 现在表示当前导航目标 yaw，用于调试输出
