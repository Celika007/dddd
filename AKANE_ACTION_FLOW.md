# AKANE 动作流程

本文档说明 AKANE 障碍赛调试流程。这里有意排除了仅用于任务赛的逻辑，例如 `GRAB` 和 `APP/action/navigation_logic.md`。

## UART7 图像主机链路

```text
UART7 DMA + IDLE
└─ HAL_UARTEx_RxEventCallback()
   └─ UniComm_UART7_Callback()
      └─ UniComm_ProcessIncomingBytes()
         └─ UniComm_ProcessFrame()
            ├─ UniComm_HandleMeasuringFrame()
            │  ├─ 更新 unicomm_x_m / y_m / bearing_deg / yaw_deg
            │  ├─ 更新 current_x / current_y / upstream_current_yaw / upstream_goal_yaw
            │  ├─ 递增 unicomm_frame_seq
            │  └─ 更新 PG1-PG8
            └─ UniComm_HandleActioningFrame()
               ├─ 更新 unicomm_action_code
               ├─ 递增 unicomm_frame_seq
               └─ 更新 PG1-PG8
```

当 `ACTION_RECORD_UART8_ENABLE == 1` 时，该链路只解析图像主机帧，不决定电机动作。

## UART8 调试主机链路

```text
Action_UART8DebugInit()
└─ USARTRegister(&huart8)
   └─ HAL_UARTEx_ReceiveToIdle_DMA()

UART8 DMA + IDLE
└─ HAL_UARTEx_RxEventCallback()
   └─ Action_UART8DebugCallback()
      └─ ActionDebug_FeedByte()
         └─ ActionDebug_HandleCommand()
            ├─ debug:run          -> state = RUN
            ├─ debug:left_turn    -> state = LEFT_TURN
            ├─ debug:right_turn   -> state = RIGHT_TURN
            ├─ debug:in_place     -> state = IN_PLACE
            ├─ debug:jumpupstairs -> state = JUMP_UPSTAIRS
            ├─ debug:stand        -> state = STAND
            ├─ debug:realse       -> state = REALSE
            ├─ debug:*+ / debug:*- -> 调整 state_detached_params[state]
            ├─ debug:frontstretch/frontlean/rightlean +/- -> 调整 STAND 姿态参数
            ├─ debug:jump_length/jump_ready/jump_land/jt7/jt6 +/- -> 调整 jumpupstairs 参数
            └─ debug:clear        -> 重置 walk_done 和 IMU 增量基准
```

## 控制循环

```text
PostureControl_task()
└─ PostureControl()
   ├─ IMU_ParseLatestFrame()
   ├─ UpdateStartupState()
   ├─ UpdateStartFlag()
   ├─ ACTION_RECORD_UART8_ENABLE == 1
   │  └─ ActionProcessDebugControl()
   │     ├─ 关闭 UART7 导航对动作的影响
   │     └─ ActionDebug_TrySendRecord()
   │        └─ HAL_UART_Transmit_DMA(&huart8, ALOG...)
   └─ switch (state)
      ├─ RUN
      │  ├─ state_detached_params[RUN]
      │  ├─ Action_ApplyRunYawCorrection()
      │  └─ gait_detached()
      ├─ LEFT_TURN / RIGHT_TURN
      │  ├─ state_detached_params[state]
      │  ├─ ApplyTurnStepLength()
      │  └─ gait_detached()
      ├─ IN_PLACE
      │  ├─ state_detached_params[IN_PLACE]
      │  └─ gait_detached()
      ├─ JUMP_UPSTAIRS
      │  ├─ 仅 UART8 debug 进入，ACTIONING 不直接启用
      │  ├─ 使用 jump_length / jt7 / jt6 等调试参数
      │  ├─ 复用 CartesianToTheta()
      │  ├─ 叠加 frontstretch / frontlean / rightlean
      │  └─ SetCoupledPosition()
      ├─ STAND
      │  ├─ StandByStartPosition()
      │  └─ 叠加 frontstretch / frontlean / rightlean
      ├─ REALSE
      │  └─ osDelay(50)
      └─ GRAB
         └─ 仅用于任务赛的路径，AKANE 障碍赛调试不使用
```

## 步态与步数统计

```text
gait_detached()
└─ CoupledMoveLeg()
   └─ CycloidTrajectory()
      ├─ 使用 freq 更新 gait_phase_accumulator
      ├─ base_phase = fmod(gait_phase_accumulator, 1.0)
      ├─ ActionDebug_UpdateWalkPhase(base_phase)
      │  └─ 相位回绕 -> walk_done++
      ├─ 通过摆线和正弦轨迹计算 foot_x / foot_y
      ├─ CartesianToTheta()
      └─ SetCoupledPosition()
         └─ DJIMotorSetRef()
```

`walk_done` 不是根据脚本时间估算的，而是由驱动 `CycloidTrajectory()` 的同一个步态相位统计得到。

## 模式划分

```text
ACTION_RECORD_UART8_ENABLE = 1
└─ AKANE debug mode
   ├─ UART7：仅解析图像主机帧
   └─ UART8：接收调试命令并发送 ALOG 记录

ACTION_RECORD_UART8_ENABLE = 0
└─ AKANE 自动模式
   └─ UART7 MEASURING/ACTIONING 驱动 ActionProcessSerialState()
```

## Jump 与 Action Code

```text
UART8 debug
└─ debug:jumpupstairs
   └─ JUMP_UPSTAIRS
      ├─ ready: jump_ready + jt7
      ├─ takeoff: jump_length + jt7
      ├─ landing prepare: jump_land + jt7
      ├─ landing buffer: jump_land + jt6
      └─ STAND

UART7 ACTIONING
├─ SAND_PIT
│  └─ 代码中保留第一步 jumpupstairs 注释占位，不直接进入 JUMP_UPSTAIRS
└─ STEPS
   └─ 代码中保留第一步 jumpupstairs 注释占位，不直接进入 JUMP_UPSTAIRS
```

`frontstretch/frontlean/rightlean` 是可叠加的 STAND 姿态调参。`STAND` 下 ALOG 以 2 Hz 返回这三个值；其他 debug 状态会持续返回 `sp`、`adj`、`jp` 和 IMU 信息，便于把物理验证结果填回正式 action 脚本。
