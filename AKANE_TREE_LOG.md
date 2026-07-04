PostureControl_task()
└─ PostureControl()
   ├─ IMU_ParseLatestFrame()
   ├─ UpdateStartupState()
   ├─ UpdateStartFlag()
   ├─ ActionProcessSerialState()
   │  ├─ MEASURING 分支
   │  │  └─ ActionProcessNavigation()
   │  │     ├─ Action_GetNavigationGoalYaw()
   │  │     ├─ Action_AngularErrorDeg()
   │  │     │  └─ Action_NormalizeYaw180()
   │  │     └─ 根据 yaw / x / y 设置 state:
   │  │        ├─ RIGHT_TURN
   │  │        ├─ LEFT_TURN
   │  │        ├─ RUN
   │  │        └─ STAND
   │  │
   │  └─ ACTIONING 分支
   │     ├─ 先 state = STAND
   │     ├─ 等待 0.5s
   │     ├─ Action_StartScript()
   │     │  └─ Action_SelectScript()
   │     │     ├─ WEAVE / WAVE -> action_script_weave
   │     │     ├─ SAND_PIT / DUCK / CLIMB 等 -> action_script_default
   │     │     └─ 未知动作 -> action_script_stand_only
   │     └─ Action_RunScript()
   │        └─ Action_ApplyScriptState()
   │           ├─ ACTION_SCRIPT_RUN        -> state = RUN
   │           ├─ ACTION_SCRIPT_LEFT_TURN  -> state = LEFT_TURN
   │           ├─ ACTION_SCRIPT_RIGHT_TURN -> state = RIGHT_TURN
   │           └─ ACTION_SCRIPT_STAND      -> state = STAND
   │
   └─ switch (state)
      ├─ RUN
      │  ├─ Action_ApplyRunYawCorrection()
      │  │  ├─ Action_GetNavigationGoalYaw()
      │  │  ├─ Action_AngularErrorDeg()
      │  │  │  └─ Action_NormalizeYaw180()
      │  │  └─ 修改左右腿 step_length_offset
      │  ├─ gait_detached()
      │  │  └─ CoupledMoveLeg()
      │  │     ├─ CycloidTrajectory()
      │  │     ├─ CartesianToTheta()
      │  │     └─ SetCoupledPosition()
      │  │        └─ DJIMotorSetRef()
      │  └─ 最终由 MotorControl_task() 周期性发给电机
      │
      ├─ STAND
      │  └─ StandByStartPosition()
      │     └─ DJIMotorSetRef()
      │
      ├─ LEFT_TURN / RIGHT_TURN
      │  ├─ ApplyTurnStepLength()
      │  ├─ gait_detached()
      │  │  └─ CoupledMoveLeg()
      │  │     ├─ CycloidTrajectory()
      │  │     ├─ CartesianToTheta()
      │  │     └─ SetCoupledPosition()
      │  │        └─ DJIMotorSetRef()
      │  └─ 最终由 MotorControl_task() 发给电机
      │
      ├─ GRAB
      │  └─ gait_detached()
      │
      ├─ STOP
      │  └─ DOWN()
      │     └─ DJIMotorSetRef()
      │
      └─ REALSE
         └─ osDelay(50)
接收链路是另一条：
UART7 DMA + IDLE
└─ HAL_UARTEx_RxEventCallback()
   └─ UniComm_UART7_Callback()
      └─ UniComm_ProcessIncomingBytes()
         └─ UniComm_ProcessFrame()
            ├─ UniComm_HandleMeasuringFrame()
            │  ├─ 更新 unicomm_x_m / y_m / bearing_deg / yaw_deg
            │  ├─ 更新 current_x / current_y / upstream_current_yaw / upstream_goal_yaw
            │  ├─ unicomm_frame_seq++
            │  └─ 设置 PG1-PG8
            └─ UniComm_HandleActioningFrame()
               ├─ 更新 unicomm_action_code
               ├─ unicomm_frame_seq++
               └─ 设置 PG1-PG8
所以核心节奏是：
串口只负责更新数据和 `frame_seq`。
`PostureControl_task` 每 8ms 调用一次 `PostureControl`。
`PostureControl` 中的 `ActionProcessSerialState` 读取最新消息并决定 `state`。
最后由 `switch(state)` 真正生成腿部目标角度。
`MotorControl_task` 再把目标角度转换为电机 CAN 输出。
