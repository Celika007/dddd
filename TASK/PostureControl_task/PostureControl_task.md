# PostureControl 评审记录

## 问题：单腿 `freq` 没有独立生效

- 位置：
  - `TASK/PostureControl_task/PostureControl_task.c`
  - `CycloidTrajectory()`
- 当前行为：
  - `CycloidTrajectory()` 使用函数级静态变量 `p` 和 `prev_t`
  - `gait_detached()` 在每个控制周期调用四次 `CycloidTrajectory()`，每条腿调用一次
  - 第一条腿更新 `prev_t` 后，同一周期内剩余三条腿看到的 `t - prev_t == 0`
  - 因此，只有最先执行的腿能使用自身的 `freq` 推进相位
  - 其他腿不会独立应用各自配置的 `freq` 或 `freq_offset`
- 影响：
  - `DetachedParam` 暴露了单腿步态参数，但单腿频率实际上并不独立
  - 如果后续调参为不同腿设置不同 `freq`，运行行为会与参数表不一致
- 范围：
  - 这会影响 `RUN`、`STAND`、`GRAB`、`LEFT_TURN` 和 `RIGHT_TURN`，因为它们都使用 `gait_detached()`
