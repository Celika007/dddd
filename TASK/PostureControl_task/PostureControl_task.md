# PostureControl Review Notes

## Issue: per-leg `freq` does not independently take effect

- Location:
  - `TASK/PostureControl_task/PostureControl_task.c`
  - `CycloidTrajectory()`
- Current behavior:
  - `CycloidTrajectory()` uses function-level static variables `p` and `prev_t`
  - `gait_detached()` calls `CycloidTrajectory()` four times per control cycle, once for each leg
  - After the first leg updates `prev_t`, the remaining three legs in the same cycle see `t - prev_t == 0`
  - As a result, only the first executed leg can advance phase using its own `freq`
  - The other legs do not independently apply their configured `freq` or `freq_offset`
- Impact:
  - `DetachedParam` exposes per-leg gait parameters, but per-leg frequency is not actually independent
  - If future tuning gives different `freq` values to different legs, runtime behavior will not match the parameter table
- Scope:
  - This affects `RUN`, `STAND`, `GRAB`, `LEFT_TURN`, and `RIGHT_TURN`, because all of them use `gait_detached()`
