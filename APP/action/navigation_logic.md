# Navigation Logic

## Overview

- Navigation coordinates are updated by `Modules/unicomm/unicomm.c`:
  - `current_x`
  - `current_y`
  - `target_x`
  - `target_y`
- Yaw for navigation now uses single-turn IMU yaw:
  - `Saber_DATA.Saber_imu_eluer.ELUER_YAW_now`
- The control loop entry is `PostureControl()` in `TASK/PostureControl_task/PostureControl_task.c`.
- `PostureControl()` calls `ActionProcessNavigation()` once every control period.

## State Usage

Navigation-related states in `APP/action/action.h`:

```c
RUN = 0,
STAND = 1,
GRAB = 2,
LEFT_TURN = 3,
RIGHT_TURN = 4,
STOP = 5,
REALSE = 6,
```

Current behavior:

- `RUN`: normal gait
- `STAND`: reached target
- `LEFT_TURN`: turning left
- `RIGHT_TURN`: turning right
- `GRAB`, `STOP`, `REALSE`: navigation logic returns immediately and does not overwrite state

## Startup Gating

- `state` is initialized to `REALSE` in `TASK/PostureControl_task/PostureControl_task.c`.
- During the first `2000 ms` after boot, `PostureControl()` forces `state = REALSE`.
- After the `2000 ms` startup window expires, `REALSE` automatically transitions to `STAND`.
- `start_flag` is initialized to `0`.
- `start_flag = 1` when `state == STAND`.
- `start_flag = 0` when `state == REALSE` or `state == STOP`.
- `RUN`, `GRAB`, `LEFT_TURN`, and `RIGHT_TURN` only execute when `start_flag == 1`.
- If a motion state is requested while `start_flag == 0`, `PostureControl()` forces the state back to `STAND` before executing gait logic.

## Position Thresholds

- Arrival threshold:
  - `abs(current_x - target_x) < 10`
  - `abs(current_y - target_y) < 10`
- Turn-complete threshold:
  - `abs(yaw error) < 2 deg`

When both arrival conditions are met:

- `state = STAND`
- `UniComm_SendArrival()` sends `RCArrivalMX` once

## Turn Trigger Logic

The turn trigger uses `ELUER_YAW_now`, whose expected range is `-180 ~ 180`.

### Yaw near 0 deg

Condition:

```c
abs(yaw - 0) <= 2
```

And:

```c
abs(current_x - target_x) < 10
```

Direction:

- `current_x - target_x > 0` -> left turn
- `current_x - target_x < 0` -> right turn

### Yaw near 180 deg

Condition:

- `[178, 180]`
- `[-180, -178]`

Implemented by normalized angular error against `180 deg`.

And:

```c
abs(current_x - target_x) < 10
```

Direction:

- `current_x - target_x > 0` -> right turn
- `current_x - target_x < 0` -> left turn

### Yaw near 90 deg

Condition:

```c
abs(yaw - 90) <= 2
```

And:

```c
abs(current_y - target_y) < 10
```

Direction:

- `current_y - target_y > 0` -> right turn
- `current_y - target_y < 0` -> left turn

### Yaw near -90 deg

Condition:

```c
abs(yaw + 90) <= 2
```

And:

```c
abs(current_y - target_y) < 10
```

Direction:

- `current_y - target_y > 0` -> left turn
- `current_y - target_y < 0` -> right turn

## Turn Counters And Target Yaw

The old `left_flag/right_flag` logic has been removed.

Navigation now uses:

- `left_count`
- `right_count`

Update rule:

- When a new left turn is triggered:
  - `left_count += 1`
- When a new right turn is triggered:
  - `right_count += 1`

Target yaw is calculated from cumulative counts:

```c
raw_target_yaw = -90 * left_count + 90 * right_count
```

Then normalized into `[-180, 180]` before storing in `target_yaw`.

Examples:

- `left_count = 1, right_count = 0` -> `target_yaw = -90`
- `left_count = 2, right_count = 0` -> `target_yaw = -180`
- `left_count = 3, right_count = 0` -> normalized `target_yaw = 90`

## Turn Completion

When the robot is already in `LEFT_TURN` or `RIGHT_TURN`:

- Navigation does not re-evaluate trigger direction.
- It only checks whether current yaw is close enough to `target_yaw`.

Completion rule:

```c
abs(normalized(current_yaw - target_yaw)) < 2
```

When satisfied:

- `state = RUN`

## Run Yaw Correction

`RUN` state now applies yaw-based step-length correction before calling `gait_detached(...)`.

Yaw error uses the same normalized shortest-path calculation as turn completion:

```c
err_yaw = normalize(current_yaw - target_yaw)
```

This keeps the `180 deg` / `-180 deg` boundary safe.

Per control cycle:

- All four `step_length_offset` values are reset to `0`
- If `err_yaw < 0`:
  - `detached_params_0.step_length_offset = RUN_YAW_CORRECTION_GAIN * abs(err_yaw)`
  - `detached_params_3.step_length_offset = RUN_YAW_CORRECTION_GAIN * abs(err_yaw)`
- If `err_yaw > 0`:
  - `detached_params_1.step_length_offset = RUN_YAW_CORRECTION_GAIN * abs(err_yaw)`
  - `detached_params_2.step_length_offset = RUN_YAW_CORRECTION_GAIN * abs(err_yaw)`
- If `err_yaw == 0`:
  - all four offsets remain `0`

Default gain:

```c
#define RUN_YAW_CORRECTION_GAIN 0.0f
```

So the correction path is wired in, but disabled until the gain is tuned.

## Angle Handling

`APP/action/action.c` contains angle helpers:

- `Action_NormalizeYaw180(float yaw)`
  - Normalizes any angle to `[-180, 180]`
- `Action_AngularErrorDeg(float current_yaw, float expected_yaw)`
  - Returns shortest signed angular error in `[-180, 180]`
- `Action_IsYawNear(float yaw, float center)`
  - Checks whether yaw is within `+-2 deg` of the target window center

This is what makes the `180 deg` boundary safe across both `180` and `-180`.

## Gait Parameters

`state_detached_params[]` still contains:

- `RUN`
- `STAND`
- `GRAB`
- `LEFT_TURN`
- `RIGHT_TURN`

The left-turn and right-turn gait entries are still placeholders with `TODO` comments.

## Known Remaining Issues

These are intentionally not changed in this revision:

- `_leg_active[4]` behavior is unchanged
- `UniComm_SendArrival()` is still blocking
