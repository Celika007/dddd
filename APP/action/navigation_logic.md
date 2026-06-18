# Navigation Logic

## Overview

- Upper computer commands are received on `UART8` and parsed in `Modules/unicomm/unicomm.c`.
- The parser now accepts line-based ASCII frames:

```text
RCPickUpBoxes\n
RCplace=0,3,count=3\n
RCNAV;seq=1;stamp_ms=1710000000123;cur_valid=1;cur_frame=map;cur_x=123.000;cur_y=42.000;cur_z=0.000;cur_yaw=0.000;goal_valid=1;goal_frame=map;goal_x=300.000;goal_y=200.000;goal_z=0.000;goal_yaw=0.000\n
```

- `PostureControl()` calls `ActionProcessNavigation()` once per control cycle.

## State Mapping

- `RCPickUpBoxes` sets `state = GRAB`
- `RCplace=...` sets `state = STAND`
- `RCNAV;...` updates:
  - `current_x`
  - `current_y`
  - `target_x`
  - `target_y`
  - `upstream_current_yaw`
  - `upstream_goal_yaw`
  - `nav_current_valid`
  - `nav_goal_valid`

## Coordinate And Yaw Rules

- Upper-computer navigation coordinates now use:
  - forward is `+y`
  - left is `+x`
- Upper-computer yaw now uses:
  - `0°` -> facing `+y`
  - `90°` -> facing `+x`
  - `-90°` -> facing `-x`
  - `180°/-180°` -> facing `-y`
  - counterclockwise is positive

- Goal yaw used by navigation:

```c
target_yaw = upstream_goal_yaw;
```

- If fused yaw is enabled, upstream current yaw uses the same sign convention:

```c
up_yaw = upstream_current_yaw;
```

- All yaw comparisons are normalized into `[-180, 180]`.

## Navigation Decision Order

Navigation does nothing when `state` is `GRAB`, `STOP`, or `REALSE`.

Decision priority:

1. If `nav_current_valid == 0` or `nav_goal_valid == 0`, hold `STAND`
2. Compare yaw first
3. Only when yaw is aligned, compare position

Yaw alignment threshold:

```c
abs(normalize(actual_yaw - target_yaw)) <= 2.0f
```

When yaw is not aligned:

- `actual_yaw > target_yaw` -> `RIGHT_TURN`
- `actual_yaw < target_yaw` -> `LEFT_TURN`

Position arrival threshold:

```c
abs(current_x - target_x) < 8.0f &&
abs(current_y - target_y) < 8.0f
```

Where:

- `current_x - target_x`: left/right error
- `current_y - target_y`: forward/backward error

When yaw is aligned:

- position reached -> `STAND`
- position not reached -> `RUN`

When target position is reached:

- `UniComm_SendArrival()` sends `RCArrivalMX` once

## Run Yaw Correction

- `RUN` state still supports yaw-based step-length correction
- correction uses fused current yaw vs. current navigation goal yaw
- `RUN_YAW_CORRECTION_GAIN` remains `0.0f`, so this path is wired but disabled by default

## Notes

- Old `SIN...ZU`, `RCGRABMX`, and `RCPUTDOWNMX` parsing has been removed
- Old quadrant-based turn trigger logic has been removed
- `target_yaw` now means the current navigation goal yaw for debug output
