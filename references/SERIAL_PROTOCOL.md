# 上位机与下位机串口通讯协议

## 1. 概述

本协议用于陡坡越野（Steeplechase）项目中**上位机**（运行 AprilTag 视觉定位的 Linux 主机）与**下位机**（机器人运动控制器，如 MCU）之间的串口通讯。上位机通过相机检测 AprilTag，计算位姿后周期性下发定位数据（MEASURING）；到达触发区后下发动作指令（ACTIONING）并周期性重发，直至动作时长结束。

通讯方向为**单向**（上位机 → 下位机），无需 ACK 回复。

## 2. 物理层

| 参数   | 默认值         | 说明         |
| ------ | -------------- | ------------ |
| 接口   | `/dev/ttyUSB0` | 串口设备路径 |
| 波特率 | `115200`       | 数据速率     |
| 数据位 | 8              | 标准串口     |
| 停止位 | 1              | 标准串口     |
| 校验位 | 无             | 由协议层校验 |
| 流控   | 无             |              |
| 读超时 | 20 ms          | 每次 read 阻塞上限 |

> 物理层的其他参数（数据位、停止位、校验位、流控）代码中未显式配置，使用 `pyserial` 默认值（8N1，无流控）。

## 3. 数据包格式

协议使用**定长 32 字节二进制帧**。每个数据包为固定 32 字节，结构如下：

```
[state: 9 bytes][payload: 21 bytes][checksum: 1 byte][end_frame: 1 byte]
                    ↓ 共 30 字节
              XOR checksum 覆盖范围
```

- **字节序**：所有多字节数值使用**小端序**（little-endian）。
- **浮点数**：IEEE 754 binary32（4 字节单精度浮点）。
- **帧校验**：对前 30 字节逐字节 XOR，结果存入第 30 字节。
- **帧尾**：固定 `0xED`（第 31 字节）。

### 3.1 帧同步

接收端通过以下方式确认帧边界：

1. 读取 32 字节。
2. 验证前 9 字节是否为 `MEASURING` 或 `ACTIONING`（ASCII）。
3. 验证第 31 字节是否为 `0xED`。
4. 重新计算前 30 字节的 XOR，与第 30 字节比对。

## 4. 校验和算法

**算法**：对帧的前 30 字节逐字节进行 XOR 异或运算。

```python
def _xor_checksum(data: bytes) -> int:
    result = 0
    for byte in data:
        result ^= byte
    return result
```

**校验规则**：接收方将收到的 checksum 与重新计算的 checksum 做严格相等比较。如不匹配，该帧视为无效并丢弃。

## 5. 消息类型

协议定义两种消息类型，均为上位机 → 下位机：

| 类型        | 方向           | 用途                                              |
| ----------- | -------------- | ------------------------------------------------- |
| `MEASURING` | 上位机 → 下位机 | 周期性发送机器人相对障碍物的位姿信息（APPROACHING 状态） |
| `ACTIONING` | 上位机 → 下位机 | 动作触发指令（ACTION_RUNNING 状态，周期性发送）    |

---

### 5.1 MEASURING — 位姿更新

**方向**：上位机 → 下位机  
**发送时机**：在 `APPROACHING` 状态下，以 `pose_period_ms`（默认 50ms）为周期持续发送。  
**目的**：让下位机实时获取机器人相对目标 AprilTag 的空间位置和姿态，以进行伺服控制。

**帧格式（32 字节）**：

| 偏移 | 大小 | 字段         | 类型          | 格式     | 单位 | 说明                                                               |
| ---- | ---- | ------------ | ------------- | -------- | ---- | ------------------------------------------------------------------ |
| 0    | 9    | `state`      | 字符串（9 字节） | 原始     | —    | 固定值 `"MEASURING"`（ASCII）                                        |
| 9    | 1    | `tag_id`     | uint8         | `0x00`~`0x07` | —  | 当前追踪的 AprilTag ID                                              |
| 10   | 4    | `x_m`        | float32 LE    | 原始     | 米   | 机器人前进方向距离（相机相对 tag 的 Z 轴分量）                      |
| 14   | 4    | `y_m`        | float32 LE    | 原始     | 米   | 横向偏移（相机相对 tag 的 -X 轴分量）                               |
| 18   | 4    | `bearing_deg`| float32 LE    | 原始     | 度   | 水平方位角，`atan2(y_m, x_m)`，机器人前进方向相对 tag 中心的角度    |
| 22   | 4    | `yaw_deg`    | float32 LE    | 原始     | 度   | tag 相对机器人的水平旋转角度                                        |
| 26   | 4    | `margin`     | float32 LE    | 原始     | —    | 检测决策边距（AprilTag 解码质量），值越大代表检测越可靠             |
| 30   | 1    | `checksum`   | uint8         | 原始     | —    | 前 30 字节的 XOR 校验和                                             |
| 31   | 1    | `end_frame`  | uint8         | `0xED`   | —    | 帧尾，固定值                                                       |

**坐标说明**：
- `x_m`：机器人前进方向的距离（正前方为正），即相机坐标系中的 Z 轴分量。
- `y_m`：横向偏移，正值表示目标在机器人左侧（相机坐标系中的 -X 轴分量）。
- `bearing_deg = atan2(y_m, x_m)`，正值表示目标在机器人左侧。

**struct 打包格式**：`<9sB5fBB`

**示例**（hex dump）：
```
4d4541535552494e47 00 9a99193f 0ad7233c 0000003f 00000000 00003442 XX ED
│─ MEASURING ─────│tag│─ x=0.6 ─│y=0.01──│bearing=0.5──│─ yaw=0 ─│margin=45│ck│end│
```

---

### 5.2 ACTIONING — 动作触发

**方向**：上位机 → 下位机  
**发送时机**：当机器人到达触发距离且姿态满足条件（距离、方位角、偏航角均在容差内且稳定持续 `stable_frames` 帧）时，上位机开始周期性地发送 ACTIONING。  
**发送后行为**：上位机停止发送 MEASURING，改为周期性发送 ACTIONING（同 `pose_period_ms` 周期），持续 `action_duration_ms` 毫秒后自动进入下一障碍物。  
**无需回复**：下位机无需回复 ACK。

#### 5.2.1 触发条件

ACTIONING 仅在以下**全部条件**满足时才会被发送。每个条件由障碍物配置参数控制（见 `field.yaml` 中各 obstacle 的配置）。

| 条件   | 判断公式                                                | 控制参数                                       |
| ------ | ------------------------------------------------------- | ---------------------------------------------- |
| 距离   | `|x_m - trigger_distance_m| <= distance_tolerance_m`    | `trigger_distance_m`, `distance_tolerance_m`   |
| 方位角 | `|bearing_deg| <= bearing_tolerance_deg`               | `bearing_tolerance_deg`                        |
| 偏航角 | `|yaw_deg - yaw_target_deg| <= yaw_tolerance_deg`      | `yaw_target_deg`, `yaw_tolerance_deg`          |

- 三个条件**必须同时满足**，缺一不可。
- 前置要求：机器人必须在 `APPROACHING` 状态（已检测到目标 tag，且 `margin >= min_margin`）。若姿势无效（`is_pose_valid()` 返回 `False`），直接跳过本轮检查。

**连续稳定性要求**：上述三条件须**连续保持** `stable_frames` 帧（默认 3 帧）。中间只要有一帧不满足，计数器 `_stable_count` 立即归零，重新累计。

**状态流转**：
```
APPROACHING → (stable_count >= stable_frames) → ACTION_PENDING → 发送 ACTIONING → ACTION_RUNNING
```

**丢失 tag**：如果在 `APPROACHING` 状态下丢失目标 tag，状态退回 `SEARCHING`，终止触发检查。

**帧格式（32 字节）**：

| 偏移 | 大小 | 字段       | 类型          | 格式           | 说明                                         |
| ---- | ---- | ---------- | ------------- | -------------- | -------------------------------------------- |
| 0    | 9    | `state`    | 字符串（9 字节） | 原始          | 固定值 `"ACTIONING"`（ASCII）                 |
| 9    | 1    | `tag_id`   | uint8         | `0x0F`         | 保留字段（对齐），固定值                     |
| 10   | 20   | `action`   | 字符串（20 字节） | ASCII，左对齐 | 动作代码，字母字段结束后用 `-` 填充至 20 字节 |
| 30   | 1    | `checksum` | uint8         | 原始           | 前 30 字节的 XOR 校验和                      |
| 31   | 1    | `end_frame`| uint8         | `0xED`         | 帧尾，固定值                                 |

**action 字段编码规则**：
- 使用 ASCII 编码。
- 左对齐（前位填充）。
- 不足 20 字节的部分用 `-`（`0x2D`）填充。
- 例如 `"WEAVE"` 编码后占用字节 10~29 为：`WEAVE---------------`（5 个字母 + 15 个 `-`）。

**action_code 当前定义**（来自 `field.yaml`，编码为 20 字节填充格式）：

| action_code | 含义     | 对应障碍物     | 编码后（20 字节）            |
| ----------- | -------- | -------------- | ---------------------------- |
| `WEAVE`     | 绕桩     | weave_poles    | `WEAVE---------------`      |
| `SAND_PIT`  | 沙坑     | sand_pit       | `SAND_PIT------------`      |
| `DUCK`      | 低头通过 | height_bar     | `DUCK----------------`      |
| `CLIMB`     | 攀爬     | large_ramp     | `CLIMB---------------`      |
| `CROSS_A`   | 过桥 A   | bridge_a       | `CROSS_A-------------`      |
| `CROSS_B`   | 过桥 B   | bridge_b       | `CROSS_B-------------`      |
| `RAMP`      | 坡道     | small_ramp     | `RAMP----------------`      |
| `STEPS`     | 台阶     | t_steps        | `STEPS---------------`      |

**struct 打包格式**：`<9sB20sBB`

**示例**（hex dump）：
```
414354494f4e494e47 0f 57454156452d2d2d2d2d2d2d2d2d2d2d2d2d2d2d XX ED
│─ ACTIONING ─────│0f││── WEAVE---------------- ──││       │ck│end│
```

---

## 6. 通讯流程与状态机

### 6.1 状态转换

```
SEARCHING → APPROACHING → ACTION_PENDING → ACTION_RUNNING → DONE → ... → exit
```

### 6.2 每个障碍物的完整交互流程

```
上位机 (Host)                                  下位机 (MCU)
───────                                        ────────
                                                 等待 MEASURING 数据
┌─ SEARCHING ──────────────────────────────────────────────┐
│  未检测到目标 tag，不发送任何数据                          │
│  检测到目标 tag → 进入 APPROACHING                         │
└──────────────────────────────────────────────────────────┘

┌─ APPROACHING ───────────────────────────────────────────┐
│  每 50ms 发送一次 MEASURING（32 字节二进制帧）：            │
│  ──────────────────────────────────────────────────→     │
│  [MEASURING] tag=0 x=0.85 y=0.02 bearing=1.5 ...         │  接收 MEASURING，伺服控制运动
│  ──────────────────────────────────────────────────→     │
│  [MEASURING] tag=0 x=0.70 y=0.01 bearing=0.8 ...         │
│  ──────────────────────────────────────────────────→     │
│  [MEASURING] tag=0 x=0.60 y=0.01 bearing=0.5 ...         │  (距离到达触发条件)
│  触发条件满足 → 进入 ACTION_PENDING                       │
└──────────────────────────────────────────────────────────┘

┌─ ACTION_PENDING ────────────────────────────────────────┐
│  发送 ACTIONING，进入 ACTION_RUNNING：                    │
│  ──────────────────────────────────────────────────→     │
│  [ACTIONING] action=WEAVE                                │
└──────────────────────────────────────────────────────────┘

┌─ ACTION_RUNNING ────────────────────────────────────────┐
│  每 50ms 周期性发送 ACTIONING：                           │
│  ──────────────────────────────────────────────────→     │  收到 ACTIONING，开始执行动作
│  [ACTIONING] action=WEAVE                                │  (动作执行中...)
│  ──────────────────────────────────────────────────→     │
│  [ACTIONING] action=WEAVE                                │
│  ──────────────────────────────────────────────────→     │
│  [ACTIONING] action=WEAVE                                │
│  action_duration_ms 超时 → 进入 DONE                      │
└──────────────────────────────────────────────────────────┘

┌─ DONE ──────────────────────────────────────────────────┐
│  切换到下一个障碍物或结束                                 │
└──────────────────────────────────────────────────────────┘
```

### 6.3 路线结束

路线中所有障碍物完成后，上位机直接退出，不发任何特殊结束消息。下位机可通过检测通讯中断（连续未收到帧）来判断路线完成。

---

## 7. 与旧协议（v1 ASCII NMEA）的差异

| 项目       | v1（旧）                       | v2（当前）                         |
| ---------- | ------------------------------ | ---------------------------------- |
| 编码格式   | ASCII NMEA `$TYPE,...*XX\r\n` | 定长 32 字节二进制帧                |
| 消息类型   | POSE, ACTION, ACK, FINISH     | MEASURING, ACTIONING               |
| 通讯方向   | 双向（上位机 ⇄ 下位机）         | 单向（上位机 → 下位机）             |
| 序号       | uint16 自增 seq               | 无序号                             |
| 校验       | NMEA 风格 XOR（2 位 hex）      | 二进制 XOR（1 字节）                |
| 帧同步     | `$` 头 + `*` 分隔 + `\r\n` 尾 | 9 字节 state 头 + 0xED 尾 + XOR    |
| ACK 机制   | 有（ACTION/FINISH 需 ACK）     | 无                                  |
| 重试机制   | 有（max_retries=3）           | 无（通过周期性重发 ACTIONING 保障）  |
| 结束信号   | FINISH 消息                   | 直接停止发送                        |
| action 长度| 变长 ASCII                     | 定长 20 字节（`-` 填充）            |

---

## 8. 配置参数

所有串口协议相关参数在 YAML 配置文件的 `serial` 段中定义：

```yaml
serial:
  port: /dev/ttyUSB0
  baudrate: 115200
  timeout_ms: 20
  pose_period_ms: 50
```

| 参数             | 默认值         | 说明                                       |
| ---------------- | -------------- | ------------------------------------------ |
| `port`           | `/dev/ttyUSB0` | 串口设备路径                               |
| `baudrate`       | `115200`       | 波特率                                     |
| `timeout_ms`     | `20`           | 串口读超时（毫秒）                         |
| `pose_period_ms` | `50`           | MEASURING / ACTIONING 消息发送间隔（毫秒），即 20Hz |

---

## 9. 错误处理

### 9.1 上位机端

| 错误情况          | 处理方式                                       |
| ----------------- | ---------------------------------------------- |
| 串口写入异常      | 记录错误日志，不阻塞主循环                     |
| 串口打开失败      | 记录警告日志，以无串口模式继续运行             |
| 帧序列化异常      | `struct.pack` 类型错误由 Python 异常机制处理    |

### 9.2 下位机端建议

- 收到校验和不匹配的帧时，丢弃并等待后续帧。
- 收到首字节不是 `M` 或 `A` 的帧时，丢弃并通过滑动窗口搜索下一个可能的帧头。
- 验证 `end_frame == 0xED` 后再校验 checksum，减少误匹配。
- 如连续长时间未收到任何帧，可视为通讯中断并进入安全状态。
- ACTIONING 帧中的 `action` 字段需去除尾随 `-` 后得到实际动作代码。
- 收到新的 ACTIONING（可能含不同 action_code）时，以最新收到的为准。

---

## 10. 下位机帧解析参考

### 10.1 MEASURING 解析伪代码

```c
// 32 字节缓冲区
if (buf[31] != 0xED) return;  // 错误的 end_frame

uint8_t csum = 0;
for (int i = 0; i < 30; i++) csum ^= buf[i];
if (csum != buf[30]) return;  // 错误的 checksum

if (memcmp(buf, "MEASURING", 9) == 0) {
    uint8_t  tag_id      = buf[9];
    float    x_m         = *(float*)(buf + 10);
    float    y_m         = *(float*)(buf + 14);
    float    bearing_deg = *(float*)(buf + 18);
    float    yaw_deg     = *(float*)(buf + 22);
    float    margin      = *(float*)(buf + 26);
    // 使用位姿数据
}
```

### 10.2 ACTIONING 解析伪代码

```c
if (memcmp(buf, "ACTIONING", 9) == 0) {
    // tag_id = buf[9]（始终为 0x0F，忽略）
    char action[21];
    memcpy(action, buf + 10, 20);
    action[20] = '\0';
    // 去除尾部 '-' 字符
    for (int i = 19; i >= 0 && action[i] == '-'; i--)
        action[i] = '\0';
    // 此时 action 中包含类似 "WEAVE" 的动作代码
}
```

---

## 11. 参考实现

- 上位机协议编解码：`steeplechase/protocol.py`
- 上位机串口 I/O：`steeplechase/serial_link.py`
- 上位机状态机集成：`steeplechase/main.py` / `steeplechase/obstacles.py`
- 单元测试：`steeplechase/tests/test_protocol.py` / `steeplechase/tests/test_serial_link.py`
- 运行时配置：`steeplechase/configs/field.yaml`
