# unicomm 模块说明

## 1. 功能概述

`unicomm` 用于接收上位机通过 `UART7` 发送的 32 字节二进制定长帧，并完成：

- `MEASURING` 位姿帧解析
- `ACTIONING` 动作帧解析
- 最近一次有效 tag/action/位姿数据缓存
- PG1-PG8 阶段指示输出控制

模块初始化后通过 `BSP/usart` 注册 `UART7`，使用 `DMA + IDLE` 方式接收数据。协议定义见 `references/SERIAL_PROTOCOL.md`。

`UART8` 当前可被其他任务用于调试输出，`MEASURING/ACTIONING` 协议帧不再从 `UART8` 接收。

---

## 2. 对外接口

头文件：`Modules/unicomm/unicomm.h`

### 2.1 对外变量

```c
extern float current_x;
extern float current_y;
extern float target_x;
extern float target_y;
extern float upstream_current_yaw;
extern float upstream_goal_yaw;
extern uint8_t nav_current_valid;
extern uint8_t nav_goal_valid;

extern UniCommStage_e unicomm_latest_stage;
extern uint8_t unicomm_current_tag_id;
extern uint8_t unicomm_has_current_tag;
extern float unicomm_x_m;
extern float unicomm_y_m;
extern float unicomm_bearing_deg;
extern float unicomm_yaw_deg;
extern float unicomm_margin;
extern char unicomm_action_code[UNICOMM_ACTION_CODE_MAX_LEN + 1U];
```

说明：

- `unicomm_latest_stage` 表示最近一次有效帧阶段：`UNICOMM_STAGE_MEASURING` 或 `UNICOMM_STAGE_ACTIONING`。
- `unicomm_current_tag_id` 保存最近一次有效 `MEASURING` 的 `tag_id`，范围 `0x00` 到 `0x07`。
- `unicomm_has_current_tag` 表示是否已有可用于 `ACTIONING` 的 tag。
- `unicomm_x_m`、`unicomm_y_m`、`unicomm_bearing_deg`、`unicomm_yaw_deg`、`unicomm_margin` 保存 `MEASURING` 原始字段。
- `current_x/current_y` 为兼容原导航逻辑，当前由 `x_m/y_m * 100` 得到，单位为 cm；`target_x/target_y` 当前置为 `0`。
- `upstream_current_yaw` 使用 `yaw_deg`，`upstream_goal_yaw` 当前置为 `0`。

### 2.2 对外函数

```c
void UniComm_UART7_Init(void);
```

- `UniComm_UART7_Init()`：注册 UART7 接收实例。

---

## 3. 协议格式

协议使用固定 32 字节二进制帧：

```text
[state:9][payload:21][checksum:1][end_frame:1]
```

通用规则：

- `state` 为 ASCII：`"MEASURING"` 或 `"ACTIONING"`，固定 9 字节。
- 多字节数值为 little-endian。
- 第 30 字节保留为上位机协议中的 checksum 字段，但当前下位机不等待、不校验该字段。
- `end_frame` 固定为 `0xED`。

### 3.1 MEASURING 位姿帧

```text
offset  size  field
0       9     "MEASURING"
9       1     tag_id, 0x00-0x07
10      4     x_m, float32 LE
14      4     y_m, float32 LE
18      4     bearing_deg, float32 LE
22      4     yaw_deg, float32 LE
26      4     margin, float32 LE
30      1     checksum
31      1     0xED
```

处理结果：

- 若 `tag_id > 0x07`，丢弃该帧。
- 缓存 tag 和位姿字段。
- 设置 `nav_current_valid = 1`、`nav_goal_valid = 1`。
- 当前 tag 对应 PG 输出 `GPIO_RESET`，其余 PG 输出 `GPIO_SET`。

### 3.2 ACTIONING 动作帧

```text
offset  size  field
0       9     "ACTIONING"
9       1     reserved tag, 固定 0x0F
10      20    action, ASCII, '-' 填充
30      1     checksum
31      1     0xED
```

处理结果：

- 若第 9 字节不是 `0x0F`，丢弃该帧。
- 复制 action 字段，并去除尾部 `'-'` 后保存到 `unicomm_action_code`。
- 使用最近一次有效 `MEASURING.tag_id` 决定 PG 输出。
- 若还没有有效 tag，则 PG1-PG8 全部输出 `GPIO_RESET`。

---

## 4. PG1-PG8 指示逻辑

PG 引脚映射：

```text
tag_id 0x00 -> PG1
tag_id 0x01 -> PG2
tag_id 0x02 -> PG3
tag_id 0x03 -> PG4
tag_id 0x04 -> PG5
tag_id 0x05 -> PG6
tag_id 0x06 -> PG7
tag_id 0x07 -> PG8
```

阶段行为：

- 收到有效 `MEASURING`：最近 tag 对应 PG 输出 `GPIO_RESET`，其余 PG 输出 `GPIO_SET`。
- 收到有效 `ACTIONING`：最近 tag 对应 PG 输出 `GPIO_SET`，其余 PG 输出 `GPIO_RESET`。

PG1-PG8 在 `Core/Src/gpio.c` 中配置为推挽输出、无上下拉、低速，并在初始化时全部拉低。

---

## 5. 数据更新流程

1. `main.c` 调用 `UniComm_UART7_Init()`。
2. `unicomm` 使用 `USARTRegister()` 注册 `UART7`，接收缓冲区大小为 255 字节。
3. `BSP/usart` 在 `DMA + IDLE` 收到数据后调用 `UniComm_UART7_Callback()`。
4. `UniComm_ProcessIncomingBytes()` 将输入字节放入 32 字节滚动窗口。
5. 当窗口满足 state 和帧尾校验时，立即调用对应帧处理函数。
6. 解析成功后更新缓存变量和 PG1-PG8 输出。

---

## 6. 当前实现的重要行为

### 6.1 会更新的情况

以下条件同时满足时，帧会被处理：

- 32 字节窗口中的 `state` 为 `"MEASURING"` 或 `"ACTIONING"`。
- 第 31 字节为 `0xED`。
- `MEASURING.tag_id` 在 `0x00-0x07` 范围内，或 `ACTIONING` 第 9 字节为 `0x0F`。

### 6.2 不会更新的情况

以下情况会丢弃当前窗口：

- state 未知。
- 帧尾错误。
- `MEASURING.tag_id > 0x07`。
- `ACTIONING` 保留 tag 字节不是 `0x0F`。

### 6.3 分包和粘包能力

当前实现不是旧版“单次回调直接解析”，而是使用 32 字节滚动窗口：

- 支持一帧被拆到多次 UART 回调中。
- 支持一次回调中包含多帧。
- 支持前导噪声后重新同步到下一帧。

---

## 7. 源文件说明

- `unicomm.h`
  - 声明初始化接口、阶段枚举、缓存变量和兼容导航变量。
- `unicomm.c`
  - `UniComm_UART7_Init()`：注册 UART7。
  - `UniComm_UART7_Callback()`：UART7 接收回调入口。
  - `UniComm_ProcessIncomingBytes()`：32 字节滚动窗口组帧。
  - `UniComm_IsValidFrame()`：校验 state 和帧尾。
  - `UniComm_HandleMeasuringFrame()`：解析 `MEASURING` 并拉低当前 tag 对应 PG、拉高其余 PG。
  - `UniComm_HandleActioningFrame()`：解析 `ACTIONING` 并设置对应 PG。
  - `UniComm_SetActionPin()`：按最近 tag 设置 PG1-PG8。

---

## 8. 测试建议

### 8.1 MEASURING 测试

发送合法 `MEASURING` 帧，分别使用 `tag_id = 0x00` 到 `0x07`。

期望结果：

- `unicomm_latest_stage == UNICOMM_STAGE_MEASURING`
- `unicomm_current_tag_id` 等于发送的 `tag_id`
- `unicomm_has_current_tag == 1`
- 当前 tag 对应 PG 为低电平，其余 PG 为高电平

### 8.2 ACTIONING 测试

先发送 `tag_id = 0x03` 的合法 `MEASURING`，再发送合法 `ACTIONING`，action 字段例如 `WEAVE---------------`。

期望结果：

- `unicomm_latest_stage == UNICOMM_STAGE_ACTIONING`
- `unicomm_action_code == "WEAVE"`
- PG4 输出高电平，其余 PG 输出低电平

### 8.3 异常帧测试

建议覆盖：

- 错误帧尾
- 未知 state
- `MEASURING.tag_id = 0x08`
- `ACTIONING` 第 9 字节不是 `0x0F`

期望结果：

- 对应帧被丢弃。
- PG 输出不因错误帧进入新的有效阶段。

---

## 9. 评审结论

当前 `unicomm` 已切换为 `references/SERIAL_PROTOCOL.md` 中的 v2 二进制定长帧协议。

实现支持跨回调分包、粘包和前导噪声重同步；有效 `MEASURING` 使用最近 tag 拉低对应 PG、拉高其余 PG，有效 `ACTIONING` 使用最近 tag 点亮对应 PG、拉低其余 PG。
