# unicomm 模块说明

## 1. 功能概述

`unicomm` 用于接收上层通过 `UART8` 发送的 ASCII 协议数据，并完成两类信息解析：

- 定位数据帧
- 动作控制帧

模块初始化后会通过 `BSP/usart` 注册 `UART8`，使用 `DMA + IDLE` 方式接收数据。

---

## 2. 对外接口

头文件：`Modules/unicomm/unicomm.h`

对外变量：

```c
extern int32_t current_x;
extern int32_t current_y;
extern int32_t target_x;
extern int32_t target_y;
```

对外函数：

```c
void UniComm_UART8_Init(void);
```

---

## 3. 协议格式

### 3.1 定位数据帧

帧格式：

```text
SI N{X:xxxx,Y:xxxx},T{X:xxxx,Y:xxxx} ZU
```

实际连续字符串示例：

```text
SIN{X:0123,Y:0456},T{X:0789,Y:0001}ZU
```

解析成功后更新：

- `current_x` <- `N{X:xxxx,...}` 中的 `xxxx`
- `current_y` <- `N{...,Y:xxxx}` 中的 `xxxx`
- `target_x` <- `T{X:xxxx,...}` 中的 `xxxx`
- `target_y` <- `T{...,Y:xxxx}` 中的 `xxxx`

说明：

- 使用十进制解析
- 允许前导零
- `strtol` 支持负号，因此 `-123` 也可被解析
- 只有整帧格式完全匹配时才会更新，失败时保持旧值

### 3.2 动作控制帧

帧格式：

```text
RC GRAB MX
```

或

```text
RC PUTDOWN MX
```

实际连续字符串示例：

```text
RCGRABMX
RCPUTDOWNMX
```

解析成功后更新：

- `GRAB` -> `state = GRAB`
- `PUTDOWN` -> `state = STAND`

其余内容视为异常帧，不修改 `state`。

---

## 4. 数据更新流程

1. `main.c` 中调用 `UniComm_UART8_Init()`
2. `unicomm` 注册 `UART8`
3. `HAL_UARTEx_RxEventCallback()` 在收到一帧 DMA/IDLE 数据后调用模块回调
4. `unicomm` 从 `recv_buff` 中查找：
   - `SI ... ZU`
   - `RC ... MX`
5. 若匹配成功，则更新坐标或动作状态

---

## 5. 当前实现的重要行为

### 5.1 什么情况下会更新

以下条件同时满足时，数据会更新：

- 串口数据已经进入 `UART8` 的接收回调
- 一次回调里包含完整帧
- 帧头、帧尾正确
- 中间正文格式完全匹配

### 5.2 什么情况下不会更新

以下情况不会更新：

- 帧头或帧尾错误
- 坐标字段格式错误
- 动作正文不是 `GRAB` 或 `PUTDOWN`
- 一次回调中只收到半帧
- 上层把一帧拆成多段发送，且段间触发了空闲中断

### 5.3 当前实现限制

当前版本是“单次回调直接解析”方案，不带缓存拼接逻辑，因此：

- 适合上层一次发送一整帧
- 不适合跨多次回调组包
- 不适合复杂流式协议

如果后续上层存在分包、粘包、连续多帧流式发送，需要改成状态机或环形缓冲区解析。

---

## 6. 源文件说明

- `unicomm.h`
  - 对外声明初始化接口和坐标变量
- `unicomm.c`
  - `UniComm_UART8_Init()`
    - 注册 `UART8`
  - `UniComm_UART8_Callback()`
    - 串口回调入口
  - `UniComm_ExtractFramePayload()`
    - 按帧头帧尾提取正文
  - `UniComm_ParsePositionPayload()`
    - 解析定位正文
  - `UniComm_HandleActionFrame()`
    - 解析动作正文

---

## 7. 测试建议

建议串口发送以下内容验证：

### 定位帧测试

```text
SIN{X:0123,Y:0456},T{X:0789,Y:0001}ZU
```

期望结果：

- `current_x = 123`
- `current_y = 456`
- `target_x = 789`
- `target_y = 1`

### 动作帧测试

```text
RCGRABMX
```

期望结果：

- `state == GRAB`

```text
RCPUTDOWNMX
```

期望结果：

- `state == STAND`

### 异常帧测试

```text
RCHELLOMX
SIN{X:12,Y:34}ZU
```

期望结果：

- 坐标不更新或保持旧值
- `state` 不更新

---

## 8. review 结论

当前代码在“完整帧一次性进入回调”的条件下，接收到的数据会更新。

但要注意：

- 现在没有跨回调拼帧能力
- 如果上层发送节奏导致一帧被拆成两次 `RxEventCallback`，本次实现不会更新
- 如果后续通信场景更复杂，建议把 `unicomm` 改为流式状态机解析
