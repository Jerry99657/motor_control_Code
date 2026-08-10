# UART4 FOC 主从通信协议

## 1. 目标与链路

STM32H743 开发板作为主机，FOC 控制板作为从机：

```text
USB CDC / UART5 / ESP32
        │  77 68 ... 0A 主机命令帧
        ▼
STM32H743 Command Control
        │  协议转换
        ▼
UART4 115200 8N1
        │  换行结尾 ASCII
        ▼
FOC 控制板 USART1
```

UART5 继续服务 ESP32 和原有命令，不与 FOC 链路复用。UART4 只负责 FOC 命令与遥测。

## 2. 硬件连接和 CubeMX

本工程已经配置：

| STM32H743 | 功能 | 连接 FOC 控制板 |
| --- | --- | --- |
| PD1 | UART4_TX | PB7 / USART1_RX |
| PD0 | UART4_RX | PB6 / USART1_TX |
| GND | 共地 | GND |

两端均为 `115200 bit/s`、8 数据位、1 停止位、无校验、无硬件流控。信号必须交叉连接，并确保两块板使用兼容的 3.3 V 逻辑电平。

UART4 全局中断优先级为 6，低于 UART5 的优先级 5，也低于 TIM13 实时安全控制。`.ioc` 已同步写入该配置，后续使用 CubeMX 重新生成代码时能够保留。

## 3. 主机接收帧

FOC 设备号固定为 `0x0F`。写命令格式：

```text
77 68 LEN 0F 02 OP [PAYLOAD] 0A
```

所有 `float32` 均为 IEEE-754、小端字节序。

| OP | 功能 | LEN | PAYLOAD | UART4 输出 |
| --- | --- | --- | --- | --- |
| `01` | 速度目标 | `0B` | `float32 speed` | `Speed:<value>\n` |
| `02` | 位置目标 | `0B` | `float32 angle` | `Angle:<value>\n` |
| `03` | 转矩/电流目标 | `0B` | `float32 torque` | `Torque:<value>\n` |
| `04` | 选择电机 | `08` | `uint8 motor_id` | `Motor:0\n` 或 `Motor:1\n` |
| `05` | 配置无感模式 | `07` | 无 | `Sensorless\n` |
| `06` | 请求无感闭环锁定 | `07` | 无 | `Lock\n` |
| `07` | 保留 | — | — | 暂不开放 OpenLoop |
| `08` | 停止 FOC | `07` | 无 | `Speed:0\n` |

电机 ID 与 FOC 工程一致：`0=C2804`，`1=C2208`。

示例：`Speed:10.0` 的 `float32` 小端值为 `00 00 20 41`，完整帧为：

```text
77 68 0B 0F 02 01 00 00 20 41 0A
```

### 写入应答

主板把命令成功加入 UART4 队列后，从原输入通道返回：

```text
77 68 08 0F 02 OP RESULT 0A
```

`RESULT` 按 `int8_t` 解释：

- `0`：已加入 UART4 队列；
- `-1`：参数、长度或操作号错误；
- `-2`：发送队列已满；
- `-3`：转换后的 ASCII 行超长或格式错误。

该应答只说明 STM32H743 已接受并排队。现有 FOC 从板协议没有命令 ACK，因此它不能证明从板已经执行。

## 4. 文本调试命令

进入 LCD 的 `Command Control` 页面后，也可以通过 UART5 或 USB CDC 发送：

```text
FOC SPEED 10
FOC ANGLE 1.57
FOC TORQUE 0.4
FOC MOTOR 0
FOC MOTOR 1
FOC SENSORLESS
FOC LOCK
FOC STOP
```

每条命令以 `\r` 或 `\n` 结束，主板返回 `QUEUED` 或 `FAILED` 文本。

## 5. 遥测接收与状态读取

FOC 工程通过 USART1 每 10 ms 发送一个 68 字节 JustFloat 帧：

```text
16 × float32 + 00 00 80 7F
```

UART4 使用逐字节中断写入 512 字节环形缓冲，主循环负责查找帧尾和解析，不在 ISR 中执行浮点处理。上电时 FOC 输出的 `TEST\r\n` 会被自动跳过。

读取主板缓存的 FOC 摘要：

```text
77 68 06 0F 01 0A
```

返回固定 32 字节：

| 偏移 | 字段 |
| --- | --- |
| 0..1 | `77 68` |
| 2 | 长度 `20` |
| 3 | 设备号 `0F` |
| 4 | 读命令 `01` |
| 5 | 链路 flags |
| 6..9 | CH0 目标速度 float32 |
| 10..13 | CH1 实际速度 float32 |
| 14..17 | CH2 实际 Iq float32 |
| 18..21 | CH3 实际 Id float32 |
| 22..25 | CH8 启动/编码器状态 float32 |
| 26..29 | CH9 故障或诊断状态 float32 |
| 30 | 遥测帧计数低 8 位 |
| 31 | `0A` |

flags：

- bit0：最近遥测帧的 16 个 float 均为有限值；
- bit1：250 ms 内收到过有效遥测；
- bit2：UART4 RX 环形缓冲曾溢出；
- bit3：UART4 发生过 HAL 接收错误；
- bit4：UART4 TX 普通命令曾因队列已满而丢弃。

`Command Control` 页面显示 `FOC:UP/DOWN`、实际速度和 Iq，便于不用上位机快速确认接线。

### 5.1 合并到主板 USB CDC 的 VOFA 数据流

主板不把 FOC 的 16 通道 UART4 原始帧直接转发到 USB，也不在进入 FOC 页面时切换 VOFA 帧格式。`VOFA_Task_Process()` 保持 50 Hz 固定输出，在原有 8 路四轮电机数据后追加 6 路 FOC 数据，形成固定的 14 通道 JustFloat 帧：

| USB VOFA 通道 | 内容 | 来源 |
| ---: | --- | --- |
| CH0~CH3 | M1~M4 实际转速 rpm | H743 直流电机驱动 |
| CH4~CH7 | M1~M4 占空比 % | H743 直流电机驱动 |
| CH8 | 控制模式：0 停止、1 速度、2 位置、3 转矩 | H743 UART4 命令状态 |
| CH9 | 主机请求速度 rad/s | 最近一次成功排队的 `Speed:` |
| CH10 | 实际速度 rad/s | UART4 原始 CH1 |
| CH11 | 主机请求的本圈位置 rad | 最近一次成功排队的 `Angle:` |
| CH12 | 当前本圈位置 `[0, 2π)` rad | UART4 原始 CH13 取模 |
| CH13 | 多圈绝对机械位置 rad | UART4 原始 CH13 |

CH11 是用户给从板的“本圈目标”，CH13 是编码器累计得到的多圈绝对位置，两者坐标语义不同。

USB 帧固定为 `60` 字节（`14 × float32 + 00 00 80 7F`）。目标/实际电流、UART4 在线、编码器健康和对齐诊断目前在 `main.c` 中注释保留，后续需要时可恢复。FOC 目标缓存位于 `foc_link.c`，所有调用 `FOC_Link_SendSpeed/Angle/Torque/Stop()` 的入口共享同一状态。

## 6. 安全行为

- FOC 命令仍受 `Command Control` 页面权限控制；离开页面后不能继续写入新目标。
- 进入或离开 `Command Control`、KEY2 全局停止时都会请求 `FOC STOP`。
- STOP 是 UART4 高优先级消息：清除尚未发送的旧目标，只允许当前正在发送的一行完成，然后立即发送 `Speed:0`，避免 STOP 后旧命令再次启动电机。
- 非有限 float（NaN、Inf）不会发送给 FOC。
- UART4 使用中断收发，命令不会通过阻塞式 `HAL_UART_Transmit` 卡住主循环。

## 7. OpenLoop 暂不开放的原因

参考 FOC 工程中的 `OpenLoop:` 会设置 `g_openloop_debug=1`。现有 `Speed:0` 和 `Torque:0` 接收分支没有清除这个标志，因此主板无法用统一 STOP 保证退出开环诊断。

在 FOC 从板增加原生 `Stop` 命令之前，主机操作号 `0x07` 保留且返回参数错误。未来从板的安全 Stop 至少需要：

1. 清除 `g_openloop_debug`；
2. `Startup_Stop()`；
3. 目标速度和目标电流同时清零；
4. 切换到零转矩模式；
5. 返回明确 ACK。

## 8. 首次上板测试

1. 两块板断电后交叉连接 TX/RX 并共地。
2. 暂时不要连接电机负载，分别烧录固件。
3. 进入 `Command Control`，确认 LCD 从 `FOC:DOWN` 变为 `FOC:UP`。
4. 发送状态读取帧，检查 flags.bit1 和帧计数变化。
5. 先发送 `FOC MOTOR 0`，再发送小幅 `FOC TORQUE` 或 `FOC SPEED`。
6. 发送 `FOC STOP`，确认目标速度归零。
7. 测试离开页面和 KEY2，确认旧的排队目标不会在 STOP 后重新执行。
8. 最后再连接实际电机，并根据 FOC 板额定电流和速度逐步增加目标值。

## 9. LVGL FOC Control 页面

主菜单第 7 项为 `FOC Control`，仅用于 C2804 编码器闭环：

- 进入页面依次排队 `Speed:0` 和 `Motor:0`，确保从板先停止再切到有感电机；
- 速度环范围 `-100~100 rad/s`，步进 `10 rad/s`，实际速度来自 CH1；
- 位置环范围 `0.0~6.2 rad`，步进 `0.1 rad`，语义与从板 `JerryFOC_setPosition()` 一致，表示相对当前机械圈零点的位置；
- 页面实际位置使用 CH13 多圈机械角对 `2π` 取模，显示范围为 `[0, 2π)`；
- 只有 UART4 遥测在线、CH8 编码器健康且 CH15 对齐完成时，页面才允许下发运动目标；
- KEY2、页面退出或主板安全故障会发送高优先级 `Speed:0`；
- 安全故障存在时，页面仍可查看遥测，但不能下发新的速度或位置目标。

## 10. 浮点格式化依赖

FOC UART4 驱动使用 `snprintf("%g")` 把 float32 目标转换为从板所需的 ASCII，文本调试入口也使用 `sscanf("%f")`。本工程采用 newlib-nano，必须在顶层 `CMakeLists.txt` 保留：

```cmake
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-u_printf_float"
    "-u_scanf_float"
)
```

如果缺少 `_printf_float`，速度或位置命令会格式化失败并显示 `FOC UART4 queue failed(-3)`；如果缺少 `_scanf_float`，`FOC SPEED 10` 等文本浮点命令无法正确解析。可在最终 `.map` 文件中搜索 `_printf_float` 和 `_scanf_float` 确认链接成功。
