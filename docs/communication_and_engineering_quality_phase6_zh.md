# 第六阶段：通信和工程质量优化

## 1. 本阶段目标

本阶段保持现有外部协议、UART 波特率、USB VOFA 通道和页面操作方式不变，重点解决四类问题：

1. 通信协议不再依附于 LVGL 页面实现；
2. UART5、USB CDC 和 UART4 FOC 的异常能够被统计和定位；
3. UART 接收中断重挂失败后能够自动恢复；
4. 构建系统能够阻止隐式函数声明、错误返回类型和不兼容指针等高风险 C 语言问题进入固件。

## 2. 新的通信分层

```text
UART5 IRQ ─┐
           ├─ comm_service ─ AppEvent ─ command_protocol ─ command_control
USB CDC ───┘                                  │                │
                                              ├─ NES           ├─ 四电机
                                              ├─ FOC UART4     ├─ 麦轮
                                              └─ 原通道应答     └─ 小陀螺

UART4 IRQ ─ foc_link ─ JustFloat 解析/FOC 命令队列
USB IN    ─ telemetry_service ─ 14 通道 JustFloat
```

各模块职责如下：

- `comm_service.c`：UART5 收发环形缓冲、发送队列、USB 应用数据搬运及链路统计；
- `command_protocol.c`：二进制流重组、文本命令、长度和帧尾校验、命令分发以及原通道应答；
- `command_control.c`：Command Control 页面权限、速度限幅、麦轮和小陀螺执行、离页停止；
- `foc_link.c`：UART4 文本命令队列和从板 JustFloat 数据；
- `telemetry_service.c`：只负责周期性 VOFA JustFloat 输出，不再解析 USB 控制命令；
- `lvgl_app.c`：只显示通信状态和处理页面生命周期，不再解析通信帧。

## 3. 协议兼容性与校验

二进制协议仍使用：

```text
77 68 LEN DEV_ID CMD/PAYLOAD ... 0A
```

兼容的设备号和 ESP32 虚拟手柄格式保持不变。本阶段没有加入新的校验字节，避免破坏已有 ESP32、USB 上位机和 UART5 发送端。

解析器现在执行以下检查：

- 最短帧为 6 字节，最大帧为 16 字节；
- `LEN` 必须落在合法范围内；
- 收满 `LEN` 后最后一字节必须为 `0x0A`；
- 各命令必须满足自己的精确长度，不能再用过短帧访问不存在的载荷；
- 帧尾错误时逐字节重新同步，尽量保留坏帧后紧邻的下一帧；
- UART5 与 USB 各自保存独立解析状态，交错到达不会互相污染；
- Command Control 未激活时拒绝运动命令；NES 手机按键仍只在 NES 运行期间全局有效。

文本命令保持兼容：

- UART5/USB：`GYRO ON CW|CCW 0-100`、`GYRO OFF`、`FOC ...`；
- 仅 USB：`M1:50` 到 `M4:-100`、`STOP`。

麦轮速度模式的三个输入在 Command Control 层统一限制到 `-100~100`。距离模式沿用原协议单位和默认执行速度。

## 4. 自动恢复

UART5 和 UART4 仍采用单字节中断接收。HAL 完成回调或错误回调会立即尝试挂接下一字节；如果 HAL 暂时返回 BUSY/ERROR，则设置待恢复标志，由前台任务每 10 ms 重试。这样一次 ORE、噪声错误或状态竞争不会让串口永久停止接收。

UART5/UART4 的连续接收会在 LCD、QSPI、SD 和启动动画等阻塞式初始化结束后才开启。ESP32 即使从上电开始持续发送摇杆帧，也不会再因为前台尚未运行而填满 UART5 环形缓冲。

中断中只进行字节入环形缓冲、计数和重挂接收，不进行协议解析、LVGL 操作或阻塞发送。

VOFA 的周期 JustFloat 数据使用独立的“最新样本”发送槽。主机短暂停止读取时，旧遥测会被新遥测覆盖，不会挤占文本应答和启动日志队列；USB 枚举完成后即可发送，不再依赖欢迎文本是否已经成功发送。

## 5. Communication 诊断页

进入 `8 UI Diagnostics`，Left/Right 切换到 `8/9 COMMUNICATION`：

```text
U5 RX B... O/E ...
U5 TX Q/C ... D/E ...
USB RX B... O ...
USB TX Q/C ... D/E ...
Frm U/V ... E ...
Cmd A/R ... RD... FOC ...
```

缩写含义：

| 字段 | 含义 |
| --- | --- |
| `B` | 接收字节数 |
| `O` | 接收环形缓冲溢出 |
| `Q/C` | 成功入发送队列/发送完成 |
| `D` | 发送队列已满导致丢弃 |
| `E` | HAL/USB 启动收发失败或协议帧错误 |
| `Frm U/V` | UART5/USB 合法二进制帧数 |
| `Cmd A/R` | 协议层接受/拒绝的命令数 |
| `RD` | 命令应答无法入队次数 |
| `FOC O/E/D` | UART4 接收溢出/错误/发送丢弃 |

正常运行时，溢出、丢弃、启动错误、帧错误和应答丢弃应长期保持 0。`Q` 与 `C` 可以短暂相差发送队列中尚未完成的包数，但不应持续扩大。

`9/9 ARCHITECTURE` 汇总健康状态。新增健康标志：

| 位 | 含义 |
| ---: | --- |
| bit5 | USB 应用接收环形缓冲溢出 |
| bit6 | USB 发送队列丢弃 |
| bit7 | UART5 HAL 错误、重挂失败或发送启动失败 |
| bit8 | UART4 FOC 溢出、错误或发送丢弃 |
| bit9 | UART5/USB 二进制协议帧错误 |
| bit10 | 命令应答无法进入发送队列 |

这些标志用于诊断，不会因为一次通信错误直接冻结 UI。运动安全仍由 Command Control 权限、Safety Manager、KEY2 和 IWDG 负责。

曾出现的 `F 000003C2` 可拆分为：UART5 RX 溢出 `0x002`、USB TX 丢弃 `0x040`、UART5 错误/重挂失败 `0x080`、UART4 FOC 错误 `0x100`、协议坏帧 `0x200`。其中 UART5 溢出和随后产生的坏帧主要来自接收中断在耗时启动动画之前开启；USB 丢弃来自周期遥测与可靠消息共用小队列。本节描述的延后启动和最新遥测槽就是针对这两个根因。

## 6. 构建质量

顶层 CMake 现在：

- 使用标准的 `project(... LANGUAGES C ASM)` 初始化顺序；
- 对隐式函数声明、缺失返回值和不兼容指针按编译错误处理；
- 对 CubeMX 生成代码、FatFs、LVGL 和保留的 MJPEG 兼容函数只屏蔽已确认无害的特定告警；
- 将 Flash 初始化数组标记为只读，消除 ELF Flash 段的 RWX 权限；
- 保持浮点 `printf/scanf` 链接选项，FOC 文本命令不受影响。

## 7. 上板回归顺序

1. 开机进入 `8/9 COMMUNICATION`，确认错误、溢出和丢弃均为 0。
2. 进入 Command Control，用 ESP32 摇杆连续操作 60 秒；确认 UART5 RX 字节和合法帧持续增长，小车跟手。
3. 离开 Command Control 后继续操作手机；合法帧仍可增长，但车辆不得运动。
4. 重新进入页面，测试 GYRO ON/OFF、方向和速度，确认状态保持且退出页面立即停止。
5. 保持 ESP32 在线并让摇杆位于中位，USB 分别发送 `M1:50`、`M1:-50`，电机应持续运行而不是被中位帧覆盖；发送 `STOP` 后停止。随后再次发送 `M1:50`，主动移动手机摇杆应切换为摇杆控制，松手回中后正常停车。上述命令只能在 Command Control 生效。
6. USB 和 UART5 分别发送 FOC 文本/二进制命令，确认应答从请求原通道返回。
7. 运行 FOC 页面并观察 UART4 JustFloat；`FOC O/E/D` 应保持 0。
8. 打开 VOFA 运行至少 5 分钟，确认 USB `Q/C` 不持续拉开，`D/E` 保持 0。
9. 进入 NES，测试手机手柄和 RESET；退出 NES 后手机按键不得影响普通页面。
10. 最后查看 `9/9 ARCHITECTURE`，正常情况下 `Flags` 应为 `00000000`。

## 8. CubeMX 再生成注意事项

主要业务代码位于 `Drivers/User` 和顶层 `CMakeLists.txt`，CubeMX 不会覆盖。USB 统计代码全部放在 `usbd_cdc_if.c/.h` 的 USER CODE 区域。链接脚本中的自定义内存段和 `READONLY` 属性不带 USER CODE 标记，重新生成工程后应检查 `STM32H743ZITX_FLASH.ld` 是否仍保留这些修改。
