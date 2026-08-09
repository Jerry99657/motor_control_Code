# NES 手机虚拟手柄

## 功能概述

`ESP32_connect_Controller` 的手机网页现在包含两种互斥控制模式：

- `ROBOT`：原有麦轮双摇杆与小陀螺控制；
- `NES`：方向键、Select、Start、B、A 虚拟手柄。

网页右上角的 `OPEN NES / OPEN ROBOT` 按钮负责切换。切入 NES 前，ESP32 会先发送麦轮零速并显式关闭小陀螺；切回 ROBOT 前，会发送一次全部 NES 按键释放状态，避免切换后残留运动或按键。

手机 NES 手柄和开发板实体按键可以同时使用。STM32 对两组输入执行按位合并：例如实体方向键配合手机 A 键、手机方向键配合实体 KEY1（B）均可正常组合。

## 手机页面操作

NES 页面采用横向手柄布局：

| 手机控件 | NES 位 |
| --- | --- |
| A | bit0 |
| B | bit1 |
| Select | bit2 |
| Start | bit3 |
| Up | bit4 |
| Down | bit5 |
| Left | bit6 |
| Right | bit7 |

页面使用 Pointer Events 按指针编号保存状态，因此支持多点触控，可以同时按住方向键和 A/B。按下、抬起、触摸取消、浏览器失去焦点或页面进入后台时，都会更新或释放对应按键。

## 通信链路

```text
手机 Pointer Events
    -> WebSocket 二进制状态
    -> ESP32-C3
    -> USART 115200 bit/s
    -> STM32H743 UART5
    -> NES_Runtime 手机状态
    -> 与实体按键合并
```

手机到 ESP32 的 NES 状态包为：

```text
4E buttons sequence
```

ESP32 到 STM32 使用现有 `77 68` 帧格式，新增设备号 `0x0E`：

```text
77 68 08 0E 02 buttons sequence 0A
```

它是完整状态位图而不是单独的“按下/抬起事件”。即使某个无线包丢失，后续保活包也会覆盖旧状态，不会累计错误事件。

## STM32 输入隔离

NES 帧属于 UART5 全局通信帧，不再受 `Command Control` 页面限制，但只有 `NES_Runtime_IsActive()` 为真时才写入模拟器输入。

- 麦轮、单电机、小陀螺等控制帧仍只能在 `Command Control` 页面生效；
- NES 手机按键不会触发电机；
- KEY2、KEY3 仍只保留在开发板上，手机页面不能绕过紧急停止和目录返回逻辑；
- 手机 Start 只表示 NES Start，不触发实体 `OK+KEY1` 长按软复位功能。

合并后若出现 `Up+Down` 或 `Left+Right`，运行时会将该轴中和，防止手机与实体按键方向相反时产生不确定行为。

## 断连保护

NES 页面每 50 ms 发送完整状态。ESP32 和 STM32 都采用 350 ms 的手机按键释放超时：

- 超时只把手机按键清零；
- 不退出 NES；
- 不显示故障；
- 不冻结实体按键；
- 网络恢复后，下一帧状态自动继续生效。

这项超时不能删除，否则手机在按住方向键时断开 Wi-Fi，会使游戏一直保持该方向。

## 测试步骤

1. 分别烧录 STM32 和 `ESP32_connect_Controller` 固件。
2. 手机连接 `ESP32_JoyStick`，打开 `192.168.4.1`。
3. 在 ROBOT 页面确认原双摇杆功能正常，再点 `OPEN NES`。
4. 从 SD 页面打开 `.nes` 并按 OK 进入游戏。
5. 验证手机方向键、A、B、Select、Start。
6. 验证方向键+A/B 多点触控。
7. 同时使用手机和实体按键，确认可以交叉组合。
8. 按住手机方向键后关闭网页或断开 Wi-Fi，确认约 350 ms 后方向自动释放，实体按键仍可操作。
9. 按 KEY3 返回原 SD 目录，并确认 `.sav` 行为保持不变。

