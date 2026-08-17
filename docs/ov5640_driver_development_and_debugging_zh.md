# STM32H743 + OV5640 驱动开发与故障排查记录

> 文档日期：2026-08-17
> 适用工程：`STM32H743ZIT6_KIT`  
> 当前状态：`320 x 240 JPEG` 双缓冲连续预览约 10.7 FPS，拍照保存与相册浏览代码已经完成
> 已解决根因：主板 OV_RESET 与摄像头载板 RC 复位网络发生电气冲突；断开两者后恢复正常

## 1. 文档目的

本文记录 OV5640 功能从 CubeMX 配置、传感器寄存器驱动、DCMI/DMA 采集、JPEG 解码到 LVGL 显示的完整代码结构，并保留目前故障的演进过程和排查结论。

这份文档重点解决三个问题：

1. 后续维护时能够快速找到每一层代码的位置和职责。
2. CubeMX 重新生成代码后，能够判断哪些配置必须保留。
3. 在没有万用表、示波器和逻辑分析仪时，能够仅根据 LCD 诊断信息逐步缩小故障范围。

## 2. 当前结论摘要

此前长期停留的故障显示为：

```text
SCCB E04 L11 N0 P0R0
Step 2 ID 0x0000
```

各字段的直接含义是：

- `P0`：PF13 已将 `OV_PWDN` 拉低，摄像头已经退出掉电状态。
- `R0`：PC4 释放 `OV_RESET` 并等待 20 ms 后，复位线仍然是低电平。
- `N0`：程序发现 RESET 没有释放，因此没有发送任何 SCCB 字节。
- `Step 2`：停在复位释放和 SCCB 总线注册之前。
- `ID 0x0000`：尚未读取 `0x300A/0x300B`，不能据此判断传感器型号。
- `E04`：这里用于表示当前初始化失败；由于 `N0`，它不是一次真实的地址 NACK。

最终确认这不是软件 SCCB、DCMI 或 JPEG 问题，而是主板 `OV_RESET` 与摄像头载板自身的 2.8 V RC 复位网络互相冲突。硬件已将两者断开，摄像头复位完全交给载板管理；原 PC4 改为专用 `BUZZER` 输出，摄像头代码不得再读写 PC4。

修复后的阶段 1 验证结果：

- 进入 Camera Test 后能够显示 `Camera frame ready`。
- 按 OK 可以重复拍照并正常显示。
- 已连续拍照 100 次，无采集、解码或死机错误。
- 已反复进入/退出 Camera Test 20 次，无资源泄漏或异常。
- 已验证摄像头工作不影响 PC4 蜂鸣器。

阶段 2 在此基础上先完成了稳定的 5 FPS 连续预览。10 FPS 串行软件节拍测试实测约 5.5 FPS，采集约 103 ms、解码约 27 ms，并暴露横条花屏；随后将 OV5640 PCLK 提高到 48 MHz，采集仅降至约 81 ms，却出现 `Drop` 增加、频繁恢复和更严重花屏。首版双 JPEG 槽板测仍只有约 5.7 FPS，稳定采集耗时约 180 ms，说明此时解码已经不再是主瓶颈，OV5640 的实际 VSYNC 周期本身只有约 180 ms。当前版本改为工程内 H7参考例程使用的 15 FPS传感器时序：`0x3035=0x11`、`0x3824=0x1F`，并关闭自动夜间降帧；其目的在于提高内部帧生成率，同时通过 DVP分频避免重现48 MHz花屏。保留双 JPEG槽流水线后，实机达到约 10.7 FPS，采集耗时降至约 90 ms。

首次板测连续预览约为 3.7 FPS，同时发现上下颠倒、画面偏模糊且红蓝物体显示为黑白。后续板测结论和修复如下：

- 当前镜头模组是定焦版本，不带 VCM自动对焦机构；`CAMERA_ENABLE_AUTOFOCUS`保持为 0，不上传或启动 AF固件，界面显示 `AF:FIXED`。驱动中保留 AF实现，未来更换 AF模组后再单独启用验证。
- 方向从 `OV5640_MIRROR_FLIP`改为 `OV5640_MIRROR`，仅保留该载板需要的水平镜像，消除垂直颠倒且不增加逐帧 CPU开销。
- 明确写入 JPEG彩色输出、完整 ISP、自动白平衡、彩色矩阵和自动锐化参数。
- 黑白显示的直接原因是 `.avi`入口会调用 `JPEG_InitColorTables()`，而开机直接进入 Camera Test所走的内存 JPEG入口此前没有调用。硬件 JPEG仍能输出亮度/色度数据，但未初始化的 YCbCr到 RGB565查找表使色度转换结果近似黑白。现在两个入口都会在首次解码前完成一次性初始化。
- 预览区域保持为 `224 x 168`。

## 3. 硬件版本与原理图基准

### 3.1 最终采用的摄像头板原理图

必须以用户最后提供的 2024-05 版 OV 系列摄像头驱动板原理图为准。该版本具有以下特征：

- `OV_RESET` 直接连接 OV5640 的 RESET 引脚。
- `OV_RESET` 由 R3（10 kOhm）上拉到 `DOVDD_2V8`，并由 C13（0.1 uF）对地形成 RC 复位。
- 不存在旧原理图中的 `EX_RST + Q2` 反相复位电路。
- R10/R11（5.1 kOhm）分别将 `OV_SDA/OV_SCL` 上拉到 `DOVDD_2V8`。
- U2（TPS76328）将板上 `VCC_3V3` 转换为 `VCC_2V8`，再通过 R2（0 Ohm）生成 `DOVDD_2V8`。
- Y1 是板载 24 MHz 有源晶振，直接向 OV5640 提供 `OV_XCLK`，STM32 不需要输出 XCLK。

### 3.2 必须遵守的电平规则

| 信号 | STM32 配置 | 高电平来源 | 原因 |
|---|---|---|---|
| `OV_RESET` | **不连接 STM32** | 摄像头板 R3 上拉到 2.8 V | 载板 RC 网络独立完成上电复位，避免与主板控制冲突 |
| `BUZZER` / PC4 | 推挽输出、无内部上下拉 | STM32 3.3 V逻辑 | PC4 仅驱动蜂鸣器，与摄像头彻底解耦 |
| `OV_SCL` / PF14 | 开漏、无内部上拉 | 摄像头板 R11 上拉到 2.8 V | SCCB 总线由板载 5.1 kOhm 上拉 |
| `OV_SDA` / PF15 | 开漏、无内部上拉 | 摄像头板 R10 上拉到 2.8 V | ACK 和读取阶段必须允许传感器驱动 SDA |
| `OV_PWDN` / PF13 | 推挽输出 | STM32 3.3 V逻辑 | 高电平掉电，低电平工作 |

严禁再次采用以下配置：

- 在摄像头代码中读写 PC4或重新连接主板与载板 `OV_RESET`。
- PF14/PF15 推挽输出高电平。
- PF14/PF15 启用 STM32 的 3.3 V内部上拉。
- 将旧版带 Q2 的 `EX_RST` 原理图逻辑套用到当前摄像头板。

### 3.3 主板到摄像头板的信号映射

以下映射来自主板摄像头接口图、背部 FPC 插座图和当前 CubeMX 引脚配置。

| 功能 | STM32 引脚 | DCMI/控制功能 | 摄像头板信号 |
|---|---|---|---|
| SCCB 时钟 | PF14 | 软件 SCCB SCL / I2C4_SCL | `OV_SCL` |
| SCCB 数据 | PF15 | 软件 SCCB SDA / I2C4_SDA | `OV_SDA` |
| 硬件复位 | 不连接 | 由载板 R3/C13管理 | `OV_RESET` |
| 蜂鸣器 | PC4 | 推挽 GPIO | 不连接摄像头 |
| 掉电控制 | PF13 | 推挽 GPIO | `OV_PWDN` |
| 行同步 | PA4 | DCMI_HSYNC | `OV_HREF`/HSYNC |
| 场同步 | PG9 | DCMI_VSYNC | `OV_VSYNC` |
| 像素时钟 | PA6 | DCMI_PIXCLK | `OV_PCLK` |
| 数据 0 | PC6 | DCMI_D0 | `OV_D0` |
| 数据 1 | PC7 | DCMI_D1 | `OV_D1` |
| 数据 2 | PG10 | DCMI_D2 | `OV_D2` |
| 数据 3 | PG11 | DCMI_D3 | `OV_D3` |
| 数据 4 | PE4 | DCMI_D4 | `OV_D4` |
| 数据 5 | PD3 | DCMI_D5 | `OV_D5` |
| 数据 6 | PE5 | DCMI_D6 | `OV_D6` |
| 数据 7 | PE6 | DCMI_D7 | `OV_D7` |
| 外部时钟 | 不接 STM32 | 板载 Y1 提供 24 MHz | `OV_XCLK` |

插拔 FPC 前必须先断电。需要同时确认：

- 主板接口的 3 个 3.3 V引脚正确映射到摄像头板 J2 的 22/23/24 脚。
- GND 引脚正确映射。
- FPC 金手指方向与连接器触点方向一致。
- PF15/PF14 没有因为排线方向错误而与其他信号交换。

## 4. CubeMX 与底层外设配置

### 4.1 PC4 与 PF13

当前 `.ioc` 中：

- PC4：`GPIO_Output`，推挽输出、无上下拉，标签 `BUZZER`。
- PF13：`GPIO_Output`，默认输出高电平，标签 `OV_PWDN`。

上电默认状态是：

```text
OV_PWDN = 1  摄像头掉电
BUZZER = 0    蜂鸣器关闭
```

`Camera_Service_BootHold()` 只保持 `OV_PWDN=1`，不会访问 PC4。蜂鸣器由独立设置/告警模块管理。

### 4.2 I2C4 与软件 SCCB

CubeMX 仍然保留 I2C4：

- PF14：I2C4_SCL，开漏，无内部上拉。
- PF15：I2C4_SDA，开漏，无内部上拉。
- I2C4 时序值：`0x707075B1`。
- 模拟滤波器开启，数字滤波器为 0。

当前传感器寄存器通信最终使用 GPIO 模拟 SCCB，而不是直接调用 `HAL_I2C_Mem_Read()`。保留 I2C4 的作用是：

1. 让 CubeMX 正确管理 I2C4 时钟和引脚归属。
2. 初始化前执行 HAL I2C 状态机复位和 9 个恢复时钟。
3. 为后续切回硬件 I2C保留配置基础。

进入真正寄存器访问前，PF14/PF15 会被重新配置为开漏 GPIO，并使用板上的 2.8 V外部上拉。

### 4.3 DCMI

当前 DCMI 配置：

| 参数 | 配置 |
|---|---|
| 同步方式 | Hardware synchronization |
| 数据宽度 | 8 bit |
| PCLK 采样 | Rising edge |
| VSYNC 极性 | High |
| HSYNC/HREF 极性 | Low |
| 帧采样率 | All frames |
| JPEG 模式 | Enable |
| 采集模式 | Snapshot（运行时传入） |

### 4.4 DCMI DMA

| 参数 | 配置 |
|---|---|
| DMA | DMA1 Stream1 |
| Request | DCMI |
| 方向 | Peripheral to memory |
| 外设宽度 | Word |
| 内存宽度 | Word |
| 内存递增 | Enable |
| 模式 | Normal |
| 优先级 | Very High |
| FIFO | Enable |
| CubeMX FIFO阈值 | Full |
| Camera Service运行时阈值 | 1/4 Full |
| DCMI 中断优先级 | 6 |

Camera Service 在启动摄像头时将 DMA FIFO阈值调整为 `DMA_FIFO_THRESHOLD_1QUARTERFULL`。这样可以降低可变长度 JPEG 在帧尾结束时仍有少量数据停留在 DMA FIFO 内的概率。

### 4.5 JPEG、MDMA、Cache 和内存区域

- STM32H743 硬件 JPEG 外设由 `MX_JPEG_Init()` 初始化。
- JPEG 输入/输出 FIFO 使用 MDMA。
- D-Cache 已启用，但 DMA使用的 D2 SRAM被 MPU配置为不可缓存区域。
- 摄像头 JPEG 压缩缓冲位于 `.ram_d2`，总容量仍为 128 KiB，拆成两个 64 KiB、32 字节对齐的槽位。
- RGB565 输出使用共享媒体内存池 `.media_pool`，位于 AXI SRAM。

这种布局保证 DCMI DMA写入 JPEG、JPEG/MDMA读取压缩数据和生成 RGB565 时不会因为 D-Cache 一致性而破坏数据。

## 5. 代码模块结构

| 文件 | 主要职责 |
|---|---|
| `Drivers/User/Inc/camera_service.h` | 摄像头状态、错误码、诊断结构和公共 API |
| `Drivers/User/Src/camera_service.c` | PWDN、SCCB、ID检测、OV5640配置、DCMI/DMA快照、JPEG帧边界查找 |
| `Drivers/User/Inc/camera_album.h` | 相册目录、保存/读取错误码和公共 API |
| `Drivers/User/Src/camera_album.c` | SD挂载、目录创建、照片编号、临时文件原子写入和 JPEG读取 |
| `Drivers/User/Inc/ov5640.h` | ST OV5640组件驱动接口、分辨率和像素格式定义 |
| `Drivers/User/Inc/ov5640_af_firmware.h` | 参考模组使用的 OV5640内部 MCU自动对焦固件 |
| `Drivers/User/Src/ov5640.c` | ST OV5640初始化表和传感器控制函数 |
| `Drivers/User/Inc/ov5640_reg.h` | OV5640寄存器地址定义 |
| `Drivers/User/Src/ov5640_reg.c` | ST组件驱动寄存器读写包装 |
| `Drivers/User/Src/lvgl_app.c` | Camera Test页面、按键处理、状态机、错误显示、预览图显示 |
| `Drivers/User/Src/mjpeg_player.c` | 复用硬件 JPEG流水线，将内存中的 JPEG解码为 RGB565 |
| `Drivers/User/Src/media_memory.c` | 摄像头、MJPEG、SD动画和 QSPI动画共享 RGB帧缓冲 |
| `Core/Src/main.c` | CubeMX外设初始化、Cache/MPU配置、启动时调用 `Camera_Service_BootHold()` |
| `Core/Src/stm32h7xx_hal_msp.c` | DCMI GPIO/DMA/中断、I2C4 GPIO、JPEG MDMA配置 |
| `STM32H743ZIT6_KIT.ioc` | 可重新生成的 CubeMX配置源 |
| `STM32H743XX_FLASH.ld` | `.media_pool`、`.ram_d2`、`.ram_d3_bdma` 等内存段布局 |

所有自定义摄像头业务代码位于 `Drivers/User`，不会被 CubeMX 直接覆盖。PC4/BUZZER、PF13、PF14、PF15、DCMI、DMA等生成代码由 `.ioc` 负责保持一致。

## 6. Camera Service 状态机

### 6.1 对外状态

| 状态 | 数值 | 含义 |
|---|---:|---|
| `CAMERA_STATE_OFF` | 0 | PWDN有效，摄像头关闭；RESET由载板管理 |
| `CAMERA_STATE_INITIALIZING` | 1 | 正在上电、读取 ID或写初始化寄存器 |
| `CAMERA_STATE_READY` | 2 | OV5640配置完成，可以启动快照 |
| `CAMERA_STATE_CAPTURING` | 3 | DCMI/DMA正在采集 |
| `CAMERA_STATE_FRAME_READY` | 4 | 已找到完整 JPEG SOI/EOI |
| `CAMERA_STATE_ERROR` | 5 | 初始化、采集或帧校验失败 |

### 6.2 初始化阶段

| LCD Step | 枚举 | 含义 |
|---:|---|---|
| 0 | `CAMERA_INIT_STAGE_NONE` | 未初始化或已初始化完成 |
| 1 | `CAMERA_INIT_STAGE_I2C_RECOVERY` | 正在复位 I2C并生成恢复时钟 |
| 2 | `CAMERA_INIT_STAGE_I2C_PROBE` | 退出 PWDN并注册 SCCB总线 |
| 3 | `CAMERA_INIT_STAGE_READ_ID` | 读取 `0x300A/0x300B` |
| 4 | `CAMERA_INIT_STAGE_SENSOR_CONFIG` | 写入 ST OV5640初始化表 |
| 5 | `CAMERA_INIT_STAGE_CAPTURE` | DCMI/DMA采集阶段 |

### 6.3 初始化完整流程

```text
进入 Camera Test
    |
    v
Camera_Service_Sleep / BootHold
PWDN=1；载板独立管理 RESET
    |
    v
检查 DCMI 和 DMA句柄
    |
    v
I2C4 DeInit -> PF14/PF15输出9个恢复时钟 -> STOP -> I2C4 Init
    |
    v
PWDN=0，等待10 ms
    |
    v
等待传感器和载板 RC复位稳定20 ms
（不访问 PC4/BUZZER）
    |
    v
注册 OV5640 Bus IO，PF14/PF15切到开漏软件 SCCB
    |
    v
最多3次读取 0x300A/0x300B
    |
    +-- 不是0x5640 --> I2C/ID错误
    |
    v
OV5640_Init(320x240, JPEG)
OV5640_SetPCLK(24M)
    |
    v
等待AE/AWB稳定100 ms
    |
    v
CAMERA_STATE_READY
```

连续预览时，初始化表和 `OV5640_Start()`只执行一次，传感器保持输出，让 AE/AWB连续工作。每帧结束后只停止当前 DCMI传输，下一帧直接从 `FRAME_READY`启动新的 DCMI快照；只有退出页面或自动恢复时才停止传感器并执行完整 `Sleep()`。

## 7. SCCB 实现细节

### 7.1 地址形式

- OV5640 的 7 位 SCCB 地址是 `0x3C`。
- 代码中使用带 R/W位位置的 8 位写地址 `0x78`。
- 读地址是 `0x79`。

不要将 `0x3C` 直接传给当前软件 SCCB发送函数，否则线上地址会错误。

### 7.2 为什么不用 `HAL_I2C_IsDeviceReady()`

最初版本在 ST组件的 `Bus_Init()` 中调用：

```c
HAL_I2C_IsDeviceReady(&hi2c4, 0x78, ...)
```

该版本出现 `E20` 超时，并停在 Step 2。参考 ESP32摄像头库后确认，其探测逻辑是发送普通地址阶段并根据 ACK识别设备，不应把组件总线注册与固定地址 Ready轮询绑定。

当前 `Camera_Bus_Init()` 只负责检查总线状态和切换软件 SCCB引脚，不再做 `IsDeviceReady()`。

### 7.3 寄存器读取事务

参考 Arduino/ESP32驱动和旧 F407例程后，读取 16 位寄存器地址采用两个独立事务：

```text
START
0x78
REG_HIGH
REG_LOW
STOP

START
0x79
READ DATA
NACK
STOP
```

没有使用 `HAL_I2C_Mem_Read()` 的组合内存读取流程。

### 7.4 软件 SCCB时序

- 使用 DWT `CYCCNT` 产生约 2 us基础延时。
- SCL、SDA均为开漏输出。
- 输出逻辑 1 表示释放引脚，由板载 2.8 V电阻拉高。
- 每个字节第 9 个时钟前将 SDA切换为输入，读取传感器 ACK。
- 读取数据时 SDA保持输入，最后一个字节发送 NACK。

软件 SCCB只在初始化和传感器控制时使用，不在 DCMI高速采集过程中逐像素工作，因此阻塞式位操作不会影响图像数据吞吐。

### 7.5 为什么直接读取 ID

ST的 `OV5640_ReadID()` 会先向 `0x3008` 写软件复位，再等待 500 ms，然后读取 ID。为了缩短故障链并与 ESP32的 `ov5640_detect()` 一致，当前代码直接读取：

```text
0x300A -> ID高字节，期望0x56
0x300B -> ID低字节，期望0x40
```

软件复位由后续 `OV5640_Init()` 的初始化表执行。

## 8. DCMI 快照与 JPEG 显示流水线

### 8.1 采集阶段

`Camera_Service_StartSnapshot()` 执行：

1. 检查状态必须为 READY。
2. 调用 `OV5640_Start()`。
3. 清除 DCMI 帧、溢出、同步错误标志。
4. 只启用 FRAME、ERR、OVR中断。
5. 以 `DCMI_MODE_SNAPSHOT` 启动 DMA。
6. DMA目标为 128 KiB的 `s_jpeg_buffer`。

### 8.2 中断与主循环分工

DCMI中断回调只设置轻量事件标志：

- `HAL_DCMI_FrameEventCallback()` 设置 `s_frame_event`。
- `HAL_DCMI_ErrorCallback()` 设置 `s_error_event`。

真正的停止 DMA、计算长度和 JPEG校验在 `Camera_Service_Process()` 主循环中完成，避免在中断里执行耗时操作。

### 8.3 JPEG长度与边界

DMA使用 word宽度，快照结束后通过 DMA NDTR反推实际接收字节数。随后：

1. 从缓冲头开始寻找 JPEG SOI：`FF D8`。
2. 从 SOI之后寻找 EOI：`FF D9`。
3. 如果 SOI不在缓冲首地址，则用 `memmove()` 将完整 JPEG移到缓冲开头。
4. 若缓冲耗尽仍没有 EOI，返回 BUFFER_FULL或 INVALID_JPEG。

### 8.4 解码和 LVGL显示

Camera Test页面取得 JPEG后：

1. DCMI 将完整 JPEG 写入当前 64 KiB 槽，帧尾校验通过后将该槽标记为 `READY`。
2. UI 取得快照后，槽状态变为 `IN_USE`；Camera Service 随即在另一空闲槽启动下一帧 DCMI 快照。
3. 在下一帧采集并行进行时，上一帧继续等待 30 ms 帧尾保护，然后调用 `MJPEG_Player_DecodeMemoryToRgb565()`；该入口在首次使用时独立执行 `JPEG_InitColorTables()`，并复用硬件 JPEG/MDMA 解码流水线。
4. 解码完成或失败后都显式释放 JPEG 槽，之后 DCMI 才能重新使用该槽。
5. 首帧从共享媒体内存池申请 `320 x 240 x 2` 字节 RGB565 缓冲，后续帧持续复用，并原地缩小为 `224 x 168`。
6. 构造 `lv_img_dsc_t`，在 LVGL 预览卡片中显示，同时更新实际 FPS、JPEG 大小、采集/解码耗时、帧号和累计丢帧。

双槽采用明确的 `FREE -> READY -> IN_USE -> FREE` 所有权流程。系统只允许一帧采集、一帧解码，不建立更深队列；这样既能重叠 DCMI 与 JPEG/MDMA/LCD 工作，也不会因 UI 暂时变慢而持续占用更多内存。两个 64 KiB 槽合计仍为原来的 128 KiB，因此本次优化没有新增整帧 SRAM 开销。

首次双槽实测出现 `FPS DB AF:FIX` 约 5.7 FPS、`C` 约 180 ms。这里 `C` 包含等待 OV5640 下一次 VSYNC和接收完整 JPEG的时间；稳定在180 ms意味着传感器源帧率约为 `1000 / 180 = 5.6 FPS`。双缓冲只能隐藏约27 ms解码耗时，不能提高传感器没有产生的帧率。因此后续增加了15 FPS内部时序，统计标记也改为 `DB15`。若 `DB15` 下 `C`仍接近180 ms，应优先检查寄存器读回、AEC低照度行为和传感器时序，而不是继续增加软件目标 FPS。

由于 LCD刷新使用异步 DMA，下一次 JPEG解码覆盖共享 RGB565缓冲前必须先调用 `lv_port_disp_wait_idle()`。否则 LCD DMA可能一边读取旧帧，一边被解码器写入新帧，产生撕裂或花屏。

离开页面会：

- 停止 DCMI。
- 令 PWDN=1；不操作载板 RESET和 PC4蜂鸣器。
- 释放共享媒体内存。
- 使 LVGL图片描述符失效。

## 9. Camera Test 页面操作

| 操作 | 行为 |
|---|---|
| 进入页面 | 初始化 OV5640并自动启动24 MHz双缓冲连续预览 |
| OK | 暂停/继续连续预览；暂停发生在当前采集/解码完成之后 |
| Left | 停止摄像头、释放内存并返回主页面 |
| KEY2 / ESC | 停止摄像头并返回 |
| KEY3 | 由全局媒体退出逻辑停止当前功能并返回 |

页面内部阶段包括 OFF、INIT_PENDING、READY、CAPTURING、DECODE_PENDING、SHOWING和 ERROR。初始化、采集、解码和下一帧调度分散到不同主循环周期。单个采集坏帧会保留上一张有效画面并自动重启摄像头；如果解码器已可能覆盖 RGB缓冲，则暂时隐藏图像，避免显示半帧。连续 3 次失败才进入 ERROR，按 OK可再次初始化。

## 10. LCD 诊断字段

错误显示格式：

```text
SCCB Exx Lxy Nz Pp RST EXT
Step s ID 0xiiii
```

### 10.1 `E`：I2C/SCCB错误

| 值 | 含义 |
|---|---|
| `E00` | 没有记录到总线错误 |
| `E04` | `HAL_I2C_ERROR_AF`；硬件 I2C中表示 NACK，软件 SCCB中用于表示 ACK失败 |
| `E20` | `HAL_I2C_ERROR_TIMEOUT`；早期硬件 I2C探测曾出现该错误 |

### 10.2 `Lxy`：总线空闲电平

- `x` 是 SCL/PF14。
- `y` 是 SDA/PF15。

最新固件已经关闭 STM32内部上拉，因此 `L11` 原则上应由摄像头板 R10/R11和 `DOVDD_2V8` 产生。该结论成立的前提是主板上没有其他外部上拉。

| 值 | 初步判断 |
|---|---|
| `L11` | 两条线均处于空闲高电平，总线没有被拉死；也间接支持 2.8 V上拉可能存在 |
| `L00` | 两条 2.8 V上拉均未建立，优先怀疑摄像头板无 3.3 V或 U2无 2.8 V输出 |
| `L01` | SCL被拉低、短路或映射错误 |
| `L10` | SDA被拉低、短路、传感器卡住或映射错误 |

### 10.3 `N`：首次 NACK位置

| 值 | 含义 |
|---:|---|
| 0 | 尚未发送 SCCB，或者最近一次事务成功 |
| 1 | 写设备地址 `0x78` 时没有 ACK |
| 2 | 寄存器高字节没有 ACK |
| 3 | 寄存器低字节没有 ACK |
| 4 | 写寄存器数据时没有 ACK |
| 5 | 读设备地址 `0x79` 时没有 ACK |

### 10.4 `P` 与 `RST EXT`

| 字段 | 正常工作值 | 含义 |
|---|---:|---|
| `P` | 0 | PF13/PWDN为低，摄像头工作 |
| `RST EXT` | - | OV_RESET由摄像头载板管理，MCU不采样、不驱动 |

不要为了恢复 `R0/R1`显示而把 PC4临时切成输入；PC4现在属于蜂鸣器，这会重新引入功能冲突。

## 11. Camera Result错误码

| 返回值 | 枚举 | 说明 |
|---:|---|---|
| 0 | `CAMERA_RESULT_OK` | 成功 |
| -1 | `CAMERA_RESULT_INVALID_ARGUMENT` | 空指针或参数错误 |
| -2 | `CAMERA_RESULT_BUSY` | 摄像头或共享内存正在使用 |
| -3 | `CAMERA_RESULT_NOT_READY` | 摄像头未完成初始化 |
| -4 | `CAMERA_RESULT_I2C` | I2C恢复或 SCCB通信失败 |
| -5 | `CAMERA_RESULT_BAD_SENSOR_ID` | 读到的 ID不是 0x5640 |
| -6 | `CAMERA_RESULT_SENSOR_INIT` | OV5640初始化表或 Start/Stop失败 |
| -7 | `CAMERA_RESULT_DCMI` | DCMI/DMA初始化或采集错误 |
| -8 | `CAMERA_RESULT_TIMEOUT` | 快照超时 |
| -9 | `CAMERA_RESULT_BUFFER_FULL` | 128 KiB缓冲不足以容纳 JPEG |
| -10 | `CAMERA_RESULT_INVALID_JPEG` | 没有找到有效 SOI/EOI |

## 12. 本次故障演进记录

### 12.1 初始现象

```text
ID 0x0000
Error -4
DCMI 0x00000000
Camera failed(-4)
```

DCMI错误为 0，说明失败发生在传感器初始化阶段，不是图像采集阶段。

### 12.2 加入 I2C状态诊断

第一次详细结果：

```text
I2C E20 H20 L11
Step 2 ID 0x0000
```

含义：HAL状态为 READY，总线空闲，但 `HAL_I2C_IsDeviceReady()` 超时。

处理：

- 移除 Bus Init中的固定地址 `IsDeviceReady()`。
- 不再使用 ST `OV5640_ReadID()` 先复位再读 ID的流程。
- 改为直接读 `0x300A/0x300B`。
- 读取寄存器时在地址写阶段后发送 STOP，再开始独立读事务。

### 12.3 地址阶段 NACK

修改后结果变为：

```text
I2C E04 H20 L11
Step 3 ID 0x0000
```

错误从超时变为 AF/NACK，说明 STM32能够完成事务，但传感器没有对地址应答。

处理：

- 参考 ESP32预编译 SCCB驱动和 F407例程，实现 DWT计时的软件 SCCB。
- 增加 NACK阶段编号。
- 增加 PWDN和 RESET实际电平采样。

### 12.4 定位到 RESET未释放

诊断最终变为：

```text
SCCB E04 L11 N0 P0R0
Step 2 ID 0x0000
```

当时程序在发送 SCCB前发现 RESET仍为低，因此停止初始化，故障排查暂时停留在这里。

### 12.5 原理图版本纠正

排查过程中曾收到一张带 `EX_RST + Q2` 的旧版摄像头板原理图，并短暂据此将 PC4改为 3.3 V推挽输出。随后用户确认该图不是当前硬件。

根据最终正确的 2024-05版原理图，已完成以下安全回退：

- PC4恢复为开漏，无内部上拉。
- PF14/PF15恢复为开漏，无内部上拉。
- 软件 SCCB的高电平改为释放引脚，由板载 2.8 V电阻产生。
- `.ioc`、`main.c`、`stm32h7xx_hal_msp.c` 和 `camera_service.c` 已同步。

该阶段仍假定 MCU应控制 RESET，后来通过实际硬件断线验证推翻了这一假定。

### 12.6 最终根因与修复

最终发现开发板的 `OV_RESET` 驱动与摄像头载板上的 R3/C13复位网络存在电气冲突。采取的修复是：

1. 硬件断开主板与摄像头载板 `OV_RESET` 的连接。
2. 由摄像头载板的 2.8 V RC网络独立完成上电复位。
3. CubeMX将 PC4配置为 `BUZZER` 推挽输出。
4. `Camera_Service_BootHold()`、初始化、休眠和诊断路径全部移除 PC4访问。
5. LCD诊断将虚假的 `R0/R1`改为 `RST EXT`。

修复后 ID读取、寄存器初始化、DCMI快照、JPEG硬件解码和 LCD显示全部通过。用户已完成 100次连续拍照、20次页面进出以及蜂鸣器并行验证，阶段 1正式关闭。

## 13. 历史 Bug 的可能原因排序（仅供旧硬件排查）

本节保留当时尚未发现 RESET连接冲突时的推断过程，不代表当前硬件状态。当前版本不再通过 MCU检查 RESET。

### 高概率

1. 摄像头板 J2 的 3.3 V供电没有通过 FPC接入。
2. FPC方向反向、未插到底或锁扣未压紧。
3. U2/TPS76328未产生 `VCC_2V8`，导致 `DOVDD_2V8`、R3、R10、R11都没有高电平来源。
4. 主板 PC4没有映射到摄像头板 J2的 `OV_RESET`。

### 中等概率

1. R2（0 Ohm）或 R3（10 kOhm）焊接不良。
2. RESET线路短路到地。
3. 摄像头模组或驱动板损坏。

### 当时不是首要原因

- SCCB地址写错：程序尚未发送地址，`N0`已经证明这一点。
- 24 MHz XCLK缺失：缺少 XCLK可能导致后续 `R1/N1`，但通常不能解释 STM32直接读到 RESET低电平。
- DCMI极性错误：尚未进入 DCMI采集。
- JPEG缓冲、Cache或 LVGL错误：尚未取得任何图像数据。

## 14. 无测量设备时的排查方法

以下 `R0/R1`组合表只用于分析尚未断开 RESET冲突的旧硬件。当前硬件应先确认主板 `OV_RESET`确实保持断开，正常固件只显示 `RST EXT`。

### 14.1 第一步：确保测试的是最新安全固件

1. 重新编译并烧录最新 Debug版本。
2. 完全断电至少 10秒，避免摄像头板电容保持异常状态。
3. 断电状态下重新插拔 FPC并锁紧。
4. 再次上电进入 Camera Test。
5. 完整记录两行诊断，尤其是 `Lxy/N/P/R/Step`。

不要只按 MCU Reset。摄像头板可能仍然保持供电，因此必须做一次完整断电重启。

### 14.2 第二步：根据最新版 `Lxy/R`组合判断

| 最新结果 | 无仪器条件下的推断 | 下一步 |
|---|---|---|
| `L00 P0R0` | 2.8 V总线与 RESET上拉都未建立，强烈怀疑摄像头板没有 3.3 V或 U2无输出 | 检查 FPC方向、3.3 V与 GND映射，换排线或换摄像头板 |
| `L11 P0R0` | SCCB两条 2.8 V上拉可能存在，但 RESET仍低 | 重点检查 PC4到 J2 pin16映射、R3、RESET短路和排线接触 |
| `L01` 或 `L10` | 单根 SCCB线被拉低或映射错误 | 重新检查 PF14/PF15顺序和排线方向 |
| `L11 P0R1 N1 Step3` | 电源和 RESET基本通过，传感器没有应答地址 | 检查板载 24 MHz、传感器模组安装、SDA/SCL映射；再考虑地址扫描 |
| `L11 P0R1 N2/N3` | 传感器已 ACK设备地址，但寄存器地址阶段失败 | 降低 SCCB速度、增加延时、检查时序实现 |
| `L11 P0R1 N5` | 写地址阶段成功，读地址阶段失败 | 检查 STOP到下一次 START间隔及读地址0x79 |
| `ID 0x5640 Step4` 后 `-6` | SCCB和 ID已通过，初始化表中某个寄存器失败 | 增加“最后失败寄存器”诊断 |
| `Step5` 后 `-7/-8` | 已进入 DCMI采集 | 检查 PCLK/VSYNC/HREF和 DMA |
| `Step5` 后 `-9/-10` | 已接收数据但不是完整 JPEG | 检查 JPEG模式、帧长度、PCLK极性和缓冲容量 |

`L11` 对 2.8 V存在性的判断只是软件推断，不等价于真实电压测量。如果主板上还有未知上拉，结论会受到影响。

### 14.3 第三步：纯目视检查

在完全断电后检查：

- FPC是否有折痕、破损、氧化或插偏一位。
- 两端连接器锁扣是否完整。
- 摄像头板 U2、R2、R3、R10、R11、Y1附近是否缺件、偏焊或连锡。
- OV5640模组小排线是否插到驱动板 J1底部。
- 主板三路 3.3 V和多个 GND是否确实接到相应 FPC焊盘。

摄像头板上的 D1/D2是补光灯，不是电源指示灯。它们不亮不能直接证明摄像头板没有电。

### 14.4 第四步：交叉验证

如果手上有另一块 ESP32或参考例程对应开发板，可以使用已知可工作的摄像头程序测试同一个摄像头板和排线。

注意：提供的 Arduino工程选择了 ESP32-S3相机模型，程序会主动输出 20 MHz XCLK，并将 PWDN/RESET配置为 `-1`。它可以帮助确认传感器是否工作，但不能直接证明 STM32当前 FPC映射和板载 24 MHz电路正确。

优先采用以下交叉替换顺序：

1. 保持主板不变，更换 FPC。
2. 保持 FPC不变，更换摄像头驱动板。
3. 将当前摄像头板接到已知可工作的控制器。

每次只替换一个部件，否则无法知道故障来自哪里。

### 14.5 可以继续增加的软件诊断

如果最新结果已经变为 `R1`，后续可依次加入：

1. 扫描已知摄像头 SCCB 7位地址，并显示发现的地址。
2. 记录每次读写失败的寄存器地址和数据值。
3. 在进入 SCCB前统计 PA6/PCLK的边沿，判断 OV5640是否输出像素时钟。
4. 统计 PG9/VSYNC和 PA4/HREF的电平变化。
5. 在 DCMI阶段显示 DMA NDTR、FRAME/OVR/ERR标志。

当前为 `R0` 时不应继续扫描 SCCB，因为传感器仍在复位，扫描结果必然无效。

## 15. 建议的最小工具

虽然本文给出了无仪器排查方法，但摄像头属于电源、时钟、总线和高速并行数据共同作用的外设。若后续仍停在硬件阶段，建议至少准备：

- 一支基础数字万用表：确认 3.3 V、2.8 V、RESET和 PWDN。
- 一个低成本 USB逻辑分析仪：查看低速 SCCB、RESET、PWDN、VSYNC和 HREF。
- 若要检查 24 MHz XCLK和 PCLK质量，则需要带宽足够的示波器或逻辑分析仪。

仅确认静态电源和 RESET，一支普通万用表已经能显著缩短排查时间。

## 16. 测试记录模板

每次修改后建议在本文末尾按以下格式追加记录：

```text
日期：
固件版本/提交：
是否完整断电：是/否
摄像头板版本：
FPC方向及是否重新插拔：

LCD第1行：
LCD第2行：
底部状态栏：

本次只修改了什么：
结果相对上次如何变化：
下一步唯一动作：
```

当前基线记录：

```text
日期：2026-08-17
硬件：主板OV_RESET与摄像头载板OV_RESET已断开，PC4专用于BUZZER
固件：Camera Test阶段1收尾版本
结果：Camera frame ready；重复OK拍照正常
压力测试：连续拍照100次；页面进出20次；蜂鸣器不受影响
结论：SCCB、OV5640初始化、DCMI、JPEG/MDMA和LVGL单帧显示链路通过
后续结果：5 FPS串行连续预览已稳定通过；10 FPS串行节拍和48 MHz PCLK实验均未通过稳定性验证，随后在24 MHz基线上改用双JPEG槽流水线
```

## 17. CubeMX重新生成后的检查清单

重新生成代码后必须确认：

- [ ] PC4仍为 `BUZZER`推挽输出，摄像头代码没有读写 PC4。
- [ ] 主板与摄像头载板的 `OV_RESET`保持物理断开。
- [ ] PF13上电默认高，标签仍为 `OV_PWDN`。
- [ ] PF14/PF15仍为 I2C4开漏且 `GPIO_NOPULL`。
- [ ] DCMI仍为 8 bit、JPEG Enable、PCLK Rising、VSYNC High、HSYNC Low。
- [ ] DMA1 Stream1仍连接 DCMI，请求方向为外设到内存。
- [ ] DCMI中断优先级仍为 6。
- [ ] JPEG和两个 MDMA通道仍存在。
- [ ] `Camera_Service_BootHold()` 仍在所有相关 GPIO初始化后调用。
- [ ] `camera_service.c`、`ov5640.c`、`ov5640_reg.c` 仍被 CMake编译。
- [ ] 链接脚本仍包含 `.media_pool` 和 `.ram_d2`。

## 18. 后续开发顺序

当前完成情况如下：

1. 载板 RESET独立工作、PWDN控制正确：已完成。
2. ID=`0x5640`、SCCB通信和寄存器配置：已完成。
3. DCMI FRAME、完整 `FF D8 ... FF D9` JPEG：已完成。
4. JPEG/MDMA硬件解码和 LVGL单帧显示：已完成。
5. 100次拍照、20次页面进出、蜂鸣器隔离测试：已完成。
6. 5 FPS串行连续预览、OK暂停/继续、性能统计：已通过；48 MHz PCLK实验因花屏和丢帧失败，已回到24 MHz稳定基线。
7. 双 64 KiB JPEG 槽、单帧超前的采集/解码流水线：代码与编译验证已完成；首次板测发现传感器源帧率约5.6 FPS，已增加参考例程的15 FPS内部时序，待再次记录 FPS、C、Drop、JPEG大小和花屏情况。
8. 15 FPS传感器时序实机约 10.7 FPS、采集约 90 ms：已通过初测。
9. 拍照保存、相册目录和静态 JPEG查看器：代码与编译验证已完成，待实机回归。

## 19. 拍照保存与相册实现

### 19.1 页面操作

- Camera Test 中按 Up：保存下一张完整帧。
- Camera Test 中按 Right：停止摄像头并进入 `/DCIM/CAMERA`。
- SD 文件页选择 `.JPG/.JPEG`：进入 Photo Viewer。
- Photo Viewer 中按 Left 或 KEY3：返回相册，并恢复到刚才选中的照片。
- Photo Viewer 中按 KEY2：执行全局退出并回到主菜单第 1 项。

### 19.2 文件组织和掉电保护

照片使用 `/DCIM/CAMERA/IMG00001.JPG` 到 `IMG99999.JPG` 的命名方式。保存时先写 `/DCIM/CAMERA/CAPTURE.TMP`，分 4 KiB 写入并执行 `f_sync()`；关闭成功后才通过 `f_rename()`变为最终文件名。写卡失败时会删除临时文件，既有照片不会被覆盖。

首次实机保存曾返回 `CAMERA_ALBUM_ERR_WRITE(-6)`。原因范围位于 FatFs数据写入到 SDMMC底层之间，而不是目录创建或文件打开。当前版本不再把 D2 SRAM中的 DCMI JPEG槽直接交给 FatFs：每个4 KiB块先复制到32字节对齐的 AXI SRAM暂存区，再执行写入；同时 `SD_write()`增加与读取路径一致的“DMA失败后复位SDMMC并以轮询模式重试”。错误提示会保留 FatFs码、失败文件偏移以及“实际/请求”字节数，便于继续区分卡片、DMA和空间不足问题。

随后实机打开 JPG又偶发 `CAMERA_ALBUM_ERR_READ(-9), fs=1`。这同样属于底层磁盘I/O错误，而不是 JPEG格式错误。读取路径现在也以4 KiB为单位先读入同一个 AXI SRAM对齐暂存区，成功后再由CPU复制到 D2 JPEG槽，避免 SDMMC直接访问摄像头压缩缓冲。读取错误提示同样包含失败偏移和“实际/请求”字节数。

OV5640输出的 JPEG可能省略标准 DHT。保存前调用 `MJPEG_Player_NormalizeJpeg()`补齐 Huffman表并完成硬件 JPEG要求的字节对齐，因此生成的 JPG既能由本机硬件 JPEG解码，也便于电脑或手机软件读取。

### 19.3 内存复用

相册没有新增整帧静态缓冲。进入 Photo Viewer前先让 Camera Service进入 OFF状态，再借用一个空闲的 64 KiB JPEG槽读取文件；解码输出继续使用共享媒体 RGB565池。离开查看器时先等待 LCD异步刷新停止，再使 LVGL图片缓存失效并释放媒体池，避免显示 DMA仍在读取时覆盖缓冲。

### 19.4 建议实机回归

1. 连续保存20张，确认编号递增、预览能恢复且 `Drop`没有异常增长。
2. 按 Right进入相册，逐张打开并检查方向、颜色和完整性。
3. 从 Photo Viewer按 Left和 KEY3返回，确认焦点仍在原照片。
4. 将 SD卡放入电脑，确认 JPG可由通用图片软件打开。
5. 在写保护、拔卡或空间不足时拍照，确认显示保存错误但系统不死机，原照片不损坏。

## 20. 已知技术限制与后续优化点

- 当前连续预览使用已通过板测的 24 MHz PCLK。双 JPEG 槽使“下一帧采集”与“上一帧 30 ms帧尾保护、解码及显示”重叠；48 MHz 测试虽将采集从约 103 ms缩短到 81 ms，但 `Drop`、自动恢复和花屏明显增加，证明当前载板/FPC/DCMI链路的信号完整性不足以稳定使用该档位。
- 当前使用单个 RGB565显示缓冲，通过等待 LCD异步刷新完成来安全复用；若提高到更高帧率，应再评估双 RGB缓冲和内存占用。
- 软件 SCCB在初始化期间阻塞 CPU，但不影响 DCMI采集；后续若需要频繁动态调参，可考虑验证后切回硬件 I2C。
- 每个 JPEG槽只有 64 KiB；当前 QVGA帧必须持续低于该容量。提升分辨率、降低压缩率或遇到 `CAMERA_RESULT_BUFFER_FULL(-9)` 前必须重新规划槽容量和 SRAM布局。
- 当前只记录 NACK阶段，没有记录“最后失败寄存器”。Step 4失败时应补充该字段。
- 当前没有软件检测板载 24 MHz XCLK、PCLK、VSYNC或 HREF活动。
- `L/P` 都是数字电平，不是模拟电压，不能替代万用表；RESET当前不再由 MCU测量。
- 当前软件 SCCB将失败映射为 `HAL_I2C_ERROR_AF`，因此 `E04` 可能来自软件诊断，不一定来自 HAL硬件 I2C外设。

## 21. 相关资料路径

- 最终硬件依据：用户最后提供的 2024-05版 OV 系列摄像头驱动板原理图截图。
- Arduino参考工程：`D:/Data/Downloads/zhao_xiang_ji8.21/zhao_xiang_ji8.2`。
- F407参考工程：`D:/BaiduNetdiskDownload/资料/OVx640摄像头/OVx640摄像头/OV5640/OV5640摄像头--资料--1712/2、OV5640摄像头驱动程序/F407VE_OV5640摄像头`。
- ST驱动许可证说明：`docs/ST_OV5640_LICENSE.md`。

参考优先级必须是：

```text
当前摄像头板正确原理图
    > 当前主板接口原理图
    > 当前PCB实际连线
    > Arduino/F407参考程序
    > 通用ST组件驱动默认假设
```

参考程序中的 GPIO输出类型、XCLK来源、RESET/PWDN是否存在，都必须根据当前硬件重新判断，不能直接复制。
