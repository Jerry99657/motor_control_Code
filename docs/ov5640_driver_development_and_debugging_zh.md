# STM32H743 + OV5640 驱动开发与故障排查记录

> 文档日期：2026-08-09  
> 适用工程：`STM32H743ZIT6_KIT`  
> 当前目标：通过 Camera Test 页面完成 OV5640 单帧 `320 x 240 JPEG` 采集、硬件 JPEG 解码和 LCD 显示  
> 当前阻塞点：OV5640 的硬件复位线释放后仍被读为低电平，初始化在 SCCB 通信前停止

## 1. 文档目的

本文记录 OV5640 功能从 CubeMX 配置、传感器寄存器驱动、DCMI/DMA 采集、JPEG 解码到 LVGL 显示的完整代码结构，并保留目前故障的演进过程和排查结论。

这份文档重点解决三个问题：

1. 后续维护时能够快速找到每一层代码的位置和职责。
2. CubeMX 重新生成代码后，能够判断哪些配置必须保留。
3. 在没有万用表、示波器和逻辑分析仪时，能够仅根据 LCD 诊断信息逐步缩小故障范围。

## 2. 当前结论摘要

最近一次实际显示为：

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

因此，当前问题不在 DCMI、DMA、JPEG 解码、LVGL 或 OV5640 初始化寄存器表。程序甚至还没有开始读取传感器 ID。当前必须先解决 `OV_RESET` 无法变高的问题。

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
| `OV_RESET` / PC4 | 开漏输出、无内部上拉 | 摄像头板 R3 上拉到 2.8 V | 避免向 2.8 V 传感器引脚推挽输出 3.3 V |
| `OV_SCL` / PF14 | 开漏、无内部上拉 | 摄像头板 R11 上拉到 2.8 V | SCCB 总线由板载 5.1 kOhm 上拉 |
| `OV_SDA` / PF15 | 开漏、无内部上拉 | 摄像头板 R10 上拉到 2.8 V | ACK 和读取阶段必须允许传感器驱动 SDA |
| `OV_PWDN` / PF13 | 推挽输出 | STM32 3.3 V逻辑 | 高电平掉电，低电平工作 |

严禁再次采用以下配置：

- PC4 推挽输出高电平。
- PF14/PF15 推挽输出高电平。
- PF14/PF15 启用 STM32 的 3.3 V内部上拉。
- 将旧版带 Q2 的 `EX_RST` 原理图逻辑套用到当前摄像头板。

### 3.3 主板到摄像头板的信号映射

以下映射来自主板摄像头接口图、背部 FPC 插座图和当前 CubeMX 引脚配置。

| 功能 | STM32 引脚 | DCMI/控制功能 | 摄像头板信号 |
|---|---|---|---|
| SCCB 时钟 | PF14 | 软件 SCCB SCL / I2C4_SCL | `OV_SCL` |
| SCCB 数据 | PF15 | 软件 SCCB SDA / I2C4_SDA | `OV_SDA` |
| 硬件复位 | PC4 | 开漏 GPIO | `OV_RESET` |
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

- PC4：`GPIO_Output`，`GPIO_MODE_OUTPUT_OD`，`GPIO_NOPULL`，标签 `OV_RESET`。
- PF13：`GPIO_Output`，默认输出高电平，标签 `OV_PWDN`。

上电默认状态是：

```text
OV_PWDN = 1  摄像头掉电
OV_RESET = 0 摄像头保持复位
```

`Camera_Service_BootHold()` 会在所有 CubeMX 外设初始化后再次设置这两个安全状态。

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
- 摄像头 JPEG 压缩缓冲位于 `.ram_d2`，容量 128 KiB，32 字节对齐。
- RGB565 输出使用共享媒体内存池 `.media_pool`，位于 AXI SRAM。

这种布局保证 DCMI DMA写入 JPEG、JPEG/MDMA读取压缩数据和生成 RGB565 时不会因为 D-Cache 一致性而破坏数据。

## 5. 代码模块结构

| 文件 | 主要职责 |
|---|---|
| `Drivers/User/Inc/camera_service.h` | 摄像头状态、错误码、诊断结构和公共 API |
| `Drivers/User/Src/camera_service.c` | 电源/复位、SCCB、ID检测、OV5640配置、DCMI/DMA快照、JPEG帧边界查找 |
| `Drivers/User/Inc/ov5640.h` | ST OV5640组件驱动接口、分辨率和像素格式定义 |
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

所有自定义摄像头业务代码位于 `Drivers/User`，不会被 CubeMX 直接覆盖。PC4、PF13、PF14、PF15、DCMI、DMA等生成代码由 `.ioc` 负责保持一致。

## 6. Camera Service 状态机

### 6.1 对外状态

| 状态 | 数值 | 含义 |
|---|---:|---|
| `CAMERA_STATE_OFF` | 0 | RESET有效、PWDN有效，摄像头关闭 |
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
| 2 | `CAMERA_INIT_STAGE_I2C_PROBE` | 检查 RESET电平、注册 SCCB总线 |
| 3 | `CAMERA_INIT_STAGE_READ_ID` | 读取 `0x300A/0x300B` |
| 4 | `CAMERA_INIT_STAGE_SENSOR_CONFIG` | 写入 ST OV5640初始化表 |
| 5 | `CAMERA_INIT_STAGE_CAPTURE` | DCMI/DMA采集阶段 |

### 6.3 初始化完整流程

```text
进入 Camera Test
    |
    v
Camera_Service_Sleep / BootHold
PWDN=1, RESET=0
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
PC4开漏写高，释放 RESET，等待20 ms
    |
    +-- R0 --> Step 2立即失败，不发送 SCCB
    |
    v R1
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

1. 从共享媒体内存池申请 `320 x 240 x 2` 字节 RGB565缓冲。
2. 调用 `MJPEG_Player_DecodeMemoryToRgb565()` 复用硬件 JPEG/MDMA解码流水线。
3. 将 320 x 240 RGB565原地缩小为 200 x 150。
4. 构造 `lv_img_dsc_t`，在 LVGL预览卡片中显示。
5. 显示 JPEG大小、采集耗时和解码耗时。

离开页面会：

- 停止 DCMI。
- 令 PWDN=1、RESET=0。
- 释放共享媒体内存。
- 使 LVGL图片描述符失效。

## 9. Camera Test 页面操作

| 操作 | 行为 |
|---|---|
| 进入页面 | 先关闭摄像头并显示初始化占位信息 |
| OK | 初始化摄像头并自动拍摄；显示图像后再次按 OK可重新拍摄 |
| Left | 停止摄像头、释放内存并返回主页面 |
| KEY2 / ESC | 停止摄像头并返回 |
| KEY3 | 由全局媒体退出逻辑停止当前功能并返回 |

页面内部阶段包括 OFF、INIT_PENDING、READY、CAPTURING、DECODE_PENDING、SHOWING和 ERROR。初始化和 JPEG解码故意分散到不同主循环周期，避免一次 LVGL处理占用过长时间。

## 10. LCD 诊断字段

错误显示格式：

```text
SCCB Exx Lxy Nz PpRr
Step s ID 0xiiii
```

### 10.1 `E`：I2C/SCCB错误

| 值 | 含义 |
|---|---|
| `E00` | 没有记录到总线错误 |
| `E04` | `HAL_I2C_ERROR_AF`；硬件 I2C中表示 NACK，软件 SCCB中也用于表示 ACK失败或复位检查失败 |
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

### 10.4 `P` 与 `R`

| 字段 | 正常工作值 | 含义 |
|---|---:|---|
| `P` | 0 | PF13/PWDN为低，摄像头工作 |
| `R` | 1 | PC4/RESET已被板载 R3拉到逻辑高 |

注意：`R` 只是 STM32数字输入阈值判断，不是电压测量。它不能区分 1.9 V、2.8 V或其他高于 VIH的电压。

## 11. Camera Result错误码

| 返回值 | 枚举 | 说明 |
|---:|---|---|
| 0 | `CAMERA_RESULT_OK` | 成功 |
| -1 | `CAMERA_RESULT_INVALID_ARGUMENT` | 空指针或参数错误 |
| -2 | `CAMERA_RESULT_BUSY` | 摄像头或共享内存正在使用 |
| -3 | `CAMERA_RESULT_NOT_READY` | 摄像头未完成初始化 |
| -4 | `CAMERA_RESULT_I2C` | RESET、I2C或 SCCB失败 |
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

程序在发送 SCCB前发现 RESET仍为低，因此停止初始化。当前故障就停留在这里。

### 12.5 原理图版本纠正

排查过程中曾收到一张带 `EX_RST + Q2` 的旧版摄像头板原理图，并短暂据此将 PC4改为 3.3 V推挽输出。随后用户确认该图不是当前硬件。

根据最终正确的 2024-05版原理图，已完成以下安全回退：

- PC4恢复为开漏，无内部上拉。
- PF14/PF15恢复为开漏，无内部上拉。
- 软件 SCCB的高电平改为释放引脚，由板载 2.8 V电阻产生。
- `.ioc`、`main.c`、`stm32h7xx_hal_msp.c` 和 `camera_service.c` 已同步。

最后一次 `P0R0/L11` 是在上述最终安全配置烧录前得到的，必须用最新固件重新测试一次。最新固件关闭了 PF14/PF15内部上拉，因此新的 `Lxy` 更有诊断价值。

## 13. 当前 Bug 的可能原因排序

### 高概率

1. 摄像头板 J2 的 3.3 V供电没有通过 FPC接入。
2. FPC方向反向、未插到底或锁扣未压紧。
3. U2/TPS76328未产生 `VCC_2V8`，导致 `DOVDD_2V8`、R3、R10、R11都没有高电平来源。
4. 主板 PC4没有映射到摄像头板 J2的 `OV_RESET`。

### 中等概率

1. R2（0 Ohm）或 R3（10 kOhm）焊接不良。
2. RESET线路短路到地。
3. 摄像头模组或驱动板损坏。

### 当前不是首要原因

- SCCB地址写错：程序尚未发送地址，`N0`已经证明这一点。
- 24 MHz XCLK缺失：缺少 XCLK可能导致后续 `R1/N1`，但通常不能解释 STM32直接读到 RESET低电平。
- DCMI极性错误：尚未进入 DCMI采集。
- JPEG缓冲、Cache或 LVGL错误：尚未取得任何图像数据。

## 14. 无测量设备时的排查方法

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
日期：2026-08-09
固件：最终开漏安全配置烧录前的诊断版本
LCD第1行：SCCB E04 L11 N0 P0R0
LCD第2行：Step 2 ID 0x0000
结论：PWDN退出成功；RESET未释放；SCCB未发送
下一步：烧录最终开漏、无内部上拉版本，完整断电后重新记录Lxy/P/R
```

## 17. CubeMX重新生成后的检查清单

重新生成代码后必须确认：

- [ ] PC4仍为 `GPIO_MODE_OUTPUT_OD + GPIO_NOPULL`。
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

建议严格按以下顺序继续，前一步没有通过时不要跳到下一步：

1. `P0R1`：确认掉电和复位控制正确。
2. `N0/E00` 且 ID=`0x5640`：确认 SCCB通信正确。
3. Step 4完成：确认 OV5640寄存器配置正确。
4. 捕获到 PCLK/VSYNC/HREF活动：确认并行输出存在。
5. DCMI FRAME中断出现：确认采集同步正确。
6. JPEG缓冲包含 `FF D8 ... FF D9`：确认完整图像帧。
7. 硬件 JPEG解码成功：确认 MDMA和共享内存正确。
8. LVGL显示单帧正确：完成 Camera Test第一阶段。
9. 再考虑连续预览、双缓冲、帧率控制和通过 ESP32/Wi-Fi传输。

## 19. 已知技术限制与后续优化点

- 当前 Camera Test以单帧快照为目标，每次完整采集后都会停止并让摄像头回到安全状态，不适合直接作为连续预览架构。
- 软件 SCCB在初始化期间阻塞 CPU，但不影响 DCMI采集；后续若需要频繁动态调参，可考虑验证后切回硬件 I2C。
- 128 KiB JPEG缓冲适用于当前 QVGA测试，提升分辨率或降低压缩率前必须重新评估容量。
- 当前只记录 NACK阶段，没有记录“最后失败寄存器”。Step 4失败时应补充该字段。
- 当前没有软件检测板载 24 MHz XCLK、PCLK、VSYNC或 HREF活动。
- `R/L/P` 都是数字电平，不是模拟电压，不能替代万用表。
- 当前软件 SCCB将失败映射为 `HAL_I2C_ERROR_AF`，因此 `E04` 可能来自软件诊断，不一定来自 HAL硬件 I2C外设。

## 20. 相关资料路径

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
