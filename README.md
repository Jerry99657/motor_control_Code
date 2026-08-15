# STM32H743ZIT6_KIT

基于 **STM32H743ZIT6** 的麦克纳姆轮机器人控制与 LVGL 人机交互工程。工程使用 STM32CubeMX 生成 HAL 外设初始化，采用 CMake/Ninja 构建，集成四路直流电机闭环控制、麦轮运动学、MPU6500 姿态采集、USB/UART5 通信、SD 卡媒体播放、WS2812 灯带和 LVGL 图形界面。

## 1. 项目功能总览

- 四路直流电机速度闭环和位置闭环，支持 `-100~100%` 速度设定。
- 麦克纳姆底盘 X/Y 平移、Z 轴旋转以及“速度 + 距离”混合轨迹控制。
- MPU6500 六轴数据采集与姿态融合，提供平面航向角和航向角速度。
- 小陀螺模式：车体连续自转，同时通过航向补偿保持世界坐标方向运动。
- LVGL 菜单、固定页面骨架、页面过渡、焦点反馈、数值惯性和 UI 性能诊断。
- SD 卡目录浏览，播放 `.bin` 动画、`.avi` MJPEG 视频和 `.gif` 动画。
- NES 模拟器：支持 iNES Mapper 0/1/2/3、QSPI ROM 缓存、MMC1 和电池 `.sav` 存档，无音频输出。
- 媒体播放暂停/继续、前后跳帧、停止和返回目录。
- WS2812 RGB 控制及上电状态提示。
- USB CDC、UART5 文本/二进制控制协议和 ESP32-C3 虚拟摇杆。
- UART4 FOC 主从桥接：把上游统一二进制帧转换为 FOC 从板的 ASCII 指令，并接收 100 Hz JustFloat 遥测。
- 独立 FOC Control 页面：C2804 编码器闭环速度/位置控制、目标值与实时值显示。
- VOFA+ 50 Hz、14 通道 JustFloat 实时上传四轮电机数据和 FOC 速度、位置反馈。

## 2. 代码结构

```text
Core/Src/main.c                 系统初始化、主循环、定时器回调、USB/UART 分发
Drivers/User/Src/lvgl_app.c    LVGL 页面、按键逻辑、命令解析
Drivers/User/Src/mecanum.c     麦轮逆解、距离轨迹、小陀螺航向补偿
Drivers/User/Src/dc_motor_ol.c 电机测速、速度环、位置环、PWM 输出
Drivers/User/Src/imu.c         MPU6500 采样和姿态融合
Drivers/User/Src/imu_service.c 平面航向快照、航向角速度和安装方向处理
Drivers/User/Src/foc_link.c    UART4 FOC 指令队列、停止优先级和遥测解析
Drivers/User/Src/media_control.c 媒体按键、暂停、跳帧和返回状态机
Drivers/User/Src/mjpeg_player.c AVI/MJPEG 解码与播放调度
Drivers/User/Src/nes_rom_cache.c NES ROM 的 SD→QSPI 缓存、快速命中和 CRC 校验
Drivers/User/Src/nes_cpu.c     Ricoh 2A03/6502 指令、NMI/IRQ 和故障现场
Drivers/User/Src/nes_runtime.c Mapper、PPU、APU 兼容时序、LCD 输出和 `.sav`
Drivers/User/Src/sd_diskio.c   SD 读写、DMA、Cache 维护
Drivers/User/Src/ui_*.c        Theme、页面骨架、过渡、反馈、动画和性能诊断
Drivers/User/Src/ws2812*.c     WS2812 定时器 DMA 驱动
```

### 定时任务

`HAL_TIM_PeriodElapsedCallback()` 中的任务分工如下：

| 定时器 | 周期 | 任务 |
| --- | ---: | --- |
| TIM6 | 1 ms | `lv_tick_inc(1)`，LVGL 时基 |
| TIM7 | 由配置决定 | MJPEG 播放调度 |
| TIM13 | 10 ms | 安全检查、`Mecanum_Tick10ms()`、`DCMotor_OL_Tick10ms()` |
| TIM16 | 由配置决定 | ADC 采样和电源数据更新 |

主循环执行通信处理、IMU 服务、ADC 服务、LVGL 任务和 VOFA 上传；10 ms 电机/底盘控制放在定时器回调中，避免被 UI 或 SD 卡操作长期阻塞。

## 3. 按键和页面通用操作

LVGL 当前使用以下逻辑按键：

| 按键 | 作用 |
| --- | --- |
| Up/Down | 上移/下移焦点；编辑数值时改变数值 |
| Left/Right | 进入/退出子页面；编辑数值时减小/增大；媒体页后退/快进 |
| OK | 进入编辑、确认设置；媒体页暂停/播放 |
| KEY2 | 全局停止/退出保护键；媒体播放中停止播放 |
| KEY3 | 媒体播放中停止当前媒体并返回 SD 文件目录；普通 LVGL 页面作为返回键 |

页面采用统一的“标题栏 + 内容区 + 状态栏”骨架。通常使用 Right/OK 进入，Left/KEY3 返回，KEY2 负责停止当前运动或媒体。进入 Command Control、Mecanum Control 或媒体播放前后，工程都会清理上一页面遗留的控制状态。

## 4. 主页面和各功能页面

主菜单共 10 项：

1. `Motor Control`
2. `Command Control`
3. `SD Card Files`
4. `Mecanum Control`
5. `MPU6500 Data`
6. `WS2812 Control`
7. `FOC Control`
8. `UI Diagnostics`
9. `Camera Test`
10. `Display Settings`

### 4.1 Motor Control

进入后先显示二级菜单：`Motor Speed`、`Servo Angle`、`Back`。

- `Motor Speed`：选择 M1~M4，按 OK 进入编辑，Left/Right 每次改变 10%，范围 `-100~100%`；再次 OK 发送设定，KEY3/ESC 取消编辑。
- `Servo Angle`：选择舵机通道，角度范围 `0~270°`，步进 10°，操作流程与电机速度相同。
- 页面同时刷新 Set（设定值）和 Act（实际值），便于观察闭环响应。
- KEY2 可立即停止所有电机并退出当前编辑。

### 4.2 Command Control

该页面是 USB/UART5 远程控制的安全门。**只有当前页面为 Command Control 时，USB/UART5 的电机、麦轮、舵机、小陀螺和 UART4 FOC 控制指令才会执行**；进入页面会先停止并清空上一轮指令，离开页面也会停止本机电机、小陀螺和 FOC 从板。

页面显示：

- M1~M4：`Set` 设定速度和 `Act` 实际速度。
- FOC：UART4 链路 UP/DOWN、从板实际速度和 Iq。
- `Gyro: ON/OFF`、旋转方向、速度百分比和角速度（deg/s）。
- 虚拟摇杆的 LX/LY/RX/RY 原始值。

OK 可以在页面内直接编辑电机或舵机行；Left/Right 修改数值，OK 确认。KEY2 为全局停止，KEY3/ESC 返回主菜单。

### 4.3 SD Card Files

进入页面后扫描 SD 卡当前目录：

- Down/Up 选择文件或文件夹。
- Right/OK 进入文件夹或开始播放媒体。
- Left/KEY3 返回上一级目录；在根目录再次返回主菜单。
- KEY2 停止正在播放的媒体。

当前识别的媒体包括 `.bin`、`.avi`、`.gif` 和 `.nes`。AVI 文件既可按扩展名识别，也可通过 RIFF/AVI 文件头识别，播放返回后会重新扫描当前目录。

FatFs 保持 CubeMX 的 CP936 长文件名配置：目录扫描得到的原始 CP936/GBK 名称用于 `f_open()`、`f_opendir()` 和媒体接口；文件列表、目录路径、播放标题及状态文本在显示前通过 `ff_convert()` 转为 LVGL 所需的 UTF-8。转换使用临时缓冲，不会把 UTF-8 名称误传给 FatFs，也不会为 48 个目录项长期保存第二份名称。完整 GB2312 字库采用 16 px、2 bpp，Unicode 映射和字形度量保留在 MCU Flash，约 396 KB 位图存放于 W25Q64 最后 512 KB 分区，并通过 QSPI Memory-Mapped 窗口和 8 项 RAM 字形缓存读取；NES 缓存擦写期间自动切换为安全的间接读取。未覆盖的 CP936 扩展字明确显示为 `?`，不再无提示消失。首次使用请把 [`assets/GB2312.FNT`](assets/GB2312.FNT) 复制到 SD 卡根目录并重启，固件会执行全量 CRC32 后一次性安装。详细结构见 [`docs/lvgl_chinese_font_w25q64_zh.md`](docs/lvgl_chinese_font_w25q64_zh.md)。

选择 `.nes` 后先检查 W25Q64 缓存；同一 ROM 会显示 `Cache hit 100%`，否则执行 SD→QSPI 复制和 CRC32 校验。缓存完成后按 OK 开始游戏。NES 支持 Mapper 0/1/2/3；KEY3 正常退出并为带电池标志的游戏写入同名 `.sav`，KEY2 保持紧急退出且不等待存档。游戏运行时既可使用开发板实体按键，也可将 ESP32 手机网页切换到 NES 手柄，两组输入能够同时组合。

### 4.4 Mecanum Control

页面有 7 行：

1. `X Dist(cm)`：X 方向距离
2. `Y Dist(cm)`：Y 方向距离
3. `Z Rot(deg)`：原地旋转角度
4. `X Speed`：X 方向速度
5. `Y Speed`：Y 方向速度
6. `Z Speed`：Z 方向角速度
7. `EXECUTE` / `STOP ACTIVE`

距离按 10 cm 修改，速度范围为 `-100~100`，按 OK 进入编辑。执行后，带距离的轴使用 10 ms 轨迹控制逐步推进；四轮到位后自动停止，不会留下某个轮子继续转动。执行期间第 7 行变为 `STOP ACTIVE`，再次 OK 或 KEY2 可停止。

速度换算关系：X/Y 的界面值乘以 10 转换为 mm/s，Z 角速度保持 deg/s；距离 cm 转换为 mm。麦轮逆运动学使用四轮速度组合：

```text
V1 = Vx + Vy + W
V2 = -Vx + Vy + W
V3 = -Vx - Vy + W
V4 = Vx - Vy + W
```

输出会按最大轮速统一缩放，保持运动方向比例。

### 4.5 MPU6500 Data

显示加速度、角速度、姿态角、平面航向角和航向角速度等实时数据。该页用于确认传感器方向、零偏和数据刷新是否正常；没有磁力计时，航向角只能作为相对角度使用，长期会产生漂移。

### 4.6 WS2812 Control

通过 RGB 三个滑块调节灯带颜色，并实时发送到 WS2812。上电初始化时短暂显示绿色提示，初始化完成后应自动熄灭；若持续亮起，优先检查初始化流程和 DMA 更新完成状态。

### 4.7 FOC Control

该页面只开放 FOC 板的 C2804 编码器闭环控制。进入页面会先发送高优先级 `Speed:0`，再发送 `Motor:0`，从而退出从板可能遗留的无感或开环状态并选择有感电机。页面包含两行：

- `Speed`：速度环目标范围 `-100~100 rad/s`，Left/Right 步进 `10 rad/s`；同时显示 JustFloat CH1 实际速度。
- `Turn Pos`：位置环目标为当前机械圈零点的相对位置，范围 `0.0~6.2 rad`，Left/Right 步进 `0.1 rad`；实际值由 JustFloat CH13 多圈机械角对 `2π` 取模得到。

Up/Down 选择控制环，OK 进入编辑，再按 OK 将目标加入 UART4 队列。顶部显示当前活动环、UART4 UP/DOWN、编码器健康状态和对齐状态。只有 UART4 在线、编码器健康且对齐完成后才能发送运动目标。KEY2、安全故障、Left/KEY3 退出页面都会发送高优先级 FOC STOP；安全故障未解除时页面拒绝发送新的速度或位置目标。

### 4.8 UI Diagnostics

UI Diagnostics 分为 9 个分页：Overview、Display、Memory、Reliability、Safety、Motion Loop、Chassis Odom、Communication 和 Architecture。使用 Left/Right 切换，KEY2/KEY3/ESC 返回。Communication 页显示 UART5、USB CDC、命令协议和 UART4 FOC 的收发、溢出、丢弃及错误计数；详细定义和回归方法见 [`docs/communication_and_engineering_quality_phase6_zh.md`](docs/communication_and_engineering_quality_phase6_zh.md)。

### 4.9 Camera Test

进入页面后按需唤醒 OV5640、校验传感器 ID，并自动采集一帧 `320×240 JPEG`。JPEG 使用 STM32H7 硬件 JPEG 外设解码为 RGB565，再缩放为 `200×150` 显示在 LVGL 预览区；页面底部同时显示 JPEG 大小、采集耗时和解码耗时。

- OK：重新初始化摄像头并采集下一帧。
- Left/KEY2/KEY3：停止当前采集、拉低 `OV_RESET`、进入掉电状态并返回主菜单。

### 4.10 Display Settings

提供屏幕旋转、按键声音、中英文切换和低电压蜂鸣器报警开关。屏幕旋转后五维按键方向同步映射；设置写入 W25Q64，掉电后保持。
- 初始化或采集失败时，页面显示传感器 ID、Camera 错误码和 DCMI 错误码；按 OK 可重试。

该页面只做单帧拍照测试，不持续占用 DCMI。离开页面会释放共享 RGB565 媒体内存池，摄像头压缩帧缓冲与显示帧缓冲分别位于 D2 SRAM 和 AXI SRAM，保证硬件 JPEG 解码时输入、输出能够同时存在。

## 5. 媒体播放操作

媒体播放由 `media_control.c` 统一处理按键，播放器本身负责解码和帧调度。

| 操作 | `.bin` | `.avi` MJPEG | `.gif` |
| --- | --- | --- | --- |
| OK 短按 | 暂停/播放 | 暂停/播放 | 暂停/播放 |
| Left 短按 | 后退 10 帧 | 后退 10 帧 | 后退 10 帧 |
| Right 短按 | 前进 10 帧 | 前进 10 帧 | 前进 10 帧 |
| Left/Right 长按 | 每 250 ms 重复跳帧 | 每 250 ms 重复跳帧 | 每 250 ms 重复跳帧 |
| KEY2 | 停止并退出播放 | 停止并退出播放 | 停止并退出播放 |
| KEY3 | 停止并返回 SD 目录 | 停止并返回 SD 目录 | 停止并返回 SD 目录 |

短按和长按采用独立的边沿/重复计时，跳帧后会更新播放基准，避免出现“回退一帧又被正常播放立即推进”的来回抖动。AVI 播放使用 TIM7 调度，SD DMA 缓冲区需位于 DMA 可访问内存，并在 Cache 开启时执行对应的 Clean/Invalidate。

常见错误码定义在 `mjpeg_player.h`，包括挂载失败、文件失败、格式失败、I/O 失败、解码失败、停止和缓冲区过大等。看到 `failed(-5)` 时优先检查 SD 读错误、DMA 完成状态、Cache 维护和缓冲区越界；看到 `failed(-6)` 时优先检查 MJPEG 帧头、JPEG 解码和帧缓冲区大小。

## 6. 小陀螺模式实现逻辑

### 6.1 目标

小陀螺模式让车体绕自身中心旋转，同时保持用户指定的世界坐标运动方向。例如开启后让车体顺时针自转，摇杆向前仍然沿开启瞬间的“世界前方”移动，而不是随着车头转动改变方向。

### 6.2 传感器和坐标处理

MPU6500 没有磁力计，因此工程不能得到绝对地磁北向。`imu_service.c` 对校准后的 Z 轴陀螺数据积分得到相对平面航向角：

```text
yaw(k) = wrap(yaw(k-1) + gyro_z_dps * dt)
```

启动时根据传感器初始安装方向自动确定 Z 轴正负；航向角被限制在 `[-180°, 180°]`。进入小陀螺模式的瞬间保存参考角 `yaw_ref`，该参考角就是本次运动的世界坐标零点。由于没有磁力计，长时间运行仍会有陀螺漂移。

### 6.3 指令和执行

小陀螺参数为：

- `ON/OFF`：是否启用。
- 方向：CW 顺时针或 CCW 逆时针。
- 速度：`0~100%`。
- 基准角速度：200 deg/s，实际角速度为 `speed_percent × 200 / 100`。

启用后每 10 ms 执行：

1. 读取当前平面航向，并按采样延迟做短时外推。
2. 计算车体当前角度相对 `yaw_ref` 的差值。
3. 把用户的世界坐标速度 `(Vx_world, Vy_world)` 旋转到车体坐标：

   ```text
   Vx_body = cos(theta) * Vx_world + sin(theta) * Vy_world
   Vy_body = -sin(theta) * Vx_world + cos(theta) * Vy_world
   ```

4. 将平移速度和限幅后的旋转速度一起送入麦轮逆解。
5. 角速度使用约 400 deg/s² 的斜坡变化，反向时先经过零速，减少机械冲击。

小陀螺模式会在以下情况关闭或停止：收到显式 `GYRO OFF`、离开 Command Control、KEY2 全局停止、底盘停止或 IMU 安全故障。当前通信设计不再对小陀螺命令单独设置 300 ms 超时；ESP32 只在 ON/OFF 状态变化时发送模式命令，摇杆超时只将平移/旋转摇杆归零，不会伪造 `GYRO OFF`。

## 7. 通信协议

### 7.1 文本命令

文本命令以 `\r` 或 `\n` 结束：

```text
M1:50                 设置 M1 为 +50%
M3:-30                设置 M3 为 -30%
STOP                  停止全部电机
GYRO ON CW 60         顺时针 60%
GYRO ON CCW 40        逆时针 40%
GYRO OFF              关闭小陀螺
FOC MOTOR 0           选择 C2804 电机
FOC SPEED 10          设置 FOC 速度目标
FOC TORQUE 0.4        设置 FOC 转矩/电流目标
FOC STOP              FOC 停止
```

`GYRO ...` 可由 USB CDC 或 UART5 接收，但只在 Command Control 页面生效。`M1:xx`/`STOP` 的快速调试入口位于 USB CDC 的 VOFA 接收路径。

### 7.2 二进制帧

通用格式为：

```text
77 68 LEN DEV_ID CMD PAYLOAD ... 0A
```

`LEN` 为整帧长度，当前接受 `0x04~0x10`，尾字节必须为 `0x0A`。普通写命令使用 `CMD=0x02`：

| DEV_ID | 用途 | 主要负载 |
| --- | --- | --- |
| `0x01` | 四电机速度 | Byte5~8 为 M1~M4 的 int8 速度 |
| `0x02` | 单电机速度 | Byte5 电机号，Byte6 int8 速度 |
| `0x03` | 麦轮混合控制 | mode、Vx、Vy、Wz；mode=0 速度，mode=1 距离 |
| `0x05` | 舵机角度 | 舵机端口和角度 |
| `0x0D` | 小陀螺 | Byte5 使能，Byte6 方向，Byte7 速度百分比 |
| `0x0F` | UART4 FOC 桥接 | Byte5 操作号，后续为可选 float32 小端负载 |

小陀螺 60% 顺时针示例：

```text
77 68 09 0D 02 01 01 3C 0A
```

`DEV_ID=0x0C、LEN=0x0A` 为 ESP32 虚拟摇杆帧，Byte4~7 依次是 `LX、LY、RX、RY`（int8）：

```text
wz = LX * 2
vy = RX * 10
vx = RY * 10
```

读命令和写入应答从请求原通道返回：UART5 请求回 UART5，USB CDC 请求回 USB CDC。

### 7.3 UART4 FOC 控制板

H743 使用 `PD1/UART4_TX` 连接 FOC 板 `PB7/USART1_RX`，使用 `PD0/UART4_RX` 连接 `PB6/USART1_TX`，并共地。两端均为 115200 8N1。上游继续使用 `77 68 ... 0A` 帧；H743 在内部转换为 FOC 固件现有的 `Speed:`、`Angle:`、`Torque:`、`Motor:`、`Sensorless` 和 `Lock` 换行命令。

UART4 使用中断收发和 8 项发送队列，收到停止请求时会丢弃尚未发出的旧目标，并在当前行发送完成后优先发送 `Speed:0`。FOC 板在同一串口输出的 16 路 JustFloat 数据由主循环解析，Command Control 页面显示链路、实际速度和 Iq。完整操作号、应答、状态回包和首次上板步骤见 [UART4 FOC 主从通信协议](docs/foc_uart4_master_protocol_zh.md)。

### 7.4 USB CDC / VOFA+

USB CDC 始终以 50 Hz 输出一个固定的 14 通道 JustFloat 帧，不随当前 LVGL 页面切换数据源。VOFA+ 应选择 `JustFloat`，并按 CH0~CH13 配置：

| 通道 | 数据 | 单位/取值 |
| ---: | --- | --- |
| CH0~CH3 | 主板 M1~M4 实际转速 | rpm |
| CH4~CH7 | 主板 M1~M4 PWM 占空比 | % |
| CH8 | FOC 控制模式 | 0 停止、1 速度、2 位置、3 转矩 |
| CH9 | 主机请求速度 | rad/s |
| CH10 | FOC 实际速度 | rad/s |
| CH11 | 主机请求的本圈位置 | rad，`0~2π` |
| CH12 | FOC 当前本圈位置 | rad，归一化为 `[0, 2π)` |
| CH13 | FOC 多圈绝对机械位置 | rad，可正可负 |

CH8、CH9 和 CH11 由 UART4 驱动在命令成功进入发送队列时统一更新，因此通过 LVGL、USB CDC 或 UART5 下发的 FOC 命令都会反映到同一组曲线。目标/实际电流和链路诊断项目前已在代码中注释保留。帧长度固定为 `14 × float32 + 4 字节帧尾 = 60 字节`。

### 7.5 ESP32-C3 虚拟手柄

配套程序位于工程内的 `ESP32_connect_Controller`。浏览器右上角按钮可在麦轮遥控和 NES 虚拟手柄之间切换；切换时会主动释放上一模式的输入。

麦轮页面提供左右摇杆、小陀螺 ON/OFF 按钮和 `-100~100` 的方向/速度滑条。当前发送策略为：

- 摇杆以约 50 Hz 发送，使用最新值，不排队发送旧帧。
- WebSocket 缓冲区超过阈值时丢弃本次旧帧，避免网络拥塞造成延迟累积。
- 摇杆停止或连接超时时只清零摇杆值。
- 小陀螺 ON/OFF 为状态事件，不会因摇杆刷新超时自动关闭。

NES 页面提供方向键、Select、Start、B、A，并支持方向键与 A/B 多点触控。独立的 `HOLD RESET` 按钮需长按 1 秒，用于软复位当前 ROM、返回游戏标题画面。手机按键以完整 8 位状态图每 50 ms 发送，STM32 只在 NES 运行期间接收，并与实体按键按位合并。断连 350 ms 后仅释放手机按键，不退出游戏、不报故障，详见 [NES 手机虚拟手柄](docs/nes_mobile_controller_zh.md)。

手机端连接不稳定时，先确认手机已连接 ESP32 热点，再刷新网页；若数据延迟持续增大，优先检查浏览器页面是否为最新版本、WebSocket 缓冲区和热点信道干扰。

## 8. 安全策略

- Command Control 是远程控制的页面级权限门，离开页面立即停止。
- UART4 FOC STOP 会抢占待发送目标；未给 FOC 从板增加原生 Stop 前，不开放无法被 `Speed:0` 可靠退出的 OpenLoop 调试命令。
- Mecanum Control 执行距离轨迹时，所有受限轴到位后统一清零。
- TIM13 每 10 ms 执行安全控制；运动过程中 IMU 数据超过约 30 ms 未更新会触发 `SAFETY_FAULT_IMU_STALE` 并急停。
- KEY2 是最高优先级的本地停止输入；媒体播放中先处理 KEY2，再处理暂停和跳帧。
- 电机目标速度和麦轮速度统一限制在 `-100~100`，避免上层输入溢出。

## 9. 构建和调试

### STM32

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

烧录后检查：USB CDC 是否枚举、UART5/UART4 波特率和引脚是否正确、TIM13 是否稳定 10 ms、编码器方向是否正确、SD 卡是否能正常挂载。

### ESP32-C3

在虚拟手柄目录执行：

```powershell
pio run
pio run -t upload
```

刷写后重新打开手机网页，确认浏览器控制台没有 WebSocket 连接错误。

### 常见排查顺序

1. 电机不动：确认当前是否在 Command Control，检查 KEY2 是否处于按下状态以及 IMU 安全故障标志。
2. 麦轮方向错误：单独测试 X/Y/Z，核对四轮编号、编码器方向和电机正负号。
3. AVI 卡顿或退出：检查 SD DMA 缓冲区位置、Cache Clean/Invalidate、文件连续读取和 JPEG 帧长度。
4. 手机摇杆延迟：观察 WebSocket 连接、浏览器页面是否持续产生旧帧积压；重新连接热点后再测试。
5. 小陀螺方向不对：在 MPU6500 Data 页面确认 Z 轴角速度符号，再分别发送 CW/CCW 小速度命令。
6. FOC 显示 DOWN：检查 UART4 与 USART1 是否交叉连接并共地，确认两端都是 115200 8N1，再检查 FOC 板是否持续输出 JustFloat 帧。

## 10. 已知限制

- MPU6500 无磁力计，小陀螺的“世界坐标”是进入模式瞬间建立的相对坐标，不是绝对地理坐标；长时间运行会有航向漂移。
- SD 卡、JPEG 解码和显示刷新仍受外部存储器时序、Cache 一致性和帧缓冲区大小影响。
- USB/UART5 二进制控制命令必须遵守帧头、长度和尾字节格式；应答和读回包返回原请求通道。
- 当前 FOC 从板没有命令级 ACK，主板应答只代表命令已进入 UART4 队列，不能证明从板已经执行。
- 媒体播放期间 UI 主循环工作量较大，建议使用合理分辨率、帧率和 JPEG 压缩质量。

## 11. 主要入口函数

```text
LVGL_App_Init() / LVGL_App_Process()
lvgl_app_show_main_menu()
lvgl_app_show_command_control()
lvgl_app_cmd_parse()
Mecanum_MixedControl() / Mecanum_Tick10ms()
Mecanum_GyroEnable() / Mecanum_GyroDisable()
DCMotor_OL_Tick10ms()
IMU_Service_Process()
FOC_Link_Init() / FOC_Link_Process()
MJPEG_Player_PlayFile()
SD_StartAnim_PlayFile()
VOFA_Task_Process()
```

如需继续扩展，建议优先在现有页面骨架、`media_control` 状态机和 `Mecanum_MixedControl` 接口上增加功能，避免在 LVGL 事件回调中直接执行长时间 SD/电机操作。
