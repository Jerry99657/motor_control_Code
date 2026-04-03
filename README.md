# STM32H743ZIT6 KIT 启动动画排障记录

## 目标

本项目的启动动画支持两条路径：

- SD 卡播放启动动画
- QSPI Flash 回退播放启动动画

本次排障的目标是：

- 解决 SD 播放黑屏、读取失败和回退异常
- 解决启动时 HardFault / SIGTRAP 问题
- 降低 RAM_D2 占用
- 明确异步播放与同步播放的差异
- 记录后续“真正异步”的实现计划

## 问题回顾

### 1. 启动动画黑屏

最初的现象是：

- SD 动画播放失败
- 超时后回退到外置 Flash 也失败
- 画面黑屏

### 2. SD 首帧读取失败

后续日志显示：

- `SDA: frame0 read fail`
- `QSA: ok`

这说明：

- QSPI 播放链路正常
- SD 动画失败集中在首帧读取或其后续显示链路

### 3. HardFault / SIGTRAP

在调试器中，程序进入了异常停顿状态。

通过 `g_hardfault_snapshot` 最终定位到：

- `stacked_pc = 0x0801AB5E`
- `stacked_lr = 0x0801AB0D`
- `cfsr = 0x00000400`
- `hfsr = 0x40000000`

这代表：

- 精确数据总线错误
- 被升级成 HardFault

最终追到的故障点是 LCD 异步 DMA 路径中的 DCache 维护函数，而不是 SD 读本身。

### 4. RAM_D2 占用偏高

由于 SD 启动动画使用双帧缓冲，RAM_D2 占用一度较高，需要优化内存布局。

## 已完成的修复

### 1. 增加 HardFault 快照

在 `Core/Src/stm32h7xx_it.c` 中加入了：

- `g_hardfault_snapshot`
- `HardFault_HandlerC()`
- 堆栈寄存器、CFSR、HFSR、BFAR、MMFAR 采集

作用：

- 让 HardFault 不再“无现场”
- 可以直接定位故障 PC

### 2. SD 读取增强

在 `Drivers/User/Src/sd_start_anim.c` 中加入：

- `f_lseek()` 跳到动画数据区
- 分块读取
- 读取失败详细日志
- 中转 stage buffer

作用：

- 让读取错误更容易定位
- 避免直接把大块数据一次性压到目标缓冲区

### 3. QSPI 链路稳定化

在 `Drivers/User/Src/qspi_w25q64.c` 和 `Drivers/User/Src/qspi_start_anim.c` 中做了加固：

- QSPI 读写重试
- 播放日志
- 播放正常后可稳定回退

当前结果：

- `QSA: ok`

### 4. LCD 异步链路修复

最终定位到异常来自 LCD 异步 DMA 路径中的 DCache 维护。

做过的处理包括：

- 增加 `LCD_ResetTransferState()`
- 避免超时后状态污染
- 将异步播放路径修正为适合 DMA 读源数据的 cache 语义
- 对 D2 SRAM 进行 MPU 非缓存配置
- 让 LCD 驱动只对真正缓存的 AXI SRAM 做 cache 维护

### 5. RAM_D2 优化

已经把 SD 启动动画的帧缓冲恢复到 `.ram_d2`，同时通过 MPU 把 D2 配置成非缓存区，避免 cache op 触发异常。

当前内存结果：

- RAM 总占用约 52.54%
- RAM_D2 占用约 39.06%

### 6. 测速功能关闭

为减少干扰，已将 SD benchmark 默认关闭。

## 关键结论

### 异步播放不等于一定更快

异步的作用是：

- 让 CPU 不必一直等待 SPI 传输完成
- 允许“读下一帧”和“发上一帧”重叠

但如果当前代码仍然是：

- 单缓冲
- 每帧仍然等待完成
- 动画帧间隔固定

那么异步在观感上通常不会有明显差异。

### 速度瓶颈主要在三个地方

- LCD SPI 带宽
- 动画文件的 frame delay
- 是否真的实现了流水线并行

## 当前可调参数

### 1. LCD SPI 分频

位置：

- `Drivers/User/Src/lcd_spi_154.c`

当前代码里存在：

- `LCD_SPI.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;`

CubeMX 里也能改：

- `SPI6 -> Parameter Settings -> Clock Parameters -> Prescaler for Baud Rate`

建议：

- 先试 `8`
- 稳定后试 `4`
- 如果面板允许，再试 `2`

### 2. QSPI 播放倍率

位置：

- `Drivers/User/Inc/qspi_start_anim.h`

当前参数：

- `QSPI_START_ANIM_PLAYBACK_SPEED_NUM = 4U`
- `QSPI_START_ANIM_PLAYBACK_SPEED_DEN = 1U`

含义：

- 当前约为 4 倍速

### 3. SD 动画帧延时

位置：

- 动画文件头中的 `frame_delay_ms`
- `Drivers/User/Src/sd_start_anim.c`

当前播放节奏仍然跟随帧延时。

如果要更快，可以：

- 重新打包动画时减小 `frame_delay_ms`
- 或给 SD 播放也加一个倍率参数

## CubeMX 中的 SPI 修改位置

如果要改 SPI6 分频：

- 打开 CubeMX
- 进入 `Pinout & Configuration`
- 找到 `Connectivity -> SPI6`
- 打开 `Parameter Settings`
- 修改 `Clock Parameters -> Prescaler for Baud Rate`

注意：

- 运行时驱动里如果再次手动设置 SPI 分频，会覆盖 CubeMX 的配置
- 需要保证 CubeMX 与代码中的值一致

## 真正异步的实现计划

目前的“异步”更多是：

- DMA 发送异步
- 但读取与显示并没有充分流水线化

如果要做成真正异步，建议按下面方案推进：

### 计划 1：双缓冲流水线

目标：

- 当前帧 DMA 发屏时，后台读取下一帧

实现思路：

- 保留两个帧缓冲
- buffer A 发送时，buffer B 读取下一帧
- 发送完成后交换缓冲区

收益：

- 真正重叠 I/O 和显示
- 才能明显缩短总播放时间

### 计划 2：把帧读取与发送解耦

目标：

- 读帧和发屏不要写在同一个阻塞循环里

实现思路：

- 读取任务负责填充下一帧
- 显示任务负责把当前帧交给 LCD DMA
- 用状态机管理两个阶段

### 计划 3：统一缓冲区内存策略

目标：

- 让 DMA 读写缓冲区都放在明确策略下的内存区

实现思路：

- DMA 输入缓冲使用非缓存区，或严格做 invalidate/clean
- 需要 cache 的区域只给 CPU 侧使用
- 避免把 DMA buffer 和 cache maintenance 混用

### 计划 4：加入真正的性能度量

目标：

- 判断到底是 LCD、SD 还是动画节奏限制了速度

实现思路：

- 统计每帧读取耗时
- 统计每帧显示耗时
- 统计总帧率
- 决定是调 SPI、调帧间隔还是调度策略

## 最后建议

如果只想先“看起来更快”，优先做这三件事：

1. 把 LCD SPI 分频从 16 调小到 8 或 4
2. 把动画文件的 `frame_delay_ms` 调小
3. 如果要保留异步，就继续推进双缓冲流水线

如果要“真正加速”，核心是让读取和发送重叠，而不是仅仅把发送改成异步。

## 时间轴总结

### 第一阶段：最开始移植 W25Q64

目标：

- 让外置 QSPI Flash 正常初始化、读 ID、读写数据

遇到的问题：

- QSPI 初始化不稳定
- 读写流程与原工程环境不一致
- 用户层 QSPI 文件需要重新接回工程
- CubeMX 重新生成后，外部 Flash 相关代码容易遗漏

最终处理：

- 修正 QSPI 初始化流程
- 增加读写重试和复位
- 将 `qspi_w25q64.c`、`qspi_start_anim.c` 等文件重新纳入构建
- 先把 W25Q64 的基础读写稳定下来

### 第二阶段：用串口写入 W25Q64

目标：

- 通过串口 / CDC 把动画数据写入 W25Q64

遇到的问题：

- 普通 CDC 通道会和下载数据流冲突
- 如果没有先进入下载模式，主循环中的 CDC echo 会干扰写入流
- 下载过程需要明确区分“运行模式”和“下载模式”

最终处理：

- 增加串口下载入口
- 在启动时区分正常运行和下载模式
- 进入下载模式后再接收完整写入流
- 避免正常 CDC echo 抢占写入数据

### 第三阶段：播放 W25Q64 中的动画

目标：

- 从外置 W25Q64 读取动画并显示到 LCD

遇到的问题：

- 动画数据头解析和数据偏移需要严格一致
- 播放帧率和 LCD 刷新速度要匹配
- 异步 DMA 发送路径涉及 cache 维护
- 早期一度出现 HardFault / SIGTRAP

最终处理：

- 统一动画头格式
- 增加帧读取与播放日志
- 修正 LCD 异步 cache 维护语义
- 通过 MPU 和内存布局规避 DMA 缓冲冲突

### 第四阶段：启动动画集成到 SD / QSPI 双路径

目标：

- SD 优先播放，失败后回退 QSPI

遇到的问题：

- SD 首帧读取失败
- 回退链路在 cache / DMA 上出问题
- RAM_D2 占用偏高

最终处理：

- 引入 HardFault 快照
- 定位到 LCD 异步传输链路
- 调整 D2 SRAM 的 MPU 策略
- 保留异步播放能力，同时把内存策略收敛到稳定状态

### 最终结论

整个工程的主线可以概括为：

1. 先让 W25Q64 基础读写稳定
2. 再让串口写入流程不被正常 CDC 任务干扰
3. 然后让 W25Q64 动画播放稳定
4. 最后把 SD / QSPI 启动动画集成到统一播放链路里

这条时间轴里最核心的风险点是：

- 外设 DMA 与 cache 维护
- CDC 正常模式和下载模式的通道冲突
- 动画帧率、SPI 带宽和内存布局的耦合

## 本次对话中的故障总表

### 故障 1：SD 启动动画黑屏

现象：

- SD 动画不能播放
- 屏幕黑屏
- 失败后回退外置 Flash 也不稳定

根因：

- 启动动画播放链路没有稳定跑通
- 后续定位到 SD 首帧读取与 LCD 发送链路叠加后会出问题

最终处理：

- 增加启动阶段日志
- 增加 SD 读取分块和失败诊断
- 加固 QSPI 回退链路
- 加固 LCD 传输状态恢复

### 故障 2：SD 首帧读取失败

现象：

- 日志出现 `SDA: frame0 read fail`
- 但 `QSA: ok`

根因：

- SD 链路失败集中在首帧读取或其后续显示路径
- 后来确认根因并不在 SD 读本身，而在 LCD 异步路径的 cache 维护

最终处理：

- SD 读取增加 `f_lseek()` 和分块读取
- 增加 `fr/req/got` 详细日志
- 调整 LCD 异步路径的缓存策略

### 故障 3：HardFault / SIGTRAP

现象：

- 烧录后直接进入异常
- 调试器停在 `SIGTRAP`
- `g_hardfault_snapshot` 一开始还是全 0

根因：

- 真正进入了 HardFault，但最初没有拿到有效现场
- 之后通过 `g_hardfault_snapshot` 和反汇编定位到 `SCB_CleanDCache_by_Addr()` / `SCB_InvalidateDCache_by_Addr()`
- 这是 LCD 异步 DMA 相关的 cache 维护问题

最终处理：

- 增加 HardFault 快照抓取
- 定位到 LCD 异步缓存维护路径
- 调整 D2 SRAM 的 MPU 策略
- 让 LCD 驱动只对真正缓存区域执行 cache maintenance

### 故障 4：RAM_D2 占用偏高

现象：

- RAM_D2 占用一度接近 78%

根因：

- SD 启动动画使用双帧缓冲
- 帧缓冲直接放在 D2 区域

最终处理：

- 优化帧缓冲布局
- 配置 D2 SRAM 为非缓存区
- 降低 cache 相关风险

### 故障 5：异步播放“看起来没更快”

现象：

- 异步启用后，观感上与同步播放差别不大

根因：

- 异步只减少 CPU 等待，不等于总播放时间必然减少
- 代码里尚未形成真正的读写流水线
- 播放速度还受 frame delay 和 LCD 带宽限制

最终处理：

- 保留异步能力
- 但明确后续要做真正双缓冲流水线

## 本次对话中的最终解决方法汇总

- 通过 HardFault 快照定位精确故障点
- 将故障从 SD 读取链路进一步缩小到 LCD 异步 cache maintenance
- 用 MPU 和缓存策略修正 DMA 缓冲区使用方式
- 保留异步播放，但让其在正确的内存策略下运行
- 通过 SPI 分频和帧延时作为主要提速参数

## 后续仍可继续优化的方向

- 让 SD 读取和 LCD 发送真正流水线并行
- 进一步缩短动画文件的 `frame_delay_ms`
- 继续下调 SPI6 分频，验证屏幕稳定上限

