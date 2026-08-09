# NES 模拟器第二阶段说明

> 本文记录第二阶段 Mapper 0 版本。当前工程已经进入第四阶段，并进一步支持 Mapper 1/2/3、完整手柄输入、缓存快速命中与电池存档；请同时参考 `nes_emulator_phase3_zh.md` 和 `nes_emulator_phase4_zh.md`。

## 本阶段范围

本阶段在第一阶段“SD 卡 `.nes` 文件缓存到 W25Q64 并完成 CRC32 校验”的基础上，加入可实际运行并显示画面的 NES 核心。

当前支持：

- iNES 1.0；
- Mapper 0（NROM）；
- 16 KB / 32 KB PRG ROM；
- 8 KB CHR ROM 或 8 KB CHR RAM；
- 水平、垂直和四屏名称表镜像；
- 6502 全部官方指令及常用非官方指令；
- 背景、8×8/8×16 精灵、精灵优先级、Sprite 0 Hit；
- 256×240 NES 画面左右各裁掉 8 像素，以 240×240 RGB565 显示；
- 无音频运行。

暂不支持 Mapper 1/2/3/4 等带 Bank Switching 的卡带，也不模拟 APU 音频和 APU IRQ。

## 代码结构

| 文件 | 作用 |
| --- | --- |
| `nes_rom_cache.c/.h` | SD 到 QSPI 的分块缓存、iNES 解析、CRC 和 Memory-Mapped 生命周期 |
| `nes_cpu.c/.h` | GCC 原生 C 版 Ricoh 2A03/6502 指令解释器 |
| `nes_runtime.c/.h` | Mapper 0、CPU 总线、PPU、逐帧调度、LCD 输出和按键退出 |
| `media_memory.c/.h` | 为 NES 分配共享 240×240 RGB565 帧缓冲 |
| `lvgl_app.c` | 从 NES Cache 页面启动运行时，退出后恢复原 SD 目录 |
| `qspi_partition.h` | W25Q64 启动画面、NES 元数据和 ROM 的固定分区 |

`nes_runtime.c` 的 CPU RAM、PRG RAM、CHR RAM、名称表、OAM 和扫描线工作区共约 24 KB，放在 DTCM。DTCM 段为 `NOLOAD`，因此运行时每次启动都会显式清零；活动标志单独留在普通 `.bss`，避免上电后读取未初始化 DTCM。

完整 RGB565 帧缓冲继续使用共享媒体内存池，不会与 MJPEG、启动动画或 Camera Test 同时占用。ROM 不复制到 SRAM，而是直接读取 QSPI `0x90000000 + 0x401000` 的 Memory-Mapped 地址。

## 运行和显示流程

1. 在 `SD Card Files` 中选择 `.nes` 文件。
2. 第一阶段把 ROM 缓存到 QSPI 并显示 `Cache ready 100%`。
3. 按 OK 启动模拟器。
4. CPU/PPU 约按 60 Hz 推进；LCD 每两帧异步提交一次，即目标显示约 30 FPS。
5. NES 运行时直接拥有 LCD，LVGL 定时器暂时冻结，避免 LVGL Flush 覆盖游戏画面；主循环中的通信和安全任务仍继续运行。
6. KEY3 停止当前 NES 并返回原来的 SD 目录，提示 `Returned by KEY3`；KEY2 执行全局停止并退出。

基础按键映射为：

| 开发板按键 | NES 手柄 |
| --- | --- |
| Up/Down/Left/Right | 方向键 |
| OK | A |
| KEY1 | Start |
| KEY3 | 返回 SD 目录，不发送给游戏 |
| KEY2 | 全局停止，不发送给游戏 |

由于本阶段目标是先稳定显示画面，B 和 Select 暂未分配。

## 上板测试顺序

1. 优先使用页面显示为 `Mapper 0`、`PRG 16/32 KB`、`CHR 0/8 KB` 的 ROM。
2. Cache Ready 后按 OK，观察是否在数秒内出现稳定画面。
3. 连续运行至少 5 分钟，确认无花屏、黑屏、HardFault 或 SD 目录丢失。
4. 播放中按 KEY3，确认返回进入 ROM 前所在的 SD 目录。
5. 再次选择同一 ROM并启动，确认 QSPI 能反复进入和退出 Memory-Mapped 模式。
6. 运行时按 KEY2，确认电机保持停止并返回目录。

若页面显示 `NES failed (-5)`，表示 6502 执行到 JAM/未实现语义，应记录 ROM 名称、CRC32、PC 和操作码后补充兼容性；若显示 `NES failed (-6)`，优先检查 LCD 上一次 LVGL Flush 是否真正结束、LCD 异步传输状态和共享媒体池所有者。

## 参考来源

扫描线组织方式和 RGB565 调色板参考工程内：

- `例程/stm32f407_nes-master/NES`
- `例程/STM32H7-retrogame-master/Middlewares/NES`

参考工程原来的 6502 核心使用 Keil ARMASM 语法，当前 GCC 工程无法直接链接，因此本阶段重新实现了原生 C 版 CPU 核心，并在源码中保留参考来源说明。
