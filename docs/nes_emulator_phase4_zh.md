# NES 模拟器第四阶段：兼容性与存档

## 本阶段目标

第四阶段建立在 Mapper 0/2/3、QSPI ROM 缓存和完整按键输入已经通过实机验证的基础上，主要完成：

- 相同 ROM 的 QSPI 缓存快速命中；
- Mapper 1（MMC1）；
- 电池 PRG-RAM 的 SD 卡存档；
- 不输出声音但保留兼容性所需的 APU Frame IRQ；
- CPU、PPU、Mapper 和 FatFs 故障诊断。

## QSPI 缓存快速命中

缓存元数据版本升级为 2。每份缓存记录：

- ROM 路径 CRC32；
- FAT 修改日期和时间；
- ROM 大小和完整 iNES 头；
- ROM 尾部最多 4 KiB 的 CRC32；
- 已写入 QSPI 的完整 ROM CRC32和元数据 CRC32。

再次选择同一文件且以上特征一致时，页面直接显示 `Cache hit 100%`，不再擦除和重写 W25Q64。升级固件后的第一次选择会因为旧元数据版本不匹配而正常重建缓存，第二次开始才能快速命中。

## Mapper 1（MMC1）

当前实现包括：

- MMC1 5 位串行移位寄存器和 bit7 复位；
- PRG 32 KiB、固定低 16 KiB、固定高 16 KiB 三类映射；
- CHR 8 KiB 和双 4 KiB 映射；
- 单屏低页、单屏高页、垂直和水平镜像；
- PRG-RAM 使能/禁用；
- RMW 指令连续写过滤；
- 最高 512 KiB PRG，并支持使用 CHR bank bit4 选择 PRG 高区的 SUROM 类布局。

为避免降低 PPU 绘制速度，当前 PRG/CHR Bank 在寄存器提交时预先计算，CPU 和 PPU 的高频读取路径不执行整数取模。

## `.sav` 电池存档

只有 iNES `flags6.bit1` 置位的 ROM 才启用存档。当前 PRG-RAM 固定为 8 KiB：

1. 游戏启动时读取同目录、同文件名的 `.sav`；
2. `$6000-$7FFF` 内容发生变化后设置 dirty 标志；
3. KEY3 正常返回目录时才写卡；
4. KEY2 仍是紧急退出，不等待 SD 写入；
5. 没有修改 PRG-RAM 时不会产生无意义写入。

PRG-RAM 位于 DTCM，SDMMC DMA 无法直接访问。因此存档通过位于 D2 SRAM、32 字节对齐的 4 KiB 缓冲区分块传输。

写入采用以下恢复链：

```text
name.sav.tmp  写满 8 KiB并 f_sync
name.sav      若存在则改名为 name.sav.bak
name.sav.tmp  改名为 name.sav
name.sav.bak  新文件生效后删除
```

启动加载依次检查 `.sav`、`.sav.bak`、`.sav.tmp`，只接受大小严格等于 8 KiB 的完整映像。这样即使在重命名步骤之间掉电，也能找到上一份或刚同步完成的存档。

## 无音频 APU 兼容层

项目仍然不生成声音，但新增：

- `$4015` Frame IRQ 状态读取和清除；
- `$4017` 四步/五步模式、IRQ inhibit 和计数器复位；
- 6502 maskable IRQ 请求、清除和 `$FFFE/$FFFF` 向量处理；
- OAM DMA 停顿期间 APU 时钟继续前进。

音频通道长度计数器和 DMC IRQ 尚未实现，因此 `$4015` 的低 5 位保持为零。

## 故障诊断

运行时保存以下信息：

- Mapper、当前 PRG/CHR Bank 和 MMC1 控制寄存器；
- 6502 最后执行的 PC 和 Opcode；
- 当前 PPU 扫描线、CPU 总周期和帧数；
- 存档是否加载、是否 dirty，以及最后一个 FatFs 错误码。

若 CPU 或显示出现错误，返回 SD 页面后的状态栏会显示类似：

```text
NES failed -5 M1 PC:C123 OP:02 L:120
```

## 实机测试顺序

1. 先复测已经运行正常的 Mapper 0、2、3 游戏，确认画面、按键和 KEY3 返回没有回归。
2. 连续选择同一个 ROM 两次；第一次可能重建版本 2 缓存，第二次应立即出现 `Cache hit 100%`。
3. 选择页面显示 `Mapper 1` 的 ROM，重点观察标题画面、场景切换、滚屏和精灵图块。
4. 对带电池标志的游戏产生存档内容，按 KEY3 返回；状态栏应显示 `save written`，目录中生成同名 `.sav`。
5. 再次进入游戏，确认进度能够恢复。
6. 修改游戏进度后用 KEY2 退出，再进入时应保持上一次 KEY3 保存的内容，证明紧急退出没有阻塞写卡。
7. 每类 Mapper 连续运行至少 10 分钟；若失败，记录页面状态栏中的 Mapper、PC、Opcode 和扫描线。

## 当前边界

- 支持 Mapper 0、1、2、3，尚不支持 Mapper 4/MMC3；
- Mapper 1 PRG-RAM 当前固定为 8 KiB，不支持多 WRAM Bank 板型；
- 不输出音频，也未实现音频通道长度计数器和 DMC；
- `.sav` 是卡带电池 RAM，不是 CPU/PPU 即时存档；
- LCD 仍采用约 60 Hz 模拟、约 30 FPS 异步显示。

