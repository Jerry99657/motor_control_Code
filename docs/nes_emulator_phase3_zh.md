# NES 模拟器第三阶段说明

> 本文记录 Mapper 0/2/3 阶段。当前工程已经进入第四阶段，并进一步支持 Mapper 1、QSPI 快速命中、电池 `.sav` 和无音频 APU IRQ；请同时参考 `nes_emulator_phase4_zh.md`。

## 改进目标

第三阶段在已通过实机验证的 Mapper 0 显示、LCD 异步刷新和 KEY3 返回逻辑上继续扩展：

- 补齐 A、B、Start、Select 和方向键；
- 增加软复位；
- 支持 Mapper 2（UxROM）和 Mapper 3（CNROM）；
- 保持无音频设计，不改变 KEY2/KEY3 的安全职责。

## 按键方案

| 操作 | 功能 |
| --- | --- |
| OK | A |
| KEY1 | B |
| OK + KEY1 | Start |
| KEY1 双击 | Select（第二次按下时暂时屏蔽 B） |
| Up / Down / Left / Right | NES 方向键 |
| OK + KEY1 保持 1.2 秒 | NES 软复位 |
| KEY3 | 停止当前游戏并返回原 SD 目录 |
| KEY2 | 全局停止并退出游戏 |

开发板的 Up、Down、Left、Right、OK 是同一颗五维按键，内部方向不可能同时触发，因此运行时不再使用 Up+Down 或 Left+Right 组合。OK 和独立的 KEY1 可以同时按下，Start/软复位仍采用这组跨器件组合。

独立的模拟器暂停功能已取消。需要暂停游戏时使用 `OK+KEY1` 发送游戏自身的 Start；这样不会占用 KEY2 的全局停止和 KEY3 的目录返回功能。

软复位会执行以下操作：

- 6502 从复位向量 `$FFFC/$FFFD` 重新启动；
- 清理 PPU 控制寄存器、滚动地址锁存和 CPU DMA/周期债务；
- Mapper 2 PRG Bank 和 Mapper 3 CHR Bank 回到 0；
- 保留 CPU RAM、PRG RAM、名称表和 OAM，行为接近主机 Reset，而不是重新加载 ROM。

## Mapper 2：UxROM

CPU 地址映射为：

```text
$8000-$BFFF  可切换的 16 KB PRG Bank
$C000-$FFFF  固定为最后一个 16 KB PRG Bank
```

写入 `$8000-$FFFF` 会更新低地址 PRG Bank。支持至少 32 KB、以 16 KB 为单位的 PRG ROM，以及 8 KB CHR RAM或单 Bank CHR ROM。

## Mapper 3：CNROM

PRG 映射保持与 NROM 相同，写入 `$8000-$FFFF` 切换整块 8 KB CHR Bank。当前支持 16/32 KB PRG，以及一个或多个 8 KB CHR ROM Bank。

## 兼容范围

当前运行时支持：

| Mapper | 名称 | PRG | CHR |
| ---: | --- | --- | --- |
| 0 | NROM | 16/32 KB | 8 KB ROM 或 RAM |
| 2 | UxROM | ≥32 KB，16 KB 分 Bank | 8 KB ROM 或 RAM |
| 3 | CNROM | 16/32 KB | ≥8 KB，8 KB 分 Bank |

暂不支持 Mapper 1、Mapper 4、存档写回 SD、APU 音频和 APU IRQ。少数 Mapper 2/3 卡带具有总线冲突或特殊板级变体，若出现画面 Bank 错乱，需要根据 ROM CRC 增加板型兼容规则。

## 推荐测试

1. 先复测已经通过的 Mapper 0 ROM，确认 KEY3 行为没有回归。
2. 在标题画面验证 OK、KEY1、OK+KEY1 和 KEY1 双击。
3. 使用 OK+KEY1 验证游戏自身的开始/暂停功能。
4. 保持 OK+KEY1 1.2 秒，确认游戏回到复位画面，并且不会退出 SD 页面。
5. 分别选择页面显示为 Mapper 2 和 Mapper 3 的 ROM，观察切换场景时是否出现花屏或错误图块。
6. 每种 Mapper 连续运行至少 5 分钟，再按 KEY3 返回并重新进入，检查 QSPI Memory-Mapped 生命周期。
