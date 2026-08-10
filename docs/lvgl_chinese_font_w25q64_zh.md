# LVGL 中文字库与 W25Q64 存储说明

## 目标

SD 卡使用 FatFs CP936/GBK 文件名，LVGL 使用 UTF-8。本功能在不改变文件系统原始名称的前提下，为 SD 文件列表、目录路径和媒体标题提供完整 GB2312 汉字显示，并避免把约 396 KB 字形位图占用在 STM32H743 的内部 Flash 中。

未收录在 GB2312 或原有小字库中的 CP936 扩展字符会显示为 `?`，不会再出现“字符直接消失、用户无法判断文件名内容”的情况。

## W25Q64 分区

| 范围 | 大小 | 用途 |
| --- | ---: | --- |
| `0x000000–0x3FFFFF` | 4 MB | 启动画面 |
| `0x400000–0x400FFF` | 4 KB | NES 缓存元数据 |
| `0x401000–0x77FFFF` | 约 3.5 MB | NES ROM 缓存 |
| `0x780000–0x7FFFFF` | 512 KB | LVGL GB2312 字库包 |

字库分区不会覆盖启动动画。NES ROM 可用空间由约 4 MB 调整为约 3.5 MB，对当前 Mapper 0/1/2/3 的常见 ROM 容量没有实际影响。NES 元数据校验会采用新的分区上限，异常超大旧缓存不会越界访问字库。

## 代码结构

- `tools/generate_lvgl_gb2312_font.ps1`：调用 `lv_font_conv` 从 Windows 黑体生成 16 px、2 bpp、无压缩 GB2312 字库；拆出位图并生成 W25Q64 安装包。
- `assets/GB2312.FNT`：可复制到 SD 卡的字库安装包，包含 32 字节头、位图长度、CRC32、字形数量和约 396 KB 位图。
- `Drivers/User/Src/lv_font_gb2312_16.c`：只保留 Unicode 映射、字形尺寸、偏移和 LVGL 字体描述，位图数组已移除。
- `Drivers/User/Src/ui_font_storage.c`：字库检查、SD 安装、写后回读校验以及 8 项字形缓存。
- `Drivers/User/Inc/ui_font_asset.h`：由生成脚本写出的长度、版本和 CRC32 常量，保证 MCU 侧描述与外置位图严格匹配。
- `Drivers/User/Inc/qspi_partition.h`：W25Q64 固定分区边界和编译期重叠检查。
- `Drivers/User/Src/lvgl_app.c`：CP936 转 UTF-8；只对界面显示名称使用 UTF-8，FatFs 打开文件仍使用原始 CP936 字节。

## 首次安装

1. 编译并烧录新的 STM32 固件。
2. 将工程中的 `assets/GB2312.FNT` 原样复制到 SD 卡根目录，目标路径必须是 `/GB2312.FNT`。
3. 插入 SD 卡后重启开发板。
4. 固件发现 W25Q64 字库头无效或全量 CRC32 不匹配时会自动安装。安装过程依次执行所需扇区擦除、SD 数据 CRC 校验、W25Q64 写入、W25Q64 全量回读 CRC 校验，最后才写入 32 字节有效头。
5. 安装成功后可以删除 SD 卡中的 `GB2312.FNT`；后续启动直接使用 W25Q64，不会重复擦写。

如果没有放置安装包或安装失败，系统不会卡死，仍会回退到原有约 1000 字的小字库，并把其余字符显示为 `?`。

## 掉电与兼容处理

安装时先写位图、最后提交有效头。安装中途掉电时，下一次启动会把该分区判断为无效，并在 SD 卡仍有安装包时重新安装，不会使用半包数据。

固件会同时检查 magic、版本、头长度、位图长度、CRC32、字形数量、字号和 bpp，并在每次启动时对外置位图执行一次全量 CRC32。只要重新生成过描述或位图，旧的 W25Q64 字库就不会被误用；把新的 `GB2312.FNT` 放回 SD 根目录并重启即可升级。

正常 LVGL 页面会让 QSPI 保持 Memory-Mapped 模式，字库通过带 D-Cache 的 `0x90000000` 窗口读取，并额外使用 8 项 RAM 字形缓存。NES 缓存开始前，现有缓存状态机会退出 Memory-Mapped 模式；在擦除、写入和校验期间，字库回调自动切换到 QSPI 间接读取，不会访问失效的映射窗口。NES 缓存结束或退出游戏后，主循环会重新恢复映射。映射失败时每 1 秒最多重试一次，避免 QSPI 异常拖慢 UI 主循环。

如果 W25Q64 字库不可用，主页面底部会提示 `Chinese font offline(...)` 和安装文件名。系统仍可进入所有页面，已有小字库中的汉字正常显示，其他字符转换成可见的 `?`。

## 重新生成字库

在工程根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File tools\generate_lvgl_gb2312_font.ps1
```

默认使用 `C:\Windows\Fonts\simhei.ttf`，生成：

- `Drivers/User/Src/lv_font_gb2312_16.c`
- `Drivers/User/Inc/ui_font_asset.h`
- `assets/GB2312.FNT`

更换字体文件时可传入 `-FontPath`。生成后必须重新编译 STM32 固件，并把新 `GB2312.FNT` 重新复制到 SD 根目录完成升级，不能只更新其中一侧。

## 建议测试文件名

- `冒险岛.nes`
- `导弹坦克.nes`
- `中文视频测试.avi`
- 含生僻 CP936 扩展字的文件名，用于确认未覆盖字符显示为 `?`

测试时同时确认进入目录、打开媒体和返回上级目录正常，确保 UTF-8 只用于显示，没有误传给 FatFs 文件接口。
