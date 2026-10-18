# linoragx

Linux 平台 Neo Geo 模拟器，NeoRageX（经典 Windows 模拟器）的最终构建向现代栈的移植重写：
Wayland + Vulkan，纯 C/C++，无重型依赖。

A Neo Geo emulator for Linux — a from-scratch modern rewrite (Wayland + Vulkan) of the
classic Windows emulator NeoRageX (final build, ~2006).

## 特性

- **CPU**：Motorola 68000（MAME 生成核）+ Z80（cpu_z80，栈与回调 RAM 一致性修复）
- **音频**：YM2610（YM2151 + YM3012 紧凑 FM 近似），含状态寄存器 D7 忙位、定时器握手
- **图形**：Vulkan 渲染器（离屏 + Wayland 窗口），GLSL 着色器构建期嵌入（`shaders_spv.h`），
  可选扫描线滤镜，整数缩放
- **卡带仿真**：MAME swlist XML 驱动加载；CMC42/CMC50/PVC 音频/程序解密；
  bootleg 保护窗口（kof2k3 系列 PX 解密、SMA/CTHD 等 RAM 窗）；看门狗；
  cart_audio 银行复用（与 MAME 语义对齐）
- **兼容性**：uni-bios 3.2/3.3 等多 BIOS，按游戏自动切换（如 mslug3 → asia-s3）
- **NeoRageX 遗产**：读取原版 `neorage.ini` 配置与键位（`--ini`），存档状态兼容框架
- **内置剖析器**：`PROF=1` SIGPROF 采样 + dladdr 符号（无需 perf/ptrace 权限）

## 构建

依赖：`libvulkan-dev`、`libwayland-client0`、`zlib1g-dev`、GCC（C11/C++17）。

```sh
make -j$(nproc)        # 产物 build/linoragx
python3 tools/embed_shaders.py   # 修改 shaders/*.glsl 后重新嵌入 SPIR-V
```

## 使用

```sh
linoragx --rom <游戏.zip> [--bios <BIOS目录>] [--ini neorage.ini]
         [--scale N] [--scanlines] [--no-audio]
         [--headless --frames N --dump out.bmp]   # 无窗口跑 N 帧并截屏
         [--state FILE | --save-state FILE]       # 读/写存档状态
```

窗口模式需要 Wayland 会话（`WAYLAND_DISPLAY` 环境变量）。ROM 与 BIOS 需自备
（Neo Geo BIOS：`NEO-GEO.ROM`、`NG-SFIX.ROM`、`NG-LO.ROM`、`sm1.sm1` 等）。

## 性能

单核 headless 渲染（kof2000，320×224）：

| 阶段 | fps |
|---|---|
| 初始移植 | 77 |
| 热路径 `getenv` 缓存化（曾占 CPU 56%） | 909 |
| `-O3 -march=native` | 1143+ |

68K/Z80/YM2610 按 chunk 交织调度，解密与 CRC 预解码多线程并行。

## 兼容性实测

208 张游戏卡批量测试（f=8000/12000 帧截图分类，MAME 0.285 为金标准对照）：

- **194** 张正常启动出画
- **7** 张与 MAME 行为一致（等投币黑屏 / 超慢启动，非缺陷）
- **2** 张超慢启动（kof2001/kof96，f≥12000 出画）
- **4** 张 bootleg 边缘卡遗留（kf2k3pl/kf2k3upl/kf2k2ps2/kof99ae，保护仿真未完成）

## 目录结构

```
src/        模拟器核心（neogeo.c 总线/卡带, video.c PIFB, render_vulkan.c,
            ym2610.c, z80_wrap.cpp, rom.c, config.c, wl_input.c, main.c）
vendor/     MAME m68k 核、cpu_z80、softfloat
shaders/    GLSL 源（构建期嵌入）
generated/  SPIR-V 嵌入头
tools/      构建辅助脚本
```

## 声明

本仓库不含任何 ROM、BIOS 或二进制固件。Neo Geo 与 NeoRageX 相关权利归各自持有者。
