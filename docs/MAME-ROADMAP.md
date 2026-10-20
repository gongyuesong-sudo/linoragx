# linoragx 架构复审：向 MAME 路线对齐

日期：2026-10-20。基线：commit 7adf645（部署 md5 6fbc7048，batch29：OK 59 / GLITCH 45 / DARK 42 / BLACK 29 / FEW 26 / BIOS 6）。

## 1. 现状架构

纯 C 单体（约 6000 行），无设备框架：

| 模块 | 行数 | 职责 |
|---|---|---|
| src/neogeo.c | 2906 | 总线 r16/w16 大 switch、锁存器、banking、SRAM、保护（ctab 静态表）、帧调度 ng_run_frame |
| src/video.c | 271 | 整帧渲染 ng_render（bg fill → 精灵 → 固定层），逐扫描线列表已按 MAME |
| src/render_vulkan.c | 923 | Vulkan 渲染器，三后端 wayland/x11/headless，WSI + 离屏 dump |
| src/ym2610.c | 177 | YM2610 简化核（FM 不全、ADPCM-A 无回放、无 YM3802） |
| src/z80_wrap.cpp | 87 | AAE Z80 核封装 |
| vendor/ | — | Musashi m68k、AAE z80、softfloat |
| src/rom.c + swlist | 192 | neogeo_swlist.xml 加载器（MAME softwarelist 子集） |
| src/ctl.c | 367 | localhost TCP 控制通道（LINORAGX_CTL） |
| tools/mcp_linoragx.py | — | MCP stdio 服务器（15 工具，已端到端验证） |

帧调度：`ng_run_frame` 以 16 周期 chunk 交替跑 68K/Z80/YM（比例 1 : 1/3 : 2/3，与 MAME 时钟比一致），帧末一次性 `ng_render` 整帧。

## 2. 与 MAME（src/mame/snk/neogeo*.cpp）的差距矩阵

| # | 维度 | MAME 做法 | 现状 | 差距影响 | 阶段 |
|---|---|---|---|---|---|
| G1 | 渲染时机 | screen_device 逐扫描线更新（sprite_line_timer→update_partial+parse_sprites），帧中改 VRAM/CRAM 立即生效 | ~~帧末整帧渲染~~ 已改逐行交织（f4f8844） | 光栅效果游戏正确性 | ✅P1 |
| G2 | 总线解码 | memory_map 声明式（map("maincpu") + bank regions + delegates） | r16/w16 手写大 switch | 可维护性；新保护移植易错 | P3 |
| G3 | 保护 | 每板独立 device 类（prot_kof98/prot_cmc/pvc MCU 仿真） | ctab 静态解密表近似 | mslug5 等 PVC 卡 fix 层残留花屏（静态表在 C 尾区不准） | P4 |
| G4 | 音频 | 完整 ym2610 + ym3802 + ym2151 | 177 行简化核 | PCM2 卡（mslug4/rotd/pnyaa）ADPCM-B 无回放；音质 | P2 |
| G5 | MCU | HD38080/TMP87PH46U/I8039 程序仿真 | 无 | mslug5 彻底修复的前置 | P4 |
| G6 | 状态存档 | device serialize | 手写 NRXSTATE09 | 新增设备后同步成本 | P5 |
| G7 | 视频输出 | soft/opengl + 窗口/全屏 | Vulkan 三后端已有；全屏缺失 | 用户新需求 | P0 |
| G8 | 联网 | MAME netplay（延迟锁步） | 无 | 用户新需求 | P6 |

已对齐项（勿动）：帧时序 202752 周期/帧、Z80/YM 时钟比、精灵优先级列表 + 链式状态机、固定层列主序 0x7000、CRAM 位格式、绘制顺序（bg→sprites→fix）、看门狗、IRQ2 display counter、LS74 vblank 语义、watchdog 3244030 master cycles。

## 3. 路线图

- P0 视频输出补全（本轮）：X11 `_NET_WM_STATE_FULLSCREEN` + Wayland `xdg_toplevel_set_fullscreen`，F11 / Alt+Enter 切换，`--fullscreen` 启动参数；letterbox 保持 320:224 比例（viewport 内缩，不改着色器——本机无 glslc，generated/shaders_spv.h 不可再生）。
- P1 逐扫描线渲染（MAME 核心语义）：ng_render 拆为 ng_begin_frame/ng_draw_line；在 ng_run_frame chunk 边界按 scanline 推进逐行画。行映射：帧首 = MAME 行 240（vblank 起点），可见行 = 本帧 scanline s ∈ [40,264) → fb 行 s-40（VTOTAL 0x108、VBEND 0x10、VBSTART 0xF0）。完成后重新启用 CRAM bank（vblank 时刻快照）。验收：bstars2 bank 行为 + 208 卡批测零回归。
- P2 音频对齐：vendor MAME ym2610/ym3802/ym2151（写 device 框架 shim），补 ADPCM-A 回放与 PCM2。
- P3 总线表化：r16/w16 重构为区间表（基址/掩码/handler），行为逐字节不变，批测护栏。
- P4 MCU 仿真：vendor HD38080 核 + pvc_mslug5 程序仿真，替换静态 PVC 表（mslug5 fix 层根治）；随后 TMP87PH46U。
- P5 存档 serialize 化：随 P1/P4 的新状态一并做。
- P6 联网对战：LAN lockstep（见 §4）。

## 4. 联网对战设计（P6，lockstep）

Neo Geo 是 2 玩家街机板，双机各跑一份确定性仿真、只同步输入，无需传视频。本项目确定性是硬约束（全缓冲 calloc 零初始化已保证，md5 复验过），lockstep 成立：

- 角色：host（P1 本地）/ client（P2 本地），TCP 长连接（默认端口 55300，`--net host|client <ip>[:port]`）。
- 每帧：双方交换 (frame_no, P1 掩码, P2 掩码) 8 字节帧；收齐对端输入后才跑本帧（延迟 N=2 帧吸收抖动，LAN RTT 1–5ms 下无感）。
- 输入注入点：ng_set_button 已有；netplay 把远端位掩码直接写入 buttons[]，本地按键只写本机玩家位。
- 掉线保护：对端超时 2s → 打印告警并暂停（不静默改输入）。
- MCP/CTL 不受影响（单机路径不变）。

## 5. 需求映射与实施状态

| 用户需求 | 状态 |
|---|---|
| 按 MAME 路线实现 | P1 逐扫描线渲染已落地（f4f8844）；P2-P5 见 §3 |
| x11/wayland 窗口 + 全屏可切换 | 已落地（e5f12d3）：F11/Alt+Enter 切换、--fullscreen、letterbox 保比例；Wayland 路径为标准 xdg 调用（本机无 Wayland 会话未实测） |
| Vulkan 加速 | 已有（render_vulkan.c，RADV 实测） |
| MCP 服务 | 已有（tools/mcp_linoragx.py + CTL 通道） |
| 多机联网游玩 | 已落地（da2a35a）：--net host/client lockstep，本机回环 6200 帧双端像素一致验证 |

## 6. 已落地实施记录

- P0 全屏（e5f12d3）：X11 EWMH `_NET_WM_STATE_FULLSCREEN`（进出时放宽/恢复尺寸约束）；Wayland `xdg_toplevel_set_fullscreen`；letterbox 用 viewport 内缩实现（着色器冻结不可重编译）；实测 2560x1440 全屏黑边正确。
- P1 逐扫描线渲染（f4f8844）：MAME `sprite_line_timer_callback` 语义——每行开始时画上一行 + parse_sprites 当前行，帧中 VRAM/CRAM 写入即时可见。`ng_set_fb` 装载 fb 后 `ng_run_frame` 在 chunk 边界逐行绘制（fb 行 = scanline-40）；`ng_render` 保留整帧路径给 headless/ctl dump。验证：headless dump md5 与旧版一致；kof2000/sdodgeb 窗口交织 vs headless 整帧像素零差异。顺带删除重复 tile pass。
- P6 联网（da2a35a）：src/netplay.c，8 字节/帧 {frame,P1,P2}，TCP_NODELAY，握手带游戏 CRC 防错卡，10s 超时判掉线；host=P1/client=P2；LINORAGX_INPUT 加 4 字段形式（frame:player:bit:hold）。回环验证：双端 6200 帧锁步同帧率（773.5fps）、dump 互相一致且区别于无输入基线。
- 输入注入时机教训：BIOS coin 屏只在 attract 循环相位读投币口（f<100 注入 coin 无效果，f=5000 有效）——测输入因果要用 f≥5000 或 CTL press。
