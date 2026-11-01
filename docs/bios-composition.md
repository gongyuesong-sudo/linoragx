# Neo Geo BIOS 构成分析与 linoragx 对标

分析对象：`/home/gill/linoragx/` 全部 BIOS 固件。方法：字节序修正（Neo Geo ROM 为 lo/hi 字交错存储，
线性大端镜像 = 相邻字节两两交换）→ capstone M68000 线性反汇编 + 块级相似度矩阵 + 运行时探针交叉验证
（EXCPROBE/ACK3C/RAMWIN）。Ghidra 11.3.1 headless（68000:big:32，加载基址 $C00000）反编译补充见文末。

分析用镜像与脚本：`~/.hermes/cache/scratch/bios_dis/`（`bios_struct.py`、`bios_matrix.py`）。

## 1. BIOS 清单与家族关系

| 文件 | 大小 | 家族 | 说明 |
|---|---|---|---|
| NEO-GEO.ROM | 128K | 官方日版 | 金标基准（MAME wh2/mslug4 等官方卡用） |
| asia-s3.rom | 128K | 官方 s3 亚版 | mslug3/b2b/fightfev 首选 pref |
| sp-s.sp1 / sp-e.sp1 | 128K | 官方 SP 系 | 早期机 BIOS |
| usa_2slt.bin | 128K | 官方 2-slot 美版 | |
| vs-bios.rom | 128K | VS 系统（独立血缘） | 与官方家族仅 2/32 块相同 |
| uni-bios_3_2.rom | 128K | 第三方改写 | linoragx 默认 BIOS |
| neodebug.bin | 128K | 官方调试 BIOS | 与官方家族 13/32 块相同 |
| NG-LO.ROM | 64K | 非 68K BIOS（AES LO 芯片） | 向量表无效，排除 |
| sm1.sm1 | 128K | Z80 音频 BIOS | 另册（Z80，capstone zdis.py） |
| sp1v1.bin | 128K | ADSP 采样+SH1 代码 | 另册 |
| sfix.sfix / NG-SFIX.ROM | 128K | SFIX 精灵芯片固件 | MAME 仅数据使用，不执行 |

4K 块一致率矩阵（相同块/32）：官方五变体两两 18-20/32（同源变体）；vs-bios、uni-bios 与官方家族仅 2/32
（仅向量表+公共头）；neodebug 与官方 13/32。

## 2. 向量表协议（8 张 68K BIOS 完全一致）

```
SSP=$10F300  RESET=$C00402  NMI=$C00408  未用异常=$C00426(iret stub)
IRQ1=$C00432  IRQ2(VBlank)=$C00438→jmp $C11646  IRQ3=$C0043E→jmp $C118CA
服务跳板：$C00444→$C112D2  $C0044A(TimerC)→jmp $C1167A  $C00450→$C117CE  $C00458→$C11820
```

要点：低区 $C00402-$C004FF 全部是 6 字节 `jmp` 跳板（Ghidra 反汇编证实），一一对应向量表项。
BIOS 的 VBlank 服务体挂在 IRQ2 向量（$C00438）上，游戏 VBlank handler 尾部
`jmp $C00438` 主动进入；$C0044A（TimerC 服务 $C1167A）是游戏协程让出点，游戏以
`jsr $C0044A` 挂起/恢复协程。linoragx 运行时实测与此一致（wh2 handler $20009C:
tst $10FD80; bmi; jmp $C00438）。

## 3. 内存映射（NEO-GEO.ROM 反汇编 I/O 引用频次）

| 地址 | 引用数 | 用途 |
|---|---|---|
| $300000 | 134 | 输出锁存（LED/coin counter/watchdog $300001） |
| $3A0000 | 128 | 板级控制 |
| $10F000 | 93 | work RAM 变量区（a5=$10F300 基址） |
| $380000 | 55 | SRAM |
| $3C0000 | 46 | 视频寄存器（$3C0008 IRQCTRL、$3C000C ack、$3C0004/5 display counter） |
| $320000 | 25 | 输入端口（$320000 P1、$320001 AUDIO 轮询） |
| $340000 | 5 | 固定 bank 切换 |

## 4. NEO-GEO.ROM 构成地图（capstone 53848 条指令 + 字符串聚类）

| 区域 | 内容 |
|---|---|
| $C00000-$C00100 | 异常向量表 |
| $C00080 | 版本串 "NEO-GEO SYSTEM PROGRAM VERSION 0 COPYRIGHT BY SNK CORPORAT..." |
| $C00402-$C004FF | 异常入口 stub 群（RESET/NMI/服务跳板 $C00438/$C0044A） |
| $C02000-$C03FFF | 硬币画面字符图案数据（非文本） |
| $C04000-$C10FFF | 自检 + 硬币画面 + demo 状态机（uni-bios 全重写区） |
| $C11000-$C157FF | 核心服务框架：VBlank 服务体 $C11646、TimerC 服务 $C1167A、协程调度 |
| $C15800-$C197FF | 任务派发器 $C1871E、主循环门 $C1870C（与官方 86% 同源） |
| $C19800-$C1FFFF | 字体/文本画面资源（uni-bios 重写区） |

高频子程序入口（jsr/jmp 目标计数）：$C13A3C×40、$C16AAA×38、$C1870C×34、$C1871E×28、$C1334C×27、$C17D34×24。

## 5. 关键机制（运行时探针交叉验证）

- **主循环门**：$C1870C 置 $10FE8C=1 → $C18714 `tst.b $10FE8C; bne` 等待服务体清零。服务体 $C11668
  清 $10FE8C 并写 $3C000C=#6 ack。这是 BIOS 每帧一次的节流阀。
- **VBlank 服务体** $C11646-$C11678：`tst $10FEE3` 门控 → movem 保存 → jsr $C1871E（任务派发）→
  清 $10FE8C → `move.w #$6,$3C000C` ack。
- **任务派发器** $C1871E：遍历 $10FF00 任务队列，按 d0<14 查跳转表 $C1877E（14 项），jsr (a3)。
- **游戏侧 VBlank handler**（wh2 例）：work RAM $20009C：`tst.b $10FD80; bmi; jmp $C00438`。
- **FREE PLAY/attract 计时**：BIOS 主循环任务链驱动，金标 wh2 约 400f/步；uni-bios 下 wh2 计时
  546f/步且 attract demo 脚本注册链断裂（脚本长度字段恒 0）→ attract 恒黑。

## 6. uni-bios vs 官方差异图（difflib 字节级，2K 粒度）

| 区域 | 一致率 | 含义 |
|---|---|---|
| $C00000-$C02000 | 5% | 头+版本串重写（fast-boot 入口） |
| $C04000-$C11000 | 0% | 自检/硬币画面/demo 全重写 |
| $C11000-$C15800 | 高 | 服务框架保留 |
| $C15800-$C19800 | 86% | 任务派发框架同源（高频入口地址逐一相同） |
| $C19800-$C20000 | 1% | 字体/文本重写 |

结论：uni-bios 保留官方向量协议与任务调度骨架（高频入口地址逐一相同），重写自检/硬币画面/字体区。

## 7. 对标结论：BIOS 敏感卡机制

部分官方卡（mslug4、bstars、wh2、b2b、fightfev、mslug3）的行为依赖官方 BIOS 自检/硬币画面区
（$C04000-$C11000）的初始化副作用，uni-bios fast-boot 跳过该路径导致：
- mslug4：音频命令流首条 SL1 命令时序错位（f25 vs 金标 f237）→ attract BGM 静默
- bstars：VRAM 0xF0-0xFF 缩放链 sprite 表未初始化 → attract 花屏
- wh2：attract demo 脚本注册链断裂（长度字段恒 0）→ attract 恒黑
- mslug3：uni-bios 下 boot 循环进 RAM test → 需 asia-s3
- b2b/fightfev：调用 s3 BIOS 装在 $11012 的自检 stub → 需 asia-s3

linoragx 对策：per-card BIOS pref 表（`src/neogeo.c` swlist 匹配块内 `load_bios_pref`），
命中卡强制 NEO-GEO.ROM 或 asia-s3.rom。wh2/wh2j/wh2h 于 task-232 加入（NEO-GEO.ROM，
验证：默认路径 f6000=74 色、f20000=74、f21500=47 场景推进；uni-bios 恒 1 色）。

## 8. Ghidra 反编译补充（11.3.1 headless，68000:BE:32:default，基址 $C00000）

脚本 `~/.hermes/cache/scratch/bios_dis/export_bios.py`：全量线性反汇编 + 调用引用建函数 +
KEYS 地址强制建函数 + 导出函数清单 JSON 与关键服务伪代码。产物 `ghidra_out/funcs_*.json`、
`pseudocode_*.c`。

函数规模：官方 MVS 各版 258-283（代码区 $C04000 + $C10000-$C1BFFF，约 32K）、uni-bios 793
（自检/画面/字体全重写，代码翻倍）、neodebug 340。

关键服务伪代码定案（NEO-GEO.ROM，uni-bios 3.2 逐字节同构）：

- **VBlank 服务体 $C11646**：`$3C000C=4` ack → 喂狗 `$300001` → 若 `$10FEE3==0`：任务派发
  `$C1871E` + TimerC 服务 `$C1167A` + 清主循环门 `$10FE8C` → VBlank 计数 `$10FEE4++`。
- **TimerC 服务 $C1167A**：TimerC 计数 `$10FE88++` → 子服务 `$C126B4`/`$C1850E` →
  `$10FEE1` 倒计时，到 0 且 `$10FD80` 高位置位则跑 `$C116C8`/`$C1183C`；`$10FD82!=0 &&
  $10FDAF==2` 时跑 `$C12BE6`。
- **任务派发器 $C1871E**：重入锁 `$10FDC2`；遍历队列 `$10FF00`（块首字节=下一块指针，块内
  `[1]`=任务 id），id<0xE 查跳转表 `$C1877E`（14 项 4 字节）执行；id≥0xE 停止（游戏任务由
  游戏自身调度器执行）。
- **跳转表 14 项 = BIOS 服务原语**：nop×2、任务参数字节写 `[a4+2]`、word 拷贝、流读取
  （`[a1]` + 指针 `$10FEC6` 自增）、流回退 `-4`、word memset、递增 memset、文本 VRAM 写原语
  `$C18838`（按 d5 标志 4 模式：字节查找/字查找/定值填充/递增填充）、`[a6]=[a1]+d6`、
  tile 拷贝 `$C18876/$C188DC`（`jsr (a2)` 回调）。
- **VRAM 清屏任务 $C13A3C**：`$3C0004=1`、`$3C0000=$7000` 写使能，循环 0x500 次写
  `$3C0002=$20`（display counter）并喂狗 `$300001=$20`。
- **向量服务 handler**：默认异常 `$C11002`（`$3C000C=7`+`$3C0006=$4000`+`$C11A84`）；IRQ3
  `$C118CA`（ack=2）；boot 清 RAM `$C112D2`（`$10FEE1=$FF`、清 `$10FD80` bit7、RAM $402000
  起清零）；BCD 倒数服务 `$C117CE`（`$10FDB0/$10FDB1`，机台计时）；`$C11820`（`$10FEE0=0`、
  `$10FD81=3`、`$C1299A`）。

对标要点：linoragx 已实现的 VBlank ack/主循环门/协程 resume 语义与上述伪代码一致；未建模的
服务原语（文本 VRAM 写 4 模式、流指针 `$10FEC6`、BCD 计时 `$10FDBx`）仅 BIOS 自身画面使用，
游戏卡带路径不经过，无需补。
