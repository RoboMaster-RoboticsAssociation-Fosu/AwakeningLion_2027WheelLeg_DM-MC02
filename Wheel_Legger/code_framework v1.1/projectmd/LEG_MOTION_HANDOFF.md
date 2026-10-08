# 倒地自起封装交接

最新工作快照见 [2026-10-08 自起工作交接](WORK_HANDOFF_2026-10-08_RECOVERY.md)。当前参数为 0.39 m 扫腿、1000 ms 收腿、500 ms 交接；上一轮固件构建对应修改前配置。

交接日期：2026-10-08。依据当前工作区源码整理，相关改动尚未提交。当前动作改为转到 2.8、收腿、两腿一起转到 1.7 后交接 NORMAL，保留提前收腿窗口和现有输出保护。最新验证结果在下方单独记录；此前接口封装及等效回放仅作历史记录。尚未烧录或实车验证。

## 接手顺序

1. 调用或修改单腿动作时，先读 [接口用法](LEG_MOTION.md) 和 [leg_motion.h](../User/Controller/leg_motion.h)，核对 phi0 弧度、方向及 start 的使用。
2. 修改自起阶段、重启、超时或交接时，读 [chassis_recovery.h](../User/Controller/chassis_recovery.h) 和 [chassis_recovery.c](../User/Controller/chassis_recovery.c)。修改底盘接入时，再读 [chassis_task.c](../User/APP/chassis_task.c) 的 chassis_recovery_control()、falling_down_detect() 和 VMC_translate()。
3. 控制变化完成后记录实车角度与腿长跟踪、阶段切换、交接结果及任务周期，再依据数据调整轨迹时间、增益或超时。

## 文件与职责

| 文件 | 当前职责 |
| --- | --- |
| [leg_motion.h](../User/Controller/leg_motion.h)、[leg_motion.c](../User/Controller/leg_motion.c) | 单腿命令、反馈、输出和上下文；phi0 定向选路、连续角展开、同步插值、PD 与重力近似、完成/超时判断。公共函数为 LegMotion_Run。 |
| [chassis_recovery.h](../User/Controller/chassis_recovery.h)、[chassis_recovery.c](../User/Controller/chassis_recovery.c) | 自起参数、双腿上下文、阶段及计时；首次启动、扫腿/收腿/等待/第二段转动/保持、共享截止、交接及结果。公共函数为 ChassisRecovery_Run 和 ChassisRecovery_Reset。 |
| [chassis_task.c](../User/APP/chassis_task.c) | 反馈/输出适配、模式切换、开启判姿和正常倒地检测、最终清零及 VMC 映射检查；全局 chassis_recovery 供调试。 |
| [Keil 工程](../MDK-ARM/CtrlBoard-H7_IMU.uvprojx) | Controller 分组注册两个模块；沿用 User/Controller 包含路径。 |
| [leg_motion_verify.js](../mdk_check/leg_motion_verify.js) | 编译实际单腿模块、自起模块及底盘适配验证。 |
| [leg_motion_verify.c](../mdk_check/leg_motion_verify.c)、[chassis_recovery_verify.c](../mdk_check/chassis_recovery_verify.c)、[recovery_verify.c](../mdk_check/recovery_verify.c) | 分别验证单腿命令、自起公共模块、底盘接入及最终输出保护。 |
| [LEG_MOTION.md](LEG_MOTION.md) | 五个命令参数、逐字段赋值示例和返回状态的日常参考。 |

旧 spin 控制函数、相应运行状态和禁用的旧级联倒地控制实现已替换。LQR、LESO、遥控、CAN 映射和 RTOS 周期沿用现有实现。

## 接口中已经确定的约定

- 单腿每控制周期调用一次，左右腿各自保留初始清零的上下文；模块无 HAL、CAN、阻塞延时和动态分配。
- 命令为 direction、target_phi0_rad、duration_ms、length_m、timeout_ms。目标直接使用 VMC 的 phi0，单位 rad，竖直向下为 π/2；反馈速率为 rad/s，腿长为 m，腿长速率为 m/s。
- 正方向是 phi0 增大，负方向是 phi0 减小。左右镜像已在反馈和输出端处理，两腿共用方向约定。目标以 2π 为周期，等价姿态不额外转圈；旧 angle_mode、angle_deg 及相对整圈模式已移除。
- start 只在开始动作的当周期置 1；置 1 就重新捕获实际角度、腿长和时刻，即使命令相同也会重启。后续置 0，命令被锁存，command 可传 NULL。一直置 1 会使轨迹每拍重新开始。
- duration_ms 是角度和腿长共同的线性参考变化时间，0 表示即时参考；实际到位仍需满足容差。timeout_ms 是从启动计时的总截止时间。
- 内部连续角仍使用 q=phi0−π/2，连续误差用于跟踪和完成判断，重力近似仍使用物理姿态。相邻采样间实际转动须小于 π，展开算法无法识别漏采样后的额外整圈。
- DONE 在参考变化时间结束、实际角误差小于 0.25 rad 且腿长误差小于 0.02 m 后锁存，继续输出以保持目标，不要求角速度为零。TIMEOUT/INVALID 输出零并锁存，新的 start 才能重新执行。
- 完成后持续保持，不因原 timeout_ms 到期变成超时；保持期间反馈无效仍会进入 INVALID。调用者停止调用时须主动清除底盘输出。

控制增益、限幅及重力参数集中在 leg_motion.h，详细数值见接口说明。F0 单位 N，Tp 单位 N·m；当前 PD 使用反馈速率阻尼，未增加参考速度前馈。

## 自起模块与底盘接入

`ChassisRecovery_Run(ctx, input, now_ms, output)` 的输入包含 enabled、pitch_rad 和 leg[2] 单腿反馈，输出 leg[2] 的 F0/Tp。返回状态依次为 CHASSIS_RECOVERY_IDLE、RUNNING、HANDOFF、DONE、FAULT（均带 CHASSIS_RECOVERY_ 前缀）。上下文持有 motion[2]、phase[2]、result、entry_tick 和 handoff_tick；phase 为 CHASSIS_RECOVERY_SWING、RETRACT、WAIT_ALIGN、ALIGN、HOLD（同样带此前缀）。左右腿下标分别为 0、1，与底盘腿下标一致。

- 模块内部明确顺序调用左腿、右腿；左腿失败立即返回，不推进右腿。右腿失败时清除左腿已计算的本拍输出。
- ChassisRecovery_Reset() 复位阶段和计时，保留单腿上下文供诊断。FAULT 锁存至复位；禁用调用会复位并返回 IDLE、清零输出，下次启用重新捕获两腿反馈。DONE 后继续调用仍保持两腿输出，由底盘适配交回正常控制。
- `falling_down()` 和 `falling_to_down()` 均只调用 `chassis_recovery_control()`。适配函数逐字段准备反馈，将模块输出回写两腿，清零轮输出，并根据结果切换底盘模式。
- 失败或禁用时适配函数完成腿虚拟输出、关节力矩和轮输出的最终清零。开启判姿、正常状态倒地检测、VMC 映射后检查保留在底盘原调用位置。
- `recovery_output_pending` 在 VMC 映射时读取并清除，确保交回 NORMAL 的最后一帧仍检查自起输出。

本次保留单腿接口、PD 增益及输出限幅，修改自起阶段和终点。动作共享截止为 600000 ms，交接超时为当前配置 500 ms；两腿收完后增加同步启动的第二段转动。

## 底盘自起阶段

```text
OFF→ON 位姿检查
  ├─ 位姿符合正常范围 → NORMAL
  └─ 位姿倒地 → FALLING_DOWN
                  每腿独立：SWING → RETRACT → WAIT_ALIGN
                  两腿均 WAIT_ALIGN → 下一拍同时 ALIGN
                  每腿 ALIGN 完成 → HOLD
                  两腿均 HOLD → FALLING_TO_NORMAL
                                  ├─ 下一拍起，交接条件满足 → NORMAL
                                  └─ 交接超时 → ZERO_FORCE

在线 NORMAL 检测到倒地 → ZERO_FORCE，等待再次 OFF→ON
自起超时、无效反馈或自起力矩映射异常 → 当周期 ZERO_FORCE
```

### SWING：扫腿

参数集中在 chassis_recovery.h：SPIN_SWEEP_DIR 为负方向，SPIN_TARGET_PHI0_RAD=2.8f rad，参考变化 4000 ms，目标腿长 0.39 m。初始实际腿角、腿长作为同步插值起点。

满足以下任一条件即可收腿：

- 原提前窗口：归一化 q=phi0−π/2 位于 [-0.9, 0] rad，包含边界；启动时已在窗口也立即收腿。正方向配置的窗口仍为 [0, 0.9]。
- 正常终点：斜坡已经结束，且当前连续角实际值与目标的误差严格小于 SPIN_ANGLE_WINDOW=0.05 rad。不额外要求腿长先到 0.39 m，也不能仅靠单腿原 0.25 rad 的 DONE 容差切阶段。

正常终点当拍先更新单腿反馈并验证，再以该拍实际角和腿长重启收腿命令；当拍输出采用收腿命令的结果。

### RETRACT 与 WAIT_ALIGN：固定角度收腿及等待

进入收腿时仅捕获一次实际 phi0，之后即使离开窗口也保持此目标；腿长参考用 1000 ms 从该拍实际长度变化到 0.139 m。沿用单腿 DONE 判据：斜坡结束、连续角误差小于 0.25 rad、腿长误差小于 0.02 m。

完成后进入 WAIT_ALIGN，继续调用已完成的单腿上下文保持输出。两腿均为 WAIT_ALIGN 的当拍不启动第二段；下一周期同时启动，两腿使用同一个 now_ms，各自重新捕获实际角度与长度。等待期间沿用 DONE 锁存，不新增速度条件。

### ALIGN：收腿后一起转向 1.7

SPIN_ALIGN_TARGET_PHI0_RAD=1.7f rad，SPIN_ALIGN_TIME_MS=1000U，目标腿长保持 SPIN_RETRACT_LENGTH=0.139f。

启动时按各腿实际 phi0 选择到 1.7 的最短方向并锁存：正常从 2.8 向负方向转；提前收腿角小于 1.7 时向正方向转。同姿态不额外绕圈，半圈按原扫腿方向。后续反馈改变不会重新选方向或重启斜坡。

单腿 DONE、斜坡结束以及当前连续角误差小于 0.05 rad 同时满足，才进入 HOLD。即使单腿 DONE 已锁存，实际角还差 0.1 rad 也要继续等待。先完成的腿保持，直到另一腿完成。

### HOLD 与 FALLING_TO_NORMAL：保持和交接

两腿都 HOLD 当拍只切入交接并开始 500 ms 计时；下一周期开始检查正常平衡条件。期间继续调用两个完成的单腿上下文，start=0、command=NULL。

| 条件 | 当前判据 |
| --- | --- |
| 机身俯仰 | 有限，且绝对值小于 0.2 rad。 |
| 每条腿姿态 | phi0 与 1.7 rad 的圆周误差小于 0.05 rad，且原始 phi0 在 [0.4, 2.5] rad 内。 |
| 每条腿腿长 | 距 0.139 m 的误差小于 0.02 m。 |
| 每条腿腿长变化率 | 绝对值小于 0.05 m/s。 |

先检查就绪，再检查交接超时；截止边界满足条件仍可进入 NORMAL。原始 phi0 范围避免仅因圆周姿态等价交回后立刻被倒地检测中止。两腿收完或第二段参考时间结束，本身都不会直接切入 NORMAL。

### 超时和清零

扫腿、收腿、等待及第二段转动共用进入自起时开始的 600000 ms 总截止。每次启动新动作只分配剩余时间；第二段不会延长截止。运行中先检查总截止，到达边界即中止；交接的 500 ms 另行计算。

左腿失败立即返回，不推进右腿阶段或动作上下文；右腿失败清除两腿当拍 F0/Tp。底盘中止路径同拍清零腿虚拟输出、映射关节力矩、轮力矩及电流。OFFLINE 或 ZERO_FORCE 时 VMC_translate() 清零并返回。

recovery_output_pending 保留到当拍 VMC 映射后消费；即使适配函数已经将模式改为 NORMAL，最后一帧自起映射仍检查力矩是否有限。

## 参考时序：从 phi0=-0.57 开始

假设两腿同步、实际完全跟踪参考、初始腿长 0.139 m，且未触发提前窗口：

| 时刻 | 动作与参考 |
| --- | --- |
| 0～4 s | 连续 phi0 从 -0.57 减小至 2.8−2π≈-3.483185 rad；VMC 显示跨越 -π/+π 后到 2.8。腿长从 0.139 增至 0.39 m。 |
| 4～5 s | 捕获实际 phi0≈2.8，角度不变，腿长从 0.39 收至 0.139 m。 |
| 5 s | 两腿均 WAIT_ALIGN；下一控制周期同时启动第二段。 |
| 约 5.001～6.001 s | 两腿从 2.8 向负方向转到 1.7，目标腿长 0.139 m。 |
| 约 6.001 s 后 | 两腿均 HOLD 当拍进入交接；下一周期条件满足后 NORMAL。 |

以上按 1 ms 控制周期计算，不含实际跟踪滞后。若提前收腿，或两腿完成时间不同，第二段开始时刻以较晚一腿收完后的下一周期为准。

## 验证记录与复现

### 2026-10-08：当前参数的交接复验

本轮只更新交接资料，并按当前 0.39 m 扫腿、1000 ms 收腿、500 ms 交接配置重跑主机测试：739 + 1047 + 230 = 2016 项断言通过。当前参数未重新做 Keil 固件构建；下节构建和产物对应上一轮 0.35 m、500 ms 收腿、1000 ms 交接配置。详见工作快照。

### 历史：2026-10-08 实现 2.8 → 收腿 → 同步转向 1.7

| 验证 | 结果与边界 |
| --- | --- |
| 主机 C 联合验证 | 单腿 739 项、自起模块 1047 项、底盘接入 230 项，共 2016 项断言通过。直接编译真实模块，覆盖反馈轨迹、跨界、提前窗口、双腿不同步、同步启动、最短方向、0.05 rad 边界邻域、阶段锁存、重启、共享截止、交接与故障顺序。 |
| 底盘保护 | 实际适配、清零、检测及 VMC 函数提取编译；包括交回 NORMAL 当拍映射异常的最终清零。 |
| VMC/Jacobian | 两个数值脚本通过；几何与固件不同，仅验证公式。 |
| Keil ArmClang 6.16 隔离完整重编 | 0 Error(s)、0 Warning(s)，退出码 0；[构建日志](../build/recovery_align_keil/build.log)。 |
| 工具环境 | 沙箱内 gcc 无法启动，Keil armasm 报 DLL 重定位错误；改在沙箱外执行后主机测试及固件构建均通过，没有遗留被阻碍的检查。 |
| 硬件 | 未烧录，未实车验证。 |

本次隔离工程为 [recovery_align_check.uvprojx](../build/recovery_align_keil/recovery_align_check.uvprojx)，产物为 [CtrlBoard-H7_IMU.hex](../build/recovery_align_keil/Objects/CtrlBoard-H7_IMU.hex) 和 [CtrlBoard-H7_IMU.axf](../build/recovery_align_keil/Objects/CtrlBoard-H7_IMU.axf)。构建从当前权威工程生成；新模块已在权威工程注册，本次没有新增源文件。

此次有意改变动作时序，因此以新轨迹和保护条件为验收依据。下方旧等效回放证明的是此前接口封装，不代表本次动作与旧版等效。复现当前隔离构建时，使用后文命令并将工程路径改为 build/recovery_align_keil/recovery_align_check.uvprojx，日志路径相应改为 build/recovery_align_keil/build.log。

### 历史：2026-10-08 phi0 接口与独立自起模块

| 验证 | 结果与边界 |
| --- | --- |
| 主机 C 联合验证 | 单腿 739 项、自起公共模块 197 项、底盘接入 198 项，共 1134 项断言通过，退出码 0；日志见 [recovery_phi0_verification.log](../build/recovery_phi0_verification.log)。单腿用例包含 ±3、±10 个 2π 的等价目标浮点舍入回归，以及非等价小角度的定向选路。 |
| VMC/Jacobian 数值检查 | 通过；脚本几何与固件不同，仅验证公式。 |
| 编码与行尾 | 原文件编码及行尾保持；新自起模块 UTF-8 无 BOM、CRLF；三份 Markdown 为 UTF-8 BOM、CRLF。 |
| 修改前后同反馈回放 | 517 个场景、2263 帧、88257 项字段对比，差异 0，最大浮点误差 0。报告见 [report.json](../build/recovery_phi0_equivalence/report.json)。 |
| 隔离 Keil ArmClang 6.16 完整构建 | 0 Error(s)、0 Warning(s)，退出码 0。日志见 [build.log](../build/recovery_phi0_keil/build.log)。 |
| 权威工程产物保护 | 修改前检查的 MDK-ARM 下 667 个文件中，仅授权的 uvprojx 变更，其余 666 个文件哈希保持。 |
| 硬件 | 未烧录，未做实车测试。 |

本次隔离工程为 [recovery_phi0_check.uvprojx](../build/recovery_phi0_keil/recovery_phi0_check.uvprojx)，产物为 [CtrlBoard-H7_IMU.hex](../build/recovery_phi0_keil/Objects/CtrlBoard-H7_IMU.hex) 和 [CtrlBoard-H7_IMU.axf](../build/recovery_phi0_keil/Objects/CtrlBoard-H7_IMU.axf)。同反馈回放在工程根目录用 `node build/recovery_phi0_equivalence/compare.js` 复现；该脚本依赖本次修改前快照和本机临时构建，不随 Git 交付。

重复本次隔离完整构建：

```powershell
$recoveryProject = Join-Path (Get-Location) 'build/recovery_phi0_keil/recovery_phi0_check.uvprojx'
$recoveryArgs = @('-r', ('"' + $recoveryProject + '"'), '-t', 'CtrlBoard-H7_IMU', '-j0', '-o', 'build.log')
$recoveryProcess = Start-Process -FilePath 'C:/Keil_v5/UV4/UV4.exe' -ArgumentList $recoveryArgs -WindowStyle Hidden -Wait -PassThru
Get-Content -LiteralPath 'build/recovery_phi0_keil/build.log' -Tail 8
$recoveryProcess.ExitCode
```

### 历史：首次封装与 2026-10-05 风格整理

以下为旧接口实现的历史验证，不能替代本次 phi0 接口与自起模块的验证：

| 验证 | 结果与边界 |
| --- | --- |
| 动作模块 C 验证 | 350 项断言通过；涵盖正负 360°/720°、绝对方向选路、跨周界、同姿态、实际未到位、保持、重启、双腿独立、计时回绕、即时参考和无效输入。 |
| 底盘阶段 C 验证 | 137 项断言通过；涵盖阶段锁存、先完成腿保持、共享截止、交接范围/超时、禁用、故障当周期清零，以及 NORMAL 交接帧的映射检查。 |
| VMC/Jacobian 脚本 | 有限差分核对通过；脚本几何与固件不同，仅验证公式。 |
| Keil ArmClang 6.16 完整重编 | 0 Error(s)、0 Warning(s)，退出码 0。 |
| 编码与文件保护 | 既有 GBK 编码、各文件行尾保持；原 MDK-ARM 下 521 个受检查文件哈希未变。 |
| 硬件 | 未烧录，未做实车测试；控制成功率和实际任务耗时尚无数据。 |

2026-10-05 风格整理复验：单腿模块 350 项断言、自起流程 198 项断言通过。新增用例覆盖扫腿及交接期间左腿失败后不再推进右腿，以及右腿失败时清除左腿已计算输出。单腿模块源码和公共接口未改动，原有阶段、超时及交接输出保护用例继续通过。

隔离 Keil 完整重编结果为 0 Error(s)、0 Warning(s)，退出码 0；当时日志为 [style_build.log](../build/leg_motion_keil/style_build.log)。尚未烧录或进行实车验证。

### 复现方法

在工程根目录运行：

```powershell
node mdk_check/leg_motion_verify.js
node mdk_check/vmc_verify.js
node mdk_check/jacobian_verify.js
```

动作验证需要 Node.js 与支持 C99 的主机编译器，默认 gcc，可通过环境变量 CC 指定可执行文件。脚本直接编译实际 leg_motion.c 和 chassis_recovery.c，自起阶段验证使用真实新模块；底盘接入验证提取实际适配、清零及 VMC 保护代码，时钟、VMC 映射和电机使能使用桩。验证不覆盖真实 CAN、RTOS 调度和机械响应。

底盘源码为 GBK，脚本按 GB18030 解码，提取片段以 UTF-8 写入临时文件。底盘适配函数或提取边界变化时，同步维护 leg_motion_verify.js。目标编译器若启动失败，应记录未完成的主机验证，并与固件 Keil 构建结果分开报告。

历史隔离工程和结果位于被 Git 忽略的 build/ 下，不代表本次新代码：

- 2026-10-05 工程：[leg_motion_check.uvprojx](../build/leg_motion_keil/leg_motion_check.uvprojx)。
- 旧日志：[build.log](../build/leg_motion_keil/build.log) 和 [style_build.log](../build/leg_motion_keil/style_build.log)。
- 旧产物：[CtrlBoard-H7_IMU.hex](../build/leg_motion_keil/Objects/CtrlBoard-H7_IMU.hex) 和 [CtrlBoard-H7_IMU.axf](../build/leg_motion_keil/Objects/CtrlBoard-H7_IMU.axf)。
- 首次封装前快照：[manifest.json](../build/leg_motion_baseline_7p2dihus/manifest.json)。
- 此前接口封装快照：build/recovery_phi0_baseline_5kiry7zy；本次阶段调整前快照：build/recovery_align_baseline_az1by8xi。

隔离工程源路径使用本机绝对路径，迁移工作目录后须从权威 Keil 工程重新生成；临时产物不会随 Git 提交交付。新源文件注册到权威工程后，正常 Keil 编译可直接使用。原 MDK-ARM 下的产物未覆盖，不代表本次新代码。

隔离构建使用 Start-Process -WindowStyle Hidden -Wait，要求 0 错误且无新增警告。正常工程的编译与 CMSIS-DAP 下载方式见 [CLAUDE.md](../CLAUDE.md)。

## 实车续接与观测

优先观测 chassis_recovery.motion[0]/[1] 的 actual_angle_rad、reference_angle_rad、target_angle_rad、reference_length_m、result 和锁存 command，同时记录 chassis_recovery.phase[0]/[1]、result、entry_tick、handoff_tick，Chassis.chassis_mode、两腿 phi0/L0/d_L0、F0/Tp、torque_set[]、机身俯仰及 fb_dt/t_sum。

phase[0]/[1] 的值为 0=SWING、1=RETRACT、2=WAIT_ALIGN、3=ALIGN、4=HOLD。重点比较两腿 motion[].start_tick 是否同拍、command.direction 是否分别锁存、command.target_phi0_rad 是否在收腿时捕获并在 ALIGN 变为 1.7。

单腿上下文角度以竖直向下为零，采用连续弧度；与 phi0 比较时先减 π/2 并核对周数。command.target_phi0_rad 则直接采用 VMC 坐标。

自起模块的 FAULT 本身锁存至复位，但底盘中止路径会调用 ChassisRecovery_Reset()，因此故障后 chassis_recovery.result 通常已经回到 IDLE，阶段和计时也已重置。单腿 motion[0]/[1] 保留供诊断，需结合 Chassis.chassis_mode、motion[].result 和映射输出定位；映射异常时单腿可能仍为 DONE 而底盘已经 ZERO_FORCE。

实车按以下顺序收集数据：

1. 确认实际任务周期、左右腿方向与参考跟踪；记录正常开启、倒地开启和 OFF→ON 重启结果。
2. 覆盖前倒、后倒、两腿姿态不同及一腿提前完成，记录进入收腿的实际 phi0、完成时间和机身俯仰。
3. 核对当前 1 秒收腿、等待另一腿、1 秒第二段转动及最后一帧交接的输出变化；记录进入 NORMAL 后是否保持平衡，以及失败时输出是否归零。
4. 根据记录调整动作时间、提前窗口、容差或控制参数。600000 ms 是保留配置，最终超时由实测完成时间确定。

现有倒地检测仍只看 phi0 和俯仰，未增加侧倾、触地条件或检测滞回；交接未增加角速度门限。这些属于后续有数据后再评估的控制优化。

LegMotion_Run 和 ChassisRecovery_Run 本身无阻塞延时；ZERO_FORCE 中原有 DM_Enable() 电机重使能和 osDelay(1) 路径仍保留，分析周期时应区分这一路径与动作模块计算耗时。

## 继续修改与提交

- 修改源码时按原编码和行尾字节读写；chassis_task.c 为 GBK/CRLF，chassis_task.h 仍为 GBK/LF。leg_motion.c/.h 与 chassis_recovery.c/.h 均为 UTF-8 无 BOM/CRLF（含中文宏注释）。Markdown 使用 UTF-8 BOM + CRLF。
- Git 仓库根目录是固件目录的上两级。本次未暂存或提交，当前状态无已暂存改动；保留工作区其他现有修改，提交时逐项审查并显式选择文件。
- 交付时将新模块和验证文件一并纳入；保留 CLAUDE.md 原有用户修改。
- 修改代码后重跑对应 C 验证与 Keil 构建；控制变化完成后补实车记录。只改文档无需重编固件。
