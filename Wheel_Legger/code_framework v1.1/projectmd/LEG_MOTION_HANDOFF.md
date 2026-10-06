# 倒地自起封装交接

交接日期：2026-10-05。依据当前工作区源码整理，相关代码尚未提交。动作封装和底盘接入已完成；主机验证通过，Keil 完整构建为 0 错误、0 警告；尚未烧录或实车验证。

## 接手顺序

1. 调用或修改单腿动作时，先读 [接口用法](LEG_MOTION.md) 和 [leg_motion.h](../User/Controller/leg_motion.h)，核对角度坐标、单位及 start 的使用。
2. 修改自起流程时，读 [chassis_task.c](../User/APP/chassis_task.c) 的 Recovery_Phase、falling_down()、falling_to_down() 和 VMC_translate()。以本文件的阶段说明核对实际切换条件。
3. 先做实车观测，再决定轨迹时间、增益或超时是否需要调整。完成标准是记录角度与腿长跟踪、阶段切换、交接结果及任务周期，而非只看到机器人偶然起身。

## 文件与职责

| 文件 | 本次内容 |
| --- | --- |
| [leg_motion.h](../User/Controller/leg_motion.h) | 单腿命令、反馈、输出、独立上下文、结果枚举与控制参数；唯一公共函数 LegMotion_Run。 |
| [leg_motion.c](../User/Controller/leg_motion.c) | 方向选路、连续角展开、角度与腿长同步插值、PD 与重力近似、完成/超时判断。 |
| [chassis_task.c](../User/APP/chassis_task.c) | 主流程明确调用左腿、右腿；falling_leg_control() 合并单腿阶段处理与反馈/输出适配，保留交接条件及故障清零。 |
| [chassis_task.h](../User/APP/chassis_task.h) | 清理旧自起 PID 及重力参数宏，相关参数移至新模块。 |
| [Keil 工程](../MDK-ARM/CtrlBoard-H7_IMU.uvprojx) | Controller 分组新增 leg_motion.c；沿用原 User/Controller 包含路径。 |
| [leg_motion_verify.js](../mdk_check/leg_motion_verify.js) | 编译实际 C 模块及从当前底盘源码提取的阶段代码，运行两个主机验证程序。 |
| [leg_motion_verify.c](../mdk_check/leg_motion_verify.c)、[recovery_verify.c](../mdk_check/recovery_verify.c) | 动作语义和底盘阶段/故障边界的验证用例。 |
| [LEG_MOTION.md](LEG_MOTION.md) | 调用示例、六个命令参数及返回状态的日常参考。 |

旧 spin 控制函数、相应运行状态和禁用的旧级联倒地控制实现已替换。LQR、LESO、遥控、CAN 映射和 RTOS 周期沿用现有实现。

## 接口中已经确定的约定

- 单腿调用，左右腿各自保存上下文，初始清零。函数每控制周期运行一次，无 HAL、CAN、延时和动态分配。
- 命令包含 direction、angle_mode、angle_deg、duration_ms、length_m、timeout_ms。角度参数为度；反馈角及上下文角为弧度，反馈速率为 rad/s，腿长为 m，腿长速率为 m/s。
- 绝对模式以腿相对机身垂直向下为 0°，对应 phi0=π/2；按指定方向到达该姿态。相对模式表示实际转过的非负角量，360°/720°分别为一圈/两圈，方向另行指定。
- 正方向是 phi0 增大，负方向是 phi0 减小。反馈和输出已处理左右镜像，两腿共用方向约定。
- start 只在开始动作的当周期置 1；置 1 就重新捕获实际角度、腿长和时刻，即使命令相同也会重启。后续置 0，命令被锁存，command 可传 NULL。一直置 1 会导致轨迹一直从当前反馈重新开始。
- duration_ms 是参考变化时间，角度和腿长同时从启动反馈线性变化；它不保证电机在该时刻已经实际到位。timeout_ms 是从启动计时的总截止时间。
- 连续角误差用于跟踪和完成判断，重力近似仍使用物理姿态。相邻采样间实际转动须小于 180°；展开算法不能识别漏采样后的额外整圈。
- DONE 在参考变化时间结束、实际角误差小于 0.25 rad 且腿长误差小于 0.02 m 后锁存，并继续输出以保持目标。DONE 不要求角速度已经为零。TIMEOUT/INVALID 输出零并锁存，新的 start 才能重新执行。
- 已完成动作可持续保持，不因原 timeout_ms 到期变成超时；保持期间反馈无效仍会进入 INVALID。调用者停止调用时须主动清除底盘输出。

控制增益、限幅及重力参数集中在 leg_motion.h，详细数值见接口说明。F0 是力，单位 N；Tp 是摆腿力矩，单位 N·m。当前 PD 使用反馈速率阻尼，未加入参考速度前馈。

## 底盘自起函数组织

- `falling_down()` 顺序处理禁用清零、首次启动、总超时、左腿、右腿及进入交接。每条腿执行后立即检查结果；左腿失败时不再调用右腿，右腿失败时一并清除左腿当周期输出。
- `falling_leg_control(leg, start, now)` 合并原适配函数和单腿阶段处理，按 SWING / RETRACT / HOLD 设置命令，直接调用 `LegMotion_Run()` 并回写 F0/Tp。命令与反馈逐字段赋值，两腿仍保留独立上下文。
- `falling_to_down()` 顺序保持左腿、右腿，再判断平衡交接条件或交接超时。HOLD 传入 NULL 命令并保持 start=0，不重新捕获目标。
- 清零、中止、复位函数及 `recovery_output_pending` 保留；标记在 VMC 映射时读取后清除，确保交接最后一帧仍受检查。

本次仅整理流程表达和中文注释，`LegMotion_Run()` 公共接口、控制参数、阶段条件及 600000/1500 ms 超时保持不变。

## 底盘自起阶段

```text
OFF→ON 位姿检查
  ├─ 位姿符合正常范围 → NORMAL
  └─ 位姿倒地 → FALLING_DOWN
                  每条腿独立：SWING → RETRACT → HOLD
                  两腿都 HOLD → FALLING_TO_NORMAL
                                   ├─ 交接条件满足 → NORMAL
                                   └─ 交接超时 → ZERO_FORCE

在线 NORMAL 检测到倒地 → ZERO_FORCE，等待再次 OFF→ON
自起超时、无效反馈或自起力矩映射异常 → 当周期 ZERO_FORCE
```

### SWING：扫腿

当前配置在 chassis_task.c 顶部：负方向、绝对目标 +39.143°、参考变化 4000 ms、目标腿长 0.30 m。初始实际腿角、腿长作为插值起点。

旧宏 SPIN_TARGET_ANGLE=5.6f 表示沿扫腿方向的角量。原负方向的终点 spin=-5.6 rad，与 +0.683185 rad 为同一物理姿态，即 +39.143°，对应 phi0≈2.254 rad。新宏 SPIN_TARGET_ANGLE_DEG 表示绝对姿态；迁移参数时同时核对坐标和模式。

实际角满足任一条件就进入收腿，而不要求扫腿模块先返回 DONE：

- 当前负方向的提前窗口：归一化 spin=phi0-π/2 位于 [-0.9, 0] rad，边界包含。
- 实际角与绝对终点的圆周误差小于 0.25 rad。

实际角在进入自起时已满足窗口的腿，可在第一周期直接收腿。正方向配置的提前窗口为 [0, 0.9] rad；改变方向时，应同时核算目标姿态和正常检测窗。

### RETRACT：固定角度收腿

首次进入时重启该腿模块，命令为相对 0°。角度目标固定为这一刻的实际角度，腿长参考用 1000 ms 从当时实际长度变化到 0.13 m。

收腿阶段锁存，实际角后来离开窗口也会继续跟踪已捕获目标。模块返回 DONE 后进入 HOLD；不会回到扫腿或再次伸至 0.30 m。

### HOLD 与 FALLING_TO_NORMAL：保持和交接

先完成的一条腿继续保持，等待另一条腿完成。两腿都 HOLD 后开始独立的 1500 ms 交接计时；交接期间继续调用两个已完成上下文。

交回 NORMAL 同时要求：

| 条件 | 当前判据 |
| --- | --- |
| 机身俯仰 | 有限，且绝对值小于 0.2 rad。 |
| 每条腿姿态 | 满足提前窗口或终点窗口，并且原始 phi0 在 [0.4, 2.5] rad 内。 |
| 每条腿腿长 | 距 0.13 m 的误差小于 0.02 m。 |
| 每条腿腿长变化率 | 绝对值小于 0.05 m/s。 |

正常检测仍按 phi0 范围及俯仰绝对值 0.3 rad 判断。交接增加原始 phi0 范围约束，避免仅因圆周姿态等价而交回 NORMAL，随后立刻被正常检测判为倒地。

### 超时和清零

扫腿、收腿及等待另一腿共用进入 FALLING_DOWN 时开始的 600000 ms 总截止时间。收腿启动时仅分配剩余时间，不会把总截止时间再延长。底盘先检查总截止，到达边界即中止；交接的 1500 ms 另行计算。

任一腿 TIMEOUT/INVALID、交接异常或超时，都使底盘当周期进入 ZERO_FORCE，并清零两腿 F0/Tp、映射后的关节力矩、轮力矩及轮电流命令。OFFLINE 或 ZERO_FORCE 时 VMC_translate() 直接清零并返回，避免无效雅可比与零相乘得到 NaN。

recovery_output_pending 标记本周期由自起生成的输出。falling_to_down() 先改模式再做 VMC 映射，因此交回 NORMAL 的最后一帧仍须用该标记检查映射力矩是否有限；该保护已经有回归用例。

## 本次相对旧实现的行为变化

| 变化 | 实车需要确认的效果 |
| --- | --- |
| 扫腿时角度与腿长参考同时线性插值 | 起始伸腿与转动时的接地关系、跟踪滞后和电机峰值输出。 |
| 收腿改成明确的 1 秒参考变化 | 能否及时建立支撑并减少收腿冲击；时间不是已经验证的最佳值。 |
| 提前窗口只触发一次；随后保持捕获角度 | 扰动后是否能持续支撑，以及提前进入收腿时机是否合适。 |
| 两腿分别锁存完成并保持 | 左右完成时间不同是否影响机身姿态或造成拖地。 |
| 交接额外要求正常检测的 phi0 范围 | 接近范围边缘时能否正常交接；当前没有追加角度对齐阶段。 |

## 验证记录与复现

封装首次实现完成时的结果：

| 验证 | 结果与边界 |
| --- | --- |
| 动作模块 C 验证 | 350 项断言通过；涵盖正负 360°/720°、绝对方向选路、跨周界、同姿态、实际未到位、保持、重启、双腿独立、计时回绕、即时参考和无效输入。 |
| 底盘阶段 C 验证 | 137 项断言通过；涵盖阶段锁存、先完成腿保持、共享截止、交接范围/超时、禁用、故障当周期清零，以及 NORMAL 交接帧的映射检查。 |
| VMC/Jacobian 脚本 | 有限差分核对通过；脚本几何与固件不同，仅验证公式。 |
| Keil ArmClang 6.16 完整重编 | 0 Error(s)、0 Warning(s)，退出码 0。 |
| 编码与文件保护 | 既有 GBK 编码、各文件行尾保持；原 MDK-ARM 下 521 个受检查文件哈希未变。 |
| 硬件 | 未烧录，未做实车测试；控制成功率和实际任务耗时尚无数据。 |

2026-10-05 风格整理复验：单腿模块 350 项断言、自起流程 198 项断言通过。新增用例覆盖扫腿及交接期间左腿失败后不再推进右腿，以及右腿失败时清除左腿已计算输出。单腿模块源码和公共接口未改动，原有阶段、超时及交接输出保护用例继续通过。

隔离 Keil 完整重编结果为 0 Error(s)、0 Warning(s)，退出码 0；本次日志为 [style_build.log](../build/leg_motion_keil/style_build.log)。尚未烧录或进行实车验证。

在工程根目录运行：

```powershell
node mdk_check/leg_motion_verify.js
node mdk_check/vmc_verify.js
node mdk_check/jacobian_verify.js
```

动作验证需要 Node.js 与支持 C99 的主机 C 编译器；默认 gcc，也可通过环境变量 CC 指定编译器可执行文件。当前机器可用 E:/mingw64/bin/gcc.exe。测试编译真实模块；底盘验证从当前 GBK 源码中提取实际阶段函数，时钟、VMC 映射和电机使能使用桩，未覆盖真实 CAN、RTOS 调度和机械响应。验证脚本严格按 GB18030 解码底盘源码，提取片段以 UTF-8 写入临时文件；用函数定义定位清零及后续阶段代码，支持中文注释。源码选择标记变化时，需同步维护 leg_motion_verify.js 的提取位置。

本次隔离工程和结果均在被 Git 忽略的 build/ 下：

- 工程：[leg_motion_check.uvprojx](../build/leg_motion_keil/leg_motion_check.uvprojx)。
- 日志：[build.log](../build/leg_motion_keil/build.log)。
- 产物：[CtrlBoard-H7_IMU.hex](../build/leg_motion_keil/Objects/CtrlBoard-H7_IMU.hex) 和 [CtrlBoard-H7_IMU.axf](../build/leg_motion_keil/Objects/CtrlBoard-H7_IMU.axf)。
- 改动前快照与哈希：[manifest.json](../build/leg_motion_baseline_7p2dihus/manifest.json)。

隔离工程源路径使用本机绝对路径，迁移工作目录后须从权威 Keil 工程重新生成；这些临时文件不会随 Git 提交交付。本次源文件已注册到权威工程，正常 Keil 编译可直接使用。原 MDK-ARM 下的旧产物未被覆盖，不代表本次新代码。

在当前目录重复隔离完整构建：

```powershell
$motionProject = Join-Path (Get-Location) 'build/leg_motion_keil/leg_motion_check.uvprojx'
$motionArgs = @('-r', ('"' + $motionProject + '"'), '-t', 'CtrlBoard-H7_IMU', '-j0', '-o', 'build.log')
$motionProcess = Start-Process -FilePath 'C:/Keil_v5/UV4/UV4.exe' -ArgumentList $motionArgs -WindowStyle Hidden -Wait -PassThru
Get-Content -LiteralPath 'build/leg_motion_keil/build.log' -Tail 8
$motionProcess.ExitCode
```

目标为 0 错误且无新警告。原 Keil GUI 打开同一权威工程时继续使用隔离构建；正常工程的编译与 CMSIS-DAP 下载方式见 [CLAUDE.md](../CLAUDE.md)。

## 实车续接与观测

优先观测 chassis_leg_motion[0]/[1] 的 actual_angle_rad、reference_angle_rad、target_angle_rad、reference_length_m、result 和锁存 command，同时记录 recovery_phase[0]/[1]、Chassis.chassis_mode、两腿 phi0/L0/d_L0、F0/Tp、torque_set[]、机身俯仰及 fb_dt/t_sum。上下文里的角度为连续弧度；与 phi0 比较时先减去 π/2 并核对周数。

故障后上下文保留供调试，但 recovery_phase 被重置。映射异常可出现模块仍为 DONE 而底盘已 ZERO_FORCE 的情况，需结合映射输出定位。

下一步按以下顺序收集数据：

1. 确认实际任务周期、左右腿方向与参考跟踪；记录正常开启、倒地开启和 OFF→ON 重启的结果。
2. 覆盖前倒、后倒、两腿姿态不同及一腿提前完成，记录进入收腿的实际角度、完成时间和机身俯仰。
3. 核对 1 秒收腿和最后一帧交接的输出变化；记录成功进入 NORMAL 后是否保持平衡，以及失败时输出是否归零。
4. 根据这些记录再调整动作时间、提前窗口、容差或控制参数。600000 ms 是保留的现有配置，最终超时应由实测完成时间确定。

现有倒地检测仍只看 phi0 和俯仰，没有加入侧倾、触地条件或检测滞回；交接没有新增角速度门限。这些属于后续有数据后再评估的控制优化，本次封装没有改变对应策略。

LegMotion_Run 本身无阻塞延时；ZERO_FORCE 中原有 DM_Enable() 电机重使能和 osDelay(1) 路径仍保留。分析周期记录时应区分这一路径与动作模块的计算耗时。

## 继续修改与提交

- 修改 chassis_task.c/.h 时使用字节读写保留 GBK；当前 .c 为 CRLF，.h 为 LF。新模块为 ASCII/CRLF。Markdown 使用 UTF-8 BOM + CRLF。
- Git 仓库根目录是固件目录的上两级。工作区原有用户暂存内容，本次实现尚未暂存或提交；提交时逐项审查，显式选择文件。
- 新代码和验证文件当前为未跟踪文件，交付源码时需一并纳入；CLAUDE.md 已有用户暂存修改，新增交接更新与其合并保留。
- 代码修改后重跑对应 C 验证与 Keil 构建；控制变化完成后再补实车记录。只改文档无需重编固件。
