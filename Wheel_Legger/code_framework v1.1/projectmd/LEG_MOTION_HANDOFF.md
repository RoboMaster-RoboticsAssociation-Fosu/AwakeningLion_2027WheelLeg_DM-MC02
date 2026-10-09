# 倒地自起封装交接

最新工作快照见 [2026-10-08 自起工作交接](WORK_HANDOFF_2026-10-08_RECOVERY.md)。当前有效原始 phi0∈[0.4,2.82]（含边界）或扫腿到位即锁存就绪，先就绪腿 Tp=0 保持进入时实际长度，等两腿都就绪后同拍收腿。共同 300 ms 收至 0.139 m，收完后下一拍同步用 600 ms 转到 1.6 再交接 NORMAL。扫腿时间仍为 4000/500 ms。

交接日期：2026-10-08。依据当前工作区源码整理，相关改动尚未提交。本轮已通过主机回归和隔离完整构建，未烧录或实车验证；先前用户确认 PID 调参解决自起停留问题，记录保留于下方。历史接口及构建记录不代表本轮参数和启动规则。

## 接手顺序

1. 调用或修改单腿动作时，先读 [接口用法](LEG_MOTION.md) 和 [leg_motion.h](../User/Controller/leg_motion.h)，核对 phi0 弧度、方向及 start 的使用。
2. 修改自起阶段、重启、超时或交接时，读 [chassis_recovery.h](../User/Controller/chassis_recovery.h) 和 [chassis_recovery.c](../User/Controller/chassis_recovery.c)。修改底盘接入时，再读 [chassis_task.c](../User/APP/chassis_task.c) 的 chassis_recovery_control()、falling_down_detect() 和 VMC_translate()。
3. 控制变化完成后记录实车角度与腿长跟踪、阶段切换、交接结果及任务周期，再依据数据调整轨迹时间、增益或超时。

## 文件与职责

| 文件 | 当前职责 |
| --- | --- |
| [leg_motion.h](../User/Controller/leg_motion.h)、[leg_motion.c](../User/Controller/leg_motion.c) | 单腿命令、反馈、输出和上下文；phi0 定向选路、连续角展开、同步插值、PD 与重力近似、完成/超时判断。公共函数为 LegMotion_Run。 |
| [chassis_recovery.h](../User/Controller/chassis_recovery.h)、[chassis_recovery.c](../User/Controller/chassis_recovery.c) | 自起参数、双腿上下文、阶段及计时；首次启动、扫腿/收腿/等待/第二段转动/保持、共享截止、交接及结果。公共函数为 ChassisRecovery_Run、ChassisRecovery_Reset 和 ChassisRecovery_InEarlyWindow。 |
| [chassis_task.c](../User/APP/chassis_task.c) | 反馈/输出适配、模式切换、开启判姿和正常倒地检测、最终清零及 VMC 映射检查；全局 chassis_recovery 供调试。 |
| [Keil 工程](../MDK-ARM/CtrlBoard-H7_IMU.uvprojx) | Controller 分组注册两个模块；沿用 User/Controller 包含路径。 |
| [leg_motion_verify.js](../mdk_check/leg_motion_verify.js) | 编译实际单腿模块、自起模块及底盘适配验证。 |
| [leg_motion_verify.c](../mdk_check/leg_motion_verify.c)、[chassis_recovery_verify.c](../mdk_check/chassis_recovery_verify.c)、[recovery_verify.c](../mdk_check/recovery_verify.c) | 分别验证单腿命令、自起公共模块、底盘接入及最终输出保护。 |
| [LEG_MOTION.md](LEG_MOTION.md) | 七个命令参数、逐字段赋值示例和返回状态的日常参考。 |

旧 spin 控制函数、相应运行状态和禁用的旧级联倒地控制实现已替换。LQR、LESO、遥控、CAN 映射和 RTOS 周期沿用现有实现。

## 接口中已经确定的约定

- 单腿每控制周期调用一次，左右腿各自保留初始清零的上下文；模块无 HAL、CAN、阻塞延时和动态分配。
- 命令为 angle_control、direction、target_phi0_rad、angle_duration_ms、length_duration_ms、length_m、timeout_ms。angle_control 启动时锁存，0=POSITION、1=FREE；所有构造点显式赋值。目标直接使用 VMC 的 phi0，单位 rad，竖直向下为 π/2；反馈速率为 rad/s，腿长为 m，腿长速率为 m/s。
- 正方向是 phi0 增大，负方向是 phi0 减小。左右镜像已在反馈和输出端处理，两腿共用方向约定。目标以 2π 为周期，等价姿态不额外转圈；旧 angle_mode、angle_deg 及相对整圈模式已移除。
- start 只在开始动作的当周期置 1；置 1 就重新捕获实际角度、腿长和时刻，即使命令相同也会重启。后续置 0，命令被锁存，command 可传 NULL。一直置 1 会使轨迹每拍重新开始。
- angle_duration_ms / length_duration_ms 分别控制角度 / 长度参考，两条轨迹同拍开始、独立插值，先完成的参考保持目标；对应时间 0 表示即时参考。POSITION 单腿完成等待两个时间和容差，FREE 只等待长度时间及容差、忽略角度时间。timeout_ms 仍是从启动计时的总截止。原 duration_ms 已移除，所有调用点及示例均显式赋值两个时间。
- 内部连续角仍使用 q=phi0−π/2，连续误差用于跟踪和完成判断，重力近似仍使用物理姿态。相邻采样间实际转动须小于 π，展开算法无法识别漏采样后的额外整圈。
- DONE 在参考时间结束、腿长误差小于 0.02 m 后锁存；POSITION 还要求实际连续角误差小于 0.25 rad。FREE 不检查角度到位，始终 Tp=0，只保持腿长。TIMEOUT/INVALID 输出零并锁存，新的 start 才能重新执行。
- 完成后持续保持，不因原 timeout_ms 到期变成超时；保持期间反馈无效仍会进入 INVALID。调用者停止调用时须主动清除底盘输出。

FREE 仍检查反馈有限、长度为正和相邻采样小于半圈；实际连续角照常更新，参考角及目标角跟随实际角。摆角 PD、阻尼、摆角重力补偿关闭，F0 的腿长 PD 和伸缩重力补偿保留。

控制增益、限幅及重力参数集中在 leg_motion.h，详细数值见接口说明。F0 单位 N，Tp 单位 N·m；当前 PD 使用反馈速率阻尼，未增加参考速度前馈。

## 自起模块与底盘接入

`ChassisRecovery_Run(ctx, input, now_ms, output)` 的输入包含 enabled、pitch_rad 和 leg[2] 单腿反馈，输出 leg[2] 的 F0/Tp。返回状态依次为 CHASSIS_RECOVERY_IDLE、RUNNING、HANDOFF、DONE、FAULT（均带 CHASSIS_RECOVERY_ 前缀）。上下文持有 motion[2]、phase[2]、result、entry_tick、handoff_tick 和 sweep_arrived[2]；phase 为 CHASSIS_RECOVERY_SWING、RETRACT、WAIT_ALIGN、ALIGN、HOLD（同样带此前缀）。左右腿下标分别为 0、1，与底盘腿下标一致。

- 模块内部明确顺序调用左腿、右腿；左腿失败立即返回，不推进右腿。右腿失败时清除左腿已计算的本拍输出。
- ChassisRecovery_Reset() 复位阶段和计时，保留单腿上下文供诊断。FAULT 锁存至复位；禁用调用会复位并返回 IDLE、清零输出，下次启用重新捕获两腿反馈。DONE 后继续调用仍保持两腿输出，由底盘适配交回正常控制。
- `falling_down()` 和 `falling_to_down()` 均只调用 `chassis_recovery_control()`。适配函数逐字段准备反馈，将模块输出回写两腿，清零轮输出，并根据结果切换底盘模式。
- 失败或禁用时适配函数完成腿虚拟输出、关节力矩和轮输出的最终清零。开启时统一进入自起；逐腿收腿范围判断由自起模块完成，正常状态倒地检测和 VMC 映射后检查保留在底盘原调用位置。
- `recovery_output_pending` 在 VMC 映射时读取并清除，确保交回 NORMAL 的最后一帧仍检查自起输出。

单腿 angle_control 字段及 PD 增益、输出限幅、现有阶段编号保持。动作共享截止为 600000 ms，交接超时为当前配置 500 ms；两腿收完后增加同步启动的第二段转动。

## 底盘自起阶段

```text
OFF→ON 统一复位 → FALLING_DOWN（上电已在线同样适用）
每腿启动及 SWING：
  ├─ 原始 phi0∈[0.4,2.82] 或扫腿到位 → sweep_arrived=1
  │    SWING 中 FREE 等待（Tp=0，保持进入时实际腿长）
  └─ 未就绪 → POSITION 扫腿
两腿都就绪 → 同拍 RETRACT（Tp=0，300 ms 收腿）
每腿收完 → WAIT_ALIGN（Tp=0，仅保持短腿长）
两腿均 WAIT_ALIGN → 下一拍同时 ALIGN（最短方向到 1.6）
每腿 ALIGN 完成 → HOLD
两腿均 HOLD → FALLING_TO_NORMAL
               ├─ 下一拍起，交接条件满足 → NORMAL
               └─ 交接超时 → ZERO_FORCE

在线 NORMAL 检测到倒地 → ZERO_FORCE，等待再次 OFF→ON
自起超时、无效反馈或自起力矩映射异常 → 当周期 ZERO_FORCE
```

### SWING：扫腿、就绪与自由等待

每次 OFF→ON 进入 FALLING_DOWN，上电已在线同样适用。启动及 SWING 中，有限原始 phi0∈[SPIN_RETRACT_PHI0_MIN_RAD, SPIN_RETRACT_PHI0_MAX_RAD]（当前 [0.4,2.82]，含边界）即收腿就绪，不看 pitch。上界引用第一目标 SPIN_TARGET_PHI0_RAD；InEarlyWindow() 签名保持，查询原始角，不归一化或按方向反转。

范围外使用 POSITION 扫向 2.82，角度/长度独立用 4000/500 ms 向 2.82/0.39 m 插值。角度时间结束、当前连续角误差严格小于 SPIN_ANGLE_WINDOW=0.05，也可就绪；不要求长度斜坡结束。

就绪锁存在 sweep_arrived[leg]，phase 仍为 SWING，但命令切为 FREE，两个参考时间为 0，捕获进入就绪时实际腿长并保持。摆角漂移不取消就绪，不继续伸腿或独立缩腿。单腿动作 DONE 在此只表示长度保持命令完成，不能当作 RETRACT 已收完。

两腿都就绪后同拍 RETRACT，共用 now_ms，重新捕获各自实际腿长并清除标志；两腿启动均在范围内，则首拍共同收腿。复位或禁用清除两个标志，保留单腿诊断。范围/到位条件先只读计算，再依次推进左、右腿，左失败不修改右腿就绪状态。

运行中 SWING 切换 FREE 前，先调用原动作更新验证本拍反馈（包括半圈跳变），失败立即按原顺序中止；验证输出不发送，成功后建立 FREE 并只回写最终输出。已就绪等待仍持续检查腿反馈，共享总截止不重置。

### RETRACT 与 WAIT_ALIGN：自由摆角收腿及等待

进入收腿时锁存 FREE 命令（angle_duration_ms=0、length_duration_ms=SPIN_RETRACT_TIME_MS）、实际腿长和时刻，后续 start=0，不因摆角变化重启；腿长参考用 SPIN_RETRACT_TIME_MS=300 ms 变化到 0.139 m。Tp 全程为零；完成只看参考时间结束和腿长误差小于 0.02 m。

完成后进入 WAIT_ALIGN，继续调用 FREE 上下文保持短腿长及 Tp=0。两腿均为 WAIT_ALIGN 的当拍不启动第二段；下一周期同时启动，两腿使用同一个 now_ms，各自重新捕获实际角度与长度。等待期间沿用 DONE 锁存，不新增速度条件。

### ALIGN：收腿后一起转向 1.6

SPIN_ALIGN_TARGET_PHI0_RAD=1.6f rad，SPIN_ALIGN_TIME_MS=600U，目标腿长保持 SPIN_RETRACT_LENGTH=0.139f。

启动时恢复 POSITION，angle_duration_ms 和 length_duration_ms 均使用 SPIN_ALIGN_TIME_MS，按各腿实际 phi0 选择到 1.6 的最短方向并锁存：此时已自由摆动，不能假定仍在 2.82；使用此时的实际角选择方向。同姿态不额外绕圈，半圈按原扫腿方向。后续反馈改变不会重新选方向或重启斜坡。

单腿 DONE、斜坡结束以及当前连续角误差小于 0.05 rad 同时满足，才进入 HOLD。即使单腿 DONE 已锁存，实际角还差 0.1 rad 也要继续等待。先完成的腿保持，直到另一腿完成。

### HOLD 与 FALLING_TO_NORMAL：保持和交接

两腿都 HOLD 当拍只切入交接并开始 500 ms 计时；下一周期开始检查正常平衡条件。期间继续调用两个完成的单腿上下文，start=0、command=NULL。

| 条件 | 当前判据 |
| --- | --- |
| 机身俯仰 | 有限，且绝对值严格小于 SPIN_HANDOFF_PITCH_MAX_RAD（当前 0.5 rad）。 |
| 每条腿姿态 | phi0 与 SPIN_ALIGN_TARGET_PHI0_RAD（1.6 rad）的圆周误差小于 0.05 rad，且原始 phi0 在收腿上下界宏定义的 [0.4, 2.82] rad 内。 |
| 每条腿腿长 | 距 0.139 m 的误差小于 0.02 m。 |
| 每条腿腿长变化率 | 绝对值小于 0.05 m/s。 |

自起收腿不检查 pitch；在线倒地检测保留原 0.3 rad 阈值。先检查就绪，再检查交接超时；截止边界满足条件仍可进入 NORMAL。原始 phi0 范围避免仅因圆周姿态等价交回后立刻被倒地检测中止。两腿收完或第二段参考时间结束，本身都不会直接切入 NORMAL。

### 超时和清零

扫腿、收腿、等待及第二段转动共用进入自起时开始的 600000 ms 总截止。每次启动新动作只分配剩余时间；第二段不会延长截止。运行中先检查总截止，到达边界即中止；交接的 500 ms 另行计算。

左腿失败立即返回，不推进右腿阶段或动作上下文；右腿失败清除两腿当拍 F0/Tp。底盘中止路径同拍清零腿虚拟输出、映射关节力矩、轮力矩及电流。OFFLINE 或 ZERO_FORCE 时 VMC_translate() 清零并返回。

recovery_output_pending 保留到当拍 VMC 映射后消费；即使适配函数已经将模式改为 NORMAL，最后一帧自起映射仍检查力矩是否有限。

## 参考时序：两腿范围内 phi0=2.1 开始

假设腿反馈有效、实际腿长按时到位，且最终符合 pitch 等交接条件，控制周期为 1 ms：

| 时刻 | 动作与参考 |
| --- | --- |
| 0～300 ms | 两腿同拍 FREE 收腿，Tp=0，腿长从各自实际值插值到 0.139 m。 |
| 300 ms | 两腿均 WAIT_ALIGN，仍 Tp=0。 |
| 301～901 ms | 两腿同拍捕获当前实际角，分别沿最短方向转到 1.6。 |
| 901 ms | 两腿均满足 ALIGN 完成条件后进入 HANDOFF。 |
| 902 ms 起 | 全部交接条件满足后进入 NORMAL。 |

实际若一腿晚到，等待期间继续 FREE；第二段始终在较晚腿收完后的下一拍共同启动。已在 1.6 的同姿态启动也保留上述参考时间及交接检查。

## 参考时序：范围外 phi0=-0.57 扫腿到目标

假设两腿同步、实际完全跟踪参考、初始腿长 0.139 m，且未触发提前窗口：

| 时刻 | 动作与参考 |
| --- | --- |
| 0～4 s | 连续 phi0 从 -0.57 减小至 2.82−2π≈-3.463185 rad；VMC 显示跨越 -π/+π 后到 2.82。腿长参考在前 500 ms 从 0.139 增至 0.39 m，其后保持。 |
| 4～4.3 s | 两腿同拍 Tp=0，摆角随受力变化，腿长参考从各自实际值收至 0.139 m。 |
| 4.3 s | 两腿均 WAIT_ALIGN；下一控制周期同时启动第二段。 |
| 约 4.301～4.901 s | 两腿从自由摆动后的实际角，分别沿最短路径转到 1.6，目标腿长 0.139 m。 |
| 约 4.901 s 后 | 两腿均 HOLD 当拍进入交接；下一周期条件满足后 NORMAL。 |

以上按 1 ms 控制周期计算，不含实际跟踪滞后。若提前收腿，或两腿完成时间不同，第二段开始时刻以较晚一腿收完后的下一周期为准。

## 验证记录与复现

### 2026-10-08：[0.4,2.82] 锁存就绪，双腿共同收腿

- 原始角就绪范围上界引用 SPIN_TARGET_PHI0_RAD，当前 2.82。范围内或原扫腿终点就绪后，SWING 使用 FREE 保持进入时腿长、Tp=0，sweep_arrived[2] 锁存；另一腿就绪后同拍重新捕获实际长度，共同收腿并清除标志。取消单腿提前收腿例外，阶段编号与公开函数签名保持。
- 用户报告从 2.78 启动时 0/1/0/0 停留，摆到同一位置则可收腿。实际模块回放确认固定负向选路产生 -6.243186 rad 行程，合并范围前的快照同样复现。修复后原回放 phases=1/1、Tp=0。新范围等待回归修复前失败、修复后通过。
- SWING 切到 FREE 前验证原动作反馈，检查输出不发送；保护半圈跳变及无效反馈，左腿失败不推进右腿，右腿失败清零两腿。单腿控制、PID（当前角度 150/2、长度 1800/100）、在线保护和现有轨迹时长未改。
- 主机 C 回归：单腿 1247、自起模块 1896、底盘接入及保护 879；扫腿配置及轨迹 2394，合计 6416 项断言。轮输出 117 项、正常腿长斜坡 65399 项通过。覆盖就绪边界/2π 偏移、长度保持及摆角漂移、两腿同步收腿和转动、切换反馈故障、超时、禁用及重启。
- Keil / ArmClang 6.16 隔离完整重编：0 Error(s)、0 Warning(s)，退出码 0。工程：[recovery_sync_range_check.uvprojx](../build/recovery_sync_range_keil/recovery_sync_range_check.uvprojx)。日志：[构建](../build/recovery_sync_range_keil/build.log)、[主机](../build/recovery_sync_range_keil/host.log)、[轮输出](../build/recovery_sync_range_keil/wheel.log)、[腿长斜坡](../build/recovery_sync_range_keil/ramp.log)。
- 当前产物：[HEX](../build/recovery_sync_range_keil/Objects/CtrlBoard-H7_IMU.hex)、[AXF](../build/recovery_sync_range_keil/Objects/CtrlBoard-H7_IMU.axf)。未烧录或实车验收；主工程产物保留，修改前快照为 build/recovery_sync_range_baseline。原故障及修复后日志：[red](../build/recovery_start_target_probe/repro.log)、[green](../build/recovery_start_target_probe/repro_fixed.log)。

### 历史：2026-10-08 统一原始 phi0 收腿范围

- 原提前窗口与正常角范围合并为 [SPIN_RETRACT_PHI0_MIN_RAD, SPIN_RETRACT_PHI0_MAX_RAD]=[0.4, 2.5]，含边界。启动及 SWING 逐腿判断，不看 pitch、不归一化、不分方向；删除首次正常姿态专用判断。保留 InEarlyWindow 函数签名，交接原始角范围复用上下界宏。
- 保留用户现有 4000 / 500 ms 扫腿、300 ms / 0.139 m 收腿、600 ms / 1.6 rad 转动。单腿控制、在线倒地检测、PID、正常腿长斜坡和保护参数未改。
- 主机 C 回归：单腿 1247、自起模块 1421、底盘接入及保护 771；扫腿配置及范围进入轨迹 670，合计 4109 项断言通过。覆盖原始角边界与 2π 偏移、大/无效 pitch 下收腿、SWING 范围进入、收腿离开范围不重启、先完成腿等待、第二段同拍、扫腿到目标规则及交接保护。轮输出 117 项、正常腿长斜坡 65399 项通过。
- Keil / ArmClang 6.16 隔离完整重编：0 Error(s)、0 Warning(s)，退出码 0。工程：[recovery_unified_range_check.uvprojx](../build/recovery_unified_range_keil/recovery_unified_range_check.uvprojx)。日志：[构建](../build/recovery_unified_range_keil/build.log)、[主机](../build/recovery_unified_range_keil/host.log)、[轮输出](../build/recovery_unified_range_keil/wheel.log)、[腿长斜坡](../build/recovery_unified_range_keil/ramp.log)。
- 当时产物：[HEX](../build/recovery_unified_range_keil/Objects/CtrlBoard-H7_IMU.hex)、[AXF](../build/recovery_unified_range_keil/Objects/CtrlBoard-H7_IMU.axf)。未烧录或实车验证；主工程产物保留，修改前快照为 build/recovery_unified_range_baseline。

### 历史：2026-10-08 扫腿角度与腿长斜坡独立配置

- 拆分宏 SPIN_SWING_ANGLE_TIME_MS / SPIN_SWING_LENGTH_TIME_MS，默认均 4000 ms。单腿命令原 duration_ms 拆为 angle_duration_ms / length_duration_ms，同拍启动并独立计时，先结束的参考保持目标。
- POSITION 单腿 DONE 等待两个斜坡及现有到位容差；FREE 只用长度时间，Tp=0。自起扫腿终点仅检查角度斜坡及角度误差，仍允许长度斜坡未结束时同拍收腿。提前窗口、正常启动直接收腿、300 ms 收腿及 600 ms 转向 1.6 的时序保持。
- 主机验证：单腿 1247、自起模块 1341、底盘接入及保护 739；额外扫腿配置共 466 项，合计 3793 项断言。配置覆盖 4 s/4 s、4 s/8 s、8 s/4 s、0 s/4 s、4 s/0 s，使用实际源码临时副本，仅改两个时间宏。轮输出 117 项、正常腿长斜坡 65399 项通过。
- Keil / ArmClang 6.16 隔离完整重编：0 Error(s)、0 Warning(s)，退出码 0。工程：[recovery_split_ramp_check.uvprojx](../build/recovery_split_ramp_keil/recovery_split_ramp_check.uvprojx)。日志：[构建](../build/recovery_split_ramp_keil/build.log)、[主机](../build/recovery_split_ramp_keil/host.log)、[轮输出](../build/recovery_split_ramp_keil/wheel.log)、[腿长斜坡](../build/recovery_split_ramp_keil/ramp.log)。
- 当时产物：[HEX](../build/recovery_split_ramp_keil/Objects/CtrlBoard-H7_IMU.hex)、[AXF](../build/recovery_split_ramp_keil/Objects/CtrlBoard-H7_IMU.axf)。未烧录或实车验证；主工程产物保留，修改前快照为 build/recovery_split_ramp_baseline。

### 历史：2026-10-08 正常范围内启动也先收腿、同步转到 1.6

- OFF→ON 全部进入自起；首拍正常姿态直接双腿 FREE 收腿，运动途中不重新选择启动路径。保留 300 ms / 0.139 m 收腿、600 ms / 1.6 rad 对齐及当前所有控制参数，未改动单腿接口、PID、正常平衡腿长斜坡、在线倒地检测或输出保护。
- 新入口用例“上电已在线、两腿 2.1”在修改前失败、修改后通过。覆盖正常边界、角度分处目标两侧、同姿态时间及不绕圈、实际长度捕获、先收完腿等待、同步转动、未到位禁止 NORMAL、OFF→ON 重启和非正常混合提前窗口。模块回归另验证正常范围只在首拍选择、共享超时、无效反馈及左右故障顺序。
- 修正旧测试写死的 1.7 目标和 0.2 pitch 门限：使用 SPIN_ALIGN_TARGET_PHI0_RAD / SPIN_HANDOFF_PITCH_MAX_RAD，保留边界内外检查。
- 主机 C 回归：单腿 851 + 自起模块 1341 + 底盘接入及保护 739，共 2931 项断言通过；轮输出 117 项、正常腿长斜坡 65399 项检查通过。
- Keil / ArmClang 6.16 隔离完整重编：0 Error(s)、0 Warning(s)，退出码 0。工程：[recovery_normal_start_check.uvprojx](../build/recovery_normal_start_keil/recovery_normal_start_check.uvprojx)。日志：[构建](../build/recovery_normal_start_keil/build.log)、[主机](../build/recovery_normal_start_keil/host.log)、[轮输出](../build/recovery_normal_start_keil/wheel.log)、[腿长斜坡](../build/recovery_normal_start_keil/ramp.log)。
- 当时产物：[HEX](../build/recovery_normal_start_keil/Objects/CtrlBoard-H7_IMU.hex)、[AXF](../build/recovery_normal_start_keil/Objects/CtrlBoard-H7_IMU.axf)。未烧录、未实车验证；主工程产物保留，修改前快照为 build/recovery_normal_start_baseline。

### 2026-10-08：theta=-0.18 时自起停留，用户确认 PID 调参解决

- **现象**：用户观察到腿看似摆到位，机体 theta 约 -0.18 rad，保持姿态而没有进入平衡。
- **实车状态**：`Chassis.chassis_mode=0`（FALLING_DOWN）、`chassis_recovery.result=1`（RUNNING）、`phase[0]=3`（左腿 ALIGN）、`phase[1]=4`（右腿 HOLD）。左腿尚未满足第二段完成判据，因此两腿尚未共同进入 HANDOFF；当时并非卡在 pitch 交接门限。
- **排查证据**：第二段完成需单腿 DONE、斜坡结束且连续角误差严格小于 SPIN_ANGLE_WINDOW。仅凭外观看似到位不能确认上述条件。以 theta=-0.18、两腿实际角/腿长到位且腿长速度为零回放实际入口和交接代码，可进入并保持 NORMAL；这只验证状态切换，不等同于复现实车机械响应。
- **最终结论**：用户明确确认是 PID 参数问题，调整后问题已解决。本次排查未修改控制逻辑，未放宽 pitch、摆角或腿长交接条件。
- **记录边界**：用户未提供具体调整的是哪个 PID 项及调整前后数值；不推定为某个单独增益或某项具体误差。该结论来自用户实车反馈，未由代理独立完成硬件复验。
- **后续同类问题**：先读取底盘模式与双腿 phase。出现 RUNNING + ALIGN/HOLD 时检查未完成腿的动作 result、目标/实际连续角和腿长跟踪及 PID；两腿进入 HANDOFF 后再检查 pitch 等交接条件。

### 历史：2026-10-08 窗口内启动先对齐，再进入 NORMAL

- 修复 OFF→ON 正常姿态分支绕过自起的问题：任一腿窗内也进入 FALLING_DOWN，复用原自由收腿及双腿同步转向 1.7 的流程。公开只读窗口查询，原算法、阶段、参数和在线 NORMAL 保护保持。
- 新增入口回归用例在修复前失败（上电已在线、双腿窗内却直接 NORMAL），修复后通过。覆盖左右单腿/双腿、边界内外、窗口外原启动、两腿先后收完、交接前角度再偏离、姿态/长度/速度约束及 OFF→ON 重启。
- 单腿 851、自起模块 1269、底盘接入 427，共 2547 项主机断言通过；轮输出 117 项检查通过。
- Keil ArmClang 6.16 隔离完整重编：0 错误、0 警告。工程：[recovery_entry_check.uvprojx](../build/recovery_entry_keil/recovery_entry_check.uvprojx)；[构建日志](../build/recovery_entry_keil/build.log)、[主机日志](../build/recovery_entry_keil/host.log)、[轮输出日志](../build/recovery_entry_keil/wheel.log)。
- 当时产物：[HEX](../build/recovery_entry_keil/Objects/CtrlBoard-H7_IMU.hex)、[AXF](../build/recovery_entry_keil/Objects/CtrlBoard-H7_IMU.axf)。本轮未烧录、未实车验证；修改前快照位于 build/recovery_entry_baseline。

### 历史：2026-10-08 双腿同步收腿与 FREE 摆角模式

- 单腿 851、自起模块 1251、底盘接入 264，共 2366 项主机断言通过；轮输出 117 项检查通过。新增覆盖正常同步收腿、先到后偏离、提前窗口两侧组合、自由摆动跨界、不因摆角偏离阻止完成、等待期间 Tp=0、重新捕获角度、半圈异常、时钟回绕、共享截止、故障顺序及禁用重启。
- Keil ArmClang 6.16 隔离完整重编：0 错误、0 警告。工程：[recovery_free_retract_check.uvprojx](../build/recovery_free_retract_keil/recovery_free_retract_check.uvprojx)；[构建日志](../build/recovery_free_retract_keil/build.log)、[主机日志](../build/recovery_free_retract_keil/host.log)、[轮输出日志](../build/recovery_free_retract_keil/wheel.log)。
- 当时产物：[HEX](../build/recovery_free_retract_keil/Objects/CtrlBoard-H7_IMU.hex)、[AXF](../build/recovery_free_retract_keil/Objects/CtrlBoard-H7_IMU.axf)。主工程原有产物和用户改动保留；build 文件为本机验证记录。
- 尚未烧录或实车验证，动作效果、机械响应及实际任务耗时仍需上车记录。


### 历史：2026-10-08 放开摆角前的交接复验

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

phase[0]/[1] 的值为 0=SWING、1=RETRACT、2=WAIT_ALIGN、3=ALIGN、4=HOLD。重点比较两腿 motion[].start_tick 是否同拍、command.direction 是否分别锁存、command.angle_control 是否从 POSITION 切为 FREE、在 ALIGN 恢复 POSITION；Tp 在 RETRACT 和 WAIT_ALIGN 必须为零。所有路径都要核对两腿收腿 start_tick 相同；SWING + sweep_arrived=1 + FREE 表示就绪等待，长度参考应为进入时实际值。FREE 的 command.target_phi0_rad 仅为启动记录，上下文参考角和目标角随实际角更新；ALIGN 命令目标为 SPIN_ALIGN_TARGET_PHI0_RAD，当前 1.6。

单腿上下文角度以竖直向下为零，采用连续弧度；与 phi0 比较时先减 π/2 并核对周数。command.target_phi0_rad 则直接采用 VMC 坐标。

自起模块的 FAULT 本身锁存至复位，但底盘中止路径会调用 ChassisRecovery_Reset()，因此故障后 chassis_recovery.result 通常已经回到 IDLE，阶段和计时也已重置。单腿 motion[0]/[1] 保留供诊断，需结合 Chassis.chassis_mode、motion[].result 和映射输出定位；映射异常时单腿可能仍为 DONE 而底盘已经 ZERO_FORCE。

实车按以下顺序收集数据：

1. 确认实际任务周期、左右腿方向与参考跟踪；覆盖收腿范围边界及内外、大 pitch、两腿 2.1、目标两侧、已在 1.6 附近、范围内/外混合、上电已在线和 OFF→ON 重启。记录进入自起及 NORMAL 时刻，确认两腿到 1.6 附近并满足交接条件后才进入平衡。
2. 覆盖前倒、后倒、两腿姿态不同及一腿提前完成，记录进入收腿的实际 phi0、收腿期间自然摆动幅度、完成时间和机身俯仰。
3. 记录两个扫腿时间参数和实际/参考角度、腿长，核对长度较慢时角度到位仍可收腿；同时核对范围内就绪及长度保持、SWING 中进入范围锁存、两腿就绪才共同收腿、300 ms 自由摆角收腿、先收完腿继续 Tp=0、600 ms 同步第二段转动及最后一帧交接的输出变化；记录进入 NORMAL 后是否保持平衡，以及失败时输出是否归零。
4. 根据记录调整动作时间、提前窗口、容差或控制参数。600000 ms 是保留配置，最终超时由实测完成时间确定。

现有倒地检测仍只看 phi0 和俯仰，未增加侧倾、触地条件或检测滞回；交接未增加角速度门限。这些属于后续有数据后再评估的控制优化。

LegMotion_Run 和 ChassisRecovery_Run 本身无阻塞延时；ZERO_FORCE 中原有 DM_Enable() 电机重使能和 osDelay(1) 路径仍保留，分析周期时应区分这一路径与动作模块计算耗时。

## 继续修改与提交

- 修改源码时按原编码和行尾字节读写；chassis_task.c 为 GBK/CRLF，chassis_task.h 仍为 GBK/LF。leg_motion.c/.h 与 chassis_recovery.c/.h 均为 UTF-8 无 BOM/CRLF（含中文宏注释）。Markdown 使用 UTF-8 BOM + CRLF。
- Git 仓库根目录是固件目录的上两级。本次未暂存或提交，当前状态无已暂存改动；保留工作区其他现有修改，提交时逐项审查并显式选择文件。
- 交付时将新模块和验证文件一并纳入；保留 CLAUDE.md 原有用户修改。
- 修改代码后重跑对应 C 验证与 Keil 构建；控制变化完成后补实车记录。只改文档无需重编固件。
