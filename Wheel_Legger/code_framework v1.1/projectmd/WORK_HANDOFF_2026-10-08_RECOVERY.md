# 2026-10-08 自起流程工作交接

工作目录：`E:\ROBOT_NEW\9.22wheel_legger6\wheel_legger\Wheel_Legger\code_framework v1.1`。DM-MC02 / STM32H723VG，Keil MDK / ArmClang 6.16 为权威工程。

最新自起规则：**有效原始 phi0∈[0.4,2.82]（含边界）或扫腿到位，即锁存为收腿就绪；先就绪腿 Tp=0、保持当时实际腿长，等两腿都就绪后同拍收腿**。启动和 SWING 使用相同规则，就绪/收腿不检查 pitch，摆角漂移不取消就绪。

每次 OFF→ON 进入自起，上电已在线同样适用。双腿启动均就绪时，首拍共同收腿；单腿先就绪时在 SWING 中用 FREE 等待。共同用 300 ms 收至 0.139 m，两腿收完后的下一拍同时按最短路径用 600 ms 转到 1.6，再满足全部交接条件后 NORMAL。

扫腿角度/长度斜坡仍为 4000/500 ms。当前 PID、正常腿长斜坡、轮输出和在线保护保留。本轮主机回归及隔离完整构建通过，未烧录、未实车验证。此前轮电流修复见 [起身打转排查](RECOVERY_SPIN_DIAGNOSIS_2026-10-08.md)。

## 本轮实车报告与回放：从 2.78 启动停留

用户提供 chassis_mode/result/phase[0]/phase[1]=0/1/0/0，已超过 4 秒，phi0 约 2.78；随后确认扫腿摆到该位置能收腿，直接从该位置启动则不能。实际模块回放显示：固定负方向从 2.78 转向 2.82 会选择 -6.243186 rad，静止反馈下仍 SWING、Tp=-12；正常负向摆到该位置则可收腿。合并范围前的快照也复现同样起点差异。

本轮扩大并统一就绪范围，取消单腿先收例外，改为就绪腿自由等待、两腿共同收腿。原故障回放修复后 phases=1/1、Tp=0，主机与固件构建通过；尚未由代理烧录或实车验收。

## 此前实车反馈：PID 调参解决另一项停留问题（2026-10-08）

用户报告腿看似摆到位、机体 theta 约 -0.18 rad 时保持姿态，无法进入 NORMAL。实车读数为 chassis_mode=0、recovery.result=1、phase[0]=3、phase[1]=4，即自起运行中、左腿仍 ALIGN、右腿已 HOLD；尚未进入检查 pitch 的交接阶段。

**用户确认原因是 PID 参数问题，调参后已解决。** 本次排查未修改控制逻辑，也未通过放宽 pitch 或到位判据处理。具体调整的 PID 项及前后数值未提供，不将当前源码参数推定为已确认的实车调参记录。详细经过见 [自起交接的验证记录](LEG_MOTION_HANDOFF.md#验证记录与复现)。

下文参数和构建结果已更新为本轮同步就绪与收腿修改后的工作区快照；上述用户实车结论仅覆盖 PID 调参解决的停留问题，不代表本轮同步就绪与收腿已完成实车验收。

## 接手顺序

1. 修改代码前读 [CLAUDE.md](../CLAUDE.md)，保持每个文件原编码和行尾。
2. 阶段、保护和实车观测见 [自起模块详细交接](LEG_MOTION_HANDOFF.md)；单腿接口见 [LEG_MOTION.md](LEG_MOTION.md)。
3. 从当前隔离产物开始实车验证，记录收腿期间自由摆动、第二段重新捕获角度和交回 NORMAL 的表现；根据实测再调整参数。

## 当前行为与接口

- 开启入口：OFF→ON 复位并进入 FALLING_DOWN，上电已在线同样适用。模块启动及 SWING 先只读计算双方收腿就绪，再依次执行左、右腿。
- `ChassisRecovery_InEarlyWindow(float phi0)` 签名保持，查询有效原始角 [SPIN_RETRACT_PHI0_MIN_RAD,SPIN_RETRACT_PHI0_MAX_RAD]（0.4～2.82），不归一化、不依赖方向、不看 pitch。上界引用第一目标 SPIN_TARGET_PHI0_RAD。
- 上下文新增 `sweep_arrived[2]`，统一锁存启动与运行中的就绪。就绪腿仍 SWING，用 FREE、两个参考时间 0 保持进入时实际腿长，后续命令锁存；摆角漂移不取消状态。两腿就绪才共同 RETRACT，同拍重新捕获实际腿长，收腿/复位清除标志。在线 NORMAL 不因进入范围重启自起。
- 单腿命令为 angle_control、direction、target_phi0_rad、angle_duration_ms、length_duration_ms、length_m、timeout_ms；逐字段赋值，start=1 仅用于新动作启动当拍，之后 start=0，命令锁存。
- `angle_control`：`LEG_MOTION_POSITION=0` 为现有摆角位置控制；`LEG_MOTION_FREE=1` 始终 Tp=0，取消摆角 PD、角速度阻尼和摆角重力补偿，只控制腿长。
- FREE 保留实际连续角更新、反馈有限性、正腿长及半圈采样歧义检查。上下文 actual/reference/target_angle_rad 均采用 q=phi0−π/2 连续角；FREE 中目标角和参考角随实际角变化，不是固定控制目标。command.target_phi0_rad 仍锁存，仅作启动记录。
- 范围外仍扫腿：单腿角度斜坡结束且当前连续角误差严格小于 0.05 rad 也可锁存就绪。就绪腿 Tp=0 保持当时腿长，不要求回到扫腿目标，不单独缩腿；无需等待扫腿长度斜坡或伸到 0.39 m。
- SWING 运行中切换 FREE 前，先调用原单腿动作验证当前反馈，包含半圈采样歧义；失败立即中止且不推进另一腿。验证输出不发送，成功后建立 FREE，仅回写最终输出。
- `RETRACT` / `WAIT_ALIGN`：采用 FREE，Tp 全程为零；F0 保留腿长插值、PD、伸缩重力补偿和限幅。时间结束且长度误差小于 0.02 m 即完成，摆角偏离不阻止完成。先收完腿只保持短腿长；收腿参考时间为 300 ms，目标 0.139 m。
- 两腿均 WAIT_ALIGN 的下一拍，恢复 POSITION，共用 now_ms，各自捕获实际角和长度，按最短路径用 600 ms 转向 1.6 rad。自由摆动后不预设方向；半圈沿扫腿方向，同姿态不绕圈。
- `ALIGN` 仍要求单腿 DONE、斜坡结束和当前连续角误差小于 0.05 rad；两腿 HOLD 当拍只进入 HANDOFF，下一拍才检查 NORMAL。
- 交接仍要求 phi0 距 `SPIN_ALIGN_TARGET_PHI0_RAD`（1.6 rad）小于 0.05 rad、原始 phi0∈收腿上下界宏定义的 [0.4,2.82]、有限 |pitch|<`SPIN_HANDOFF_PITCH_MAX_RAD`（当前 0.5 rad）、腿长误差<0.02 m、|d_L0|<0.05 m/s；先就绪后超时。收腿不检查 pitch；在线倒地检测的原 0.3 rad 阈值保持。

阶段编号保持：0=SWING（POSITION 扫腿或 FREE 就绪等待）、1=RETRACT、2=WAIT_ALIGN、3=ALIGN、4=HOLD。phase=0 需结合 sweep_arrived 与 angle_control 区分；FREE 等待的单腿 DONE 不代表已经收完。底盘两个倒地模式均通过 chassis_recovery_control() 调用模块。

## 当前参数（本轮构建快照）

| 参数 | 当前值 |
| --- | --- |
| 第一段目标 / 方向 | 2.82 rad / 负方向 |
| 扫腿角度 / 腿长斜坡时间 | SPIN_SWING_ANGLE_TIME_MS=4000 ms / SPIN_SWING_LENGTH_TIME_MS=500 ms |
| 扫腿目标长度 | 0.39 m |
| 收腿时间 / 长度 | 300 ms / 0.139 m |
| 第二段目标 / 时间 | 1.6 rad / 600 ms |
| 交接 pitch 阈值 | SPIN_HANDOFF_PITCH_MAX_RAD=0.5f，严格小于 |
| 原始 phi0 就绪范围 / 到位误差 | [0.4, 2.82] rad（两端包含）/ 0.05 rad |
| 共享总超时 / 交接超时 | 600000 ms / 500 ms |
| 单腿角度 Kp/Kd | 150 / 2，仅 POSITION 生效 |
| 单腿长度 Kp/Kd | 1800 / 100 |
| 单腿 Tp/F0 限幅 | ±12 N·m / ±40 N；FREE 的 Tp=0 |

自起参数位于 chassis_recovery.h，单腿参数位于 leg_motion.h。本轮统一范围内就绪与双腿共同收腿，现有时间及其余用户调参保留；该表不是已确认的实车 PID 调参前后记录。

## 验证与产物

| 检查 | 本轮结果 |
| --- | --- |
| `node mdk_check/leg_motion_verify.js` | 单腿 1247 + 自起 1896 + 底盘接入 879；扫腿配置及范围轨迹 2394，共 6416 项断言通过。 |
| `node mdk_check/wheel_output_verify.js` | 117 项检查通过，保留轮力矩先限幅再转换 int16_t 电流的修复。 |
| `node mdk_check/leg_length_ramp_verify.js` | 65399 项检查通过，正常平衡腿长斜坡保留。 |
| Keil 隔离完整重编 | ArmClang 6.16，0 错误、0 警告，退出码 0。 |
| 硬件 | 本轮同步收腿修改未烧录、未实车验证；先前用户确认 PID 调参解决自起停留问题，见最新实车反馈。主机测试不覆盖真实 CAN、RTOS 时序和机械效果。 |

保留并复验 2.78 启动故障回归。覆盖 0.4、1.6、2.5、2.78、2.82、边界内外及 2π 原始偏移；左右就绪等待、实际长度保持、摆角漂移后仍就绪、伙伴进入范围或扫腿到位、共同收腿时刻、先收完腿继续等待及第二段同拍。原独立提前收腿预期已改为共同收腿；保留无效反馈、半圈采样、左右故障顺序、共享超时和禁用重启。当前及不等/零时间配置均通过。

当前隔离工程：[recovery_sync_range_check.uvprojx](../build/recovery_sync_range_keil/recovery_sync_range_check.uvprojx)。产物：[HEX](../build/recovery_sync_range_keil/Objects/CtrlBoard-H7_IMU.hex)、[AXF](../build/recovery_sync_range_keil/Objects/CtrlBoard-H7_IMU.axf)；日志：[构建](../build/recovery_sync_range_keil/build.log)、[自起主机验证](../build/recovery_sync_range_keil/host.log)、[轮输出验证](../build/recovery_sync_range_keil/wheel.log)、[正常腿长斜坡](../build/recovery_sync_range_keil/ramp.log)。

隔离工程来自当前权威 Keil 工程，源路径为本机绝对路径；build 文件不随 Git 交付。迁移目录后应从权威工程重新生成。此前 recovery_unified_range_keil 为 [0.4,2.5] 独立提前收腿，不能代表本轮同步规则。修改前快照为 build/recovery_sync_range_baseline；其包含被替换的仅启动目标附近等待分支。2.78 故障回放及修复后结果位于 build/recovery_start_target_probe。

## 保留的保护与实车续接

- 左腿失败立即返回、不推进右腿；右腿失败清零两腿当拍输出。收腿启动条件在执行前只读判断：范围查询使用本拍原始角，扫腿终点使用当前反馈推算连续角，而非上一拍到位标志。
- 动作和等待共用首次自起启动的总截止，新阶段只分配剩余时间；交接单独计时。OFFLINE / ZERO_FORCE 跳过 VMC；故障或禁用清零 F0/Tp、映射关节力矩及轮输出。交接 NORMAL 最后一帧仍检查自起 VMC 映射。
- OFF→ON 复位并进入自起，逐腿按原始角范围分流并捕获反馈；在线 NORMAL 判倒地进入 ZERO_FORCE。Reset 保留单腿诊断记录，故障定位需结合底盘模式。
- 上车观察 phase[0]/[1]、motion[].command.angle_control、start_tick、实际/参考腿长、phi0/d_phi0、F0/Tp、pitch、torque_set[]、轮电流及 fb_dt/t_sum。观察 sweep_arrived[0]/[1]：先就绪腿等待，所有收腿情况的两腿 start_tick 都须相同；RETRACT/WAIT_ALIGN 的 Tp 必须为零，但缩腿关节力矩可以非零。
- 实车重点增加 2.78 直接启动、范围边界、大 pitch、单腿先就绪时长度保持，以及 SWING 中进入范围时的动作记录；继续覆盖不等角度/长度扫腿时间，记录两条参考各自到目标及触发收腿的时刻；同时覆盖双腿 2.1、目标两侧、已在 1.6 附近、正常边界、非正常混合姿态的开启，以及上电已在线和 OFF→ON 重启；记录启动分流、两腿 phi0、Tp/F0、阶段、共同转动时刻及进入 NORMAL 的时刻。正常扫腿与混合路径继续按原要求验收。

本轮未暂存或提交。保留任务开始时用户已有的主工程产物、Keil 工作区设置、2.82 rad 调参及上一轮修改；提交时逐个选择源文件、验证文件和文档。动作/自起源码采用 UTF-8 无 BOM + CRLF，底盘原 GBK 文件保持；Markdown 使用 UTF-8 BOM + CRLF。
