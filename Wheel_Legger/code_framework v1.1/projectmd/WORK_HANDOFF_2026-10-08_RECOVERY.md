# 2026-10-08 自起流程工作交接

工作目录：`E:\ROBOT_NEW\9.22wheel_legger6\wheel_legger\Wheel_Legger\code_framework v1.1`。固件运行于 DM-MC02 / STM32H723VG；以 Keil MDK、ArmClang 6.16 工程为准。

当前已实现“转到 phi0=2.8 → 固定角度收腿 → 等两腿收完 → 同时转到 phi0=1.7 → 条件满足后 NORMAL”。原提前收腿窗口保留。本轮只整理文档并复验当前参数的主机测试，未烧录、未做实车验证。

## 1. 接手顺序与当前状态

1. 先读本文确认当前参数、已验证范围和下一步，再读 [自起模块详细交接](LEG_MOTION_HANDOFF.md) 核对阶段与保护行为。
2. 调整目标和动作时间时查看 [chassis_recovery.h](../User/Controller/chassis_recovery.h)；修改单腿接口前读 [LEG_MOTION.md](LEG_MOTION.md)。
3. 修改代码前读 [CLAUDE.md](../CLAUDE.md)，按各文件原编码和行尾进行字节读写。
4. 下一步是用当前参数重新做固件隔离构建，并记录实车阶段、角度、腿长及交接表现；修改参数后以源码为准更新验证记录。

自起源文件及公共模块测试仍是未跟踪文件，相关底盘、单腿和文档也有未提交改动。本次没有暂存或提交。提交时逐个选择文件，包含新增的 `chassis_recovery.c/.h`、`chassis_recovery_verify.c`，保留工作区其他修改。

## 2. 已确认的接口和运动规则

| 模块 | 职责与入口 |
| --- | --- |
| [leg_motion.c/.h](../User/Controller/leg_motion.h) | `LegMotion_Run()`：单腿定向选路、连续角展开、角度与长度同步插值、PD 与重力补偿、完成后保持。 |
| [chassis_recovery.c/.h](../User/Controller/chassis_recovery.h) | `ChassisRecovery_Run()` / `ChassisRecovery_Reset()`：两腿动作上下文、阶段、共享截止和交接条件。 |
| [chassis_task.c](../User/APP/chassis_task.c) | `chassis_recovery_control()`：传入反馈、回写 F0/Tp、轮输出清零、模式切换、故障最终清零；判姿及 VMC 检查保留在底盘。 |
| [controller.c/.h](../User/Controller/controller.h) | 原通用控制工具；本轮自起通过前两个模块执行。 |

`falling_down()`、`falling_to_down()` 都只调用一次 `chassis_recovery_control()`。

单腿命令逐字段赋值：`target_phi0_rad`、`direction`、`duration_ms`、`length_m`、`timeout_ms`。目标直接使用 VMC phi0 弧度，竖直向下为 π/2；正方向增大、负方向减小。左右腿已经做镜像转换，共用方向约定。旧绝对/相对模式和度数命令已移除；同姿态不额外转圈。

`start=1` 只在新动作启动当拍使用，捕获实际角度、腿长和时刻，之后保持 `start=0`；命令被锁存。收腿角度只捕获一次。上下文 `actual_angle_rad/reference_angle_rad/target_angle_rad` 使用 q=phi0−π/2 的连续角，不能直接当作 phi0 读取。

```text
SWING → RETRACT → WAIT_ALIGN → ALIGN → HOLD
                  两腿均到此：下一拍同时启动 ALIGN
两腿均 HOLD：当拍进入 HANDOFF，下一拍才检查 NORMAL 条件
```

- SWING：正常终点要求斜坡结束且当前连续角误差严格小于 0.05 rad；不额外要求腿长先到扫腿长度。原提前窗口 q∈[-0.9,0]，即 phi0 约 0.671～1.571 rad；进入即收腿，启动时已在窗内也直接收腿。
- RETRACT：固定进入阶段当拍的实际 phi0，向短腿长插值；沿用单腿 DONE 的 0.25 rad 角误差、0.02 m 长度误差及斜坡结束条件。
- WAIT_ALIGN：先收完腿持续保持，等另一腿也收完。两腿使用同一下一周期时刻启动第二段，各自重新捕获实际反馈。
- ALIGN：方向只在启动时按最短路径锁存；2.8→1.7 为负方向，提前收腿角小于 1.7 时为正方向。半圈沿扫腿方向，同姿态不转。必须同时满足单腿 DONE、斜坡结束、当前连续角误差小于 0.05 rad 才进 HOLD。
- 交接：两腿实际 phi0 距 1.7 小于 0.05 rad，原始 phi0 位于 [0.4,2.5]，有限俯仰绝对值小于 0.2 rad，腿长距目标小于 0.02 m，腿长速度绝对值小于 0.05 m/s。先检查就绪，再检查交接超时。

## 3. 当前源码参数及调参位置

本次读到的参数已在上一轮实现完成后调整；下表是交接时的当前值。

| 宏/参数 | 当前值 | 所在文件 |
| --- | --- | --- |
| `SPIN_TARGET_PHI0_RAD` / `SPIN_SWEEP_DIR` | 2.8 rad / 负方向 | chassis_recovery.h |
| `SPIN_RAMP_TIME_MS` / `SPIN_SCAN_LENGTH` | 4000 ms / **0.39 m** | chassis_recovery.h |
| `SPIN_RETRACT_TIME_MS` / `SPIN_RETRACT_LENGTH` | **1000 ms** / 0.139 m | chassis_recovery.h |
| `SPIN_ALIGN_TARGET_PHI0_RAD` / `SPIN_ALIGN_TIME_MS` | 1.7 rad / 1000 ms | chassis_recovery.h |
| `SPIN_EARLY_ANGLE` / `SPIN_ANGLE_WINDOW` | 0.9 rad / 0.05 rad | chassis_recovery.h |
| `SPIN_SCAN_TIMEOUT_MS` | 600000 ms，覆盖扫腿、收腿、等待和第二段 | chassis_recovery.h |
| `SPIN_HANDOFF_TIMEOUT_MS` | **500 ms**，两腿 HOLD 后另计 | chassis_recovery.h |
| 角度 Kp/Kd | 50 / 2 | leg_motion.h |
| 腿长 Kp/Kd | 1000 / 100 | leg_motion.h |
| Tp/F0 限幅 | ±12 N·m / ±40 N | leg_motion.h |

上一轮通过固件构建时是 0.35 m 扫腿、500 ms 收腿、1000 ms 交接超时。现有隔离 hex/axf 对应上一轮配置，使用当前参数前须重新构建。

若从 phi0=-0.57 开始，未触发提前窗口、两腿同步且完全跟踪：0～4 秒负向转到 2.8（连续表示为约 -3.483185 rad），长度向 0.39 m 插值；4～5 秒固定角收至 0.139 m；两腿收完后下一周期启动，约 5.001～6.001 秒转到 1.7；再下一周期检查交接。实际跟踪滞后或两腿不同步会推迟这些时刻。

## 4. 保护行为与编程风格

继续修改时须保持：

- 左腿失败立即返回，右腿阶段及上下文不推进；右腿失败清除两腿本拍输出。
- 动作共用首次启动时的总截止，新阶段只分配剩余时间；第二段不重置总计时。
- 两腿第二段完成当拍仅进入交接，下一拍才检查 NORMAL；交接边界先判断就绪，再判断超时。
- 禁用或故障当拍清零两腿 F0/Tp、映射关节力矩、轮力矩和电流。OFFLINE/ZERO_FORCE 跳过 VMC；`recovery_output_pending` 确保模式已切 NORMAL 的最后一帧仍检查自起映射。
- OFF→ON 重新判姿并重新捕获反馈；在线 NORMAL 判倒地进入 ZERO_FORCE，等待再次 OFF→ON。Reset 复位阶段/计时，保留 motion 诊断记录。

用户风格：四空格、沿用括号风格；明确依次调用左腿和右腿；命令、反馈逐字段赋值；直观 if/switch 和提前返回；关键步骤简短中文注释，宏后写中文含义。格式调整局限于修改位置。

源码按原编码和行尾做字节级编辑；底盘 .c 为 GBK/CRLF，.h 为 GBK/LF；当前动作及自起模块为 UTF-8 无 BOM/CRLF。Markdown 使用 UTF-8 BOM/CRLF。

## 5. 验证证据与续接事项

| 验证 | 结果及适用范围 |
| --- | --- |
| 本次按当前源码重新运行主机测试 | 单腿 739、自起模块 1047、底盘接入 230，共 **2016 项断言通过**。包括提前窗口、不同步、第二段同拍启动、最短选路、连续角路径、0.05 rad 边界、腿长未到位、重启、超时、故障顺序及交接帧 VMC 清零。 |
| 上一轮 Keil 隔离完整构建 | ArmClang 6.16：0 错误、0 警告；适用于上一轮参数。当前 0.39 m / 1000 ms 收腿 / 500 ms 交接配置尚未重新进行固件构建。 |
| 上一轮 VMC/Jacobian | 两个数值脚本通过；几何与固件不同，仅验证公式。 |
| 硬件 | 本轮未烧录，未做实车验证；成功率、任务耗时及机械表现尚无验证数据。 |

主机测试入口：

```powershell
node mdk_check/leg_motion_verify.js
```

直接编译实际 C 模块；底盘接入测试提取当前适配、判姿、清零和 VMC 函数，硬件依赖使用桩。测试不覆盖真实 CAN、RTOS 调度或机械响应。默认编译器 gcc，可用 CC 指定；本机沙箱内编译器出现启动错误时，沙箱外本地执行已通过。

本次主机输出记录：[recovery_handoff_20261008_host.log](../build/recovery_handoff_20261008_host.log)。上一轮构建日志：[build.log](../build/recovery_align_keil/build.log)。日志及隔离工程在本机 build/ 下，随 Git 交付时不自动携带；迁移工作目录后需从权威工程重新生成隔离工程。

下一轮按以下顺序续接，每步保留结果：

1. 核对最新宏值，重新隔离构建，要求 0 错误、无新增警告；保留主工程已有产物及用户改动。新自起模块已注册到权威 Keil 工程。
2. 按用户后续安排上车验证，记录 phi0、L0、d_L0、pitch、F0/Tp、阶段和任务周期，覆盖正常扫腿、提前收腿、两腿不同步和 OFF→ON 重启。
3. 观察 `chassis_recovery.phase[0]/[1]`：0=SWING、1=RETRACT、2=WAIT_ALIGN、3=ALIGN、4=HOLD；核对两腿 `motion[].start_tick` 同拍，`command.direction` 锁存，收腿目标只捕获一次。
4. 同时观察 `Chassis.chassis_mode`、`motion[].result`、实际/参考连续角及长度、`torque_set[]`、`fb_dt/t_sum`。底盘故障中止会 Reset，自起 result 可能已回 IDLE；保留的 motion 记录与底盘模式需一起判断。
5. 记录第二段到位后的交接等待时间、是否进入 NORMAL、进入后能否平衡，以及故障当拍输出是否归零，再依据实测调整参数。
