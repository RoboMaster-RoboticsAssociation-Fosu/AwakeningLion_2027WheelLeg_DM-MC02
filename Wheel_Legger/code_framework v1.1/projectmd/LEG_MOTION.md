# 腿部动作封装

当前自起参数与续接状态见 [2026-10-08 工作交接](WORK_HANDOFF_2026-10-08_RECOVERY.md)；动作数值以源码配置为准。

接口位于 `User/Controller/leg_motion.h`，统一调用 `LegMotion_Run()`。每条腿保留独立的 `LegMotion_Context`，每个控制周期调用一次。函数根据反馈更新参考值并计算 F0/Tp，由底盘继续完成 VMC 映射和 CAN 发送。

## 动作参数

| 参数 | 含义 |
| --- | --- |
| `direction` | `LEG_MOTION_POSITIVE` 表示 phi0 增大，`LEG_MOTION_NEGATIVE` 表示 phi0 减小。左右腿已在反馈及输出端处理镜像，使用相同方向约定。 |
| `target_phi0_rad` | VMC 坐标下的目标 phi0，单位为弧度，竖直向下为 π/2。沿指定方向到达目标姿态，等价姿态不额外转圈。 |
| `duration_ms` | 角度和腿长参考值从启动时的实际反馈同时线性插值到目标的时间；0 表示立即设置参考值。 |
| `length_m` | 目标腿长，单位为米，必须大于 0。 |
| `timeout_ms` | 从本次启动计算的总超时，必须大于 0；包含轨迹执行及等待实际到位的时间。 |

只接受目标姿态；旧 `angle_mode`、`angle_deg` 以及相对整圈命令已移除。目标相差 2π 的整数倍时表示同一姿态；启动时的同姿态判断按输入 float 精度适当放宽容差，避免等价目标因舍入误差额外转圈，不增加接口参数或范围限制。内部仍用 q=phi0−π/2 的连续弧度展开，跟踪误差沿所选方向计算。相邻两次调用间实际转动必须严格小于 π，否则无法可靠判断跨越的角度边界。

## 调用示例

下面表示沿负方向到达 phi0=2.8 rad，在 4 秒内将腿长参考同时变化到 0.39 米，最多等待 6 秒：

~~~c
static LegMotion_Context context = {0};
LegMotion_Command command;
LegMotion_Feedback feedback;
LegMotion_Output output;
LegMotion_Result result;

command.direction = LEG_MOTION_NEGATIVE;
command.target_phi0_rad = 2.8f;
command.duration_ms = 4000U;
command.length_m = 0.39f;
command.timeout_ms = 6000U;

feedback.phi0_rad = vmc->phi0;
feedback.angular_velocity_rad_s = vmc->d_phi0;
feedback.length_m = vmc->L0;
feedback.length_velocity_m_s = vmc->d_L0;

/* start 只在开始动作的当拍置 1。 */
result = LegMotion_Run(&context, &command, &feedback,
                       HAL_GetTick(), start, &output);
vmc->F0 = output.F0;
vmc->Tp = output.Tp;
~~~

`start` 只在开始新动作的当周期置 1，后续置 0。每次置 1 都从当前反馈重新启动，包括重复执行完全相同的命令。启动后命令参数被锁存；后续调用的 `command` 被忽略，也可以传 NULL。示例的 6000 ms 是单次动作示例值，当前底盘自起仍保留 600000 ms 总截止时间。

保持当前角度并收腿时，在启动当拍给 `target_phi0_rad` 赋当前反馈的 `phi0_rad`，同时指定目标腿长和斜坡时间；后续以 `start=0` 运行。角度目标只捕获一次，不随新反馈重设。

停止调用不会自动清除底盘保存的输出。退出动作或禁用底盘时，调用者必须清零 F0/Tp、映射后的关节力矩和轮输出。

## 状态与控制参数

- `IDLE`：尚未启动，输出为零。
- `RUNNING`：执行轨迹或等待实际到位。
- `DONE`：轨迹时间结束，连续角误差小于 0.25 rad、腿长误差小于 0.02 m。状态锁存，继续调用会保持固定目标。
- `TIMEOUT`：到总超时时仍未完成，输出为零，状态锁存。
- `INVALID`：命令或反馈无效，输出为零，状态锁存。

TIMEOUT/INVALID 必须通过新的 `start` 才能重新执行。DONE 后反馈无效仍会进入 INVALID。

控制器保留原速率形式 PD 与重力近似：当前角度 Kp/Kd 为 50/2，腿长为 1000/100，Tp 限幅 ±12 N·m，F0 限幅 ±40 N。本次阶段调整保留这些现有配置。这些参数集中在 `leg_motion.h`；参考轨迹没有增加速度前馈。

## 当前倒地自起接入

自起流程由 `User/Controller/chassis_recovery.c/.h` 集中管理。`falling_down()` 和 `falling_to_down()` 均只调用 `chassis_recovery_control()`；适配函数负责反馈传入、输出回写、轮输出清零和模式切换。模块内部明确按左腿、右腿顺序调用单腿控制，左腿失败时不推进右腿，右腿失败时清除左腿本拍输出。

1. `SWING`：负方向用 4 秒转向 phi0=2.8 rad，腿长同步向 0.39 m 插值。斜坡结束且连续实际角误差严格小于 0.05 rad 时收腿，不额外要求先伸到 0.39 m。
2. 原提前窗口继续保留：负方向归一化 q=phi0−π/2 位于 [-0.9, 0]，即 phi0 约 0.671～1.571 rad；进入即提前收腿，启动时已位于窗口也直接收腿。
3. `RETRACT`：锁存进入当拍的实际 phi0，用 1 秒将腿长参考收至 0.139 m。沿用单腿 DONE 条件，完成后进入 `WAIT_ALIGN`，保持角度及短腿长。
4. 两腿都进入 `WAIT_ALIGN` 的下一周期，同拍启动 `ALIGN`。各腿捕获当时实际反馈，用 1 秒沿最短方向转到 phi0=1.7 rad，目标腿长为 0.139 m。2.8→1.7 为负方向，提前收腿角度小于 1.7 时为正方向；同姿态不转，半圈沿扫腿方向。方向只在第二段启动时锁存。
5. `ALIGN` 必须同时满足单腿 DONE、斜坡结束及当前连续角误差小于 0.05 rad，才进入 `HOLD`。先完成腿继续保持；两腿都 HOLD 当拍只进入交接，下一周期才检查 NORMAL 条件。
6. 交接要求两腿 phi0 距 1.7 rad 的圆周误差小于 0.05 rad、原始 phi0 在 [0.4, 2.5]、俯仰绝对值小于 0.2 rad、腿长距 0.139 m 小于 0.02 m、腿长速度绝对值小于 0.05 m/s。先检查就绪，再检查超时。
7. 扫腿、收腿、等待及第二段转动共用 600000 ms 截止时间，交接另限 500 ms。参数集中在 `chassis_recovery.h`；两段目标分别是 `SPIN_TARGET_PHI0_RAD` 和 `SPIN_ALIGN_TARGET_PHI0_RAD`。
8. 超时、无效反馈或自起输出映射为非有限力矩时，当周期进入 ZERO_FORCE 并清零两腿和两轮。交接到 NORMAL 的最后一帧也检查自起输出；零力和禁用状态跳过 VMC 力矩映射。

Keil 可观察 `chassis_recovery.motion[0]` 和 `[1]` 的 `actual_angle_rad`、`reference_angle_rad`、`target_angle_rad`、`result`、`reference_length_m` 和锁存命令，以及 `chassis_recovery.phase[0]/[1]`、`result`、`entry_tick`、`handoff_tick`。`phase[]` 的数值依次为 0=SWING、1=RETRACT、2=WAIT_ALIGN、3=ALIGN、4=HOLD；`motion[].start_tick` 用于核对两腿第二段同拍启动，`command.direction` 用于核对锁存方向。上下文中的角度以竖直向下为零，锁存命令中的目标则直接使用 phi0，比较时需换算 π/2 的偏移。底盘故障中止后会复位自起上下文，因此自起 result 可已回到 IDLE；单腿记录保留，诊断时同时观察 `Chassis.chassis_mode` 和 `motion[].result`。

## 验证与实车调试

运行 `node mdk_check/leg_motion_verify.js`，直接编译实际 C 单腿模块及自起模块，并验证底盘适配和 VMC 输出保护。检查内容包括 phi0 定向路径、跨角度边界、等价姿态、重启、超时、计时回绕、保持、阶段锁存、交接和故障当周期清零。

本次验证记录、隔离构建产物、实车续接顺序和提交注意事项统一见 [倒地自起封装交接](LEG_MOTION_HANDOFF.md)。

尚未进行实车验证。上车需记录任务周期、IMU 俯仰、两腿实际与参考角度、腿长、阶段切换和电机输出，确认 4 秒扫腿、1 秒收腿、双腿等待和 1 秒第二段转动的实际效果。现有 600000 ms 超时仍保留，需按实测动作耗时确定最终值。
