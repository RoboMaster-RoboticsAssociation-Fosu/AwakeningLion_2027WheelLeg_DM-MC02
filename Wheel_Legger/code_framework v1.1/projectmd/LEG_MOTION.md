# 腿部动作封装

接口位于 `User/Controller/leg_motion.h`，统一调用 `LegMotion_Run()`。每条腿保留独立的 `LegMotion_Context`，每个控制周期调用一次。函数根据反馈更新参考值并计算 F0/Tp，由底盘继续完成 VMC 映射和 CAN 发送。

## 动作参数

| 参数 | 含义 |
| --- | --- |
| `direction` | `LEG_MOTION_POSITIVE` 表示 phi0 增大，`LEG_MOTION_NEGATIVE` 表示 phi0 减小。左右腿已在反馈及输出端处理镜像，使用相同方向约定。 |
| `angle_mode` | `LEG_MOTION_ABSOLUTE` 指定最终姿态；`LEG_MOTION_RELATIVE` 指定转过的角度。 |
| `angle_deg` | 单位为度。绝对模式以腿相对机身垂直向下为 0°，对应 VMC 的 phi0=π/2；沿指定方向到达目标，相同姿态不多转一圈。相对模式必须非负，360° 表示完整一圈，720° 表示两圈。 |
| `duration_ms` | 角度和腿长参考值从启动时的实际反馈同时线性插值到目标的时间；0 表示立即设置参考值。 |
| `length_m` | 目标腿长，单位为米，必须大于 0。 |
| `timeout_ms` | 从本次启动计算的总超时，必须大于 0；包含轨迹执行及等待实际到位的时间。 |

角度内部连续展开。相邻两次调用间实际转动必须严格小于 180°，否则无法可靠判断转过的圈数。角度参考使用连续误差，因此相对模式的一圈不会被当作原地到位。

## 调用示例

下面表示沿负方向转一圈，在 4 秒内将腿长参考同时变化到 0.30 米，最多等待 6 秒：

~~~c
static LegMotion_Context context = {0};
const LegMotion_Command command = {
    .direction = LEG_MOTION_NEGATIVE,
    .angle_mode = LEG_MOTION_RELATIVE,
    .angle_deg = 360.0f,
    .duration_ms = 4000U,
    .length_m = 0.30f,
    .timeout_ms = 6000U,
};
LegMotion_Feedback feedback = {
    vmc->phi0, vmc->d_phi0, vmc->L0, vmc->d_L0
};
LegMotion_Output output;
LegMotion_Result result = LegMotion_Run(
    &context, &command, &feedback, HAL_GetTick(), start, &output);
vmc->F0 = output.F0;
vmc->Tp = output.Tp;
~~~

`start` 只在开始新动作的当周期置 1，后续置 0。每次置 1 都从当前反馈重新启动，包括重复执行完全相同的命令。启动后命令参数被锁存；后续调用的 `command` 被忽略，也可以传 NULL。

停止调用不会自动清除底盘保存的输出。退出动作或禁用底盘时，调用者必须清零 F0/Tp、映射后的关节力矩和轮输出。

## 状态与控制参数

- `IDLE`：尚未启动，输出为零。
- `RUNNING`：执行轨迹或等待实际到位。
- `DONE`：轨迹时间结束，连续角误差小于 0.25 rad、腿长误差小于 0.02 m。状态锁存，继续调用会保持固定目标。
- `TIMEOUT`：到总超时时仍未完成，输出为零，状态锁存。
- `INVALID`：命令或反馈无效，输出为零，状态锁存。

TIMEOUT/INVALID 必须通过新的 `start` 才能重新执行。DONE 后反馈无效仍会进入 INVALID。

控制器保留原速率形式 PD 与重力近似：角度 Kp/Kd 为 15/2，腿长为 300/9，Tp 限幅 ±12 N·m，F0 限幅 ±20 N。这些参数集中在 `leg_motion.h`；参考轨迹没有增加速度前馈。

## 当前倒地自起接入

底盘主流程明确调用左腿、右腿，共用 `falling_leg_control(leg, start, now)` 处理单腿阶段、反馈和输出：

1. 扫腿使用负方向、绝对目标 +39.143°，对应 phi0≈2.254 rad；参考变化时间 4 秒，目标腿长 0.30 米。
2. 实际腿角进入原提前窗口或终点窗口后，锁存收腿阶段。捕获当时实际角度，用相对 0° 保持该角度，在 1 秒内将腿长参考收至 0.13 米。
3. 收腿完成后保持固定角度及腿长；外部扰动不会使该腿返回扫腿阶段。
4. 两腿都完成后进入交接，检查机身俯仰、腿长、腿长变化率和角度窗口；phi0 必须同时处于正常检测范围 [0.4, 2.5]。
5. 扫腿与收腿共用原 600000 ms 截止时间，交接另限 1500 ms。
6. 超时、无效反馈或自起输出映射为非有限力矩时，当周期进入 ZERO_FORCE 并清零两腿和两轮。交接到 NORMAL 的最后一帧也检查自起输出；零力和禁用状态跳过 VMC 力矩映射。

Keil 可观察 `chassis_leg_motion[0]` 和 `[1]` 的 `actual_angle_rad`、`reference_angle_rad`、`target_angle_rad`、`result`、`reference_length_m` 和锁存命令。

## 验证与实车调试

运行 `node mdk_check/leg_motion_verify.js`，直接编译实际 C 动作模块，并提取实际底盘阶段代码进行主机验证。检查内容包括多圈计数、方向、重启、超时、计时回绕、保持、阶段锁存、交接和故障当周期清零。

本次验证记录、隔离构建产物、实车续接顺序和提交注意事项统一见 [倒地自起封装交接](LEG_MOTION_HANDOFF.md)。

尚未进行实车验证。上车需记录任务周期、IMU 俯仰、两腿实际与参考角度、腿长、阶段切换和电机输出，确认同时插值及 1 秒收腿的实际效果。现有 600000 ms 超时仍保留，需按实测动作耗时确定最终值。
