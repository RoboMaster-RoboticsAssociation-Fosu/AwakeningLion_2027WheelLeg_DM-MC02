# 离地检测接入与调试说明

## 1. 实现范围与当前状态

本功能接入当前 DM-MC02/STM32H723 固件，实施基线为 `9681ffc`；参考同一辆车的 Wheel-legged 远端提交 `3b8cb2e`。气弹簧已拆除，因此没有气弹簧力回补、额外伸腿推力、速度辅助落地或小陀螺专用判据。

检测、LESO 门控、空中控制已实现，默认只观测。主机验证和隔离 Keil 构建通过；尚未烧录或实车验证。控制周期、自起流程、遥控映射、腿长与 Roll 增益、LQR/LESO 系数沿用实施前配置。

## 2. 文件与公开接口

| 内容 | 位置 |
| --- | --- |
| 检测状态、参数与接口 | `../User/Controller/ground_contact.h` |
| 检测实现 | `../User/Controller/ground_contact.c` |
| 雅可比共享及反馈逆解 | `../User/Algorithm/VMC/VMC_calc.c/.h` |
| 反馈快照、位置处理、腿长分支及最终停轮 | `../User/APP/chassis_task.c` |
| 空中腿角状态反馈 | `../User/Algorithm/LQR/LQR.c` |
| 触地门控、重新初始化与补偿渐入 | `../User/Algorithm/LESO/LESO.c` |
| 主机验证 | `../mdk_check/ground_contact_verify.js/.c` |

```c
void GroundContact_Reset(void);
void GroundContact_Update(float dt, float accel_z, const float torque_fb[2][2]);
uint8_t VMC_Force_Calc(vmc_leg_t *vmc, float torque_back, float torque_front,
                       float *F0_feedback, float *Tp_feedback);
```

`GroundContact_Reset()` 清空估计、滤波历史、有效性、标志及计时，保留本轮锁存的控制开关。`GroundContact_Update()` 在 OFFLINE 时采纳控制开关，默认复位；`ground_offline_debug_enable=1` 时 OFFLINE 继续观测并忽略模式限制。ONLINE 下非 NORMAL 模式仍复位，NORMAL 更新两腿估算和判定。`VMC_Force_Calc()` 成功返回 1，输入或几何无效返回 0；反算结果不覆盖 `vmc.F0/Tp` 控制指令。

新模块已注册到权威 Keil 工程 Controller 分组，沿用已有 Controller 包含路径。原未调用的 `ground_detectionL/R`、两组四点均值缓存和旧 `vmc.FN` 字段已移除。

## 3. 坐标、单位和数据来源

- 腿数组：`LEFT_Leg=0`、`RIGHT_Leg=1`；轮数组：`LEFT_Wheel=1`、`RIGHT_Wheel=0`，两者不能混用。
- 前关节为 phi4，后关节为 phi1。`torque_fb[leg][0]` 是后关节，`[1]` 是前关节，单位 N*m。
- 右腿原始电机反馈力矩在检测模块内取反一次，与反馈角度和输出命令的镜像约定统一。传给 `VMC_Force_Calc()` 的力矩已经归一化。
- 腿长单位 m，角度 rad，速度 m/s 或 rad/s，二阶导数 m/s^2 或 rad/s^2，确认时间 s。
- `theta = Pitch + phi0 - pi/2`；本车 VMC 的广义力满足 F0 与 dL0 共轭、Tp 与 dphi0 共轭。
- `accel_z` 使用 `INS.MotionAccel_n[2]`，即大地系去重力运动加速度。姿态、加速度、关节位置/速度/力矩及轮速在一个短临界区取得快照，随后的计算正常开中断。
- 关节未使能时以 NAN 向检测表示输入无效，不将零反馈误认为离地。

## 4. 支撑力模型和参数

复用 `VMC_calc_1()` 原始二阶导数，只在检测中分别低通。共享现有正向映射矩阵：

```text
[T_back, T_front]^T = J * [F0, Tp]^T
D = j11*j22 - j12*j21
F0_feedback = (j22*T_back - j12*T_front)/D
Tp_feedback = (j11*T_front - j21*T_back)/D
```

从轮轴竖直位置展开加速度：

```text
z_wheel = z_body - L0*cos(theta)
a_w,z = accel_z - dd_L0*cos(theta)
      + 2*d_L0*d_theta*sin(theta)
      + L0*dd_theta*sin(theta)
      + L0*d_theta^2*cos(theta)

m_end = 1.65 + 0.537 = 2.187 kg
Fn_raw = F0_feedback*cos(theta)
       - Tp_feedback*sin(theta)/L0
       + m_end*(9.81 + a_w,z)
Fn_static = body_mg/2 + m_end*9.81 = 99.85447 N
Fn_ratio = Fn_filtered/Fn_static
```

摆矩投影采用减号：由 `L0*cos(theta)` 对 phi0 的导数为 `-L0*sin(theta)`，与当前虚功映射一致。新增主机验证使用本车杆长和双精度有限差分参照，覆盖倾角非零且 Tp 非零的纯竖直力投影。不能将其他工程公式的符号脱离坐标约定直接替换。

IMU 已扣除重力，因此质量项显式加 g。腿轮采用参考工程的集总近似；它不是完整五连杆刚体动力学模型，需要观察本车运动残差。反馈逆解来自电机实际力矩估计，Roll、腿长 PD 等真实电机作用已经包含其中，不再额外扣减。气弹簧已拆除，该项为零。

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| 机体重力基准 | `body_mg = 16*9.8` N | 与现有控制参数同源 |
| 腿/轮质量 | 1.65 / 0.537 kg | 同车参考记录 |
| 导数和支撑力低通时间常数 | 各 0.020 s | `y += dt/(tau+dt)*(x-y)` |
| 离地/落地比例 | 0.20 / 0.35 | 对应约 19.97 / 34.95 N |
| 离地/落地连续确认 | 0.030 / 0.050 s | 使用 DWT 实际时间累计 |
| 检测 dt 有效范围 | `0 < dt <= 0.050` s | 异常周期不采用控制回退值计时 |
| 最小腿长 | 0.0001 m | 避免除零与退化几何 |
| 雅可比退化阈值 | 0.000001 | 分母与行列式检查 |
| 额外伸腿推力 | 0 N | 保持原目标和斜坡 |

## 5. 判定、有效性和复位

首次有效样本将二阶导数滤波置零，支撑力滤波直接建立当前值，首帧不累计确认时间。正常周期按实际 dt 低通；离地比例小于等于 0.20 连续 30 ms 置位单腿离地；已离地后比例大于等于 0.35 连续 50 ms 清除。中间区保持已确认状态，条件中断清对应计时。比较时的 1e-7 s 仅补偿浮点累加舍入。

无效周期将 `valid=0`、`initialized=0` 并清确认计时，保留最近确认的单腿状态和最近有效观测值。恢复后的首个有效样本重新建立滤波初值。关节/整车电机反馈在线、关节使能、有限输入和非奇异几何由已有状态与集中数值检查共同保证。

整车离地为两腿标志的逻辑与。单腿确认触地即可退出整车离地；另一腿尚悬空或估算无效时 LESO 仍关闭。检测不因估算无效而主动触发 ZERO_FORCE；原有倒地、自起故障、电机掉线及最终命令清零优先。

## 6. 控制时序与行为

```text
DWT原始dt / 在线检查
 -> 短临界区反馈快照
 -> VMC正解与原始导数
 -> 离地检测
 -> 位置积分和落地航向对齐
 -> 原有航向处理 / 倒地判断 / 腿长斜坡
 -> 模式控制
 -> LESO更新或暂停
 -> VMC映射
 -> 最终停轮检查 / 限幅 / CAN发送
```

| 状态（控制已启用） | 行为 |
| --- | --- |
| 双腿触地 | 完整 LQR、原腿长和 Roll 控制；允许 LESO |
| 单腿悬空 | 原平衡控制；LESO 暂停更新和补偿 |
| 双腿离地 | 两轮力矩为零；仅腿角/角速度 LQR；原腿长斜坡和 PD；无机体重力前馈和 Roll |
| 任一腿落地 | 当拍恢复原控制、清位移并对齐航向保持目标；LESO 等双腿有效触地 |

LESO 门在本拍更新和注入前检查，避免使用上一拍的悬空补偿状态。重新触地复用原进入边沿清历史、seed、名义 0.5 s 等待和补偿渐入。观察模式不会因检测标志改变 LESO 或控制输出。当前用户轮补偿默认 0.2、腿补偿默认 1.0，保持实施前设置。

腿长目标和斜坡不随腾空改变，因此无需复制参考工程的腿长目标锁存/还原。当前腿长和 Roll 是直接 PD，无新增积分复位对象。速度辅助落地暂不接入：主动伸腿与到达当前目标的自然减速必须先通过本车数据区分。

## 7. 调试开关与 Watch

`ground_control_enable` 默认 0。OFFLINE 周期才将它采纳到 `Chassis.ground.control_enabled`；在线修改请求不会立即切换控制。自起与零力期间清检测，但保留已采纳的开关。

### OFFLINE 抱车观测

先下载新版程序，保持 `Chassis.chassis_enable=OFFLINE`、`ground_control_enable=0`，在 Watch 设置 `ground_offline_debug_enable=1`，保持程序运行以更新观测。OFFLINE 下无论检测标志或控制开关如何，发送的轮电流及四个关节力矩仍为零；默认关闭调试开关时恢复每拍复位。

主要看：`Chassis.ground.all_off_flag` 为两腿均离地；`off_flag[0]` 为左腿，`off_flag[1]` 为右腿。配合 `valid[0/1]`、`Fn[0/1]`、`Fn_ratio[0/1]` 和 `Chassis.all_motors_online` 观察。无效输入会保持最后确认的标志，`valid=0` 时不能将标志当成本拍有效结论。该调试开关保留既有反馈在线、关节使能及几何检查。

这不是触地开关，OFFLINE 零力矩抱车不保证直接置 1：若反馈逆解力和运动项均近零，模型中的腿轮自重项仍约 21.45 N、比例约 0.215，略高于离地门限 0.20；腿完全伸直时还可能几何退化，使估算无效。该场景首先用于采集数据，不能代替正常支撑控制下的真实离地验证，不为让抱车标志置位而单独放宽阈值。

先只观测：看 `Chassis.ground.F0_feedback/Tp_feedback`、`Fn_raw/Fn/Fn_ratio/Fn_static`、`valid/initialized/off_flag/all_off_flag`、`off_time_s/land_time_s`，并结合 `vmc.L0/theta/d_L0/d_theta`、INS 加速度和 `fb_dt/t_sum`。

本次未烧录。实车应先确认静立、三个腿长档位和正常运动不误置离地；分别观察单腿悬空、双腿抬起和放回的判定、有效性及恢复时间。验证数据后，在 OFFLINE 设置 `ground_control_enable=1`，再 ONLINE 经原自起流程验证控制输出和落地恢复。主机合成反馈不能替代真实接触和任务时序验证。

## 8. 已完成验证（2026-10-11）

| 项目 | 结果 |
| --- | --- |
| `node mdk_check/ground_contact_verify.js` | 617 项检查通过，含 OFFLINE 观测和零输出 |
| 基线对照 | 同一脚本加 `build/ground_contact_baseline` 参数；观察模式 LQR/腿长输出与实施前对照通过 |
| 自起/单腿动作 | 1247 + 1896 + 1045 项检查；五组扫腿时间配置通过 |
| 遥控来源 | 两种编译配置、协议、掉线、快照、腿长映射及 RTOS 配置通过 |
| 电机在线/自动使能 | 21036 项检查通过 |
| 轮输出 | 117 项检查通过，保留先限幅再换算电流 |
| 腿长斜坡 | 48666 项检查通过，含任务 OFFLINE 观测历史保留 |
| LESO | 828 项检查通过；实施前源码运行同组回归也通过 |
| 原 VMC/雅可比脚本 | 通过；其几何不同，新增验证使用固件实际杆长 |
| 隔离 Keil 全量构建 | ArmClang 6.16，0 Error(s)、0 Warning(s) |
| 硬件 | 未烧录、未实车验证 |

旧 LESO 测试原先断言轮补偿默认 0，与实施前已存在的 0.2 不符；本次只修正测试预期，未改用户参数。其他回归仅扩展新状态类型与依赖，保留原有验证。

隔离构建为 `../build/ground_contact_keil/ground_contact_check.uvprojx`，日志 `build.log`，产物 `Objects/CtrlBoard-H7_IMU.axf/.hex`。新增 C/H 为 UTF-8 无 BOM、CRLF，旧 GBK/UTF-8 源码和各自行尾保持；本 Markdown 为 UTF-8 BOM、CRLF。
