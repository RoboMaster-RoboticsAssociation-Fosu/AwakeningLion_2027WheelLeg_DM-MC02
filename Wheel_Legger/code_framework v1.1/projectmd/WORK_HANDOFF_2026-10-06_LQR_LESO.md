# 项目完整交接：LQR / LESO 与站立抖动

交接日期：2026-10-06。供下一条 Work 对话继续开发。无实车根因结论；离线计算改造已完成。

## 1. 当前目标、范围与首要上下文

主目标：定位并解决正常站立时整车剧烈抖动，验证 LESO 补偿链路。

用户确认的现象：进入站立约半秒后开始剧烈抖动；把 `leso_comp_scale` 设为0有效缓解；用户没有注释 `LESO_Service()`。用户确认zjx正常，但未确认两车硬件和标定参数完全相同。

已完成子任务：用户HKU脚本离散化，以及与zjx脚本同参数输出对照。实车抖动未复现、固件补偿链未修复。

本轮实施授权为“只改脚本并验证”；计算脚本的物理参数/Q/R/网格保留，旧TXT、固件系数及烧录状态未由本会话更新。后续按新任务确定替换范围。

新对话必须保留：

1. 用户自己的脚本是 `CALC/lqr_k_calc/hku_lqr_k_calc.py`；`CALC/ABK_LQR.py`属于zjx。归属曾纠正，不能再颠倒。
2. HKU现在已经是ZOH+离散DARE；无需重复离散化。
3. 当前输出开关为 `USE_K=0, USE_L=0, USE_AD_BD=1`。实施验证时为1/0/0，之后开关发生变化；保持当前选择，不能把旧状态写回。
4. 同参数结果一致；两脚本默认Q/R不同，分别默认运行时K仍不同。
5. 现有固件表与两份本地脚本11kg默认输出不同；新系数没有写回固件。不要把脚本修改当成机器人已使用新K。
6. 本车真实质量、实际控制周期和电机寄存器尚未取得实测记录；代码13kg前馈、脚本11kg、可复现固件的16.43kg不是同一个已确认硬件事实。
7. 用户已调用 `diagnosing-bugs` 技能；继续实车诊断先取得能捕捉实际振荡的记录。已有数值检查没有复现整车抖动。
8. 新Work对话的默认cwd可能属于别的工作树；以第2节E盘主工程和实时Git状态为准。

## 2. 路径、版本与当前工作区

| 项目 | 位置或状态 |
|---|---|
| 固件目录 FW | `E:/ROBOT_NEW/9.22wheel_legger6/wheel_legger/Wheel_Legger/code_framework v1.1` |
| 计算目录 CALC | `E:/ROBOT_NEW/9.22wheel_legger6/wheel_legger/Wheel_Legger` |
| Git根目录 | `E:/ROBOT_NEW/9.22wheel_legger6/wheel_legger` |
| 分支 / HEAD | main / 5f3824c，提交“改了版倒地自启，未验证” |
| zjx本地固件 | `E:/ROBOT_NEW/4.2 zjx/Wheel-legged`，旧HEAD 25a1113且有未提交修改 |
| zjx远程 | `https://github.com/QuickQ-a11y/Wheel-legged.git` |
| 本会话核对的远程提交 | 3b8cb2eda8581f4fe77caed131017c1a4c691194，2026-09-24；底盘路径现为Chassis/ |

目前未提交的实现/记录：`CALC/lqr_k_calc/hku_lqr_k_calc.py`、`CALC/lqr_k_calc/compare_k_calc.py`、`FW/projectmd/LQR_LESO_GENERATOR_COMPARISON.md`。均是工作区修改，未暂存；本会话未提交或创建分支。

此外已有 `FW/projectmd/WORK_HANDOFF_2026-10-06.md`，来自并行自起交接，本文件未覆盖它；本交接也是新增文档。旧“几十个构建产物未提交/单腿模块未跟踪”状态已过时，自起等历史修改已进入5f3824c。

交接时SHA-256：

```text
hku_lqr_k_calc.py
bca3abda3609b15a8a4a874d34d7ddc48e736153eeabfb97f540956dafef69d9
compare_k_calc.py
59630566046a05c09bd3392fae7b6e4e6c46b46b211ab508eb7f86c8be67ead0
ABK_LQR.py
a4cea0e4270f79a59af4ee95a7e0488ed3e5e4d0f9649ca75befb32e1b979b33
```

远程曾独立克隆到临时裸仓库 `C:/Users/lenovo/AppData/Local/Temp/zjx-leso-audit-2919697c751848eb81ecd2637431345d.git`；这是可丢弃缓存，不是交付依赖。本地zjx工作区未被覆盖。

## 3. 工程结构与运行链

硬件：达妙 DM-MC02，STM32H723VG；四个 DM-J8009 关节电机，MIT 纯力矩模式；两个 DJI M3508 轮电机；板载 BMI088。软件：C99、STM32 HAL、FreeRTOS、CMSIS-RTOS v1。

| 目录 | 继续开发所需职责 |
| --- | --- |
| `User/APP/` | 底盘、姿态和遥控任务 |
| `User/Controller/` | 单腿动作封装及通用控制工具 |
| `User/Algorithm/` | VMC、LQR、LESO、PID、Mahony 等 |
| `User/Devices/` | 电机、IMU、遥控设备协议 |
| `User/Bsp/` | CAN/UART/USB/DWT 接口 |
| `Core/`、`USB_DEVICE/` | CubeMX 初始化、任务创建、USB CDC |
| `MDK-ARM/` | 权威 Keil 工程和被 Git 跟踪的编译产物 |
| `mdk_check/` | 实际 C 模块及底盘流程的主机验证、VMC 数值检查 |
| `projectmd/` | 工程、自起与生成器对比文档 |
| 上一级 `ABK_LQR.py`、`lqr_k_calc/` | 离线建模、LQR/LESO 求解与系数拟合 |

任务：INS 为 Realtime、约 1 ms；遥控为 High、30 ms；底盘为 AboveNormal、`osDelayUntil()` 目标 1 ms，实际周期看 `fb_dt`。`CHASSIS_TASK` 栈为 1024，`INCLUDE_vTaskDelayUntil=1`。PS2 任务未创建。姿态使用 Mahony，EKF 编入但未使用。

共享数据：`INS`、`remote_ctrl`、`Chassis`；CAN/UART 回调和任务直接读写全局结构体，业务链未通过队列或互斥锁交接。

底盘周期顺序固定为：

```text
chassis_feedback_update → YAW_Parameter_Processing → falling_down_detect
→ 按 chassis_mode 调用模式函数 → LESO_Service → VMC_translate → Chassis_CanTransimit
```

NORMAL 控制：LQR 写轮力矩和腿摆力矩 Tp，腿长/roll PD 写 F0。自起写 F0/Tp、轮力矩归零；再统一 VMC 映射和 CAN 发送。

## 4. 关键文件与函数

以下路径相对固件目录，函数名用于定位；不要依赖历史行号。

| 文件 | 关键入口及用途 |
| --- | --- |
| `User/APP/chassis_task.c` | `chassis_task()` 调度；`chassis_feedback_update()` 更新 VMC/车体反馈；`falling_down_detect()` 判断入口及倒地卸力 |
| 同上 | `falling_down()` 启动、总截止、左腿、右腿、进入交接；`falling_to_down()` 保持两腿、交接或超时；`falling_leg_control(leg,start,now)` 合并单腿阶段与数据适配 |
| 同上 | `chassis_recovery_at_stance()` 提前/终点窗口；`chassis_recovery_handoff_ready()` 平衡交接判据 |
| 同上 | `chassis_zero_outputs()` 清全部输出；`chassis_recovery_abort()` 当周期 ZERO_FORCE 并复位阶段；`chassis_recovery_reset()` 复位自起标志，保留单腿上下文供观察 |
| 同上 | `zero_force()` 清零并保留电机重使能路径；`VMC_translate()` 映射及自起输出有效性检查；`Chassis_CanTransimit()` 下发、限幅、LESO 回灌 |
| `User/APP/chassis_task.h` | Chassis 结构、模式/使能枚举、正常腿长与 roll 参数、总线/ID/索引 |
| `User/Controller/leg_motion.c/.h` | 公共函数 `LegMotion_Run()`；命令、反馈、输出、独立上下文和动作结果 |
| `User/Algorithm/VMC/VMC_calc.c` | `VMC_calc_1()` 正解及解析速率；`VMC_calc_2()` F0/Tp 到关节力矩 |
| `User/Algorithm/LQR/LQR.c/.h` | `LQR_Calc()`、`Fitting_K_Calc()`；`P[40][6]` 与状态误差限幅 |
| `User/Algorithm/LESO/LESO.c/.h` | `LESO_Update()`、`LESO_Service()`、`LESO_Seed()`、`LESO_Feedback()`；AdP/BdP/LP 系数与补偿参数 |
| `User/APP/Remote_task.c` | `remote_task()`：s2 使能，s1 腿长档位，仅 NORMAL 更新腿长 |
| `User/Devices/Remote_Control/Remote_Control.c/.h` | 遥控解码及已有 `Remote_Is_Offline()`，当前无业务调用 |
| `User/Bsp/bsp_can.c` | CAN 接收按硬编码 ID 写入 Chassis，修改 ID 必须同步发送定义 |
| `Core/Src/freertos.c`、`Core/Inc/FreeRTOSConfig.h` | 任务优先级、栈和绝对周期配置 |
| `User/Devices/PC_Comm/PC_Comm.c` | `PC_Info_Upload()` 当前为空，未形成控制反馈上传链 |
| `User/APP/INS_task.c` | Mahony姿态、INS.Gyro原始赋值、INS.YawTotalAngle连续航向 |
| `User/Devices/DM_Motor/DM_Motor.h/.c` | DM协议PMAX/VMAX/TMAX、MIT编码解码 |
| `CALC/lqr_k_calc/hku_lqr_k_calc.py` | 用户生成器：solve_lqr、compute_K、fit_all_coefficients、main |
| `CALC/lqr_k_calc/compare_k_calc.py` | 实际参数读取、verify_discrete、441点K/拟合验证 |
| `CALC/ABK_LQR.py` | zjx本地参考：subs_AB、c2d、dlqr、leso_gain、fit_poly22 |
| `mdk_check/leg_motion_verify.js` | 严格 GB18030 解码底盘源码，提取实际阶段函数，临时片段 UTF-8，编译两个 C 验证程序 |
| `mdk_check/leg_motion_verify.c`、`recovery_verify.c` | 单腿语义、自起阶段、清零、交接最后一帧及失败调用顺序验证 |

## 5. 当前自起行为与不可丢失的约定

```text
OFF→ON：phi0/pitch 判为正常 → NORMAL；判为倒地 → FALLING_DOWN
在线 NORMAL 判倒地 → ZERO_FORCE，等待 OFF→ON
FALLING_DOWN：每条腿独立 SWING → RETRACT → HOLD
两腿都 HOLD → FALLING_TO_NORMAL → 条件满足才 NORMAL
自起或交接失败 → 当周期 ZERO_FORCE
```

- 倒地判据：任一腿原始 phi0 不在 `[0.4,2.5] rad`，或 `|pitch|>0.3 rad`。开启时保留姿态复检；正常运行倒地后不直接自动扫腿。
- 模式与使能分别是 `chassis_mode` 和 `chassis_enable`。复位/上电时遥控已在 ON 档，也会形成首次开启检查。
- 左右腿每周期先左后右计算，动作各自推进；左腿失败立即中止，右腿失败清掉左腿本周期已算出的输出。
- SWING 用绝对目标和指定方向，角度与腿长同时插值；实际进入提前/终点窗口便切收腿，不要求扫腿返回 DONE。起始已在窗口内可直接收腿。
- RETRACT 首次置 start=1，捕获实际角度；相对转角 0°、目标腿长 0.13 m。阶段锁存，扰动不使它重新扫腿；DONE 后进入 HOLD。
- HOLD 使用锁存命令，start=0、command=NULL，继续输出保持；先完成的一腿等待另一腿，不重新捕获角度。
- 平衡交接要求：有限 pitch 且绝对值 `<0.2 rad`；每腿满足姿态窗口且原始 phi0 在 `[0.4,2.5]`；腿长距 0.13 m `<0.02 m`；`|d_L0|<0.05 m/s`。
- `recovery_output_pending` 表示本周期输出来自自起。VMC 读取后清除；模式已交回 NORMAL 的最后一帧仍检查自起映射力矩。
- OFFLINE/ZERO_FORCE 时跳过 VMC 并直接清零 F0/Tp、最终关节力矩、轮力矩和轮电流；不能依靠异常雅可比乘零实现清零。
- `zero_force()` 仍会在关节 state!=1 时调用 DM_Enable 并延时；零力状态不是驱动器失能或故障锁止。

单腿接口：命令角度单位度，反馈/内部连续角为 rad，速率为 rad/s，腿长为 m。start=1 只用于启动当周期；持续置 1 会反复重启。绝对角以机身垂直向下为 0°；相对角量非负，方向单独指定。DONE 锁存并保持，不因原超时到期失效；TIMEOUT/INVALID 零输出锁存，重新 start 才执行；保持期间无效反馈仍会失败。相邻实际角变化必须小于 180°，漏采样额外整圈无法识别。duration_ms 是参考变化时间，不保证实际到位。

坐标及电机索引：右腿反馈已乘 mirror=-1，输出关节力矩再次取负；两腿模块共用符号，不额外镜像。`spin=phi0−π/2=−alpha`；`theta=pitch+phi0−π/2`。腿索引左0右1，轮索引左1右0。前关节对应 phi4，后关节对应 phi1。FDCAN1 接两轮及右腿，FDCAN2 接左腿；轮 RX 0x201 为右、0x202 为左。

## 6. 重要参数

| 参数/条件 | 当前有效值 |
| --- | --- |
| 扫腿方向、绝对目标 | LEG_MOTION_NEGATIVE；+39.143°，对应 phi0 约2.254 rad |
| 扫腿参考时间、腿长 | 4000 ms；0.30 m |
| 收腿参考时间、腿长 | 1000 ms；0.13 m |
| 提前窗口 | 当前负方向 spin∈[-0.9,0] rad，含边界 |
| 终点角窗口/动作完成角误差 | 严格小于0.25 rad |
| 动作完成腿长误差 | 严格小于0.02 m |
| 自起总截止 | 600000 ms；包含扫腿、收腿、等待另一腿；收腿只分配剩余时间 |
| 交接截止 | 1500 ms，自两腿 HOLD 进入交接另计 |
| 单腿角度 Kp/Kd | 15 / 2 |
| 单腿腿长 Kp/Kd | 300 / 9 |
| 单腿 Tp/F0 限幅 | ±12 N·m / ±20 N |
| 单腿重量参数 | 全腿14.33 N，下腿10.71 N |
| 正常腿长 Kp/Kd_RATE、限幅 | 1600 / 300；PD项±90 N |
| roll Kp/Kd、限幅 | 340 / 80；±60 N |
| 正常重力补偿 | body_mg=13×9.8 N；与离线模型质量不可混为同一参数 |
| s2 / s1 | s2=1或3 ON，2或0 OFF；s1=2或0腿长0.15 m，1为0.25 m，3为0.20 m |
| LQR误差限幅 | x±6、yaw±1.25、左右腿theta±0.45、机身pitch±0.25 |
| 关节力矩、电流指令限幅 | 关节±40 N·m；轮电流±16384；力矩到电流系数左−3330、右+3330 |
| LESO接入 | ONLINE且NORMAL；入口重seed；累计0.5 s后接入 |
| LESO补偿比例及斜坡 | leso_comp_scale=1.0；LESO_COMP_RATE=0.003；调试可用scale=0建立对照，比例经斜坡变化 |
| 离线采样周期与极点 | Ts=0.001 s；LESO状态极点0.4、扰动极点0.985 |

LESO当前仅累加 stable_t，并以一阶斜坡接入；`LESO_GATE_DX`、`LESO_GATE_DTHETA` 未参与门控。不要依据旧注释认为仍有速度稳定门。修改控制周期时同步重算离散模型，并检查源码写死的0.001 s累计量。

## 6.1. 当前计算参数、接口与固件系数来源

状态顺序：`[s,ds,yaw,dyaw,thetaL,dthetaL,thetaR,dthetaR,pitch,dpitch]`；输入顺序：`[TwL,TwR,TpL,TpR]`。F0单位N，Tp/轮力矩单位N·m。

poly22列序：`[p00,p10,p01,p20,p11,p02]`；X=左腿长，Y=右腿长；矩阵行主序。K/P为4×10/40×6；Ad为10×10/100×6；Bd为10×4/40×6；L为14×10/140×6。

| 参数 | 用户HKU当前值 | zjx本地ABK当前值 |
|---|---|---|
| m_b / m_l / m_w，kg | 11.0 / 1.65 / 0.537 | 相同 |
| R_w / R_l / l_c，m | 0.058 / 0.22 / 0.120 | 相同 |
| 机体包络，m | 0.415 / 0.16 / 0.260 | 相同 |
| Q对角 | [100,20,100,20,400,5,400,5,10000,1] | [100,50,1000,50,100,5,100,5,20000,5] |
| R对角 | [20,20,4,4] | [150,150,50,50] |
| 拟合范围/周期 | 0.10～0.30m、21×21；0.001s | 相同 |
| LESO极点 | 0.4 / 0.985 | 相同 |
| 输出开关 | K=0、L=0、Ad/Bd=1 | 四表计算，只有K系数打印，其余print_block被注释 |

两边默认均启用公式惯量、腿部经验公式。生效惯量为Iw=0.000903234、Ib=0.1813395833、Iz=0.2198395833；声明区0.000516/0.025/0.380不生效。0.20m腿惯量公式为0.15，实测表插值约0.010680589，约14倍差；两边共同使用公式，不构成两脚本之间的差异。

HKU接口：

- `solve_lqr(A,B,Q,R)`接受连续A/B，内部按全局Ts做ZOH，返回 `(K, DARE最大绝对残差, 闭环谱半径)`。
- `compute_K()`返回 `(K, 原连续A, 原连续B, 残差, 谱半径)`；供LESO分支只离散一次。
- `fit_all_coefficients(...,want=...)`显式选择K/L/Ad/Bd并返回系数/残差；函数本身不写文件。
- `main()`控制TXT输出；`--no-out`禁止写结果。
- `c_array_block()`五位有效数字、行尾逗号；保持既有布局。

当前固件额外参数：LESO扰动限幅 `[2,2,7,7]` N·m；stable_t固定每次加0.001；速度门宏0.1m/s、0.5rad/s未用于实际接入；DM协议PMAX=12.5、VMAX=45、TMAX=40。

### 数值复现的固件配置，区别于现脚本默认值

用ABK离散算法、质量16.43kg、网格0.10～0.35m，下列配置分别复现完整240个K系数的五位打印值：

| 参数 | 用户现固件P | zjx远程3b8cb2e固件K |
|---|---|---|
| Q | [200,50,100,50,400,10,400,10,15000,5] | [200,50,100,50,100,5,200,5,15000,5] |
| R | [10,10,1,1] | [10,10,1,1] |

两者LESO三表完整逐项相同；16.43kg+0.10～0.35m可复现其有效系数。这是反推后完整重算验证的可复现配置，不是作者源文件原文；Q/R整体缩放有等价解。

当前固件K和LESO均可对应16.43模型，不能断言现固件已经“K按11、LESO按16.43”混用。确证的是现固件与两本地脚本11kg默认输出不同。zjx的16.43来自其20.8kg整车称重，不能直接当本车实际质量。

## 7. 已完成工作与已验证结论

### 完成的代码修改

- HKU由连续CARE改为ZOH+`solve_discrete_are`。
- K计算使用离散公式；返回DARE残差和离散闭环谱半径。
- 固定腿长及拟合稳定性判据改为极点模<1，警告/输出/说明同步。
- 保持compute_K返回连续A/B，LESO路径没有二次离散。
- 对比工具读取实际参数、Q/R、腿数据与惯量分支；连续CARE只作独立参考。
- 实施时保留参数、Q/R、网格、开关、UTF-8无BOM/LF；未覆盖8份旧TXT、ABK或固件系数。第1节记录的开关变化发生在验证之后。

### 离散化后441点同参数对照

共同条件：质量11kg、公式惯量、腿公式、0.10～0.30m网格、Ts=0.001s、HKU Q/R、LESO极点0.4/0.985。四表通过内存显式want全部生成，未改开关、未写TXT。

| 输出 | 441点矩阵最大绝对差 | 拟合系数最大绝对差 |
|---|---:|---:|
| K | 2.2610e-10 | 1.0758e-9 |
| Ad | 4.1633e-17 | 3.3307e-16 |
| Bd | 1.0408e-17 | 8.3267e-17 |
| L | 6.4809e-15 | 1.3989e-14 |

- 全部通过rtol=atol=1e-8。
- 五位打印K/Ad/Bd分别240/240、600/600、240/240一致；L为795/840，剩余为浮点尾差，数值一致。
- 441公式网格、121实测表网格全部离散稳定；归一化DARE残差≤1e-10。
- 0.5ms和2ms周期的边界/非对称/公式/实测代表点通过。
- 五条方程符号差为0；A/B装配、poly22列序及行主序正确。
- 实施时`--mode all --no-out`覆盖固定K、K拟合和插值，退出0，输出文件哈希不变。

默认0.20/0.20m、HKU Q/R结果：

```text
左轮pitch/dpitch：-7.7610093861 / -0.5920149019
左腿Tp的pitch/dpitch：-37.0391226410 / -2.2075639455
闭环谱半径：0.9990368392
DARE最大绝对残差：1.67e-8；归一化残差约4.09e-14
```

### 实车诊断证据边界

- 两份真实C观测器核心，在相同矩阵和输入下结果相同。
- 无外部扰动的20Hz/0.5N·m输入模拟中，输入晚一拍误估约0.00746N·m；不能独自解释剧烈抖动。
- 已检查的理想模型稳定；不等同于硬件稳定。
- 本会话没有新Keil构建、烧录、任务周期实测或实车反馈。
- 自起此前主机/隔离Keil验证记录在LEG_MOTION_HANDOFF；当前HEAD的固件验证以实时测试为准，不能凭旧“0错误0警告”宣称最新控制已经实测。

## 8. 已否决或撤回的判断

- “离散化即可消抖”：没有实车证据；此前同Q/R连续与离散K仅约0.7%～1.3%差。
- “用户LESO表与zjx最新不同”：撤回；三表与3b8cb2e完全相同。
- “用户pitch增益比zjx大4.6倍”：基于旧25a1113，撤回；0.20m最新zjx Tp/pitch约−85.41，用户固件约−84.40。
- “zjx陀螺有高频低通，因此噪声一定更小”：撤回；filteredGyroRadps主要为启动零偏/EKF偏置处理。
- “HKU模型转写/转置/poly22列序错误”：检查未发现。
- “Q/R不同使LESO表不同”：同物理模型/周期/极点时Ad/Bd/L一致，权重只影响K。
- “所有旧TXT均失效”：旧K TXT不对应当前Q/R；旧LESO TXT仍与11kg默认模型吻合到打印精度。
- “晚一拍是唯一根因”：仅有小幅误估模拟证据，未验证硬件根因。

## 9. 现存bug与待确认项

| 项目 | 确认事实及影响边界 |
|---|---|
| LESO输入时序 | Service更新时用u_last，预测输入相对当拍测量/输出晚一拍；zjx在最终输出定稿后更新。未修复。 |
| 关节输入回灌 | 实际torque_set限幅±40后发出，但回灌仍为请求Tp；饱和时实际广义输入不同。zjx从最终关节力矩逆VMC还原。是否参与抖动待采集。 |
| 轮力矩限幅 | 电流换算/限幅/发送后才clamp wheel_T到±4.8；发送与回灌不一致。 |
| HKU --params顺序 | 先公式惯量再覆盖参数；--params m_b=16.43时质量变，但Ib/Iz仍按11计算。默认运行不触发。 |
| HKU eval_fit(Bd) | 40行重建为4×10，应为10×4；Bd导出正确，当前主流程未用它验Bd。 |
| 航向跨界 | LESO用wrapped INS.Yaw；已有INS.YawTotalAngle可用。跨±π会假残差，不能解释一般静止半秒发作。 |
| 反馈/触地 | 本车Estimate_dx无滤波；zjx速度Kalman开启。触地检测未接入本车LESO门控；噪声/接地状态待记录。 |
| 电机量程 | 本车代码VMAX45，zjx最新30；若本车寄存器30，解码速度放大1.5倍。尚未确认本车寄存器，不能认定实际量程错。代码TMAX双方40。 |
| 遥控失联 | Remote_Is_Offline未被调用，可能保持最后ONLINE输入。 |
| 坐标核对 | INS.Pitch=-mahony.pitch，底盘pitch速率=-INS.Gyro[1]；没有证据支持直接翻转LESO补偿符号。 |
| 调度/宏 | INCLUDE_vTaskDelayUntil=1必需；PID_Calc旧D未除dt；Gear_Ratio没括号，x/Gear_Ratio有优先级问题。 |
| 参数与测量 | 本车真实质量/惯量/质心、PMAX/VMAX/TMAX、实际fb_dt和振荡轨迹缺失。 |
| 数据出口 | PC_Info_Upload为空，USB接口没有实际控制反馈上传；未找到可重放CSV。 |

这些缺陷没有在本会话修复。不要一轮同时改模型、权重、周期、反馈和补偿逻辑。

## 10. 下一步任务与完成标准

1. **配置核对**：新对话读取AGENTS/CLAUDE和本交接，确认E盘主工程Git状态、HKU实时开关、实际烧录版本；确认本车质量、寄存器、fb_dt。标准：有数值及来源。
2. **采集实车复现**：同站立条件记录scale=0稳定段与scale=1接入/抖动段，接入前后约2秒。字段：时间/fb_dt、pitch/dpitch、两腿theta/dtheta/L0/dL0、四路dh、comp、四路广义请求、最终关节力矩/轮电流、模式和使能；同时记电机寄存器和固件版本。
3. **建立回放判据**：同一记录可重复捕捉实际振荡或输入失配后，才验证时序、限幅回灌、模型或噪声假设。用户已调用技能路径：`C:/Users/lenovo/.codex/skills/diagnosing-bugs/SKILL.md`。
4. **按证据逐项修复**：优先确认LESO实际输入/时序，每次一个因素，保留原始对照并复测。计算一致和理想稳定不是实车验收。
5. **系数生成与替换是后续任务**：先确认本车模型/周期/网格，记录K/Ad/Bd/L来源。物理参数或周期/网格变化时同步对应四表；仅改Q/R不改变LESO模型。替换后核对元素数240/600/240/840、编码，Keil构建和实车验证。
6. **独立脚本修复候选**：--params惯量联动、eval_fit(Bd)形状；当前均未实施，需针对实际错误的数值回归。
7. **自起实车验证**：前倒/后倒/不对称腿姿态/一腿提前完成/OFF→ON/超时卸力，记录阶段切换和交回NORMAL；详细接手条件见LEG_MOTION_HANDOFF。

## 11. 构建、验证与编辑约束

首读：FW下AGENTS.md、CLAUDE.md。改单腿接口读LEG_MOTION.md；改自起阶段/交接读LEG_MOTION_HANDOFF.md；生成器读LQR_LESO_GENERATOR_COMPARISON.md最上方离散化后章节。旧文档Git状态和连续LQR段落属于历史记录。

从FW运行：

```powershell
python -B ../lqr_k_calc/compare_k_calc.py
python -B ../lqr_k_calc/hku_lqr_k_calc.py --mode all --no-out
node mdk_check/leg_motion_verify.js
node mdk_check/vmc_verify.js
node mdk_check/jacobian_verify.js
```

第二条当前0/0/1只跑Ad/Bd支路。compare独立验证K，不依赖生成开关；四表完整441点检查在会话中内存执行，结果已记录，不能宣称compare本身单独覆盖全部L。VMC/Jacobian几何不同，只验证公式。

权威工程 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx`；target `CtrlBoard-H7_IMU`；Keil `C:/Keil_v5/UV4/UV4.exe`；ArmClang6.16、C99。PowerShell启动Keil用Start-Process -Wait，后台加-WindowStyle Hidden；占用时处理GUI或用隔离工程。构建目标0错误无新警告。实际构建日志在MDK-ARM/build.log；产物在MDK-ARM/CtrlBoard-H7_IMU/。

```powershell
$firmwareProject = Join-Path (Get-Location) 'MDK-ARM/CtrlBoard-H7_IMU.uvprojx'
$firmwareArgs = @('-b', ('"' + $firmwareProject + '"'), '-t', 'CtrlBoard-H7_IMU', '-j0', '-o', 'build.log')
$firmwareBuild = Start-Process -FilePath 'C:/Keil_v5/UV4/UV4.exe' -ArgumentList $firmwareArgs -WindowStyle Hidden -Wait -PassThru
Get-Content -LiteralPath 'MDK-ARM/build.log' -Tail 8
$firmwareBuild.ExitCode
```

上述为操作入口，本轮没有执行Keil。下载/调试用CMSIS-DAP；EIDE/J-Link脚本过时。构建会改被跟踪产物，提交逐项选文件。

编码与修改：

- 按字节读取/写入，局部修改并保留原编码行尾；GBK源码不得整体重存UTF-8。
- chassis_task.c为GBK/CRLF，.h为GBK/LF；HKU/compare为UTF-8无BOM/LF。
- Markdown为UTF-8 BOM/CRLF。
- LESO.c已有231个U+FFFD，不能增加；LESO.h有历史混合编码行。
- 保留User/APP/*.fffd.bak；避免整文件格式化。
- 新C源/包含路径注册到Keil；CubeMX再生成核对手工RTOS改动。
- 保留未提交工作，显式选择提交文件；此任务没有提交/烧录授权动作记录。

## 12. 验证记录及边界

`projectmd/LQR_LESO_GENERATOR_COMPARISON.md`保存同参数重算、固件配置反推与历史判断更正；其首节是当前离散结果，后面旧连续数据保留作基线。

`projectmd/LEG_MOTION_HANDOFF.md`为较早自起实现记录；其中“未提交/未跟踪”与当前5f3824c不符。`projectmd/WORK_HANDOFF_2026-10-06.md`为另一自起会话交接，可查该会话实际验证，但不把其cwd或动作归入本次LQR会话。

尚无实车波形、没有能复现实车抖动的自动回放命令，没有经实车验证的唯一根因或参数修复方案。
