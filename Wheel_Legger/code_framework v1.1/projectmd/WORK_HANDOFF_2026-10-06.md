# 轮腿机器人项目交接 — 下一条 Work 对话

核对日期：2026-10-06。适用范围：E 盘主工程当前源码、未提交的离线计算修改，以及本对话完成的倒地自起整理。本文件记录开发状态与执行约束，参数和后续变更以实际文件为准。

## 1. 新对话首先确认的上下文

- **实际开发工程**：`E:/ROBOT_NEW/9.22wheel_legger6/wheel_legger/Wheel_Legger/code_framework v1.1`。
- **Git 根目录**：`E:/ROBOT_NEW/9.22wheel_legger6/wheel_legger`；分支 `main`；当前 HEAD：`5f3824c926c8f1031f988bb47e0eb8f990e7ad62`，提交信息“改了版倒地自启，未验证”。
- 本对话默认 cwd 是 `C:/Users/lenovo/.codex/worktrees/170b/wheel_legger/Wheel_Legger/code_framework v1.1`。该工作树仍为 detached `4764650`，不是当前主工程。本对话最初因检查 C 盘旧工作树，错误报告当前版没有自起超时；已经更正。后续检查和修改都使用 E 盘绝对路径或显式 workdir。
- `5f3824c` 已包含单腿模块、底盘自起接入、风格整理、验证代码及交接资料。`LEG_MOTION_HANDOFF.md` 中“尚未提交”“新文件未跟踪”是旧状态；以本节和实时 `git status` 为准。提交标题的“未验证”不能抹去主机验证记录；本对话仍没有实车验证记录。
- 创建本交接文件前，工作区只有三个已修改、未暂存文件：固件目录下 `projectmd/LQR_LESO_GENERATOR_COMPARISON.md`，以及上一级 `lqr_k_calc/compare_k_calc.py`、`lqr_k_calc/hku_lqr_k_calc.py`。这些是已有工作，继续开发时保留。
- 开始代码修改前读取主工程 `AGENTS.md` 和 `CLAUDE.md`。涉及自起读 `projectmd/LEG_MOTION_HANDOFF.md`；涉及模块调用读 `projectmd/LEG_MOTION.md`；涉及生成器读 `projectmd/LQR_LESO_GENERATOR_COMPARISON.md` 最新段落。

## 2. 当前目标与完成状态

- 本对话目标：核对倒地自起的新增安全保护；按用户原有底盘代码风格整理自起流程；解释当前流程；交付新对话交接资料。
- 上述代码整理已完成。用户明确选择：保留 `LegMotion_Run()` 封装；主流程明确调用左腿、右腿；共用单腿阶段函数；命令逐字段赋值；简短中文注释；保留全部现有控制条件及保护。
- 下一开发阶段的主要缺口：自起实车验证、LESO 接入后抖动的反馈采集与复现、离线生成器修改的审查及参数一致性核对。
- 尚未实施遥控失联停机、轮力矩发送前限幅调整、惯量覆盖顺序修复、roll/触地判据或新的参数调优。

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

## 7. 已完成工作与已验证结论

- 自起动作封装、双腿独立阶段、收腿锁存及保持、共享截止、交接判据和异常当周期清零已经接入。
- 风格整理已完成：主流程明确左右腿调用；阶段处理与原 `chassis_leg_motion_run()` 搬运合并为 `falling_leg_control()`；命令/反馈逐字段赋值；中文注释；公共接口、算法与参数保持原样。
- 风格整理只改5份文件：chassis_task.c、leg_motion_verify.js、recovery_verify.c、LEG_MOTION.md、LEG_MOTION_HANDOFF.md；当时12个保留函数代码token序列不变，667份主工程MDK文件哈希不变。该结论只针对当次整理，不意味着后来用户提交的MDK产物从未变化。
- **2026-10-06本次交接复验当前代码**：单腿350项断言通过，自起198项断言通过。覆盖独立阶段、保持、重启、超时、计时回绕、反馈无效、失败顺序及交接最后一帧映射检查。
- **上次风格整理隔离完整构建**：ArmClang6.16，0 Error(s)、0 Warning(s)，退出码0；证据 `build/leg_motion_keil/style_build.log`。本次仅交接文档，没有重新编译固件。
- 主机验证的时钟、VMC映射和电机使能使用桩；不覆盖真实CAN、RTOS调度、接地或机械响应。自起成功率、实际任务耗时及烧录后的行为仍需用户实测记录。
- 已有资料记录VMC/Jacobian有限差分校验通过；脚本杆长0.215/0.254与固件0.208/0.25212不同，只验证公式。

离线计算当前状态：

- 用户自己的生成器是 `../lqr_k_calc/hku_lqr_k_calc.py`；用户确认 zjx 的本地脚本是 `../ABK_LQR.py`，不要颠倒归属。
- HKU未提交修改已从连续CARE切到Ts对应的ZOH+离散DARE；当前开关为 USE_K=0、USE_L=0、USE_AD_BD=1，公式惯量和腿部经验公式开启。报告旧表的连续LQR与输出开关只属于历史基线。
- 对比报告记录同参数441点K/Ad/Bd/L一致性，rtol=atol=1e-8通过；矩阵最大差分别约2.26e-10、4.16e-17、1.04e-17、6.48e-15。这是报告中的已有验证，本次未重跑全部网格。
- HKU默认Q=[100,20,100,20,400,5,400,5,10000,1]、R=[20,20,4,4]；本地ABK默认Q/R不同，因此默认运行的K仍会不同。
- 报告记录可复现当前固件系数的一组配置：质量16.43 kg、网格0.10～0.35 m、Q=[200,50,100,50,400,10,400,10,15000,5]、R=[10,10,1,1]，ABK离散算法。它是数值复现配置，不是本车实测质量，也不是当前HKU默认值；权重整体比例不唯一。
- poly22列序[1,ll,lr,ll²,ll·lr,lr²]，矩阵均行主序；固件K为40×6、Ad为100×6、Bd为40×6、L为140×6。实际默认脚本、现有txt、固件已粘贴表、实际烧录版本必须分别核对。

## 8. 已否决或本轮未采用的方案

- 用户本轮未选择“仅统一排版”或“连单腿公共接口一起简化”；采用保留模块、整理底盘流程的方案。
- 不恢复NORMAL倒地后立即扫腿；保留ZERO_FORCE等待人工OFF→ON。开启时按姿态复检，不能短路成每次ON都自起。
- 不以F0/Tp设零后继续VMC计算代替最终输出清零；不能删除交回NORMAL最后一帧的自起来源标记。
- 不反复重启收腿或追着实际角更新固定目标；保留首次捕获、阶段锁存、完成后保持。
- 不因风格整理删除数值保护、超时或交接条件；600000/1500 ms本轮明确保持。改成6 s是待实测评估事项。
- 不额外对右腿单腿控制取反；反馈/发送端已经处理镜像。
- 不用旧K txt、过期辅助脚本硬编码权重或连续LQR历史结果作为当前生成器输出依据。
- 不把理想模型或离线矩阵一致性当成实车抖动根因验证；已有报告明确没有复现实车振荡。

## 9. 现存bug、缺口及待诊断问题

| 问题 | 已核对现状与影响 |
| --- | --- |
| 遥控失联未停机 | Remote_Is_Offline已有50 ms判断，但业务代码未调用；遥控保留最后一帧，ON和摇杆命令可能继续生效 |
| 轮力矩限幅顺序 | ±4.8轮力矩限幅在计算SET_Current及发送之后；本拍实际只受电流±16384约束，LESO回灌值与本拍实际指令可能不一致 |
| HKU命令行覆盖不联动公式惯量 | main先formula_inertia(PARAMS)，后params.update；覆盖m_b/R_w等后依赖惯量仍由旧值计算。报告的旧行号已变化，按函数定位 |
| Gear_Ratio无括号 | 宏为268.0f/17.0f；当前反馈使用括号乘法；其他表达式直接x/Gear_Ratio会改变运算结合 |
| LESO接入后抖动未复现 | 对比报告记录用户描述的剧烈抖动，但没有可重放实车轨迹；时序、模型参数、回灌限幅等仅为待检验因素 |
| 缺少采集链 | PC_Info_Upload为空；报告未找到CSV/反馈日志，USB没有实际控制反馈上传调用 |
| 倒地检测策略缺口 | 只看phi0/pitch，不看roll，不做持续时间确认；瞬态越界会卸力，侧躺可能漏判。尚无新实车判据验证 |
| LESO触地条件缺口 | ground_detectionL/R仅定义未接入；NORMAL不能保证双轮触地；速度稳定门当前闲置 |
| 自起总超时过长 | 当前10分钟；其合理性未验证，尚未调整 |
| 故障联锁覆盖有限 | 当前没有在自起上层新增过温/过流/CAN离线/传感器失联停机；零力状态仍会重使能关节。不能由此断言电机固件内部无保护 |
| 编码历史损坏 | LESO.c/VMC_calc.h/main.c/user_sys_config.h存在旧损坏注释；不要把整文件重编码当修复 |
| 工程配置同步 | CubeMX .ioc与手改RTOS配置不完全一致；重新生成可能覆盖任务栈及INCLUDE_vTaskDelayUntil |

正常模式的整体输出有效性保护范围、反馈时间戳新鲜度和上层故障联锁，未在本次风格整理中扩展。

## 10. 下一步任务与完成标准

1. **接手定位**：确认E盘HEAD和未提交文件；读本文件及相关主工程说明；当前任务由用户新指令确定。完成标准：不把C盘4764650或旧文档状态当成当前基线。
2. **自起实车验证**：记录前倒、后倒、两腿姿态不同、一腿提前完成、在线倒地、OFF→ON重启、失败卸力、恢复平衡；观察实际/参考角度、腿长、阶段、pitch和fb_dt。完成标准：能说明进入收腿的时机、保持目标、两腿完成差异和交接结果，不能仅以偶然起身作为通过。
3. **LESO抖动复现**：同一站立条件采集scale=0稳定段、scale=1接入及抖动段前后约2秒；包含时间/fb_dt、pitch/dpitch、两腿theta/dtheta/L0、四路dh、leso_dbg_comp、四路广义力矩请求、最终电机指令、模式/使能，以及电机PMAX/VMAX/TMAX和烧录版本。完成标准：建立能捕捉实际振荡的回放判据，再逐项检验原因。
4. **离线计算续接**：审查并验证三个未提交文件；优先修复覆盖参数时的惯量顺序；记录当次开关、参数、Q/R、Ts、网格和输出目录。完成标准：同参数真实脚本一致性、DARE残差和离散谱半径检查通过，明确输出是否对应固件表。
5. **已知代码问题**：按用户后续指定顺序处理遥控失联、发送前限幅及LESO回灌、触地/倒地策略；每次控制变化单独验证。完成标准：触发条件、当周期输出和恢复语义都有可复现记录。
6. **交付**：代码修改后重跑对应主机验证、Keil构建并更新实车记录；参数变更与风格变更分别说明。当前仅需要交接文档，以上待办没有在本次执行。

## 11. 构建、验证与编辑约束

主机验证（PowerShell，显式主工程路径）：

```powershell
Set-Location -LiteralPath 'E:/ROBOT_NEW/9.22wheel_legger6/wheel_legger/Wheel_Legger/code_framework v1.1'
$env:CC = 'E:/mingw64/bin/gcc.exe'
node mdk_check/leg_motion_verify.js
```

VMC公式变更时运行 `node mdk_check/vmc_verify.js` 和 `node mdk_check/jacobian_verify.js`。验证脚本从真实源码提取函数，调整函数组织时同步维护提取标记；GB18030解码、UTF-8临时片段和GCC输入字符集已经配置。

权威工程 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx`，target `CtrlBoard-H7_IMU`；Keil `C:/Keil_v5/UV4/UV4.exe`，ArmClang6.16、C99、MicroLIB、Cortex-M7硬浮点，要求0错误且无新增警告。leg_motion.c已注册。主工程MDK产物被Git跟踪，编译时会修改大量文件；GUI正在占用权威工程时使用隔离工程。

现有隔离构建（源路径为本机绝对路径，换目录后需重新生成）：

```powershell
$handoffProject = 'E:/ROBOT_NEW/9.22wheel_legger6/wheel_legger/Wheel_Legger/code_framework v1.1/build/leg_motion_keil/leg_motion_check.uvprojx'
$handoffArgs = @('-r', ('"' + $handoffProject + '"'), '-t', 'CtrlBoard-H7_IMU', '-j0', '-o', 'handoff_build.log')
$handoffProcess = Start-Process -FilePath 'C:/Keil_v5/UV4/UV4.exe' -ArgumentList $handoffArgs -WindowStyle Hidden -Wait -PassThru
Get-Content -LiteralPath 'E:/ROBOT_NEW/9.22wheel_legger6/wheel_legger/Wheel_Legger/code_framework v1.1/build/leg_motion_keil/handoff_build.log' -Tail 8
$handoffProcess.ExitCode
```

隔离产物在 `build/leg_motion_keil/Objects/`。烧录由Keil CMSIS-DAP Download完成；本对话未烧录。EIDE工程清单过期，J-Link脚本路径/产物不对应本机，不能作为权威构建或烧录入口。

- chassis_task.c/.h按GBK字节读写；.c当前CRLF，.h原为LF。其他源文件逐文件核对，不能统一当成GBK；LQR.c、LESO.c、多数设备/Bsp/Lib文件为UTF-8。
- 中文新增注释按目标文件自身编码写入。检查原编码严格解码、U+FFFD计数、行尾及字节差异；普通编辑工具整文件重存可能破坏GBK注释。Git diff受autocrlf影响，不能单凭diff判断行尾保持。
- Markdown保存UTF-8 BOM+CRLF；局部新代码四空格；保留现有模块前缀；不做整文件格式化。
- 保留 `*.fffd.bak`：chassis_task.c和Remote_task.c备份可用于恢复；chassis_task.h备份本身损坏，不作为健康来源。
- 本对话普通shell曾因sandbox-bin锁初始化失败；后续显式require_escalated经自动审核执行。该环境问题与固件编译错误无关。

## 12. 证据与后续观察入口

- 自起详细交接：[LEG_MOTION_HANDOFF.md](LEG_MOTION_HANDOFF.md)；接口：[LEG_MOTION.md](LEG_MOTION.md)；工程目录：[PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)。其中历史提交状态已被本文件更新。
- 离线计算及抖动资料：[LQR_LESO_GENERATOR_COMPARISON.md](LQR_LESO_GENERATOR_COMPARISON.md)。最新离散验证段优先；后续段落明确标为连续设计历史记录，不照搬其旧开关或辅助脚本问题描述到当前文件。
- 上次风格整理证据目录：`C:/Users/lenovo/.codex/visualizations/2026/10/05/01a10e51-9d6d-7c40-9a8f-00bb9af0b154/`，含 `recovery_style_baseline/` 原文件和manifest、`recovery_style_verification.json`。原封装快照在主工程 `build/leg_motion_baseline_7p2dihus/`。
- 调试Watch：`Chassis.chassis_mode/chassis_enable`、`recovery_phase[0/1]`、`chassis_leg_motion[0/1]` 的actual/reference/target角、reference_length、result与command；两腿phi0/L0/d_L0/F0/Tp/torque_set；轮wheel_T/SET_Current；pitch；`fb_dt/t_fb/t_mode/t_leso/t_can/t_sum`；`leso_dbg_dh/leso_dbg_comp/leso_dbg_emax`。
- 故障后单腿上下文保留，阶段会复位；映射异常时可出现单腿result仍为DONE而底盘已ZERO_FORCE。判读时结合模式、映射结果和当周期输出。
