# 倒地自启（摆腿起立）移植方案

> 状态：**方案已定，尚未改动任何源码**。2026-09-20 产出，结论以当日工作区为准（含未提交的 WIP）。（09-20 更新：斜坡照抄 `ramp_update`（§4）、新增交接门算例 §5.5、落码清单 §10；仍未动源码。）
> 源方案：`E:\ROBOT_NEW\AwakeningLion_2027SERIES_LEH_DM-MC02_zbx\DM-balance`
> （该工程 UTF-8，可直接读写；技术简报在它的 `leg-control-brief.md`）
> 目标工程：本目录 `code_framework v1.1`
>
> **动手前必读**：本目录**源码**是 GB2312 + CRLF，改源码必须走 `CLAUDE.md` 里的字节级规则
> （禁止用 Edit/Write 直接改源码）。本文档本身是 UTF-8 with BOM（VS Code 默认 gb2312，靠 BOM 覆盖）。

---

## 1. 一句话

把腿当撬棍：绕髋关节扫过一个大角度把机身从地面撬起来，扫到位后收腿，两条腿都完成后交给 LQR
平衡控制器。全程轮子给 0（只做自由滚动的被动支点）。

## 2. 当前工作区状态（未提交 WIP，树是坏的）

这不是从零起步：工作区里已经有一次半成品移植，且**编译不过、链接也会炸**。

| 事实 | 证据 |
|---|---|
| 上次构建失败 19 个错 | `MDK-ARM/CtrlBoard-H7_IMU/CtrlBoard-H7_IMU.build_log.htm`：全部是 `chassis_task.c` 400-405/414 行 undeclared identifier（`PHI0_PID_KP`、`left_phi0_err`、`left_phi0_rate`…），Target not created |
| .h 改了、.c 没跟上 | 未提交 diff：`.h` 删掉 7 个 `PHI0_*` 宏、`set_phi0_Left/Right` 改名 `set_spin_angle_*`、新增 7 个 `spin_angle_*` 全局；而 `.c` 里 `falling_leg_control()` 仍用 `PHI0_*`，`falling_down()/falling_to_down()` 仍有 8 处 `set_phi0_*` |
| 7 个全局定义写进了头文件 | `chassis_task.h:59-65`。该头被 7 个 .c include（chassis_task / Remote_task / ps2_task / bsp_can / LQR / LESO / Stepper_Motor）。`spin_angle_end = 5.5` 带初始化是**强定义**，编译一过链接必报重复符号；其余 6 个是暂定定义，不要指望 `-fcommon` |
| 474 行是中文占位符 | `chassis_task.c:474`，裸标识符，不是合法 C（clang 停在 20 错上限，还没报到这里） |
| 457-470 行变量写串了 | 左腿的两支也写进 `spin_angle_fdb_R`；且 `2π−alpha` 的归一化与源方案（`alpha<0→+2π`）不符 |
| 文件编码本身健康 | `chassis_task.c` 21013 B，U+FFFD = 0，GB2312 严格解码通过 |

**结论**：不要逐处打补丁。先把方案定死，再 `.h`/`.c` 一次改齐。

## 3. 源方案要点（已与源码核对）

- 分支：`if (任意一腿没转完) 扫腿分支 else LQR 分支`（`chassisR_task.c:259-296` / `chassisL_task.c:177-215`）。
  完成标志只在平衡分支里被 `s[0]==2` 清零。
- 相位量：`alpha = π/2 − phi0`（本工程同式，`VMC_calc.c:90`），归一到 [0,2π)，站立位 0≡2π。
- 唯一目标源：遥控 `s[0]`：2=ref 跟随 fdb 并记录 init；3=4 s 线性斜坡 `ramp_update(init→5.5 rad)`；
  2→3 边沿（`update_flag=1`）重置计时。
- 判据（都作用在反馈上）：`f≤0.9` 提前完成；`|f−5.5|<0.35` → 腿长目标 0.3→0.13 收腿；
  `|0.13−L0|<0.02` → 本腿完成。
- 控制律：`Tp = Leg_mg*L0/2*sin(f) + PID(f,ref)`（PD {30,0,3000}，max 20）；
  `F0 = −sLeg_mg*cos(f) + PID(L0, spin_Leg)`（PD {170,0,3000}，max 20）。轮子电流 0。
- 刻意不用 IMU：躺地 pitch 不可信，判据只用 alpha（该工程里旧方案用 `|pitch|<30°` 判完成，已注释掉）。
- 他们自认的已知问题（本工程可顺手修掉，见 §6）：①早退的腿仍被斜坡继续拽；②卡死无法复位、无失败检测；
  ③判据与开关状态无关；④`spin_Leg` 是跨任务共享全局会抖。

## 4. 映射表（源 → 本工程）

好消息：本工程是**单任务双腿**，源工程一半的复杂度用不上。

| 源（AwakeningLion） | 本工程对策 |
|---|---|
| 双腿各一任务 + finish_flag 互锁 | 本来就 1 ms 单任务双腿，天然同步；每腿只留一个完成标志供交接 |
| 右腿归一化要整体取负（`-((α-2π))`） | 我们右腿在**反馈端就乘了 mirror**（`chassis_task.c:223-228`），两腿 alpha 已同号 → **一套公式两腿共用**，不要照抄右腿的取负 |
| 右腿 Tp 用 `+`、左腿用 `−`（他们未镜像） | 我们输出级已对右腿取负（`chassis_task.c:292-293`，与 LQR/腿长/roll 补偿同一约定）→ 两腿写同一个 Tp |
| 轮子 `SET_Current = 0` | 我们写 **`wheel_T = 0`**：`Chassis_CanTransimit()` 的 ONLINE 分支会拿 `wheel_T` 重算 `SET_Current`，直接写电流会被覆盖（ZERO_FORCE 已踩过此坑，`chassis_task.c:429-432` 注释） |
| `ramp_update()` + `Ramp_Typedef` | **本工程没有**。只有速度限幅型 `ramp_init/ramp_calc`（`User/Lib/user_lib.c:82,98`）。**09-20 定：照抄源 `user_lib.c:77-113`（有效行 28）**——typedef 进 `user_lib.h`、函数进 `user_lib.c`，实例每腿一个放 `chassis_task.c`。三个使用要点：①它用 `HAL_GetTick` 记毫秒——本工程 `Remote_Control.c:70` 已在用，时钟是活的（已核）；②参数不变时**不会**自动重置（`start_tick` 只在参数变或 `flag=1` 时复位）——首次进入和「s2 拨断重开」都靠 `flag=1` 在进入沿强制重启；③tick 回绕它在函数内部已处理 |
| `recover_flag`（趴平）、`start_flag` | 不需要。ZERO_FORCE 已承担"倒地卸力等人重上电" |
| 两个开关 2/3 + 恢复保护 | 见 §7 岔路口 1 |
| 腿长范围 0.13~0.36、`LEG_MG 14.33 / SLEG_MG 10.71` | 与本工程一致；常数已在 `chassis_task.h:33-34`（注释写明抄自他们） |
| 结束交 LQR | 交本工程 NORMAL（`LQR()` + `LEG_Lenth_Control()`），不用改 NORMAL 本身 |

**不需要动的**（已核对）：

- **LESO**：`LESO_Service()` 只在 `enable==ONLINE && mode==NORMAL` 时工作（`LESO.c:431`），扫描期间 early-return 不注入；
  NORMAL 边沿清 `u_last` 并重新 seed。`LESO_Feedback()` 照收 `u_last`，无害。
- **remote**：`Remote_task.c:47` 写腿长目标有 `if(chassis_mode==NORMAL)` 守卫，扫描期间不抢；
  `chassis_enable` 由 s2 无条件写（`:34-46`），正好就是"OFF→ON 重上电"那条链。
- **VMC / 输出链路**：扫描期间只要在状态里写好 `F0/Tp/wheel_T`，`VMC_translate()`→`Chassis_CanTransimit()` 照常工作。

## 5. 四个必须换算或确认的点

### 5.1 符号链——按用户确认（09-20）：三步手推降为上车首查

他们两条腿用**未镜像**的关节约定，所以同一物理方向的扫描在两腿上表现为 Tp 差一个负号。
我们已经镜像，所以**大概率**：

- 归一化用左腿那套（`alpha<0 → +2π`，站点 0≡2π，从下方逼近 5.5）；
- 控制律 `Tp = +[Leg_mg*L0/2*sin(f) + PID(f,ref)]`，两腿共用。

但"大概率"不等于确认，取决于我们关节零位/正方向与其镜像后的关系。**手推实验**（不接动力）：

1. 躺姿读 `alpha_L` / `alpha_R`：应**同号且接近**。反号 → 镜像假设不成立，需退回"每腿一套归一化"。
2. 立姿用手把腿往前拨，看 alpha 增大还是减小 → 定卷绕方向（决定 5.5 / 0.9 两个窗口落在圆的哪一侧）。
3. 给恒定小 Tp（+1 N·m 调试量）看两腿是否都朝"能撬起机身"的方向转 → 定全局符号（`+ff+PID` 还是 `−ff−PID`）。

> 09-20 用户答复「转向这些应该都没问题」：按「两腿同一套归一化（`alpha<0 → +2π`）+ `Tp = +[ff+PID]`」落码。
> 上列三步降级为**上车联调第一项**；若不通过，只需改这两处（归一化方向 / Tp 全局符号），不动结构。

### 5.2 Kd 换算（他们的 3 ms 是这么来的）

他们的任务里每条 CAN 之间有 `osDelay(1)`（`chassisR_task.c:136-142` 等），一个周期至少 2 个，
自己按 **3 ms** 记账（`VMC_calc_1_right(..., CHASSR_TIME*3.0f/1000.0f)`）。
本工程是真的 1 ms（`FreeRTOSConfig.h:64` tick=1000，ONLINE 分支 CAN 发送 delay=0 不阻塞，单个 `osDelay(1)`）。

`PID_Calc` 的 D 项是每周期差分、不除 dt（`pid.c:100`），所以等效连续阻尼 = Kd × 周期：

- 他们：3000 × 3 ms = **9 N·m/(rad/s)**；
- 我们 1 ms：**Kd ≈ 9000**（角度环、腿长环同）。

更稳的写法（工程先例：`chassis_task.h:12-17` 的 `LEG_PID_KD_RATE`）：
`D = 9 × (d(ref)/dt − d_alpha)`——`vmc->d_alpha` 在 `VMC_calc_1` 里已算好（`VMC_calc.c:112`），不依赖循环周期。
注意别漏掉 ref 的变化率（斜坡斜率最大 `(5.5−init)/4s ≈ 1.375 rad/s`，对应最多 ≈12 N·m 的速度前馈）。

### 5.3 卷绕点的 D 项尖峰

`fdb` 包裹到 [0,2π) 后，alpha 跨 0 时 fdb 跳 2π，差分型 D 项当周期会打出 `Kd·2π` 的尖峰。
他们的扫描停在 5.5、到不了卷绕点所以基本不触发；保留"保持/早退"路径就会踩到。
**用 `d_alpha` 做 D 项可完全避开**（选速率形式的第二个理由）。

### 5.4 入口判据：保留我们的，不要照抄纯开关

`CLAUDE.md` §3 里 OFF→ON 位姿复检（`chassis_task.c:355-369`）是**刻意**的
（抱着车拨开关 → NORMAL，不空中甩腿）。他们入口是纯开关无检测。两者正交：
**入口用本工程的检测/复检，只替换 FALLING_DOWN 内部的控制律**。
他们"不用 IMU"是针对**完成判据**（用 alpha 判），不是针对入口（入口时车躺地上、phi0 可信）。

### 5.5 交接门必须同时过倒地检测器（09-20 补算）

`phi0 = π/2 − alpha`；扫描末端 `alpha = f − 2π` 恒成立，故 **`phi0 = 7.854 − f`**：

- `f = 5.5` → `phi0 = 2.354`，距检测器上限 `2.5` 只剩 **0.146 rad（≈8°）**；
- `f = 5.35`（收腿窗边缘）→ `phi0 = 2.504` **越界**；
- `f = 5.6` → `phi0 = 2.254`，裕量 0.25 rad。

即：**交回 NORMAL 的那一拍必须 `|Pitch| < 0.3` 且 `f ≳ 5.4`**，否则检测器下一拍（1 ms 后）立刻打回 ZERO_FORCE。

两条结论：

1. `spin_angle_end` 用 **5.6**（源码 5.5 + 0.1 裕量；动这个数比放宽检测窗安全——那组阈值是 `CLAUDE.md` 刻意保的）。
2. `FALLING_TO_NORMAL` 必须当**交接闸门**：两腿都在「站位侧」（`f ≥ spin_angle_end − 0.05` 或早退侧 `f ≤ 0.9`）、`|Pitch| < 0.2`、两腿 `|d_L0| < 0.05` 才放行；两腿完成后 1.5 s 未达标 → 回 ZERO_FORCE（不硬交权：交出去也会被检测器一拍打回，白甩一次腿）。

同一式的推论：检测器 phi0 窗 `[0.4, 2.5]` 对应 f 的两段安全区 `[0, 1.17]` 与 `[5.354, 2π)`——早退侧（f≤0.9）与逼近侧（f≈5.6）都落在里面，交权后不会误触发。

## 6. 建议落地形态

状态机骨架不动，只换三个状态的职责：

```
NORMAL --检测倒地--> ZERO_FORCE --OFF→ON（位姿复检）--> FALLING_DOWN
   ^                   卸力/清 wheel_T                     │ 扫腿（移植点）
   └────────── NORMAL <-- FALLING_TO_NORMAL <--------------┘
                 LQR       收腿 + 交接条件
```

`FALLING_DOWN` 内单周期伪码（两腿各一份 L/R）：

```c
f = alpha;  if (f < 0) f += 2*pi;              /* 归一化，两腿同一套（方向待 §5.1 确认） */
if (f <= 0.9f) done = 1;                       /* 早退：腿已在站位附近 */

if (进入本状态第一拍) { init = f; t = 0; }      /* 状态进入沿捕获起点，替代他们的 2→3 边沿 */
t += dt;  ref = init + (5.5f - init) * min(t/4.0f, 1.0f);

Tp = +LEG_MG*L0/2*sin(f) + PID(f, ref);        /* 全局符号待 §5.1 确认 */
F0 = -SLEG_MG*cos(f)    + PID(L0, spin_Leg);
wheel_T = 0;

if (fabsf(f - 5.5f) < 0.35f) { spin_Leg = 0.13f;  if (fabsf(0.13f - L0) < 0.02f) done = 1; }
else                          spin_Leg = 0.3f;

if (done) ref = f;                             /* 冻结 → 修掉源方案问题① */
```

对源方案的三处**有意改进**（都是他们的已知问题）：

1. **早退腿冻结 ref**（他们的①：早退后仍被斜坡拽向反方向）。
2. **超时兜底**：扫描超时（6 s 未进收腿窗口）→ 回 ZERO_FORCE；交接闸门再加 1.5 s 超时（§5.5）→ 同样回 ZERO_FORCE 等人重新上电（他们的②；`CLAUDE.md` 也点名过无失败判据）。
3. **每腿独立 init/ref**（`spin_angle_init_L/R`、`ref_L/R` 已预留），共用 target=5.5 与 4 s 时长；
   躺歪时两腿 init 不同，各自线性到同一点、同时到位。

`FALLING_TO_NORMAL` 建议改成"收腿 + 交接"：

- 腿长目标 0.13；注意 NORMAL 里 remote 会把 `set_L0` 写成 0.15/0.3，**交接瞬间会有一次小伸展**
  （要么接受，要么把收腿目标直接定 0.15）。
- 交接条件用"L0 到位 + |Pitch| 小 + 角速度小"。现在那套 `theta≈0.5±0.1`（`chassis_task.c:520`）
  是按旧方案（phi0 级联）调的，**必须重调**——`CLAUDE.md` §3 已预警。

## 7. 待决策岔路口

| # | 问题 | 建议 |
|---|---|---|
| 1 | 起扫触发：进 FALLING_DOWN 自动 / 遥控开关边沿 | **自动**。遥控开关都占着：s1 在 NORMAL 是腿长档位、s2 是使能；要保留边沿还得把 `Remote_task.c:34-46` 的 `last_s1/last_s2` 更新代码解注释。而"拨 s2 回中/下 → OFFLINE → 关节 0 力矩 + 轮子 0"这条随时停机的安全阀已现成 |
| 2 | 前馈/判据自变量：alpha（照搬，不用 IMU）vs theta（现有 `falling_leg_control` 用的） | **alpha**。注意我们 `theta = Pitch − alpha` 与他们 `alpha − Pitch` 反号：sin 前馈要跟着反，cos 前馈（偶函数）无影响 |
| 3 | 旧的 phi0 级联自启（`falling_leg_control` + `PHI0_*` + `set_phi0_*`）留还是删 | 建议**删**并一起清理，否则就是现在这棵编译不过的树。删前确认没有别的分支在用 |

## 8. 上车前验证清单（按风险排序）

1. **手推三步**（§5.1）——符号没定，别上电。
2. 编译 + 链接通过（先修 `.h` 全局定义与改名残留），**只读**上电：看躺姿下 `alpha_L/alpha_R` 数值、
   5.5 在圆上的相对位置（本工程有 PC_Comm 可用）。
3. 轮子不转、腿长锁 0.3，**只跑角度环**：手拨腿看跟不跟 ref，确认转向与增益。
4. 整段扫描空载（车吊起）跑一遍：看 4 s 斜坡、收腿窗口、超时兜底。
5. 地面实测，第一次务必守着 s2（随时 OFFLINE）。

## 9. 索引

本工程（`code_framework v1.1`）：

- `User/APP/chassis_task.c`：116 调用检测；221-228 右腿 mirror；282-319 CanTransimit（292-293 右腿取负）；
  321-346 LEG_Lenth_Control；348-383 falling_down_detect（355-369 OFF→ON 复检，377-382 连续检测）；
  387-421 falling_leg_control（旧方案，待删）；423-442 zero_force；450-493 falling_down（457-475 WIP 残码）；
  494-524 falling_to_down
- `User/APP/chassis_task.h`：33-34 LEG_MG/SLEG_MG；59-65 spin_angle_* 全局（**要改成 extern**）；
  120-129 enable/mode 枚举
- `User/APP/Remote_task.c`：34-46 s2→enable；47-60 s1→腿长（NORMAL 守卫）
- `User/Algorithm/VMC/VMC_calc.c`：90 alpha；109 theta；112 d_alpha
- `User/Lib/user_lib.c`：82 ramp_init / 98 ramp_calc（**无** ramp_update）
- `User/Algorithm/PID/pid.c`：78-101 PID_Calc（误差=ref−fdb，D 为每周期差分）
- `User/Algorithm/LESO/LESO.c`：431 模式门（ONLINE 且 NORMAL 才工作）
- `User/Devices/DM_Motor/DM_Motor.h`：63-64 J8009 限幅 ±40 N·m
- `Core/Inc/FreeRTOSConfig.h`：64 tick = 1000 Hz

源工程（`...\AwakeningLion_2027SERIES_LEH_DM-MC02_zbx\DM-balance`）：

- `User/APP/chassisR_task.c`：69-70 PID 增益；74-75 终点/腿长；239-240 质量常数；259-296 扫腿分支
  （261-263 归一化、265 早退、274 斜坡、281-282 控制律、285-290 窗口/完成、293-295 平衡分支与标志清零）
- `User/APP/chassisL_task.c`：177-215（180-181 归一化、183 早退、192 斜坡、199-200 控制律、204-209 窗口）
- `User/APP/remote_task.c`：53-74 标志位；120-128 s[0]=3&s[1]=2 → leg_set 0.18
- `User/Lib/user_lib.c`：77-113 ramp_update
- `User/Algorithm/VMC/VMC_calc.c`：38 alpha；48 theta（与我们的 theta 反号）

---

## 10. 落码清单（09-20 定稿，尚未落盘）

**`User/Lib/user_lib.h`**：加 `Ramp_Typedef`（源 6 行）+ `ramp_update()` 原型。
**`User/Lib/user_lib.c`**：加 `ramp_update()` 本体（照抄源 77-113 代码；注释转 ASCII 英文，遵守插入规则）。

**`User/APP/chassis_task.h`**：
- 59-65 行 7 个 `spin_angle_*` 定义 → `extern`（消链接重复符号）；`spin_angle_end` 定义移入 .c（初值 5.6f）；
- 新增 `SPIN_*` 常量组 16 个（增益 4 / 阈值 6 / 超时与交接门 6）；
- 删 `FALLING_LEG_PID_*`（26-30，随旧方案废）；删 `set_spin_angle_Left/Right`（109-110，WIP 死字段）。

**`User/APP/chassis_task.c`**：
- 42 行后：spin 全局（7 个 `spin_angle_*` 定义 + `spin_started`/两完成标志/`Ramp_L,Ramp_R`/`spin_entry_tick`/`spin_done_tick`）；
- 删 36-37 `FallingLeg_*_Pid`；387-421 `falling_leg_control()` 整段 → `spin_leg_normalize()` + `spin_leg_at_stance()` + `spin_leg_step()`；
- 450-493 `falling_down()` 重写为扫描器（进入沿 latch init + `ramp_update` + 完成标志 + 6 s 超时回 ZERO_FORCE）；
- 494-524 `falling_to_down()` 重写为交接闸门（保持收腿 + §5.5 条件 + 1.5 s 超时）；
- `zero_force()` 加 `spin_started = 0`；`falling_down_detect()` 的 OFF→ON 沿加 `spin_started = 0`（覆盖扫描中途拨断重开的再进入）。

**不动**：NORMAL/LQR、LESO、Remote_task、VMC、检测器阈值、`LEG_Lenth_Control`。
