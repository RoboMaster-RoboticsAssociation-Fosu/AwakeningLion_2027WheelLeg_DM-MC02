# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

正文中文，写给下一个会话，也给人看。最后更新：2026-10-05。

达妙 DM-MC02 板（STM32H723VG）上的双轮腿平衡机器人固件：CubeMX 生成的 HAL + FreeRTOS（CMSIS-RTOS v1），用 Keil MDK 编译。
git 仓库根在上两级 `wheel_legger/`；离线算增益的 Python 脚本在上一级（`../ABK_LQR.py`、`../lqr_k_calc/`，见 §2.5）。

## 0. 第一铁律：不要用 Edit / Write 改这里的源码

- **源码编码按文件各不相同**（首个提交起就是这样），不是统一的 GB2312：
  - GB2312/GBK：`User/APP/chassis_task.c/.h`、`User/Algorithm/{VMC,kalman,mahony}/*`、`some_config/some_para.h`、`LESO/LESO.h`、`User/Devices/DT7_Remote/*`。
  - UTF-8 无 BOM：其余带中文的 `User/` 文件（DM-MC02 框架原文件，头注释常写 `@encoding UTF-8`），包括 `LQR.c`、`LESO.c`、`INS_task.*`、`Bsp/*`、`Devices/*`、`Lib/*`。
  - 行尾也是逐文件的（CRLF、LF 都有）。git `core.autocrlf=true`，所以 `git diff` 看不出行尾变化。
- 09-19 实测：Edit 工具会把 GB2312 文件**整个**重存为 UTF-8，无法解码的字节全部变成 U+FFFD。改一次就会毁掉整个文件的中文注释，不只是你改的那几行。（"用 Edit 改纯 ASCII 是字节安全的"这个旧结论已被推翻。）
- 正确姿势：用 Python 按字节读写（`open(p,'rb')` / `open(p,'wb')`），沿用文件原来的行尾，插入的内容保持纯 ASCII。实在要写中文，就用**该文件自己的编码**。反例：`LESO.h` 第 20 行是插进 GB 文件的一行 UTF-8，行尾还是 LF。
- 改完自检**别再用 `file` 判断健康**，UTF-8 文件报 UTF-8 是正常的。改前改后对比这几项：`EF BF BD`（U+FFFD）个数不增；按原编码严格解码仍通过；CRLF/LF 个数只随插入的行变化；字节数 = 原大小 + 插入量。
- 根目录的 `.scan_enc.ps1` 别依赖：`$root` 指向旧路径，而且它查不出 U+FFFD（损坏后的文件恰好是合法 UTF-8）。
- 被外部改动过的文件，别在编辑器里直接 Ctrl+S，会按当前编码重存一遍。用户 VS Code 默认编码是 `gb2312`，所以本目录的 .md 要存成 **UTF-8 with BOM + CRLF**，靠 BOM 覆盖默认编码。
- `.clang-format` 有，但现有代码从没按它排过，别整文件格式化。

## 1. 构建 / 烧录 / 验证

以 Keil 工程为准：`MDK-ARM/CtrlBoard-H7_IMU.uvprojx`，target `CtrlBoard-H7_IMU`。MDK 5.36 + ArmClang V6.16，装在 `C:\Keil_v5`；编译选项 `-O0 -std=c99`，用 MicroLIB，`-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard`，宏 `USE_HAL_DRIVER,STM32H723xx`。产物在 `MDK-ARM/CtrlBoard-H7_IMU/CtrlBoard-H7_IMU.{axf,hex}`。

命令行编译（Git Bash，在本目录下执行；bash 会等 UV4 结束，全量编译约 5 s）：

```
"/c/Keil_v5/UV4/UV4.exe" -b MDK-ARM/CtrlBoard-H7_IMU.uvprojx -t CtrlBoard-H7_IMU -j0 -o build.log
tail -3 MDK-ARM/build.log        # -o 的路径相对工程目录 MDK-ARM/
```

- 退出码 0 = 干净，1 = 有警告，≥2 = 出错。当前基线是 `0 Error(s), 0 Warning(s)`，出现新警告就当回归看。`-r` 是全部重编。PowerShell 不会等 GUI 程序退出，要用 `Start-Process -Wait`。Keil GUI 正开着同一工程（尤其在调试中）时，别从命令行编，产物可能被占用。
- 新增 .c 文件或新目录，要在 uvprojx 里加分组和 include path。EIDE 配置不会跟着同步：根目录的 `.eide/eide.yml` 已过时，缺 `LESO.c`、`Remote_task.c`、`some_para.c`，用它编会链接失败。别把 EIDE 当成文件清单。
- 编译产物、`.uvguix.*`、`.uvoptx` 都在 git 里，编一次就会改动几十个二进制文件，提交时要挑着加。
- 烧录：在 Keil 里 Download，调试器配的是 CMSIS-DAP。根目录的 `flash.bat` / `flash.jlink`（J-Link）在本机用不了：JLink 路径写的是 `D:\Keil_v5-536\…`（本机实际在 `C:\Keil_v5\ARM\Segger\JLink.exe`），要烧的 EIDE 产物 `build/CtrlBoard-H7_IMU/*.hex` 也不存在。
- 单腿动作与底盘自起阶段的主机 C 验证为 `node mdk_check/leg_motion_verify.js`，需 Node.js 和 C99 主机编译器（默认 gcc）。覆盖及实车边界见 [自起交接](projectmd/LEG_MOTION_HANDOFF.md)。离线 VMC 数值校验使用 `node mdk_check/vmc_verify.js` 和 `node mdk_check/jacobian_verify.js`，用有限差分核对 `VMC_calc.c` 的 dL0/dphi0 解析式和 `VMC_calc_2` 的雅可比。脚本里的杆长（0.215/0.254）和固件（0.208/0.25212）不一样，所以只能验公式，验不了数值。
- 其余只能上车看：用 Keil 调试器 Watch 全局变量，例如 `fb_dt`、`t_fb/t_mode/t_leso/t_can/t_sum`（`chassis_task.c` 里的临时插桩）、`leso_dbg_*`、`chassis_leg_motion[0]/[1]`。

## 2. 架构

### 2.1 硬件与总线
- 关节：4 个 DM-J8009，MIT 模式下**纯力矩**（pos/vel/kp/kd 全给 0），限幅 ±40 N·m。轮子：2 个 DJI M3508，共用一帧 0x200 发电流；力矩到电流的系数是 ±3330，推导见 `chassis_task.h`。
- 用的是经典 CAN，不是 FD。FDCAN1 挂两个轮子（RX 0x201 是右轮，0x202 是左轮）和右腿两个关节；FDCAN2 挂左腿两个关节；FDCAN3 已配置，但没接设备。
- **改总线或 ID 要同时改两处**：发送端是 `chassis_task.h` 的 `*_CAN_hfdcan` 和 `JOINT_DM_*_ID_Set` 宏；接收端是 `bsp_can.c` 的 Rx 回调，按 ID 硬编码分发，直接写全局 `Chassis`。另外，`Joint_Motor[]` 的下标 = TxID − 1（`Chassis_Joint_ID_e`）。
- IMU 是板载 BMI088（SPI2）。遥控是 DT7/DR16 的 DBUS 帧（代码里叫 SBUS），走 UART5 DMA 双缓冲 + IDLE 中断，解析进 `remote_ctrl`。USART2（宇树电机）和 USART10（步进电机）只开了接收，控制里没用。`User/Devices/DT7_Remote/` 不在工程里。

### 2.2 任务（`Core/Src/freertos.c`，tick 1 kHz）
任务之间全靠全局结构体共享数据（`Chassis`、`INS`、`remote_ctrl`），没有队列，也没有锁。

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| `INS_TASK` | Realtime | 1 ms | BMI088 → Mahony → `INS`（Pitch/Roll/Yaw/Gyro）。约 1.5 s 后置 `INS.ins_flag`，chassis 要等它才初始化。EKF/kalman 编进去了但没用 |
| `REMOTO_TASK` | High | 30 ms | s2：1/3 → ONLINE，2/0 → OFFLINE；ch3 → `v_set`（±2 m/s），ch0 → `yaw_set_v`（±3 rad/s）；只在 NORMAL 下用 s1 切腿长 0.15/0.20/0.25 m |
| `CHASSIS_TASK` | AboveNormal | 1 ms（`osDelayUntil`） | 全部控制，见 2.3 |

`defaultTask` 只做 USB 初始化；PS2 任务的创建被注释掉了。

### 2.3 chassis_task 的 1 ms 周期
`chassis_feedback_update()` → `YAW_Parameter_Processing()` → `falling_down_detect()` → 按 `chassis_mode` 分发 → `LESO_Service()` → `VMC_translate()` → `Chassis_CanTransimit()`

- 反馈更新：`fb_dt` 用 DWT 实测；关节角经 `VMC_calc_1` 正解，得到 L0、phi0、theta 及它们的导数；轮速算出 `Estimate_dx`；静止时积分 `x`。
- **每个模式函数只写四样东西**：每条腿的 `vmc.F0`、`vmc.Tp`，以及 `Wheel_Motor[].wheel_T`。后面的 `VMC_calc_2`（雅可比，把 F0/Tp 换成关节力矩）和发送是所有模式共用的。
- NORMAL = `LQR()`（写 `wheel_T`、`Tp`）+ `LEG_Lenth_Control()`（写 `F0`：重力前馈 + 腿长 PD + roll PD，roll 项左右反号）。
- 轮子必须写 `wheel_T`，不能写 `SET_Current`：ONLINE 分支每拍都会用 `wheel_T` 重算电流，直接写的电流会被覆盖。
- OFFLINE（s2 = 2）时所有电机发 0，这是唯一的急停。

### 2.4 VMC 与坐标约定（写新控制律前先看）
- 五连杆，`l5 = 0`（两髋同轴），l1 = l4 = 0.208，l2 = l3 = 0.25212（`VMC_init`）。前关节对应 phi4，后关节对应 phi1。
- **右腿在反馈端乘了 `mirror = -1`，输出端又把关节力矩取负**，所以左右腿用同一套公式、同一个符号。新写的控制律两腿共用，不要再按腿取反。
- `alpha = π/2 − phi0`；`theta = Pitch + phi0 − π/2`（LQR 用的腿摆角）。`d_L0`、`d_phi0` 是由关节角速度解析算出来的，不是差分。
- 下标：`LEFT_Leg = 0`，但轮子是 `Wheel_Motor[LEFT_Wheel = 1]`、`[RIGHT_Wheel = 0]`。

### 2.5 LQR / LESO 与离线工具链
- 状态 `[s, ds, yaw, dyaw, θL, dθL, θR, dθR, pitch, dpitch]`，输入 `[T_wL, T_wR, Tp_L, Tp_R]`。`LQR.c` 的 `u[]` 是"目标 − 反馈"，先限幅再乘增益。
- 每个矩阵元素都是左右腿长的 poly22：列序 `[p00 p10 p01 p20 p11 p02]`（X = L_l，Y = L_r），行序为矩阵按行展平。K（4×10）存在 `LQR.c` 的 `P[40][6]`；LESO 的 Ad（10×10）、Bd（10×4）、L（14×10）分别存在 `LESO.c` 的 `AdP`、`BdP`、`LP`。
- 这些系数都是上一级目录的脚本打印出来、再**手工粘贴**进来的：`../ABK_LQR.py`（ZOH 离散 + dlqr，LESO 用极点配置，`Ts_ac = 0.001` 对应 1 kHz）；`../lqr_k_calc/hku_lqr_k_calc.py`（连续 LQR，HKU 复刻，参数看 `--help`）。`../lqr_k_calc/*.txt` 是 09-06 的旧输出，和固件里现在的表不一样。改控制周期就必须重算 Ad/Bd/L（`LESO.c` 里还写死了 `stable_t += 0.001f`）。
- LESO 只在 ONLINE 且 NORMAL 时运行，每次进入都重新 seed。0.5 s 后经一阶斜坡，从 `wheel_T`/`Tp` 里减去 `comp·dh`。`LESO_Feedback()` 在 `Chassis_CanTransimit()` 里回灌当拍的 `wheel_T`/`Tp`。`leso_comp_scale` 可以在调试器里实时改，设为 0 就只观测、不注入。模型假设双轮着地，但 `ground_detectionL/R` 写了却没被调用。
- `PID_Calc` 的 D 项是逐拍差分，没除 dt，增益会随周期变。新写的环用显式速率形式，参考 `LEG_PID_KD_RATE` 和 `LegMotion_Run`。

### 2.6 倒地 / 自启状态机（`chassis_mode`）

修改自起阶段、重启、超时或交接时，先读 [倒地自起封装交接](projectmd/LEG_MOTION_HANDOFF.md)；调用单腿动作接口时，读 [LEG_MOTION.md](projectmd/LEG_MOTION.md)。前者记录阶段边界、验证结果和实车续接事项，后者记录命令参数与调用示例。

```text
OFF→ON 位姿判为倒地 → FALLING_DOWN（两腿各自 SWING → RETRACT → HOLD）
两腿均 HOLD → FALLING_TO_NORMAL（保持 + 交接条件）→ NORMAL
在线 NORMAL 判为倒地 → ZERO_FORCE，等待 OFF→ON
自起超时、无效反馈或自起映射异常 → 当周期 ZERO_FORCE
```

- `falling_down_detect()` 保留开启时位姿复检：任一腿 phi0 ∉ [0.4, 2.5]，或 |pitch| > 0.3，则进 FALLING_DOWN；其余情况进 NORMAL。在线 NORMAL 持续检测，超出范围进入 ZERO_FORCE。
- 当前单腿控制实现为 `User/Controller/leg_motion.c/.h` 的 `LegMotion_Run()`，通过底盘适配函数接入。旧级联倒地控制实现及对应 PID 对象已清理。
- `SPIN_SWEEP_DIR` 当前为 `LEG_MOTION_NEGATIVE`；`SPIN_TARGET_ANGLE_DEG` 为绝对姿态 +39.143°，对应 phi0≈2.254 rad。原方向 -1、角量 5.6 rad 的末姿与此等价；新参数以度表示绝对姿态，详见交接文档。
- 两腿收腿阶段分别锁存，保持进入收腿时的实际角度；当前收腿参考变化时间 1 s。交接检查包括原正常 phi0 范围。
- `ZERO_FORCE` 分支保留 break；清零同时覆盖腿虚拟输出、映射关节力矩和轮输出。零力及禁用状态跳过 VMC 映射；交回 NORMAL 的最后一帧仍检查自起映射结果。

## 3. 其他陷阱

- **CubeMX 重新生成会冲掉手改**：`.ioc` 和代码不一致，下面这些改动都在 USER CODE 块之外。
  - `FreeRTOSConfig.h` 的 `INCLUDE_vTaskDelayUntil 1`：变回 0 时 `osDelayUntil` 会直接返回，chassis 任务不再阻塞，低优先级任务会被饿死。
  - `freertos.c` 的 CHASSIS 栈是 1024（`.ioc` 里是 512），PS2 任务的创建被注释掉了。
- 烧录或复位后，如果遥控 s2 已经在 ONLINE 档，上电就等同于一次 OFF→ON：车会直接进 NORMAL 平衡，或者开始扫腿。
- 轮力矩的 `mySaturate(wheel_T, ±4.8)` 写在**发送之后**。实际下发只受电流 ±16384（约 ±4.9 N·m）限制，这个限幅只影响回灌给 LESO 的值。真要限制轮力矩，得挪到算 `SET_Current` 之前。
- `Gear_Ratio` 宏没加括号（`268.0f/17.0f`），只能写在 `(Gear_Ratio*…)` 里；写成 `x/Gear_Ratio` 会算错。
- `Remote_Is_Offline()` 写了但没人调用：遥控失联后 `remote_ctrl` 停在最后一帧，s2 仍是 ONLINE，摇杆量也照样生效。

## 4. 待实测 / 已知风险

- 连续检测在瞬态超出范围时就会触发 ZERO_FORCE。这时车可能还站着，腿一软就直接砸下去，不是受控自启。
- 位姿判据只看 phi0 和 pitch，不看 roll，侧躺时可能漏判成 NORMAL。
- `SPIN_SCAN_TIMEOUT_MS` 现在是 600000（10 分钟），plan 里的设计值是 6 s。上车前确认是不是调试留下的。

## 5. 编码现状与注释恢复

- 健康：`chassis_task.c`、`chassis_task.h`（GB 编码，U+FFFD = 0）。
- 已损坏、git 历史里也没有干净版的文件，只能重写注释（括号内是 U+FFFD 个数）：`LESO.c`（231，从引入时就有；后来加的注释是好的）、`VMC_calc.h`（54）、`main.c`（41）、`user_sys_config.h`（37）。这些在 Keil 的 GB 视图里显示成"锟斤拷"。
- `User/APP/*.fffd.bak`（09-16）**都别删**：`chassis_task.c.fffd.bak` 和 `Remote_task.c.fffd.bak` 是干净的 GB 旧版（当前 `Remote_task.c` 注释里的"————"已经变成 `?`）；`chassis_task.h.fffd.bak` 本身就是损坏版（UTF-8，383 个 U+FFFD），不能拿来当恢复源。
- 恢复配方（09-19 验证过：26/26 行命中，字节精确）。损坏是确定性变换：`line_bytes.decode('utf-8', errors='replace')` 后存成 UTF-8，所以可以反查。
  1. 找干净副本：上面的 .bak，或者 VS Code 历史 `%APPDATA%\Code\User\History\<hash>\*.c`（就是文件原始字节，没有头部）。
  2. 对干净副本里每一行含非 ASCII 的内容，算出它损坏后的形态；如果损坏文件里出现了这个形态，就用干净副本的原始字节替换那一行。
  3. 必做验证：残留 `EF BF BD` = 0；按原编码严格解码能通过；行数一致；只有被替换的行字节变了，且每行的 ASCII 骨架（< 0x80 的字节序列）都没变。
