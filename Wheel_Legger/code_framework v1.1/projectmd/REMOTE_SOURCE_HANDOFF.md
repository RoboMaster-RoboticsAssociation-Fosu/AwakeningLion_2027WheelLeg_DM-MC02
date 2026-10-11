# 轮腿底盘双来源遥控接入

日期：2026-10-11。

## 选择控制来源

配置位置：`User/Config/user_sys_config.h`。当前配置使用 GIMBAL，即云台转发遥控。两种配置的写法如下；本地 DT7/DR16 接收机连接底盘 UART5。

```c
#define CHASSIS_CTRL_SOURCE CHASSIS_CTRL_SOURCE_LOCAL
```

改用云台板间遥控时，将该行改为：

```c
#define CHASSIS_CTRL_SOURCE CHASSIS_CTRL_SOURCE_GIMBAL
```

修改后使用权威 Keil 工程重新编译、烧录。配置也允许编译器定义覆盖；非法值会编译报错。两路接收始终使用独立存储，统一遥控任务只选取配置指定的来源。

## 修改操控看哪里

日常修改集中在 `User/APP/Remote_task.c` 前部；前部操控函数生成请求和目标，后部处理输入来源、失联保护、目标写入及启动记录。

| 函数 | 修改内容 |
|---|---|
| `Remote_MapModeRequest()` | 更换使能拨杆或按键，修改 S2 判断与 ENABLE / DISABLE / INVALID 返回请求 |
| `Remote_MapJoystick()` | 更换前进/转向通道，修改 ch3 / ch0；反向修改正负号 |
| `Remote_MapLegLength()` | 更换腿长操作，修改 S1 判断和各档位的映射 |

文件顶部的 `REMOTE_FORWARD_MAX_MPS` / `REMOTE_YAW_MAX_RADPS` 配置满杆速度；`REMOTE_LEG_SHORT_M` / `REMOTE_LEG_MIDDLE_M` / `REMOTE_LEG_LONG_M` 配置三个腿长。

模式映射中的 ENABLE 表示请求使能，DISABLE 表示用户主动关闭，INVALID 表示零值或无效操作。DISABLE 也是云台复联时解除锁定的操作，因此更换使能键位时只需更改模式映射，复联保护会使用相同请求。输入失联或 INVALID 都不会解除锁定。

`Remote_ControlStep()` 的流程为：读取选中来源 → 解析模式请求 → 检查使能保护 → 映射摇杆 → 应用底盘目标。腿长仅在输入有效且实际模式为 NORMAL 时应用；无效 S1 保留左右腿原目标，有效 NORMAL 输入在关闭使能时仍按原规则选择腿长。

### 使能与实际底盘模式

遥控映射控制 `chassis_enable`；`chassis_mode` 由底盘状态机维护。

```text
chassis_enable：OFFLINE → ONLINE
chassis_mode：  FALLING_DOWN → FALLING_TO_NORMAL → NORMAL
正常运行倒地或自起失败：→ ZERO_FORCE
关闭后重新使能：重新启动自起
```

更换触发操作看 `Remote_MapModeRequest()`；修改使能边沿或倒地判断看 `chassis_task.c` 的 `falling_down_detect()`；修改自起阶段与 NORMAL 交接看 `chassis_recovery_control()` 及 `chassis_recovery` 模块。当前遥控函数不直接写入 `chassis_mode`，以便底盘完成每次自起和故障锁存。

## 板间协议与接入

底盘使用 FDCAN3，与参考云台工程的 FDCAN3 通信；现有配置为经典 CAN、1 Mbit/s。以下帧均为标准数据帧，长度必须为 8 字节。

| ID | 字节定义 |
|---|---|
| `0x51` | 0/1：ch0；2/3：ch1；4/5：ch2；6/7：ch3 |
| `0x61` | 0/1：拨轮 iw；2：S1；3：S2；4/5：键盘位图；6：rc_lost；7：online_cnt |
| `0x71` | 0/1：鼠标 x；2/3：y；4/5：z；6：左键；7：右键 |

所有 16 位值使用大端字节顺序；通道和鼠标值按有符号 int16_t 解析。通道值已经减去遥控中值，不再减去 1024。参考发送端未开启 `0x71`，不会影响控制在线判定。键盘、拨轮和鼠标仅接收保存，本次控制沿用摇杆和拨杆映射。

接收中断读取完 FDCAN3 FIFO，只校验并排队保存原始帧与时间。板间任务每 1 ms 按接收顺序解析。软件队列容量为七帧；溢出会作废输入并记录失联，后续需要新的完整必需帧对。顺序处理可保留同一任务周期内短暂出现的 rc_lost 报告。

接口：`BoardComm_OnCanRx()` 供 CAN 回调使用；`BoardComm_Update()` 执行一次通信处理；`BoardComm_GetRemoteSnapshot()` 获取一致快照并返回当前有效性。任务和中断间快照使用保存、恢复 PRIMASK 的短临界区。

## 控制与失联

统一遥控任务每 1 ms 运行。速度、转向和腿长映射如下：

- 速度：`ch3 / 660 * 2.0f`，单位 m/s。
- 转向：`-ch0 / 660 * 3.0f`，单位 rad/s。
- S2 上档 1 / 中档 3：使能；下档 2 / 零值 / 无效值：OFFLINE。
- 仅 NORMAL 模式处理 S1 腿长：下档或零值 0.159 m，中档 0.20 m，上档 0.25 m。

本地输入在收到第一帧前无效，最后一帧达到 50 ms 即失联。云台输入要求 `0x51` 和 `0x61` 都收到、两帧各自年龄小于 50 ms，且 rc_lost 为零。online_cnt 仅作诊断，恒定值不影响在线判断。

失联时统一入口设置 OFFLINE 并清零速度、转向指令，现有底盘发送流程清零全部电机。首次上电获得有效输入时，沿用当前 S2 档位使能规则；本地复联也沿用当前 S2 档位。

云台运行期间发生通信超时、遥控丢失或软件队列溢出后，锁存 `remote_gimbal_rearm_required`。恢复有效通信后必须先观察到 S2 下档 2，再切到上档或中档，才重新进入原自起流程。零值和无效档位不能替代手动下档操作。失联事件通过 loss_seq 保留，即使通信先于统一遥控任务恢复，也不会自动重新使能。

启动诊断仅跟随选中的来源。`recovery_dbg_rx_on_tick` 取对应使能输入的接收时间，`recovery_dbg_task_on_tick` 记录统一入口执行时间；未选中的来源不会改写启动记录。

两个输入任务保持 High 优先级，栈均为 256 words。`INCLUDE_vTaskDelayUntil` 已恢复为 1，任务使用固定周期延时；CubeMX 配置同步保存任务栈和该开关。

## 验证结果

- `node mdk_check/remote_source_verify.js`：两种配置运行实际 C 输入模块和实际 FDCAN3 回调，覆盖 4490 个断言；显式分别编译 LOCAL / GIMBAL，包含无效 S1 保留目标和实际底盘模式不被遥控改写的检查；非法宏及 RTOS 配置检查通过。
- `node mdk_check/leg_motion_verify.js`：单腿、自起接口、集成和独立斜坡验证通过。
- `node mdk_check/wheel_output_verify.js`：117 个轮输出断言通过。
- `node mdk_check/vmc_verify.js`、`node mdk_check/jacobian_verify.js`：公式数值核对通过；验证脚本杆长与固件不同，仅验证公式。
- Keil ArmClang 6.16：本地、云台两种配置均完成编译，零错误、零警告。构建日志在 `MDK-ARM/remote_mapping_local_build.log` 和 `MDK-ARM/remote_mapping_gimbal_build.log`。
- 最终标准 Keil 输出保留当前 GIMBAL 配置；LOCAL 验证固件保存在 `build/remote_mapping_local_keil/Objects/`。
- 源码编码、行尾与原文件保持一致；底盘控制、自起模块及参数与修改前备份逐字节一致。

## 实车验收待完成

本次未烧录或运行实车。

1. 本地配置：验证原 S1/S2、前进和转向映射；拔掉遥控接收后检查 OFFLINE 与所有电机输出归零；复联检查原自起流程。
2. 云台配置：验证同一套遥控操作；同时给本地 UART 输入不同指令，确认控制跟随云台。
3. 分别中断整条 CAN、只停 `0x51`、只停 `0x61`，检查达到 50 ms 后 OFFLINE、速度归零及电机归零。
4. 云台报告 rc_lost 时检查停机；恢复后保持 S2 上档应继续 OFFLINE，再执行 S2 下档到上档，应重新自起。
5. 观察 `remote_ctrl` / `remote_last_rx_tick`、`board_remote_ctrl`、`board_comm_rx_overruns`、`remote_gimbal_rearm_required`、`Chassis.chassis_enable`、`fb_dt` 和 `t_sum`；记录 IMU、任务周期、栈余量及电机表现。
