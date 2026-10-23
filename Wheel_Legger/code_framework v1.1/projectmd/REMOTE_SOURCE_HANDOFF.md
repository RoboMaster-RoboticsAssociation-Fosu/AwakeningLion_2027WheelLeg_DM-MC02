# 轮腿底盘遥控来源与操控修改说明

更新：2026-10-18。本次将小陀螺改为 S2 上档直接开启，固定目标 +3 rad/s；iw 仅接收保存。板间接收继续由 CAN3 回调直接解析，`remote_ctrl` 始终保存当前选中的来源。

## 选择控制来源

修改 `User/Config/user_sys_config.h` 的 `CHASSIS_CTRL_SOURCE`，重新编译、烧录：

```c
/* 底盘 UART5 连接 DT7/DR16 接收机，使用底盘本地遥控；默认及最终标准固件采用此项。 */
#define CHASSIS_CTRL_SOURCE CHASSIS_CTRL_SOURCE_LOCAL

/* 使用云台板经 CAN3 转发的遥控值时，将上面的 LOCAL 改为 GIMBAL。 */
/* #define CHASSIS_CTRL_SOURCE CHASSIS_CTRL_SOURCE_GIMBAL */
```

编译器定义可覆盖配置，非法值编译报错。两个接收入口按编译宏互斥：LOCAL 下只有 UART5 更新遥控；GIMBAL 下只有 CAN3 更新。未选中的入口不会修改遥控数据、接收时间或失联序号，不会自动回退另一来源。

## 修改操控看哪里

日常修改集中在 `User/APP/Remote_task.c` 前部；后部统一处理使能保护、底盘目标和启动记录。

| 函数或参数 | 修改内容 |
|---|---|
| `Remote_MapModeRequest()` | 更换使能拨杆或按键，返回 ENABLE（请求使能）、DISABLE（主动关闭）或 INVALID（无效操作） |
| `Remote_MapNormalState()` | NORMAL 内选择 MOVE / SPIN：S2 上档请求小陀螺，中档请求正常运动 |
| `Remote_MapJoystick()` | ch3 前进，MOVE 时 -ch0 转向，SPIN 时写固定角速度；SPIN 不读取 iw 或 ch0 |
| `Remote_MapLegLength()` | 修改 S1 各档腿长；无效 S1 保留左右腿原目标 |
| `REMOTE_SPIN_YAW_RADPS` | SPIN 固定角速度，默认 +3.0 rad/s；修改数值调整转速，改为负值反向 |
| 文件顶部其余参数 | 满杆前进 2 m/s、MOVE 转向 3 rad/s；腿长 0.159 / 0.20 / 0.25 m |

统一流程：`Remote_GetSnapshot()` 取数据 → `Remote_CheckEnable()` 判断使能 → 前部映射函数生成操作 → `Remote_ApplyControl()` 写入目标。

S2 下档主动关闭，上档或中档请求使能；零值和其他值为无效操作。两路共用 `remote_rearm_required`：上电、失联、无效 S2 后锁定，必须在输入有效时先收到主动关闭请求，然后切中档或上档才能使能。失联前保存的无效输入不能解除锁定。更换使能键位只修改模式映射，解锁逻辑使用同一个 DISABLE 请求。

- 前进：`ch3 / 660 * 2.0f`，单位 m/s。
- MOVE 转向：`-ch0 / 660 * 3.0f`，单位 rad/s。
- SPIN 转速：`yaw_set_v = REMOTE_SPIN_YAW_RADPS`，默认 +3 rad/s；重新编译、烧录后生效，物理旋转方向需实车确认。
- iw 的零值、正负及幅值均不影响模式和角速度；SPIN 中 ch0 同样不改变固定转速。

| S2 操作 | 有效且已解锁时的行为 |
|---|---|
| 上档 | 请求 SPIN，固定 +3 rad/s |
| 中档 | 请求 MOVE，原 ch0 转向 |
| 下档、无效输入或失联 | OFFLINE，请求 MOVE，运动指令归零 |

- S1 下档或零值 0.159 m，中档 0.20 m，上档 0.25 m；仅在输入有效且实际模式 NORMAL 时应用，主动关闭时沿用此规则。

### 使能与实际底盘模式

遥控只写 `chassis_enable`、`normal_state_request` 和运动目标，实际 `chassis_mode` / `normal_state` 由底盘任务维护。

```text
有效主动关闭 → 切上档或中档 → OFFLINE → ONLINE
底盘自动执行：FALLING_DOWN → FALLING_TO_NORMAL → NORMAL
正常运行倒地或自起失败：ZERO_FORCE，等待关闭后再次使能
```

自起期间请求 MOVE；解锁使能后保持 S2 上档，完成自起进入 NORMAL 后自动请求 SPIN，无需再次拨动。SPIN 平移目标归零，速度参考沿共享斜坡更新；切回中档时 u[0]/u[2] 当拍恢复，角速度参考连续跟随原 ch0 输入，不等待实测停转。LQR / LESO、腿长及底盘一次短临界区消费请求与两项目标的流程保持，详见 [NORMAL 技术说明](NORMAL_STATE_MACHINE.md)。

## 接收链路与协议

```text
LOCAL： UART5 DMA / IDLE → SBUS_TO_RC() ─────┐
                                          ├→ remote_ctrl → 快照 → 1 ms 遥控任务
GIMBAL：CAN3 FIFO 回调 → BoardComm_OnCanRx() ┘
```

图中只有宏选中的一路写入。`board_comm_task.c/.h` 保留文件名和 Keil 注册，但已不含队列、解析任务或第二份遥控结构。FreeRTOS 与 CubeMX 的 BOARD_COMMUNICA 任务配置已删除；REMOTO_TASK 保持 High、256 words、1 ms，`INCLUDE_vTaskDelayUntil = 1`。

CAN3 回调读完 FIFO，检查标准 ID、数据帧、经典 CAN 及 8 字节长度；合格帧立即解析，退出回调即可在 GIMBAL 配置中观察 `remote_ctrl.rc`。中断不写底盘运动指令。

| ID | 字节定义 |
|---|---|
| `0x51` | 0/1：ch0；2/3：ch1；4/5：ch2；6/7：ch3 |
| `0x61` | 0/1：iw；2：S1；3：S2；4/5：键盘位图；6：rc_lost；7：online_cnt |
| `0x71`（可选） | 0/1：鼠标 x；2/3：y；4/5：z；6：左键；7：右键 |

16 位数据为大端；通道和鼠标按有符号 int16_t 解析。云台通道已去中值，接收不再减 1024。iw、键盘和鼠标继续接收保存；iw 不参与模式、速度或方向控制，本次未修改协议。

唯一快照接口：`Remote_GetSnapshot(snapshot, rx_tick, loss_seq)`。同一短临界区保存并恢复 PRIMASK，同时获取遥控值、时间、失联序号和有效性；后两个输出指针允许 NULL。

LOCAL 首帧前无效，最后接收年龄达到 50 ms 时失联。GIMBAL 必须收齐 `0x51` / `0x61`，两帧各自年龄均小于 50 ms，且 `rc_lost == 0`。`0x71` 不刷新必需帧时间；`online_cnt` 恒定或不发送鼠标均不影响在线判定。

接收层在覆盖旧时间前检查超时间隔，并通过 `remote_loss_seq` 保留事件；云台短暂 `rc_lost=1` 后立即恢复，也不能被后一正常帧抹掉。即使两次遥控任务之间已恢复，统一入口仍锁定使能，等待有效主动关闭。超时等待期间快照直接返回无效，无需依赖接收回调继续执行。

失联后统一入口设置 OFFLINE，清零速度和转向，原底盘发送流程将四关节及双轮输出归零。原始接收值保留最后一帧，调试时应同时检查有效性和使能，不能只看摇杆值判断在线。

## Watch 调试变量

| 变量 | 含义 |
|---|---|
| `remote_ctrl.rc` | 当前选中来源的摇杆、拨轮、S1 / S2；两种配置看同一变量，iw 仅供观察 |
| `remote_last_rx_tick` | LOCAL 最近 UART 帧时间；GIMBAL 最近 0x61 时间（用于启动事件时间），不代替两帧独立超时判断 |
| `remote_loss_seq` | 接收层保存的失联事件序号；恢复不清零，遥控任务检测变化后重新锁定 |
| `remote_rearm_required` | 1：必须先有效主动关闭；0：允许按映射请求使能 |
| `board_rx_tick[2]` / `board_rc_lost` | GIMBAL 模块内部必需帧时间及发送端遥控丢失报告 |
| `board_remote_online_cnt` | GIMBAL 报文计数，仅供观察 |
| `Chassis.chassis_enable/chassis_mode` | 使能与底盘实际模式 |
| `Chassis.normal_state_request/normal_state` | 遥控请求及实际 NORMAL 子状态 |
| `recovery_dbg_rx_on_seq` | 每次遥控实际从禁止使能到使能递增一次，包含失联后的再次使能 |

`recovery_dbg_rx_on_tick` 取对应使能输入接收时间，`recovery_dbg_task_on_tick` 取遥控任务执行时间；未选中的来源不影响启动记录。遥控任务持续写请求字段，手动修改请求不能持续接管。

## 本轮验证

- `node mdk_check/remote_source_verify.js`：显式编译 LOCAL / GIMBAL，分别 3489 / 3659 项，零失败；覆盖上档固定 +3 rad/s、iw 和 ch0 不影响 SPIN、中档转向、上档保持跨越自起、切换无新使能事件、两来源 S2 冲突时的隔离、协议与直接接收、50 ms 超时及复联锁定；非法宏和 RTOS 配置检查通过。
- `node mdk_check/leg_motion_verify.js`：单腿、自起接口、集成和独立斜坡均通过。旧测例在 0.5 rad 处误判越界，现增加等于边界不触发及相邻越界触发的断言；4 / 8 秒斜坡测试仅在临时模块副本中延长总截止时间，生产自起参数保持原值。
- `node mdk_check/pitch_stop_verify.js`：34100 项通过，覆盖上电锁定、俯仰停机、失联归零及重新使能。
- `node mdk_check/motor_online_verify.js`：21036 项通过。
- NORMAL 正负转速各 33577 项、轮输出 117 项及 VMC / Jacobian 公式验证通过；数值脚本杆长与固件不同，仅验证公式。
- 本次 Keil ArmClang 6.16 编译当前 LOCAL 配置为零错误、零警告，日志 `MDK-ARM/s2_spin_build.log`；标准 AXF / HEX 已更新。GIMBAL 遥控分支通过主机编译检查。此前双配置全量构建记录仍见 `remote_unified_gimbal_build.log` / `remote_unified_local_build.log`。
- 离地613项、腿长斜坡48666项、LESO 320832项回归通过；LESO 旧测试的轮补偿默认值断言由0.2同步到源码已有的1.0，生产参数未改。
- 本次生产仅修改 Remote_task.c 的操作映射，底盘状态机、自起、LQR / LESO、CAN 电机映射及上电/复联保护保持。源码保留原编码和行尾；本说明使用 UTF-8 BOM + CRLF。

## 实车验收与既有现场问题

本轮未烧录或运行实车。待台架验证：上电持中／上档仍禁止使能；有效下→上先自起再自动进入 SPIN；上档目标固定 +3 rad/s 且不受 iw / ch0 影响；切中档立即恢复正常运动反馈、转速参考沿原斜坡连续变化。再验证断开输入后电机归零、复联保持档位仍锁定、再次有效下档后才能重新自起。云台分别停发 0x51、停发 0x61、报告 rc_lost，并与本地不同 S2 值对比来源隔离。记录 S2、请求/当前状态、yaw_set_v、yaw_v_ref、实测角速度、轮力矩、1 ms 任务周期及栈余量，确认物理方向和切换响应。

此前现场反馈猛压机体后持续带力、下拨未停；回放已证明旧本地复联逻辑存在自动自起路径，但未证明现场一定发生过失联，也未确认“下拨未停”的原因。继续排查应同步捕获 tick、接收时间、任务执行、底盘模式和 CAN 发送状态。原 UART5 缓冲区长度与重装计数不一致、CAN 发送返回值处理等未在本次接收精简范围内修改，仍需单独核对。
