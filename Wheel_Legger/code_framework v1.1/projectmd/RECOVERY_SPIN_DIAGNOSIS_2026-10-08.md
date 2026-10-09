# 2026-10-08 起身后打转排查

对照基线：`47646502fae64d8f781c61e6929e5616c92a791a`（倒地自启大风车）。排查开始时工作区为 `97427537a8cd28c650a675348588adbfb620fcdf`，无未提交改动。用户确认旧版在同一台车、相同接线下起身后能平衡；当前现象为起身后打转，观察对象为左右轮电机，视角为车身同一侧。

## 已复现的回归及修复

旧版 `Chassis_CanTransimit()` 先将两轮 `wheel_T` 限制在 ±4.8 N·m，再换算、限制并发送电流。排查前版本将电流换算与 CAN 发送移到了力矩限幅前面。

`DJI_Motor_Data_Typedef.SET_Current` 是 `int16_t`。轮力矩约超过 32768 / 3330 ≈ 9.84 N·m 后，换算值越过其表示范围；该赋值发生在电流限幅之前。主机测试及 ArmClang 6.16 生成的代码均显示先整数转换、再截为 16 位，随后才按截断值限幅。此时可能已经翻转符号。C 语言本身不保证超范围浮点转整数的可移植行为，不能依赖事后限幅补救。

使用同一份当前 LQR、真实电流字段类型及真实 DJI CAN 打包，仅替换底盘发送函数进行对照：

| 输入或场景 | 修复前实际电流（左、右） | 旧版发送函数 / 修复后 |
| --- | --- | --- |
| wheel_T = (+1, +1) N·m | (-3330, +3330) | (-3330, +3330) |
| wheel_T = (+10, +8) N·m | (+16384, +16384) | (-15984, +15984) |
| 当前 LQR 样例输出 (+10.8847, +8.6199) N·m | (+16384, +16384) | (-15984, +15984) |

LQR 样例是人工构造、符合当前交接角度和腿长条件的输入：两腿 phi0=1.7 rad、L0=0.139 m，pitch=0.19 rad、pitch 速率=1.5 rad/s、腿 theta 速率=0.5 rad/s，yaw=0.4 rad、目标 yaw=0、yaw 速率=0.2 rad/s；其余误差/速率为零。这不是实车采集数据。它证明当前控制链路能产生触发越界的输出，不能单独证明实车所有失衡都由这一处造成。

修复仅将原有轮力矩限幅移到电流赋值之前，恢复已验证旧版的顺序。输出幅度、左右轮方向约定、CAN ID、自起轨迹、LQR/LESO 参数均未调整。LESO 回灌继续使用限幅后的轮力矩。

## 旧版起身方式与其他差异

旧版扫腿设腿长 0.39 m，每拍将目标 phi0 设为实际值减 0.3 rad；按各腿 theta 和机身 pitch 判断停止扫腿，两腿条件满足后进入收腿。收腿目标为 0.15 m，两腿 L0 小于 0.20 m 后继续调整角度；最终要求两腿 theta 距 0.5 rad 小于 0.1 rad、|pitch| 小于 0.2 rad、L0 小于 0.20 m，再交给 NORMAL 的 LQR。这里 theta=Pitch+phi0−π/2，不能将旧版 theta=0.5 直接当作 phi0=0.5。

当前使用固定 phi0 目标 2.8 → 收腿 0.139 m → 两腿同步转到 phi0=1.7 → 条件满足后 NORMAL。完整阶段见 [自起模块交接](LEG_MOTION_HANDOFF.md)。

左右轮数组下标、CAN 收发编号、反馈速度符号和电流换算符号与旧版相同；CAN 接收文件逐字节相同，DJI 打包函数仅延时处理不同。

同时存在以下尚未通过实车数据归因的差异，本次未回退：

- LQR 的 40×6 拟合表已更换，左腿和机身目标偏置由 -0.06 / -0.04 rad 改为 0。
- 正常腿长 Kp/Kd 从 400/20 调到 1600/300，roll Kd 从 40 调到 80。
- LESO 的矩阵及接入方式变化：原先需连续满足速度条件，当前 NORMAL 计时约 0.5 s 后逐渐接入。
- 控制周期改用绝对定时，发送函数的零延时调用不再执行 osDelay(0)。
- 旧版收腿函数每拍写入正常腿长目标 0.15 m；当前动作模块使用独立命令，正常腿长目标由 NORMAL 下 30 ms 遥控任务更新，交接初期可能沿用旧值。航向保持目标在两版中都未专门于自起完成时重置。

## 验证与文件

- 新增 [wheel_output_verify.js](../mdk_check/wheel_output_verify.js) / [wheel_output_verify.c](../mdk_check/wheel_output_verify.c)：编译实际 LQR、底盘发送函数和 DJI 打包函数，使用实际 DJI 电流类型，硬件调用使用捕获桩。
- 同一轮输出测试：修复前 117 项检查、20 项失败；仅换旧版发送函数后 117 项全部通过；修复后 117 项全部通过。覆盖正负输出、轮差速、±4.8 限幅边界、int16_t 越界邻域、禁用清零、CAN 顺序、LESO 回灌及真实 LQR 样例。
- `node mdk_check/leg_motion_verify.js`：739 + 1047 + 230 = 2016 项断言通过。
- VMC / Jacobian 两个有限差分脚本通过，仍只验证公式。
- 当前 0.39 m / 1000 ms 收腿 / 500 ms 交接参数及此次修复，Keil ArmClang 6.16 隔离完整重编：0 Error(s)、0 Warning(s)，退出码 0。
- GBK 源码严格解码通过、U+FFFD 字节数未增加、CRLF 保持；权威 MDK-ARM 目录原有 669 个文件哈希全部保持。
- 未烧录，未验证真实 CAN、RTOS 运行周期和起身后的机械平衡。

复现轮输出验证：`node mdk_check/wheel_output_verify.js`。可选第二参数为旧版 chassis_task.c 路径，用来单独对照发送函数；其他模块仍取当前工作区。主机编译器默认 gcc，可用 CC 指定。沙箱内 gcc 启动失败时，本机沙箱外验证已完成。

隔离产物：[HEX](../build/recovery_spin_keil/Objects/CtrlBoard-H7_IMU.hex)、[AXF](../build/recovery_spin_keil/Objects/CtrlBoard-H7_IMU.axf)、[构建日志](../build/recovery_spin_keil/build.log)。对照日志在 `build/recovery_spin_diagnosis/`：`current_before.log`、`baseline_transfer.log`、`current_fixed.log`、`recovery_regression.log`；`narrowing_probe.s` 是目标编译器的转换指令证据。这些本机 build 文件不会随 Git 提交交付。

## 实车续接

用本次隔离产物验证起身与直接站立两种进入 NORMAL 的方式。记录进入 NORMAL 前后两轮 wheel_T、SET_Current、yaw/yaw_set/d_yaw、pitch/角速度、set_L0/L0、leso_dbg_comp 与任务周期。当前发送后 wheel_T 已被限幅，若要看原始 LQR 大力矩，应在限幅前断点或采样。

通过条件：轮电流符合左轮负、右轮正的既定换算，幅度不超过 15984；起身后保持平衡且无持续非指令转向。若仍打转，应根据上述实测区分航向误差、LESO 接入、腿长目标切换和机械反馈问题；已有主机通过结果不能替代这一步。
