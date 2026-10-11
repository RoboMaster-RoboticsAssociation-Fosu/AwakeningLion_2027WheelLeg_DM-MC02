# 自起开启延迟记录（2026-10-10）

启动事件记录已加入原工程，直接使用 MDK-ARM/CtrlBoard-H7_IMU.uvprojx 构建、下载和调试；主工程 AXF 位于 MDK-ARM/CtrlBoard-H7_IMU/CtrlBoard-H7_IMU.axf。无需切换到隔离诊断工程。记录只写 RAM，未加入串口输出、阻塞等待或控制参数修改。

## 实车操作

让原工程连续运行，完成一次起身后 OFFLINE，将两腿摆回最长腿长及 phi0=2.81，再 ONLINE。动作结束后读取 Watch 中以下变量即可；记录会锁存至下一次接收端 OFF→ON。

| 变量 | 含义 |
| --- | --- |
| recovery_dbg_first_call_delay_ms | 从接收中断解析到 ONLINE，到第一次调用自起的毫秒数。 |
| recovery_dbg_reference_delay_ms | 从解析到 ONLINE，到两腿收腿参考均减小超过 2 mm 的毫秒数；适用于本次最长腿长场景。 |
| recovery_dbg_first_phi0[0]/[1] | 第一次自起调用实际收到的左右原始 phi0。 |
| recovery_dbg_first_result | 第一次调用返回结果，1=运行，4=故障。 |
| recovery_dbg_first_phase[0]/[1] | 第一次调用后的阶段，1/1 表示当拍已共同启动收腿。 |
| recovery_dbg_abort_count / abort_tick | 本次开启之后底盘自起中止次数及最近中止时刻；普通上下文复位不抹掉该记录。 |

计时/结果/阶段值为 UINT32_MAX（4294967295）表示对应事件尚未发生，不将初始化零值误认为零延迟。

进一步分解入口时序：recovery_dbg_rx_on_tick 为接收中断解析到开启的时刻，recovery_dbg_task_on_tick 为遥控任务响应时刻，recovery_dbg_enable_tick 为底盘识别 OFF→ON 时刻，recovery_dbg_first_call_tick 为首次调用自起时刻。recovery_dbg_rx_on_seq 为接收端 OFF→ON 计数；一次物理开启增加多次，说明接收状态曾反复切换。

## 验证及边界

- 从原工程提取的实际底盘控制代码回归通过，包含 8 次完整起身→OFFLINE摆位→ONLINE循环、首次角度/计时捕获、收腿参考变化、动作结束后锁存及中止记录。
- 主机验证：单腿 1247、自起模块 1896、底盘接入 1045，扫腿配置/轨迹 2394，共 6582 项断言通过；轮输出 117 项、正常腿长斜坡 48663 项检查通过。
- 两处旧验证在修改前也失败：理想轨迹的角度浮点误差被当前 Kp 放大，以及正常腿长斜坡预期写死 0.1 m/s。仅调整验证以使用浮点误差预算与当前速率宏，保留用户当前控制参数。
- 原 Keil 工程完整重编：0 Error(s)、0 Warning(s)。构建日志在 [recovery_trace_build.log](../MDK-ARM/recovery_trace_build.log)，主机日志在 build/recovery_main_trace_check/。
- 未烧录或独立完成实车复验；2 秒延迟尚未定位。软件完整周期回放未复现该延迟，后续依据实车锁存记录定位。

调参仍以当前工作区头文件为准；本轮只加入事件记录，不将 PID 或斜坡时间当作已确认的延迟原因。
