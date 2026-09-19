# 交接文档 — code_framework v1.1

写给下一个会话（也是给人看的）。最后更新：2026-09-19。

---

## 0. 第一铁律：不要用 Edit / Write 工具改这里的源文件

工程源文件是 **GB2312（代码页 936）**，行尾 **CRLF**。

**2026-09-19 实测证实：Edit 工具会把 GB2312 文件整文件重存为 UTF-8**，无法按 UTF-8 解码的字节全部变成 U+FFFD —— 一次编辑毁掉该文件**全部**中文注释，不是只动你编辑的那几行。（此前"用 Edit 做纯 ASCII 修改是字节安全的"这个结论**已被推翻**，别再照它行事。）

改这些文件的正确姿势：

- 用 Python 按字节读写：`open(p, 'rb')` / `open(p, 'wb')`，按 `b"\r\n"` 切行，插入的文本保持纯 ASCII。
- 改完自检：`file <file>`。报 **ISO-8859 = 健康**；报 UTF-8 / Unicode text = **已被毁**。
- 不要让编辑器对"被外部改动过"的文件直接 Ctrl+S（会按当前编码重存，等于再毁一次）。
- 用户的 VS Code 默认编码是 `gb2312`，所以**本目录下的 .md 用 UTF-8 with BOM 保存**——
  VS Code 靠 BOM 覆盖默认编码，否则打开是乱码。工程源码和文档是两套编码，别混。

## 1. 注释被毁后的恢复配方（09-19 验证：26/26 命中，字节精确）

损坏是**确定性变换**：`line_bytes.decode('utf-8', errors='replace')`。据此反查即可：

1. 找干净旧副本：工程里的 `<file>.fffd.bak`（**别删**），或 VS Code 历史
   `%APPDATA%\Code\User\History\<hash>\*.c`（内容是纯文件字节、无头部）。
2. 对干净副本的每一行：若有非 ASCII 字节，算出它的 mojibake 形式；若该形态在损坏文件里出现，
   就用**干净副本的原始字节**替换那一行。
3. 必做验证：残留 `EF BF BD` 计数 = 0；严格 GB2312/GB18030 解码通过；行数一致；
   逐行确认**只有这些行的字节变了、且每行 ASCII 骨架（< 0x80 的序列）未变**；
   最终字节数 = 改前大小 + 本次插入量。

## 2. 工程编码现状

- `User/APP/chassis_task.c` **健康**（20925 B，09-19 恢复后 U+FFFD = 0）。
- 仍带损坏（历史遗留、已提交进 git、本地无干净副本、**无法自动恢复，只能重写**）：
  `LESO.c` 231、`VMC_calc.h` 54、`main.c` 41、`user_sys_config.h` 37（单位 = U+FFFD 字节数）。
- 工程里 3 个 `*.fffd.bak`（chassis_task.c / chassis_task.h / Remote_task.c，09-16）是干净旧版备份，
  **是恢复用的源，别删**。

## 3. 倒地检测 / ZERO_FORCE 状态机（09-19 改，**尚未上车实测**）

```
NORMAL --(检测到倒地)--> ZERO_FORCE --(遥控 OFF→ON 重新上电)--> FALLING_DOWN
       --> (自动) FALLING_TO_NORMAL --> (自动) NORMAL
```

- 检测在 `User/APP/chassis_task.c` 的 `falling_down_detect()`：
  OFF→ON 边沿做一次位姿判断（355-369 行），在线且 NORMAL 时连续判断（377-382 行）。
- 本次三处改动：
  1. **129 行** `case ZERO_FORCE` 补 `break`：原来缺 break 会掉进 `falling_down()`，
     一进零力态就同时自启。该分支此前从未被执行过（全工程没有一处给 `chassis_mode` 赋过
     `ZERO_FORCE`），所以一直没暴露。
  2. **436-437 行** `zero_force()` 补 `wheel_T = 0`：原来只清 F0/Tp，而 ZERO_FORCE 期间
     `chassis_enable` 仍是 ONLINE，`Chassis_CanTransimit()` 走 ONLINE 分支直接拿 `wheel_T`
     发电流，轮子会保持 LQR 最后一次算出的力矩继续推（躺地上时轮子正好贴地，必须清）。
  3. **381 行** 连续检测的 `FALLING_DOWN` → `ZERO_FORCE`。
- 边沿块（355-369 行）**保留了位姿复检**：躺地上 → phi0 出 [0.4, 2.5] 或 |body theta| > 0.3
  → FALLING_DOWN；抱着车拨开关 → NORMAL（不会在空中甩腿）。这是**刻意的取舍**，
  不要为了"拨开关一定自启"把这段判据短路掉。
- `chassis_mode` 枚举在 `chassis_task.h` 126-131 行。LESO（`LESO.c:431`）和 Remote
  （`Remote_task.c:47`）都按 `== NORMAL` 判断，ZERO_FORCE 期间自动不掺和；回到 NORMAL 时
  LESO 会在 online 边沿重新 seed。

### 待实测 / 已知风险（上车前先看这几条）

- **触发瞬间车可能还站着**：连续检测是瞬态超带就触发，此时腿一软是直接砸下去，
  不是受控自启。这是本次改动相对旧行为最大的差别。
- **FALLING_DOWN 的入口姿态变了**：现在是"倒 + 软腿瘫了 N 秒才进"，而 462-463 行的
  `phi0 - 0.3` 和 467/471 行的 theta ∈ (-0.5, 1.3) 窗口是按旧入口（刚倒就进）调的，需重新实测。
- **位姿判据只看 phi0 + body theta，没看 roll**：侧躺可能漏判成 NORMAL。
- 若实测发现躺着的姿态**稳定地**被判成 NORMAL（每次都这样，不是偶发），
  要修的是**判据本身**（收窄 [0.4, 2.5] 或加 |roll| 条件），而不是把判据整个短路掉。

## 4. 其它

- 编译走 Keil MDK / EIDE（`MDK-ARM/`）。改完源码必须重新编译，`chassis_task.o` 等要重出。
- 本文件是 UTF-8 with BOM；工程源码是 GB2312 —— 两回事，别混。
