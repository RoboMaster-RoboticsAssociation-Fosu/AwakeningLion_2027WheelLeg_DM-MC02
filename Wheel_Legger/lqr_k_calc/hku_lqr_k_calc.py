#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
================================================================================
 轮腿机器人 LQR 增益 K 求解
================================================================================
 基于 HerKules_VOCAL_SJ_LQR_v4_with_data.m (HKU 开源) 的 Python 复刻,
 参数已全部本地化为自有机器人（与 ABK_LQR.py 一致）。
 模型: 上交开源的线性化方程组 (3.11)-(3.15)。

    x = [s, ds, phi, dphi, theta_ll, dtheta_ll, theta_lr, dtheta_lr, theta_b, dtheta_b]
    u = [T_wl, T_wr, T_bl, T_br]      控制律 u = -K·x

 流程:
   1) 符号转写 5 条线性方程 -> 按系数提取 M/G/B_raw
      -> J_A = -M^-1·G, J_B = -M^-1·B_raw (等价 MATLAB solve+jacobian,
         --verify-symbolic 可用原版做法交叉验证)
      -> 填装 A(10x10), B(10x4)
   2) 离散 LQR: 按 Ts 做 ZOH 离散化, P 解离散 Riccati 方程,
      K = solve(R + Bd.T@P@Bd, Bd.T@P@Ad) (同 ABK_LQR.py 的 c2d + dlqr)。
   3) 变腿长扫频 -> 逐元素 poly22 最小二乘拟合:
         p(l_l, l_r) = p00 + p10·l_l + p01·l_r + p20·l_l? + p11·l_l·l_r + p02·l_r?
      输出 40x6 系数表: 第 r 行 = K 展平后第 r 个元素(行主序),
      6 列依次 [p00, p10, p01, p20, p11, p02] (同 MATLAB coeffvalues)。
   4) LESO 扩张状态观测器(同 ABK_LQR.py): 对 ZOH 离散模型(Ts=1ms)把 4 个输入
      通道的总扰动扩张进状态, 极点配置求 L(14x10), 同样做 poly22 拟合(140x6)。
      X = [x(10); f(4)], x(k+1) = Ad·x + Bd·(u+f); 控制补偿 u = -K·x_hat - f_hat。
      Ad/Bd(ZOH 离散模型)一并导出并拟合 —— 固件用它们现场拼装
      A_e = [[Ad, Bd], [0, I]] (14x14),  B_e = [Bd; 0] (14x4),  C_e = [I, 0] (10x14)。

 机械数据: 惯性用公式估算(圆盘/盒近似), 腿部参数用经验公式或实测表插值。
 这是拿不到 CAD/实测数据时的临时方案, 数据到位后请替换(开关见文件顶部)。

 运行示例:
   python hku_lqr_k_calc.py                          # 全流程(定腿长 K + 拟合系数)
   python hku_lqr_k_calc.py --mode fixed             # 只算定腿长 K
   python hku_lqr_k_calc.py --mode fit               # 只算拟合系数
   python hku_lqr_k_calc.py --leg-left 0.15 --leg-right 0.18
   python hku_lqr_k_calc.py --leg-table              # 腿部参数用实测表(而非公式)
   python hku_lqr_k_calc.py --no-formula-inertia     # 惯性用声明值(而非公式)
   python hku_lqr_k_calc.py --verify-symbolic        # 符号法交叉验证 J_A/J_B
   python hku_lqr_k_calc.py --params R_w=0.06,m_b=12 # 覆盖物理参数
   (生成哪些数据由文件顶部的 USE_K / USE_L / USE_AD_BD 开关决定)

 输出(当前目录, --out-dir 可改, --no-out 关闭), 均为可直接粘贴的 C 数组定义:
   K_fixed.txt              float K[4][10]        定腿长 LQR 增益
   L_fixed.txt              float L[14][10]       定腿长 LESO 观测器增益
   Ad_fixed.txt             float Ad[10][10]      定腿长 ZOH 离散模型(拼 A_e/B_e 用)
   Bd_fixed.txt             float Bd[10][4]
   K_Fit_Coefficients.txt   float P[40][6]        K 的 poly22 拟合系数
   L_Fit_Coefficients.txt   float L_Fit[140][6]   L 的 poly22 拟合系数
   Ad_Fit_Coefficients.txt  float Ad_Fit[100][6]  Ad 的 poly22 拟合系数
   Bd_Fit_Coefficients.txt  float Bd_Fit[40][6]   Bd 的 poly22 拟合系数
   (上面哪些真正生成, 由文件顶部的 USE_K / USE_L / USE_AD_BD 开关决定)

 依赖: numpy, scipy, sympy
================================================================================
"""

import argparse
import os
import sys
import time

import numpy as np
import sympy
from scipy import linalg as scipy_linalg
from scipy.signal import cont2discrete, place_poles

# Windows 控制台重定向时避免中文打印崩溃(不影响交互式显示)
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(errors="replace")
    except Exception:
        pass


# =============================================================================
# ★★★ 用户开关（常用项都放这里, 改完直接运行）★★★
# =============================================================================
# ---- 输出内容: 1 = 生成该组数据, 0 = 跳过(连计算也跳过, 拟合提速) ----
USE_K = 1      # LQR 增益 K:   K_fixed.txt + K_Fit_Coefficients.txt
USE_L = 0      # LESO 增益 L:  L_fixed.txt + L_Fit_Coefficients.txt
USE_AD_BD = 0  # ZOH 离散模型: Ad/Bd_fixed.txt + Ad/Bd_Fit_Coefficients.txt (拼 A_e/B_e 用)
# ---- 机械数据来源: 机械给不出 CAD/实测数据时用公式临时估算, 到位后改 0 换实测 ----
USE_FORMULA_INERTIA = 1   # 1: I_w/I_b/I_z 公式估算(覆盖声明值); 0: 用 PARAMS 声明值
USE_LEG_FORMULA = 1       # 1: 腿部 lw/lb/Il 用经验公式; 0: 用 LEG_DATA 实测表插值


# =============================================================================
# Step 0：定义符号
# =============================================================================
R_w, R_l = sympy.symbols('R_w R_l', positive=True)      # 驱动轮半径 / 轮距的一半
l_l, l_r = sympy.symbols('l_l l_r', positive=True)      # 左右腿长
l_wl, l_wr = sympy.symbols('l_wl l_wr', positive=True)  # 驱动轮质心到腿部质心距离
l_bl, l_br = sympy.symbols('l_bl l_br', positive=True)  # 机体质心到腿部质心距离
l_c = sympy.symbols('l_c', positive=True)               # 机体质心到腿部关节中心距离
m_w, m_l, m_b = sympy.symbols('m_w m_l m_b', positive=True)  # 轮/腿/机体质量
I_w = sympy.symbols('I_w', positive=True)               # 驱动轮转动惯量
I_ll, I_lr = sympy.symbols('I_ll I_lr', positive=True)  # 左右腿部转动惯量
I_b = sympy.symbols('I_b', positive=True)               # 机体转动惯量(俯仰)
I_z = sympy.symbols('I_z', positive=True)               # z 轴转动惯量
g = sympy.symbols('g', positive=True)                   # 重力加速度

ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b = sympy.symbols(
    'ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b')
theta_ll, theta_lr, theta_b = sympy.symbols('theta_ll theta_lr theta_b')
T_wl, T_wr, T_bl, T_br = sympy.symbols('T_wl T_wr T_bl T_br')

XS      = [ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b]
THETAS  = [theta_ll, theta_lr, theta_b]
TORQUES = [T_wl, T_wr, T_bl, T_br]

# 参数代入顺序（lambdify / 数值求值共用）
PARAM_ORDER = [R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c,
               m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g]

STATE_NAMES = ['s', 'ds', 'phi', 'dphi', 'theta_ll', 'dtheta_ll',
               'theta_lr', 'dtheta_lr', 'theta_b', 'dtheta_b']
INPUT_NAMES = ['T_wl', 'T_wr', 'T_bl', 'T_br']


# =============================================================================
# Step 1：转写方程组 (3.11)-(3.15) -> J_A/J_B -> A/B
# =============================================================================
def build_equations():
    """5 条方程(均为 f(...)==0 的左端), 依次为:
    左腿 / 右腿摆杆动力学, 两轮"并联前进"动力学, 机体俯仰动力学, yaw 偏航动力学。
    每条方程对各变量均为线性 —— 这是线性化的前提。"""
    eqn1 = (I_w * l_l / R_w + m_w * R_w * l_l + m_l * R_w * l_bl) * ddtheta_wl \
         + (m_l * l_wl * l_bl - I_ll) * ddtheta_ll \
         + (m_l * l_wl + m_b * l_l / 2) * g * theta_ll \
         + T_bl - T_wl * (1 + l_l / R_w)

    eqn2 = (I_w * l_r / R_w + m_w * R_w * l_r + m_l * R_w * l_br) * ddtheta_wr \
         + (m_l * l_wr * l_br - I_lr) * ddtheta_lr \
         + (m_l * l_wr + m_b * l_r / 2) * g * theta_lr \
         + T_br - T_wr * (1 + l_r / R_w)

    c3 = m_w * R_w**2 + I_w + m_l * R_w**2 + m_b * R_w**2 / 2
    eqn3 = -c3 * ddtheta_wl - c3 * ddtheta_wr \
         - (m_l * R_w * l_wl + m_b * R_w * l_l / 2) * ddtheta_ll \
         - (m_l * R_w * l_wr + m_b * R_w * l_r / 2) * ddtheta_lr \
         + T_wl + T_wr

    c4 = m_w * R_w * l_c + I_w * l_c / R_w + m_l * R_w * l_c
    eqn4 = c4 * ddtheta_wl + c4 * ddtheta_wr \
         + m_l * l_wl * l_c * ddtheta_ll + m_l * l_wr * l_c * ddtheta_lr \
         - I_b * ddtheta_b + m_b * g * l_c * theta_b \
         - (T_wl + T_wr) * l_c / R_w - (T_bl + T_br)

    c5 = I_z * R_w / (2 * R_l) + I_w * R_l / R_w
    eqn5 = c5 * ddtheta_wl - c5 * ddtheta_wr \
         + I_z * l_l / (2 * R_l) * ddtheta_ll - I_z * l_r / (2 * R_l) * ddtheta_lr \
         - T_wl * R_l / R_w + T_wr * R_l / R_w

    return [eqn1, eqn2, eqn3, eqn4, eqn5]


def extract_linear_form(eqs):
    """把 5 条线性方程整理成 M·ddq + G·Θ + B_raw·T == 0:
        M     : 5x5 广义质量矩阵 (对 ddtheta_* 的系数)
        G     : 5x3 重力/腿角项   (对 theta_ll/lr/b 的系数, 含 g)
        B_raw : 5x4 控制项       (对 T_wl/wr/bl/br 的系数)
    由 ddq = -M^-1·(G·Θ + B_raw·T) 得 J_A = -M^-1·G, J_B = -M^-1·B_raw。
    自检: 残差展开必须恒为 0, 否则系数提取不完整(转写有误)。"""
    eqs_exp = [sympy.expand(e) for e in eqs]
    M     = sympy.Matrix(5, 5, lambda i, j: eqs_exp[i].coeff(XS[j]))
    G     = sympy.Matrix(5, 3, lambda i, j: eqs_exp[i].coeff(THETAS[j]))
    B_raw = sympy.Matrix(5, 4, lambda i, j: eqs_exp[i].coeff(TORQUES[j]))

    for i, eq in enumerate(eqs_exp):
        resid = eq - (M[i, :] * sympy.Matrix(XS))[0] \
                   - (G[i, :] * sympy.Matrix(THETAS))[0] \
                   - (B_raw[i, :] * sympy.Matrix(TORQUES))[0]
        assert sympy.expand(resid) == 0, f'eqn{i + 1} 系数提取不完整, 请检查方程转写!'
    return M, G, B_raw


def make_jacobian_functions(M, G, B_raw):
    """lambdify: 符号矩阵 -> 18 个物理参数 -> 数值矩阵 的函数。"""
    return (sympy.lambdify(PARAM_ORDER, M, 'numpy'),
            sympy.lambdify(PARAM_ORDER, G, 'numpy'),
            sympy.lambdify(PARAM_ORDER, B_raw, 'numpy'))


def jacobians_numeric(params, M_fun, G_fun, Br_fun):
    """数值求 J_A(5x3)、J_B(5x4):  J_A = -M^-1·G,  J_B = -M^-1·B_raw。"""
    pvals = [params[sym.name] for sym in PARAM_ORDER]
    M_n  = np.asarray(M_fun(*pvals), dtype=float)
    G_n  = np.asarray(G_fun(*pvals), dtype=float)
    Br_n = np.asarray(Br_fun(*pvals), dtype=float)
    return -np.linalg.solve(M_n, G_n), -np.linalg.solve(M_n, Br_n)


def verify_symbolic_path(eqs, params, M_fun, G_fun, Br_fun):
    """--verify-symbolic: 复刻 MATLAB 原版做法(solve + jacobian)做交叉验证,
    在两组不同参数下与主路径(-M^-1·G / -M^-1·B_raw)数值比对, 误差应 < 1e-9。"""
    leg_lo, leg_hi = LEG_DATA[0], LEG_DATA[-1]
    cases = [
        ('当前参数组', params),
        ('非对称腿长组',
         {**params, 'l_l': leg_lo[0], 'l_wl': leg_lo[1], 'l_bl': leg_lo[2], 'I_ll': leg_lo[3],
                      'l_r': leg_hi[0], 'l_wr': leg_hi[1], 'l_br': leg_hi[2], 'I_lr': leg_hi[3]}),
    ]
    for tag, p in cases:
        subs = {sym: p[sym.name] for sym in PARAM_ORDER}
        sol = sympy.solve([e.subs(subs) for e in eqs], XS, dict=True)[0]
        dd = [sol[x] for x in XS]
        JA_sym = sympy.Matrix(dd).jacobian(sympy.Matrix(THETAS))
        JB_sym = sympy.Matrix(dd).jacobian(sympy.Matrix(TORQUES))
        JA_num, JB_num = jacobians_numeric(p, M_fun, G_fun, Br_fun)
        dA = float(np.max(np.abs(np.array(JA_sym, dtype=float) - JA_num)))
        dB = float(np.max(np.abs(np.array(JB_sym, dtype=float) - JB_num)))
        print(f'  [{tag}]  max|J_A(符号)-J_A(数值)| = {dA:.3e},  '
              f'max|J_B(符号)-J_B(数值)| = {dB:.3e}')
        assert dA < 1e-9 and dB < 1e-9, '符号路径与数值路径结果不一致!'
    print('  [通过] 两条推导路径结果一致。\n')


def fill_AB(J_A, J_B, R_w_, R_l_, l_l_, l_r_):
    """由 J_A(5x3)/J_B(5x4) 填装 A(10x10)/B(10x4)。

    奇数行(1,3,5,7,9)是运动学: A[r, r+1]=1, 行内其余为 0;
    偶数行(2,4,6,8,10)是动力学(s?/φ?/θ?), 由 J_A/J_B 组合:
        行2 (s?)  = R_w·(ddθwl + ddθwr)/2
        行4 (φ?)  = R_w·(-ddθwl + ddθwr)/(2R_l) - l_l·ddθll/(2R_l) + l_r·ddθlr/(2R_l)
        行6/8/10 = ddθll / ddθlr / ddθb (直接取 J_A 第 3/4/5 行)
    只填第 5/7/9 列(对应 theta_ll/theta_lr/theta_b), 其余列(含第 6/8 列,
    即 φ? 中的 dθll/dθlr 项)按原模型置 0 —— 原程序的建模选择, 原样保留。"""
    A = np.zeros((10, 10))
    B = np.zeros((10, 4))
    for idx, p0 in enumerate((4, 6, 8)):          # 0-based 列 4/6/8 <- theta_ll/lr/b
        A[1, p0] = R_w_ * (J_A[0, idx] + J_A[1, idx]) / 2.0
        A[3, p0] = (R_w_ * (-J_A[0, idx] + J_A[1, idx]) / (2.0 * R_l_)
                    - l_l_ * J_A[2, idx] / (2.0 * R_l_)
                    + l_r_ * J_A[3, idx] / (2.0 * R_l_))
        A[5, p0] = J_A[2, idx]
        A[7, p0] = J_A[3, idx]
        A[9, p0] = J_A[4, idx]
    for h in range(4):                            # B: T_wl/T_wr/T_bl/T_br 四列
        B[1, h] = R_w_ * (J_B[0, h] + J_B[1, h]) / 2.0
        B[3, h] = (R_w_ * (-J_B[0, h] + J_B[1, h]) / (2.0 * R_l_)
                   - l_l_ * J_B[2, h] / (2.0 * R_l_)
                   + l_r_ * J_B[3, h] / (2.0 * R_l_))
        B[5, h] = J_B[2, h]
        B[7, h] = J_B[3, h]
        B[9, h] = J_B[4, h]
    for r0 in (0, 2, 4, 6, 8):                    # 奇数行运动学
        A[r0, r0 + 1] = 1.0
    return A, B


def solve_lqr(A, B, Q, R):
    """连续 A/B 按 Ts 做 ZOH, 再求离散 LQR。
    返回 K, DARE 最大绝对残差(应接近0), 闭环谱半径(应<1)。"""
    Ad, Bd = c2d(A, B, Ts)
    P = scipy_linalg.solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    dare_resid = float(np.max(np.abs(Ad.T @ P @ Ad - P - Ad.T @ P @ Bd @ K + Q)))
    spectral_radius = float(np.max(np.abs(np.linalg.eigvals(Ad - Bd @ K))))
    return K, dare_resid, spectral_radius


def compute_K(params, M_fun, G_fun, Br_fun, Q, R):
    """物理参数 -> 连续 A/B -> ZOH -> 离散 LQR 的 K。
    返回原始连续 A/B, 供 LESO 分支按同一 Ts 离散化一次。"""
    J_A, J_B = jacobians_numeric(params, M_fun, G_fun, Br_fun)
    A, B = fill_AB(J_A, J_B, params['R_w'], params['R_l'], params['l_l'], params['l_r'])
    K, dare_resid, spectral_radius = solve_lqr(A, B, Q, R)
    return K, A, B, dare_resid, spectral_radius


# =============================================================================
# Step 2：机器人参数（可修改；机械数据到位后替换公式/表格）
# -----------------------------------------------------------------------------
# 注: 常用开关(USE_K / USE_L / USE_AD_BD / USE_FORMULA_INERTIA / USE_LEG_FORMULA)
#     已集中放到文件最顶部"★ 用户开关"区, 方便查找。
# =============================================================================
BODY_L, BODY_H, BODY_W = 0.415, 0.16, 0.260   # 机体包络 长/高/宽 (m), 盒近似用

PARAMS = {
    'g':   9.81,         # 重力加速度 (唯一不可改变!)

    # ---- 机体与轮部参数 ----
    'R_w': 0.058,        # 驱动轮半径 (m)
    'R_l': 0.22,         # 两驱动轮间距/2 (m)
    'l_c': 0.120,        # 机体质心到腿部关节中心点距离 (m)
    'm_w': 0.537,        # 驱动轮质量 (kg)
    'm_l': 1.65,         # 腿部质量 (kg)
    'm_b': 16.0,         # 机体质量 (kg)

    # ---- 转动惯量声明值 (USE_FORMULA_INERTIA=0 时才生效) ----
    'I_w': 0.000516,     # 驱动轮转动惯量 (kg·m?)
    'I_b': 0.025,        # 机体转动惯量(俯仰) (kg·m?)
    'I_z': 0.380,        # z 轴转动惯量 (kg·m?)

    # ---- 定腿长模式默认腿长 (腿部 lw/lb/Il 运行时按来源自动重算, 下方为缺省) ----
    'l_l': 0.20, 'l_wl': 0.0814, 'l_bl': 0.1186, 'I_ll': 0.15,
    'l_r': 0.20, 'l_wr': 0.0814, 'l_br': 0.1186, 'I_lr': 0.15,
}

# ---- 腿长-腿部参数实测表: [腿长 l (m), lw (m), lb (m), Il (kg·m?)] ----
# USE_LEG_FORMULA=0 时对本表插值(表内插值, 表外线性外推)。
# 注意: 目前来自 ABK_LQR.py, 若不是你的实测数据, 拿到测量值后替换。
LEG_DATA = np.array([
    [0.10310, 0.04606, 0.05425, 0.00639871032],
    [0.14595, 0.05886, 0.08710, 0.00801157950],
    [0.16050, 0.06239, 0.09377, 0.00848580360],
    [0.16968, 0.06857, 0.10111, 0.00908386627],
    [0.17981, 0.07141, 0.10840, 0.00956107964],
    [0.19027, 0.07411, 0.11616, 0.01011646609],
    [0.19966, 0.07719, 0.12247, 0.01066091718],
    [0.20993, 0.08097, 0.12914, 0.01125512924],
    [0.21993, 0.08457, 0.13536, 0.01187332029],
    [0.22943, 0.08797, 0.14146, 0.01247068741],
    [0.24317, 0.09280, 0.14783, 0.01320485307],
])

# ---- LQR 权重 (当前本车配置, 用于 Ts 对应的离散设计) ----
# Q 对角线依次对应 10 个状态, 物理含义(括号内为固件侧常用的对应量名):
#   s          机器人沿前进方向的水平位移 (m)      —— 由两轮转角平均折算 (foot_distance)
#   ds         前进速度, 即 s 的导数 (m/s)                        (foot_speed)
#   phi        yaw 偏航角 (rad) —— 机器人左右转向的角度           (yaw_angle)
#   dphi       偏航角速度, 即 phi 的导数 (rad/s)                  (yaw_gyro)
#   theta_ll   左腿摆杆与竖直方向的夹角 (rad)                     (leg_angle_L)
#   dtheta_ll  左腿摆角速度, 即 theta_ll 的导数 (rad/s)           (leg_gyro_L)
#   theta_lr   右腿摆杆与竖直方向的夹角 (rad)                     (leg_angle_R)
#   dtheta_lr  右腿摆角速度, 即 theta_lr 的导数 (rad/s)           (leg_gyro_R)
#   theta_b    机体俯仰角, 机体与水平面夹角 (rad) —— 平衡控制的核心状态 (pitch_angle)
#   dtheta_b   机体俯仰角速度, 即 theta_b 的导数 (rad/s)          (pitch_gyro)
# 权重的意义: Q 中某项越大 = 越"看重"该状态 -> 对应的 K 增益越大、该状态收敛越快,
# 代价是输出力矩更大、对该状态测量噪声更敏感。如上 theta_b=20000 最大(平衡最重要),
# dtheta_* 都只有 5(角速度噪声大, 权重压低防抖)。
#                         s          ds         phi        dphi    theta_ll   dtheta_ll    theta_lr   dtheta_lr     theta_b    dtheta_b
Q_LQR = np.diag([     150.0,       25.0,      200.0,       20.0,      400.0,        20.0,      400.0,        20.0,    20000.0,        1.0])
# R 对角线依次对应 4 个输入力矩 (N·m): T_wl(左驱动轮) T_wr(右驱动轮) T_bl(左髋关节) T_br(右髋关节)
# R 越大越"省力" -> 力矩输出越小、动作越保守。
#                   T_wl    T_wr    T_bl    T_br
R_LQR = np.diag([ 15.0,  15.0,   4.0,   4.0])

# ---- LQR / LESO 共用采样周期及观测器参数 ----
Ts = 0.001               # 控制周期 (s); K 和 L 均基于该周期的 ZOH 离散模型
LESO_STATE_POLE = 0.4    # 10 个原状态极点 (z 域模), 带宽 -ln(0.4)/Ts ≈ 146 Hz
LESO_DIST_POLE = 0.985   # 4 个扩张扰动极点, 带宽 ≈ 2.4 Hz (扰动通道要慢, 太快会放大量测噪声)


# =============================================================================
# 公式估算（机械给不出数据时的临时方案）
# =============================================================================
def formula_inertia(p):
    """公式估算转动惯量(覆盖 I_w/I_b/I_z, 返回副本):
        I_w = 0.5·m_w·R_w?      驱动轮 ≈ 均质圆盘(质量集中轮缘/充气胎时改 1.0 系数)
        I_b = m_b·(L?+H?)/12    机体 ≈ 均质盒, 绕俯仰轴
        I_z = m_b·(L?+W?)/12    机体 ≈ 均质盒, 绕偏航轴
    """
    p = dict(p)
    p['I_w'] = 0.5 * p['m_w'] * p['R_w']**2
    p['I_b'] = p['m_b'] * (BODY_L**2 + BODY_H**2) / 12.0
    p['I_z'] = p['m_b'] * (BODY_L**2 + BODY_W**2) / 12.0
    return p


def leg_formula(l):
    """腿部参数经验公式: lw = 0.782·l ? 0.075, lb = 0.218·l + 0.075, Il = 0.4·l + 0.07。
    lw + lb ≡ l(质心把腿长按 78.2%/21.8% 分)。
    注意: Il 这条与实测表相比偏大约 15 倍, 量纲存疑, 仅为临时估计。"""
    return 0.782 * l - 0.075, 0.218 * l + 0.075, 0.4 * l + 0.07


def interp_extrap(xp, fp, x):
    """表内线性插值, 表外沿端点线性外推(np.interp 表外会截断, 与此不同)。"""
    if x <= xp[0]:
        return fp[0] + (fp[1] - fp[0]) / (xp[1] - xp[0]) * (x - xp[0])
    if x >= xp[-1]:
        return fp[-1] + (fp[-1] - fp[-2]) / (xp[-1] - xp[-2]) * (x - xp[-1])
    return float(np.interp(x, xp, fp))


def leg_params_at(l, use_leg_formula=None):
    """给定腿长 -> (lw, lb, Il)，按 USE_LEG_FORMULA 走公式或实测表。"""
    if (USE_LEG_FORMULA if use_leg_formula is None else use_leg_formula):
        return leg_formula(l)
    t = LEG_DATA
    return (interp_extrap(t[:, 0], t[:, 1], l),
            interp_extrap(t[:, 0], t[:, 2], l),
            interp_extrap(t[:, 0], t[:, 3], l))


def leg_grid(use_leg_formula=None):
    """拟合用腿长采样表 [(l, lw, lb, Il), ...]:
    公式模式: 21 个腿长 0.10~0.30 步进 0.01(同 ABK_LQR.py 网格);
    实测表模式: LEG_DATA 的 11 行。"""
    if (USE_LEG_FORMULA if use_leg_formula is None else use_leg_formula):
        return [(l, *leg_formula(l)) for l in np.arange(0.10, 0.30 + 1e-9, 0.01)]
    return [(r[0], r[1], r[2], r[3]) for r in LEG_DATA]


# =============================================================================
# LESO 扩张状态观测器（同 ABK_LQR.py; L 只依赖离散模型与极点, 与 K 相互独立）
# =============================================================================
def c2d(A_ac, B_ac, Ts_):
    """ZOH 离散化, 对应 MATLAB c2d(A, B, Ts)。"""
    Ad, Bd, _, _, _ = cont2discrete(
        (A_ac, B_ac, np.eye(10), np.zeros((10, 4))), Ts_, method='zoh')
    return Ad, Bd


def leso_gain(Ad, Bd, state_pole, dist_pole):
    """LESO 增益 L(14x10), 对应 place(A_e', C_e', poles)。

    扩张状态 X = [x(10); f(4)]: x(k+1) = Ad·x + Bd·(u + f) —— f 为每个输入通道
    的总扰动(模型误差+外力), 与控制量经同一个 Bd 进入, 故可用控制量补偿:
        A_e = [[Ad, Bd], [0, I]] (14x14),  B_e = [Bd; 0] (14x4),  C_e = [I, 0] (10x14)
        X_hat(k+1) = A_e·X_hat + B_e·u + L·(y - C_e·X_hat)
    y = 全部 10 个状态可测(轮速+Kalman / IMU / VMC); 控制律 u = -K·x_hat - f_hat。
    L 前 10 行 = 原状态估计增益(s..dtheta_b), 后 4 行 = 扰动估计增益(f_wl..f_br);
    列 = 10 个量测量, 顺序同 y。"""
    A_e = np.block([[Ad, Bd],
                    [np.zeros((4, 10)), np.eye(4)]])
    C_e = np.hstack([np.eye(10), np.zeros((10, 4))])
    poles = np.concatenate([np.full(10, state_pole), np.full(4, dist_pole)])
    L = place_poles(A_e.T, C_e.T, poles, maxiter=300).gain_matrix.T
    return A_e, C_e, L


# =============================================================================
# Step 3：变腿长扫频 + poly22 拟合
# =============================================================================
def fit_all_coefficients(params, M_fun, G_fun, Br_fun, Q, R, grid,
                         want=('K', 'L', 'Ad', 'Bd')):
    """遍历 grid 所有左右腿组合(n² 组)逐组求 K、LESO 增益 L 与 ZOH 离散模型 Ad/Bd,
    再逐元素 poly22 拟合。
    want: 要生成哪些数据('K'/'L'/'Ad'/'Bd' 的子集), 未选中的跳过计算与拟合。
    槽位(均行主序展平): K(l,m)->(l-1)*10+m; L(i,j)->i*10+j; Ad(i,j)->i*10+j; Bd(i,j)->i*4+j。
    返回 dict {选中项: (coeff, 残差)}, coeff 为 40/140/100/40 行 x 6 列,
    列序 [p00,p10,p01,p20,p11,p02] (同 MATLAB coeffvalues)。"""
    want = set(want)
    n = len(grid)
    K_samples = np.zeros((n * n, 3, 40)) if 'K' in want else None
    L_samples = np.zeros((n * n, 3, 140)) if 'L' in want else None
    Ad_samples = np.zeros((n * n, 3, 100)) if 'Ad' in want else None
    Bd_samples = np.zeros((n * n, 3, 40)) if 'Bd' in want else None
    coords = np.zeros((n * n, 2))             # 每个样本的 (l_l, l_r), 供拟合设计矩阵用

    unstable = []
    for i in range(n):                        # 左腿
        row_l = grid[i]
        for j in range(n):                    # 右腿
            row_r = grid[j]
            p = dict(params)
            p.update(l_l=row_l[0], l_wl=row_l[1], l_bl=row_l[2], I_ll=row_l[3],
                     l_r=row_r[0], l_wr=row_r[1], l_br=row_r[2], I_lr=row_r[3])
            idx = i * n + j
            coords[idx, 0] = row_l[0]
            coords[idx, 1] = row_r[0]
            J_A, J_B = jacobians_numeric(p, M_fun, G_fun, Br_fun)
            A, B = fill_AB(J_A, J_B, p['R_w'], p['R_l'], p['l_l'], p['l_r'])
            if 'K' in want:
                K, _, spectral_radius = solve_lqr(A, B, Q, R)
                if spectral_radius >= 1.0:
                    unstable.append((row_l[0], row_r[0], spectral_radius))
                K_samples[idx, 2, :] = K.reshape(40)
            if 'L' in want or 'Ad' in want or 'Bd' in want:
                Ad, Bd = c2d(A, B, Ts)
                if 'L' in want:
                    L = leso_gain(Ad, Bd, LESO_STATE_POLE, LESO_DIST_POLE)[2]
                    L_samples[idx, 2, :] = L.reshape(140)
                if 'Ad' in want:
                    Ad_samples[idx, 2, :] = Ad.reshape(100)
                if 'Bd' in want:
                    Bd_samples[idx, 2, :] = Bd.reshape(40)

    if unstable:
        print(f'  [警告] 有 {len(unstable)} 组腿长离散闭环不稳定(谱半径>=1):')
        for ll, lr, radius in unstable[:5]:
            print(f'         l_l={ll:.2f}, l_r={lr:.2f}, max |eig(Ad-BdK)|={radius:.9f}')

    x = coords[:, 0]
    y = coords[:, 1]
    Phi = np.column_stack([np.ones(n * n), x, y, x**2, x * y, y**2])
    out = {}
    for name, S in (('K', K_samples), ('L', L_samples),
                    ('Ad', Ad_samples), ('Bd', Bd_samples)):
        if S is None:
            continue
        C, *_ = np.linalg.lstsq(Phi, S[:, 2, :], rcond=None)
        out[name] = (C.T, np.abs(Phi @ C - S[:, 2, :]))
    return out


def eval_fit(coeff, x, y):
    """用 poly22 系数计算预测值(行数决定矩阵形状, 均为 10 列:
    K 40 行->4x10, L 140 行->14x10, Ad 100 行->10x10, Bd 40 行->4x10)。"""
    n_row = coeff.shape[0] // 10
    c = [coeff[:, i].reshape(n_row, 10) for i in range(6)]
    return (c[0] + c[1] * x + c[2] * y
            + c[3] * x**2 + c[4] * x * y + c[5] * y**2)


def interp_check(params, M_fun, G_fun, Br_fun, Q, R, coeff, coeff_L, use_leg_formula=None):
    """附加检查: 在采样点之间的腿长处按当前来源取腿部参数精确求 K 与 L,
    与 poly22 预测比较, 评估"运行时算出的增益"与真实值的偏差量级。"""
    print('\n---------------- 采样点之间腿长插值检查(附加) ----------------')
    for l_test in (0.115, 0.155, 0.185, 0.225, 0.275):
        lw, lb, Il = leg_params_at(l_test, use_leg_formula)
        p = {**params, 'l_l': l_test, 'l_wl': lw, 'l_bl': lb, 'I_ll': Il,
                          'l_r': l_test, 'l_wr': lw, 'l_br': lb, 'I_lr': Il}
        K_exact, A_ex, B_ex, _, _ = compute_K(p, M_fun, G_fun, Br_fun, Q, R)
        parts = []
        if coeff is not None:
            errK = float(np.max(np.abs(K_exact - eval_fit(coeff, l_test, l_test))))
            relK = errK / max(1e-12, float(np.max(np.abs(K_exact))))
            parts.append(f'K: max|exact-fit| = {errK:.4g} ({relK:.2%})')
        if coeff_L is not None:
            L_exact = leso_gain(*c2d(A_ex, B_ex, Ts), LESO_STATE_POLE, LESO_DIST_POLE)[2]
            errL = float(np.max(np.abs(L_exact - eval_fit(coeff_L, l_test, l_test))))
            relL = errL / max(1e-12, float(np.max(np.abs(L_exact))))
            parts.append(f'L: max|exact-fit| = {errL:.4g} ({relL:.2%})')
        print(f'  l={l_test:.3f} m:  ' + ';  '.join(parts))


# =============================================================================
# 输出
# =============================================================================
def c_array_block(name, rows, cols, mat):
    """生成可直接粘贴进 C 的二维数组定义:
        float <name>[rows][cols] = {
        <每行一行, 元素 %.5g, 用',  '分隔, 行尾带逗号>
        };
    行尾必须带逗号: 若缺省, C 词法会把上一行末值与下一行首值合并成
    减法表达式(如 -0.92391 -1.1468), 数据被静默改坏。
    末行也保留逗号 —— C89/C99 均允许初始化列表末尾多一个逗号。"""
    lines = [f'float {name}[{rows}][{cols}] = ', '{']
    for row in mat:
        lines.append(',  '.join('%.5g' % v for v in row) + ',')
    lines.append('};')
    return '\n'.join(lines)


def print_labeled_K(K):
    print('  K 矩阵 (行=输入, 列=状态):')
    print('        ' + ''.join(f'{n:>12}' for n in STATE_NAMES))
    for i, name in enumerate(INPUT_NAMES):
        print(f'{name:<7} ' + ''.join(f'{v:>12.5g}' for v in K[i]))


def write_out(path, text):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text + '\n')
    print(f'  已写出: {path}')


# =============================================================================
# 主流程
# =============================================================================
def parse_overrides(spec):
    """解析 --params 'R_w=0.06,m_b=12' 形式的参数覆盖。"""
    out = {}
    for item in spec.split(','):
        item = item.strip()
        if not item:
            continue
        if '=' not in item:
            raise SystemExit(f'--params 格式错误: "{item}" (应为 k=v)')
        k, v = item.split('=', 1)
        k = k.strip()
        if k not in PARAMS:
            raise SystemExit(f'未知参数 "{k}", 可选: {", ".join(PARAMS)}')
        out[k] = float(v)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description='轮腿机器人 LQR K 矩阵求解')
    ap.add_argument('--mode', choices=['all', 'fixed', 'fit'], default='all',
                    help='fixed=只算定腿长K; fit=只算拟合系数; all=两者(默认)')
    ap.add_argument('--leg-left', type=float, default=None, help='定腿长模式的左腿长 (m)')
    ap.add_argument('--leg-right', type=float, default=None, help='定腿长模式的右腿长 (m)')
    ap.add_argument('--verify-symbolic', action='store_true',
                    help='用 MATLAB 原版做法(solve+jacobian)交叉验证 J_A/J_B')
    ap.add_argument('--params', type=str, default=None, help='覆盖物理参数, 如 "R_w=0.06,m_b=12"')
    ap.add_argument('--formula-inertia', dest='formula_inertia', action='store_true',
                    default=None, help='强制 I_w/I_b/I_z 用公式估算')
    ap.add_argument('--no-formula-inertia', dest='formula_inertia', action='store_false',
                    help='强制用 PARAMS 里声明的 I_w/I_b/I_z')
    ap.add_argument('--leg-formula', dest='leg_formula', action='store_true',
                    default=None, help='强制腿部 lw/lb/Il 用经验公式')
    ap.add_argument('--leg-table', dest='leg_formula', action='store_false',
                    help='强制腿部参数用实测表插值')
    ap.add_argument('--out-dir', type=str, default='.', help='结果 txt 输出目录')
    ap.add_argument('--no-out', action='store_true', help='不写结果文件')
    args = ap.parse_args(argv)

    # ---- 按 Step 2 的输出开关决定生成哪些数据 ----
    want = set()
    if USE_K:
        want.add('K')
    if USE_L:
        want.add('L')
    if USE_AD_BD:
        want.update(('Ad', 'Bd'))
    if not want:
        raise SystemExit('USE_K / USE_L / USE_AD_BD 全为 0: 至少要打开一个')

    t0 = time.perf_counter()

    # ---- Step 0/1: 符号方程 -> M/G/B_raw -> 数值雅可比函数 ----
    eqs = build_equations()
    M, G, B_raw = extract_linear_form(eqs)
    M_fun, G_fun, Br_fun = make_jacobian_functions(M, G, B_raw)

    # ---- Step 2: 解析参数(公式开关 + 覆盖) ----
    use_fi = USE_FORMULA_INERTIA if args.formula_inertia is None else args.formula_inertia
    use_lf = USE_LEG_FORMULA if args.leg_formula is None else args.leg_formula
    params = formula_inertia(PARAMS) if use_fi else dict(PARAMS)
    if args.params:
        params.update(parse_overrides(args.params))
    grid = leg_grid(use_lf)

    print('===== Step 2 参数 =====')
    print('  参数: ' + ', '.join(f'{k}={v:.6g}' for k, v in params.items()))
    print(f'  惯性来源: {"公式估算(圆盘/盒近似)" if use_fi else "PARAMS 声明值"}'
          f';  腿部参数来源: {"经验公式" if use_lf else "LEG_DATA 实测表"}')
    print(f'  拟合网格: {len(grid)} 个腿长点 -> {len(grid) ** 2} 组左右组合\n')

    if args.verify_symbolic:
        print('===== 符号路径交叉验证 (对应 MATLAB solve + jacobian) =====')
        verify_symbolic_path(eqs, params, M_fun, G_fun, Br_fun)

    # ===================== 定腿长 K =====================
    if args.mode in ('all', 'fixed'):
        print(f'===== 定腿长离散 LQR 求解 (Ts={Ts:g} s) =====')
        p = dict(params)
        if args.leg_left is not None:
            p['l_l'] = args.leg_left
        if args.leg_right is not None:
            p['l_r'] = args.leg_right
        # 腿部参数总是按当前来源(公式/表)随腿长重算, 不存在陈旧组合
        for side in ('l', 'r'):
            lw, lb, Il = leg_params_at(p[f'l_{side}'], use_lf)
            if side == 'l':
                p.update(l_wl=lw, l_bl=lb, I_ll=Il)
            else:
                p.update(l_wr=lw, l_br=lb, I_lr=Il)
        print('  当前参数: ' + ', '.join(f'{k}={v:.6g}' for k, v in p.items()))

        if 'K' in want:
            K, A, B, dare_resid, spectral_radius = compute_K(p, M_fun, G_fun, Br_fun, Q_LQR, R_LQR)
            print_labeled_K(K)
            print(f'  闭环 max |eig(Ad-BdK)| = {spectral_radius:.9f}  (<1 即稳定)')
            print(f'  离散 Riccati 最大残差 = {dare_resid:.3e}  (应接近 0)')

            block = c_array_block('K', 4, 10, K)
            print('  ---- C 数组定义, 可直接粘贴进 C ----')
            print(block)
            if not args.no_out:
                write_out(os.path.join(args.out_dir, 'K_fixed.txt'), block)
        elif 'L' in want or 'Ad' in want or 'Bd' in want:
            J_A, J_B = jacobians_numeric(p, M_fun, G_fun, Br_fun)
            A, B = fill_AB(J_A, J_B, p['R_w'], p['R_l'], p['l_l'], p['l_r'])

        if 'L' in want or 'Ad' in want or 'Bd' in want:
            # ---- ZOH 离散模型 + LESO 观测器增益 (只依赖离散模型与极点, 与 K 无关) ----
            Ad, Bd = c2d(A, B, Ts)
        if 'L' in want:
            A_e, C_e, L = leso_gain(Ad, Bd, LESO_STATE_POLE, LESO_DIST_POLE)
            poles_obs = np.sort(np.abs(np.linalg.eigvals(A_e - L @ C_e)))
            print(f'  LESO 自检: 观测器极点模 {poles_obs.min():.4f}~{poles_obs.max():.4f} '
                  f'(设计: 10×{LESO_STATE_POLE} + 4×{LESO_DIST_POLE}), |L|max = {np.abs(L).max():.1f}')
            block_l = c_array_block('L', 14, 10, L)
            print('  ---- L: 前 10 行=状态估计增益(s..dtheta_b), 后 4 行=扰动估计(f_wl..f_br) ----')
            print(block_l)
            if not args.no_out:
                write_out(os.path.join(args.out_dir, 'L_fixed.txt'), block_l)

        if 'Ad' in want or 'Bd' in want:
            print('  ---- Ad/Bd: ZOH 离散模型(Ts=1ms); 固件现场拼装 '
                  'A_e=[[Ad,Bd],[0,I]], B_e=[Bd;0], C_e=[I,0] ----')
        if 'Ad' in want:
            block_ad = c_array_block('Ad', 10, 10, Ad)
            print(block_ad)
            if not args.no_out:
                write_out(os.path.join(args.out_dir, 'Ad_fixed.txt'), block_ad)
        if 'Bd' in want:
            block_bd = c_array_block('Bd', 10, 4, Bd)
            print(block_bd)
            if not args.no_out:
                write_out(os.path.join(args.out_dir, 'Bd_fixed.txt'), block_bd)

    # ===================== 变腿长拟合 =====================
    if args.mode in ('all', 'fit'):
        print('\n===== 变腿长 poly22 拟合 =====')
        print(f'  样本数 = {len(grid)}? = {len(grid) ** 2}')
        fits = fit_all_coefficients(params, M_fun, G_fun, Br_fun, Q_LQR, R_LQR, grid, want)
        for name in ('K', 'L', 'Ad', 'Bd'):
            if name not in fits:
                continue
            r_ = fits[name][1]
            print(f'  {name} 拟合最大残差 = {float(r_.max()):.4g}, '
                  f'RMS 残差 = {float(np.sqrt(np.mean(r_ ** 2))):.4g}')
        print('  拟合公式: p(x,y) = p00 + p10*x + p01*y + p20*x? + p11*x*y + p02*y?'
              '   (x=l_l, y=l_r)')
        print('  行 <-> 元素(均行主序展平): K/Ad 第 r 行=(r//10+1, r%10+1), '
              'L 第 r 行=(r//10+1, r%10+1; 1-100 行=状态估计, 101-140 行=扰动估计), '
              'Bd 第 r 行=(r//4+1, r%4+1)')
        print('  每行 6 列依次为: p00, p10, p01, p20, p11, p02')

        if 'K' in fits:
            block = c_array_block('P', 40, 6, fits['K'][0])
            print('\n  ---- C 数组定义, 可直接粘贴进 C ----')
            print(block)
            if not args.no_out:
                write_out(os.path.join(args.out_dir, 'K_Fit_Coefficients.txt'), block)

        if 'L' in fits:
            block_l = c_array_block('L_Fit', 140, 6, fits['L'][0])
            print('\n  ---- L 的 poly22 系数, C 数组定义 ----')
            print(block_l)
            if not args.no_out:
                write_out(os.path.join(args.out_dir, 'L_Fit_Coefficients.txt'), block_l)

        if 'Ad' in fits:
            block_ad = c_array_block('Ad_Fit', 100, 6, fits['Ad'][0])
            print('\n  ---- Ad 的 poly22 系数, C 数组定义 ----')
            print(block_ad)
            if not args.no_out:
                write_out(os.path.join(args.out_dir, 'Ad_Fit_Coefficients.txt'), block_ad)

        if 'Bd' in fits:
            block_bd = c_array_block('Bd_Fit', 40, 6, fits['Bd'][0])
            print('\n  ---- Bd 的 poly22 系数, C 数组定义 ----')
            print(block_bd)
            if not args.no_out:
                write_out(os.path.join(args.out_dir, 'Bd_Fit_Coefficients.txt'), block_bd)

        if 'K' in fits or 'L' in fits:
            interp_check(params, M_fun, G_fun, Br_fun, Q_LQR, R_LQR,
                         fits.get('K', (None,))[0], fits.get('L', (None,))[0], use_lf)

    print(f'\n总耗时 {time.perf_counter() - t0:.2f} s')


if __name__ == '__main__':
    main()
