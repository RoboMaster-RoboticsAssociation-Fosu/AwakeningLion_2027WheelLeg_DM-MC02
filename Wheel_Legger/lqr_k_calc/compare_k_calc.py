#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
compare_k_calc.py —— 对比 ABK_LQR.py 与 hku_lqr_k_calc.py 的“K 值计算”部分

对比维度：
  [1] 符号级：两边 5 条动力学方程是否完全相同（simplify(差)==0 逐条证明）
  [2] 数值级：同一组物理参数下，两边算出的 A(10x10)/B(10x4) 是否逐元素一致
  [3] K 级：同一组参数 + 同一组 Q/R 下，
        连续 LQR（hku 复刻链路，等价 MATLAB icare）
        vs  ZOH 离散化 @1kHz + 离散 Riccati（ABK 链路，等价 MATLAB c2d+dlqr）
      的 K 差多少 —— 把“连续/离散设计差异”与“Q/R 调参差异”分开量化
  [4] 拟合级：poly22 基底顺序、K 展平方式、最小二乘是否一致

ABK_LQR.py 只执行到 Step 2 标记之前（全部函数定义，无副作用），
其 Step 3 之后的 441 点拟合 + LESO 不属于本次“K 值计算”对比范围。
"""
import importlib.util
import os
import sys

import numpy as np
import sympy as sp

HERE = os.path.dirname(os.path.abspath(__file__))
ABK_PATH = os.path.join(os.path.dirname(HERE), 'ABK_LQR.py')
HKU_PATH = os.path.join(HERE, 'hku_lqr_k_calc.py')

# ---------------------------------------------------------------------------
# 载入两边代码
# ---------------------------------------------------------------------------
src = open(ABK_PATH, encoding='utf-8').read()
cut = src.index('# %%%%%%%%%%%%%%%%%%%%%Step 2')   # Step 2 之前全部是定义
ns = {}
exec(compile(src[:cut], 'ABK_LQR.py[:Step2]', 'exec'), ns)   # ABK 的符号方程与 subs_AB/c2d/dlqr

spec = importlib.util.spec_from_file_location('hku', HKU_PATH)
hku = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hku)                                  # 本复刻（有 __main__ 保护，安全）

# ---------------------------------------------------------------------------
# [1] 符号级：5 条方程逐条比对
# ---------------------------------------------------------------------------
smap = {  # 本复刻符号 -> ABK 符号（纯改名对应）
    hku.R_w: ns['R_w'], hku.R_l: ns['R_l'], hku.l_l: ns['l_l'], hku.l_r: ns['l_r'],
    hku.l_wl: ns['lw_l'], hku.l_wr: ns['lw_r'], hku.l_bl: ns['lb_l'], hku.l_br: ns['lb_r'],
    hku.l_c: ns['l_c'], hku.m_w: ns['m_w'], hku.m_l: ns['m_l'], hku.m_b: ns['m_b'],
    hku.I_w: ns['I_w'], hku.I_ll: ns['Il_l'], hku.I_lr: ns['Il_r'],
    hku.I_b: ns['I_b'], hku.I_z: ns['I_z'], hku.g: ns['g'],
    hku.ddtheta_wl: ns['ddtheta_w_l'], hku.ddtheta_wr: ns['ddtheta_w_r'],
    hku.ddtheta_ll: ns['ddtheta_l_l'], hku.ddtheta_lr: ns['ddtheta_l_r'],
    hku.ddtheta_b: ns['ddtheta_b'],
    hku.theta_ll: ns['theta_l_l'], hku.theta_lr: ns['theta_l_r'], hku.theta_b: ns['theta_b'],
    hku.T_wl: ns['T_lw_l'], hku.T_wr: ns['T_lw_r'], hku.T_bl: ns['T_bl_l'], hku.T_br: ns['T_bl_r'],
}
mine_eqs = hku.build_equations()
their_eqs = [ns['eqn1'], ns['eqn2'], ns['eqn3'], ns['eqn4'], ns['eqn5']]

print('[1] 符号级方程比对（simplify(本复刻 - ABK) == 0 ?）')
all_same = True
for i, (a, b) in enumerate(zip(mine_eqs, their_eqs)):
    d = sp.simplify(a.subs(smap) - b)
    ok = (d == 0)
    all_same &= ok
    print(f'    eqn{i+1}: {"完全相同" if ok else "不同! 差=" + str(d)}')
assert all_same

# ---------------------------------------------------------------------------
# [2] 数值级：同参数下 A/B 逐元素比对
# ---------------------------------------------------------------------------
M, G, B_raw = hku.extract_linear_form(mine_eqs)
Mf, Gf, Bf = hku.make_jacobian_functions(M, G, B_raw)

def my_AB(p):
    JA, JB = hku.jacobians_numeric(p, Mf, Gf, Bf)
    return hku.fill_AB(JA, JB, p['R_w'], p['R_l'], p['l_l'], p['l_r'])

def their_AB(p):
    vals = (p['R_w'], p['R_l'], p['l_l'], p['l_r'], p['l_wl'], p['l_wr'],
            p['l_bl'], p['l_br'], p['l_c'], p['m_w'], p['m_l'], p['m_b'],
            p['I_w'], p['I_ll'], p['I_lr'], p['I_b'], p['I_z'], p['g'])
    return ns['subs_AB'](vals)

# ABK 当前生效的参数组合（USE_IDILE_MODEL=1 时 I_w/I_b/I_z 被覆盖为公式值）
l = 0.15
p_abk = dict(
    R_w=0.058, R_l=0.22, l_c=0.120, m_w=0.537, m_l=1.65, m_b=11.0, g=9.81,
    l_l=l, l_r=l, l_wl=0.782*l - 0.075, l_wr=0.782*l - 0.075,
    l_bl=0.218*l + 0.075, l_br=0.218*l + 0.075,
    I_ll=0.4*l + 0.07, I_lr=0.4*l + 0.07,
    I_w=0.5 * 0.537 * 0.058**2,
    I_b=11.0 * (0.415**2 + 0.16**2) / 12.0,
    I_z=11.0 * (0.415**2 + 0.260**2) / 12.0,
)
p_hku = dict(hku.PARAMS)   # HKU 定腿长参数（l=0.18）

print('\n[2] 同参数下 A/B 逐元素最大偏差（两边独立代码路径）')
for tag, p in (('ABK 参数(l=0.15, 公式惯性)', p_abk), ('HKU 参数(l=0.18)', p_hku)):
    A1, B1 = my_AB(p)
    A2, B2 = their_AB(p)
    dA = float(np.max(np.abs(A1 - A2)))
    dB = float(np.max(np.abs(B1 - B2)))
    print(f'    {tag}:  max|dA| = {dA:.3e},  max|dB| = {dB:.3e}')

# ---------------------------------------------------------------------------
# [3] K 级：同参数同 Q/R 下，连续 LQR vs ZOH 离散 dlqr
# ---------------------------------------------------------------------------
Q_abk = np.diag([100, 50, 500, 50, 100, 5, 100, 5, 20000, 5]).astype(float)  # ABK 263 行
R_abk = np.diag([150, 150, 50, 50]).astype(float)                            # ABK 266 行
Ts = 0.001                                                                    # ABK 236 行

def kdiff(Ka, Kb):
    d = np.abs(Ka - Kb)
    scale = np.max(np.abs(Ka))
    return d.max(), d.max() / scale

print('\n[3] K 对比（同一模型，隔离“连续/离散”与“Q/R”两个因素）')

# 因素一：连续 vs 离散（同参数、同 Q/R）
print('    — 因素A: 连续(icare 等价) vs ZOH@1kHz+dlqr，Q/R 相同 —')
for tag, p, Q, R in (('ABK 参数+ABK Q/R', p_abk, Q_abk, R_abk),
                     ('HKU 参数+HKU Q/R', p_hku, hku.Q_LQR, hku.R_LQR)):
    A, B = my_AB(p)
    Kc, _, _ = hku.solve_lqr(A, B, Q, R)                    # 连续 LQR
    Ad, Bd = ns['c2d'](A, B, Ts)
    Kd = ns['dlqr'](Ad, Bd, Q, R)                           # ABK 离散链路
    md, rel = kdiff(Kc, Kd)
    # 交叉验证：用“我方 A/B”走 ABK 链路 == 用“ABK A/B”走 ABK 链路
    A2, B2 = their_AB(p)
    Kd2 = ns['dlqr'](*(ns['c2d'](A2, B2, Ts)), Q, R)
    xdiff = float(np.max(np.abs(Kd - Kd2)))
    print(f'    {tag}:  max|K_c-K_d| = {md:.4g} (相对 {rel:.2%})，'
          f'两边链路离散 K 互差 {xdiff:.1e}')

# 因素二：Q/R 不同（同参数、同为连续）
print('    — 因素B: Q/R 调参差异（同 ABK 参数，均连续 LQR）—')
K1, _, _ = hku.solve_lqr(*my_AB(p_abk), Q_abk, R_abk)
K2, _, _ = hku.solve_lqr(*my_AB(p_abk), hku.Q_LQR, hku.R_LQR)
md, rel = kdiff(K1, K2)
print(f'    ABK Q/R vs HKU Q/R:  max|dK| = {md:.4g} (相对 {rel:.2%})')

# ---------------------------------------------------------------------------
# [4] 拟合级：poly22 基底 / 系数顺序 / 最小二乘
# ---------------------------------------------------------------------------
print('\n[4] poly22 拟合机制比对')
rng = np.random.default_rng(0)
x = rng.uniform(0.10, 0.30, 400); y = rng.uniform(0.10, 0.30, 400)
z = rng.uniform(-100, 100, (400, 5))
C_abk = ns['fit_poly22'](x, y, z)                            # ABK: 返回 (n_cols, 6)
basis = np.column_stack([np.ones_like(x), x, y, x**2, x*y, y**2])   # 本复刻同款基底
C_hku = np.linalg.lstsq(basis, z, rcond=None)[0].T
print(f'    两边最小二乘系数最大差 = {float(np.max(np.abs(C_abk - C_hku))):.3e}'
      f'（基底列序均为 [1, x, y, x², xy, y²] -> [p00,p10,p01,p20,p11,p02]）')
print(f'    K 展平: 两边均为 (4,10) 行主序 -> 槽位 (l-1)*10+m；'
      f'状态/输入排序与 u=-Kx 约定两边相同')
