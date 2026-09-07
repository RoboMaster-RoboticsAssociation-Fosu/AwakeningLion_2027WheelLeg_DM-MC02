# -*- coding: utf-8 -*-
# _audit_independent.py — 独立复核脚本：方程/A/B 填装逻辑直接从
# HerKules_VOCAL_SJ_LQR_v4_with_data.m 转写（solve + jacobian 原版做法），
# 不参考被测实现，仅与 K_fixed.txt 比对结果。
import sympy
import numpy as np
from scipy import linalg

# ---- 符号（.m 13-38 行）----
R_w, R_l = sympy.symbols('R_w R_l', positive=True)
l_l, l_r = sympy.symbols('l_l l_r', positive=True)
l_wl, l_wr = sympy.symbols('l_wl l_wr', positive=True)
l_bl, l_br = sympy.symbols('l_bl l_br', positive=True)
l_c = sympy.symbols('l_c', positive=True)
m_w, m_l, m_b = sympy.symbols('m_w m_l m_b', positive=True)
I_w = sympy.symbols('I_w', positive=True)
I_ll, I_lr = sympy.symbols('I_ll I_lr', positive=True)
I_b = sympy.symbols('I_b', positive=True)
I_z = sympy.symbols('I_z', positive=True)
g = sympy.symbols('g', positive=True)
ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b = sympy.symbols(
    'ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b')
theta_ll, theta_lr, theta_b = sympy.symbols('theta_ll theta_lr theta_b')
T_wl, T_wr, T_bl, T_br = sympy.symbols('T_wl T_wr T_bl T_br')

# ---- 方程组（.m 45-49 行逐项转写）----
eqn1 = (I_w*l_l/R_w+m_w*R_w*l_l+m_l*R_w*l_bl)*ddtheta_wl+(m_l*l_wl*l_bl-I_ll)*ddtheta_ll+(m_l*l_wl+m_b*l_l/2)*g*theta_ll+T_bl-T_wl*(1+l_l/R_w)
eqn2 = (I_w*l_r/R_w+m_w*R_w*l_r+m_l*R_w*l_br)*ddtheta_wr+(m_l*l_wr*l_br-I_lr)*ddtheta_lr+(m_l*l_wr+m_b*l_r/2)*g*theta_lr+T_br-T_wr*(1+l_r/R_w)
eqn3 = -(m_w*R_w**2+I_w+m_l*R_w**2+m_b*R_w**2/2)*ddtheta_wl-(m_w*R_w**2+I_w+m_l*R_w**2+m_b*R_w**2/2)*ddtheta_wr-(m_l*R_w*l_wl+m_b*R_w*l_l/2)*ddtheta_ll-(m_l*R_w*l_wr+m_b*R_w*l_r/2)*ddtheta_lr+T_wl+T_wr
eqn4 = (m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wl+(m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wr+m_l*l_wl*l_c*ddtheta_ll+m_l*l_wr*l_c*ddtheta_lr-I_b*ddtheta_b+m_b*g*l_c*theta_b-(T_wl+T_wr)*l_c/R_w-(T_bl+T_br)
eqn5 = ((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wl-((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wr+(I_z*l_l)/(2*R_l)*ddtheta_ll-(I_z*l_r)/(2*R_l)*ddtheta_lr-T_wl*R_l/R_w+T_wr*R_l/R_w

# ---- 参数（.m 100-128 行；I_w_ac=(3510000)*10^(-7)）----
params = {R_w: 0.9, R_l: 0.25, l_c: 0.037, m_w: 0.8, m_l: 1.6183599, m_b: 11.542,
          I_w: 3510000 * 10**(-7), I_b: 0.260, I_z: 0.226,
          l_l: 0.18, l_wl: 0.05, l_bl: 0.13, I_ll: 0.02054500,
          l_r: 0.18, l_wr: 0.05, l_br: 0.13, I_lr: 0.02054500, g: 9.81}

# ---- solve + jacobian（对应 .m 50、54-55 行；参数先代入 —— 与 get_K_from_LQR
#      的 subs 语义一致，避免全符号 jacobian 的表达式爆炸，结果等价）----
eqns_ac = [e.subs(params) for e in (eqn1, eqn2, eqn3, eqn4, eqn5)]
sols = sympy.solve(eqns_ac,
                   [ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b], dict=True)
sol = sols[0]
dd = [sol[v] for v in (ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b)]
J_A = sympy.Matrix(dd).jacobian([theta_ll, theta_lr, theta_b])   # 5x3
J_B = sympy.Matrix(dd).jacobian([T_wl, T_wr, T_bl, T_br])        # 5x4

J_An = np.array([[float(v) for v in row] for row in J_A.tolist()])  # 5x3
J_Bn = np.array([[float(v) for v in row] for row in J_B.tolist()])  # 5x4

# ---- A/B 填装（.m 57-93 行, 1-based 逻辑照抄, 这里手动代入数值参数）----
Rw, Rl, ll, lr = 0.9, 0.25, 0.18, 0.18
A = np.zeros((10, 10))
B = np.zeros((10, 4))
for p in (5, 7, 9):                      # MATLAB p = 5:2:9
    ai = (p - 3) // 2 - 1                # A_index = (p-3)/2, 转 0-based
    p0 = p - 1
    A[1, p0] = Rw * (J_An[0, ai] + J_An[1, ai]) / 2
    A[3, p0] = (Rw * (-J_An[0, ai] + J_An[1, ai]) / (2 * Rl)
                - ll * J_An[2, ai] / (2 * Rl) + lr * J_An[3, ai] / (2 * Rl))
    for q in (6, 8, 10):                 # q = 6:2:10
        A[q - 1, p0] = J_An[q // 2 - 1, ai]
for r in range(1, 11):
    if r % 2 == 1:
        A[r - 1, r] = 1.0                # A(r, r+1) = 1（奇数行运动学）
for h0 in range(4):                      # h = 1:4
    B[1, h0] = Rw * (J_Bn[0, h0] + J_Bn[1, h0]) / 2
    B[3, h0] = (Rw * (-J_Bn[0, h0] + J_Bn[1, h0]) / (2 * Rl)
                - ll * J_Bn[2, h0] / (2 * Rl) + lr * J_Bn[3, h0] / (2 * Rl))
    for f in (6, 8, 10):
        B[f - 1, h0] = J_Bn[f // 2 - 1, h0]

# ---- LQR（对应 .m 的 icare: K = R^-1 B^T P）----
Q = np.diag([1., 2., 12000., 200., 1000., 1., 1000., 1., 20000., 1.])
R = np.diag([0.25, 0.25, 1.5, 1.5])
P = linalg.solve_continuous_are(A, B, Q, R)
K = np.linalg.solve(R, B.T @ P)

print('K (independent re-derivation, %.5g):')
mine_str = []
for row in K:
    s = ',  '.join('%.5g' % v for v in row)
    mine_str.append(s)
    print(s)

# ---- 与参考 K_fixed.txt 比较 ----
with open(r'E:\ROBOT_NEW\wheel_legger\Wheel_Legger\lqr_k_calc\K_fixed.txt', encoding='utf-8') as fh:
    ref_lines = [line.strip() for line in fh if line.strip()]
Kref = np.array([[float(x) for x in line.split(',')] for line in ref_lines])
print('shape mine=%s ref=%s' % (K.shape, Kref.shape))

absdiff = np.abs(K - Kref)
reldiff = absdiff / np.maximum(np.abs(Kref), 1e-9)
print('max |diff| absolute : %.3e' % absdiff.max())
print('max |diff| relative : %.3e  (参考文件仅 5 位有效数字, 期望 ~1e-6..1e-5 量级)'
      % reldiff.max())

same = (len(mine_str) == len(ref_lines)) and all(m == r for m, r in zip(mine_str, ref_lines))
print('5-sig-fig string-level identical to K_fixed.txt: %s' % ('YES' if same else 'NO'))
if not same:
    for i, (m, r) in enumerate(zip(mine_str, ref_lines)):
        if m != r:
            print('  line %d differs:\n    mine: %s\n    ref : %s' % (i + 1, m, r))

eig = np.linalg.eigvals(A - B @ K)
print('closed-loop max Re(eig(A-BK)) = %.6f' % np.max(np.real(eig)))
