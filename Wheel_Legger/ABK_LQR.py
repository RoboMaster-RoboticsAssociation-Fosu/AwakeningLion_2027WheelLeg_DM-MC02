"""ABK_LQR.m 的 Python 版本，并按 Others/Qi-Q26-Leg_Robot/LQR_calc.m 的思路改为离散设计。

与最初的连续版相比有三处结构性改动：
  1. A、B 先用 ZOH 离散化（对应 MATLAB c2d），再解离散黎卡提方程求 K（对应 dlqr）；
  2. 在离散模型上构造 LESO 扩张状态观测器，用极点配置（对应 place）求观测器增益 L；
  3. K、A_d、B_d、L 四块都按 (l_l, l_r) 做 poly22 曲面拟合并打印，可直接贴进 chassis_config.c。

三处因 Python 库而与 .m 不同：
  1. sympy 的 solve() 在此表达式爆炸，改为整理成 M*ddq = N*x 后代数值再用 numpy 解；
  2. MATLAB 的 dlqr() 用 scipy 解离散黎卡提方程后按 K = (R+B'SB)^-1*B'SA 求出；
  3. fit(...,'poly22') 用最小二乘代替，系数顺序与 coeffvalues() 相同。
依赖：numpy scipy sympy
"""

import numpy as np
import sympy as sp
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete, place_poles

# 模型维数：10 个状态、4 个输入、扩张 4 个扰动状态（与输入同维）
N_STATE = 10
N_INPUT = 4
N_DIST = 4

# %%%%%%%%%%%%%%%%%%%%%%%%%Step 0：定义变量%%%%%%%%%%%%%%%%%%%%%%%%%

R_w = sp.symbols('R_w')
R_l = sp.symbols('R_l')

l_l, l_r = sp.symbols('l_l l_r')
lw_l, lw_r = sp.symbols('lw_l lw_r')
lb_l, lb_r = sp.symbols('lb_l lb_r')
l_c = sp.symbols('l_c')

m_w, m_l, m_b = sp.symbols('m_w m_l m_b')

I_w = sp.symbols('I_w')
Il_l, Il_r = sp.symbols('Il_l Il_r')
I_b = sp.symbols('I_b')
I_z = sp.symbols('I_z')

ddtheta_w_l, ddtheta_w_r = sp.symbols('ddtheta_w_l ddtheta_w_r')
ddtheta_l_l, ddtheta_l_r = sp.symbols('ddtheta_l_l ddtheta_l_r')
ddtheta_b = sp.symbols('ddtheta_b')

theta_l_l, theta_l_r, theta_b = sp.symbols('theta_l_l theta_l_r theta_b')

T_lw_l, T_lw_r, T_bl_l, T_bl_r = sp.symbols('T_lw_l T_lw_r T_bl_l T_bl_r')

g = sp.symbols('g')

# %%%%%%%%%%%%%%%%%%%%%%%%% Step 1：求AB矩阵 %%%%%%%%%%%%%%%%%%%%%%%%%

# 对应上交文档式3.11~3.15
eqn1 = ((I_w*l_l/R_w + m_w*R_w*l_l + m_l*R_w*lb_l)*ddtheta_w_l
        + (m_l*lw_l*lb_l - Il_l)*ddtheta_l_l
        + (m_l*lw_l + 0.5*m_b*l_l)*g*theta_l_l
        + T_bl_l - T_lw_l*(1 + l_l/R_w))

eqn2 = ((I_w*l_r/R_w + m_w*R_w*l_r + m_l*R_w*lb_r)*ddtheta_w_r
        + (m_l*lw_r*lb_r - Il_r)*ddtheta_l_r
        + (m_l*lw_r + 0.5*m_b*l_r)*g*theta_l_r
        + T_bl_r - T_lw_r*(1 + l_r/R_w))

eqn3 = (-(m_w*R_w**2 + I_w + m_l*R_w**2 + 0.5*m_b*R_w**2)*ddtheta_w_l
        - (m_w*R_w**2 + I_w + m_l*R_w**2 + 0.5*m_b*R_w**2)*ddtheta_w_r
        - (m_l*R_w*lw_l + 0.5*m_b*R_w*l_l)*ddtheta_l_l
        - (m_l*R_w*lw_r + 0.5*m_b*R_w*l_r)*ddtheta_l_r
        + T_lw_l + T_lw_r)

eqn4 = ((m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c)*ddtheta_w_l
        + (m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c)*ddtheta_w_r
        + m_l*lw_l*l_c*ddtheta_l_l
        + m_l*lw_r*l_c*ddtheta_l_r
        - I_b*ddtheta_b
        + m_b*g*l_c*theta_b
        - (T_lw_l + T_lw_r)*l_c/R_w
        - (T_bl_l + T_bl_r))

eqn5 = ((0.5*I_z*R_w/R_l + I_w*R_l/R_w)*ddtheta_w_l
        - (0.5*I_z*R_w/R_l + I_w*R_l/R_w)*ddtheta_w_r
        + 0.5*I_z*l_l/R_l*ddtheta_l_l
        - 0.5*I_z*l_r/R_l*ddtheta_l_r
        - T_lw_l*R_l/R_w
        + T_lw_r*R_l/R_w)

M, rhs = sp.linear_eq_to_matrix(
    [eqn1, eqn2, eqn3, eqn4, eqn5],
    [ddtheta_w_l, ddtheta_w_r, ddtheta_l_l, ddtheta_l_r, ddtheta_b])

N_A = rhs.jacobian([theta_l_l, theta_l_r, theta_b])
N_B = rhs.jacobian([T_lw_l, T_lw_r, T_bl_l, T_bl_r])

syms_list = (R_w, R_l, l_l, l_r, lw_l, lw_r, lb_l, lb_r, l_c,
             m_w, m_l, m_b, I_w, Il_l, Il_r, I_b, I_z, g)
M_func = sp.lambdify(syms_list, M, 'numpy')
N_A_func = sp.lambdify(syms_list, N_A, 'numpy')
N_B_func = sp.lambdify(syms_list, N_B, 'numpy')


def subs_AB(vals):
    M_ac = np.array(M_func(*vals), dtype=float)
    J_A = np.linalg.solve(M_ac, np.array(N_A_func(*vals), dtype=float))
    J_B = np.linalg.solve(M_ac, np.array(N_B_func(*vals), dtype=float))

    R_w_v, R_l_v, l_l_v, l_r_v = vals[0], vals[1], vals[2], vals[3]

    a25 = R_w_v*(J_A[0, 0] + J_A[1, 0])/2
    a27 = R_w_v*(J_A[0, 1] + J_A[1, 1])/2
    a29 = R_w_v*(J_A[0, 2] + J_A[1, 2])/2

    a45 = (R_w_v*(-J_A[0, 0] + J_A[1, 0])/(2*R_l_v)
           - l_l_v*J_A[2, 0]/(2*R_l_v) + l_r_v*J_A[3, 0]/(2*R_l_v))
    a47 = (R_w_v*(-J_A[0, 1] + J_A[1, 1])/(2*R_l_v)
           - l_l_v*J_A[2, 1]/(2*R_l_v) + l_r_v*J_A[3, 1]/(2*R_l_v))
    a49 = (R_w_v*(-J_A[0, 2] + J_A[1, 2])/(2*R_l_v)
           - l_l_v*J_A[2, 2]/(2*R_l_v) + l_r_v*J_A[3, 2]/(2*R_l_v))

    a65, a67, a69 = J_A[2, 0], J_A[2, 1], J_A[2, 2]
    a85, a87, a89 = J_A[3, 0], J_A[3, 1], J_A[3, 2]
    a105, a107, a109 = J_A[4, 0], J_A[4, 1], J_A[4, 2]

    b21, b22, b23, b24 = [R_w_v*(J_B[0, j] + J_B[1, j])/2 for j in range(4)]
    b41, b42, b43, b44 = [R_w_v*(-J_B[0, j] + J_B[1, j])/(2*R_l_v)
                          - l_l_v*J_B[2, j]/(2*R_l_v)
                          + l_r_v*J_B[3, j]/(2*R_l_v) for j in range(4)]
    b61, b62, b63, b64 = J_B[2, 0], J_B[2, 1], J_B[2, 2], J_B[2, 3]
    b81, b82, b83, b84 = J_B[3, 0], J_B[3, 1], J_B[3, 2], J_B[3, 3]
    b101, b102, b103, b104 = J_B[4, 0], J_B[4, 1], J_B[4, 2], J_B[4, 3]

    A = np.array([
        [0, 1, 0, 0,    0, 0,    0, 0,    0, 0],
        [0, 0, 0, 0,  a25, 0,  a27, 0,  a29, 0],
        [0, 0, 0, 1,    0, 0,    0, 0,    0, 0],
        [0, 0, 0, 0,  a45, 0,  a47, 0,  a49, 0],
        [0, 0, 0, 0,    0, 1,    0, 0,    0, 0],
        [0, 0, 0, 0,  a65, 0,  a67, 0,  a69, 0],
        [0, 0, 0, 0,    0, 0,    0, 1,    0, 0],
        [0, 0, 0, 0,  a85, 0,  a87, 0,  a89, 0],
        [0, 0, 0, 0,    0, 0,    0, 0,    0, 1],
        [0, 0, 0, 0, a105, 0, a107, 0, a109, 0],
    ], dtype=float)

    B = np.array([
        [0,    0,    0,    0],
        [b21,  b22,  b23,  b24],
        [0,    0,    0,    0],
        [b41,  b42,  b43,  b44],
        [0,    0,    0,    0],
        [b61,  b62,  b63,  b64],
        [0,    0,    0,    0],
        [b81,  b82,  b83,  b84],
        [0,    0,    0,    0],
        [b101, b102, b103, b104],
    ], dtype=float)

    return A, B

def c2d(A_ac, B_ac, Ts):
    """ZOH 离散化，对应 MATLAB c2d(A, B, Ts)。"""
    Ad, Bd, _, _, _ = cont2discrete(
        (A_ac, B_ac, np.eye(N_STATE), np.zeros((N_STATE, N_INPUT))), Ts, method='zoh')
    return Ad, Bd


def dlqr(Ad, Bd, Q, R):
    """离散 LQR，对应 MATLAB dlqr(G, H, Q, R)。"""
    S = solve_discrete_are(Ad, Bd, Q, R)
    return np.linalg.solve(R + Bd.T @ S @ Bd, Bd.T @ S @ Ad)


def leso_gain(Ad, Bd, state_pole, dist_pole):
    """LESO 扩张状态观测器增益，对应 place(A_e', C_e', poles)。

    把 4 个输入通道的总扰动扩张进状态：
        A_e = [[Ad, Bd], [0, I]] (14x14)、B_e = [Bd; 0] (14x4)、C_e = [I, 0] (10x14)
        X_hat(k+1) = A_e*X_hat(k) + B_e*u(k) + L*(y(k) - C_e*X_hat(k))
    C_e 取全状态可测：固件里这 10 个状态都有量（s/ds 来自轮速+Kalman，
    phi/dphi 与 theta_b/dtheta_b 来自 IMU，theta_l 来自 VMC）。
    """
    A_e = np.block([[Ad, Bd],
                    [np.zeros((N_DIST, N_STATE)), np.eye(N_DIST)]])
    C_e = np.hstack([np.eye(N_STATE), np.zeros((N_STATE, N_DIST))])
    poles = np.concatenate([np.full(N_STATE, state_pole),
                            np.full(N_DIST, dist_pole)])
    L = place_poles(A_e.T, C_e.T, poles, maxiter=300).gain_matrix.T
    return A_e, C_e, L


def interp1(xp, fp, x):
    if x <= xp[0]:
        return fp[0] + (fp[1] - fp[0])/(xp[1] - xp[0])*(x - xp[0])
    if x >= xp[-1]:
        return fp[-1] + (fp[-1] - fp[-2])/(xp[-1] - xp[-2])*(x - xp[-1])
    return float(np.interp(x, xp, fp))


def poly22_basis(x, y):
    """poly22 基底，列序与 MATLAB coeffvalues() 一致：p00 p10 p01 p20 p11 p02。"""
    return np.column_stack([np.ones_like(x), x, y, x**2, x*y, y**2])


def fit_poly22(x, y, z):
    """对 z 的每一列做 poly22 曲面拟合，返回 (列数, 6) 的系数矩阵。"""
    basis = poly22_basis(x, y)
    return np.linalg.lstsq(basis, z, rcond=None)[0].T


def print_block(title, coefficients):
    """按 chassis_config.c 可直接粘贴的格式打印一块系数。"""
    print(f'/* {title} */')
    last = len(coefficients) - 1
    for n, row in enumerate(coefficients):
        print('    ' + ',  '.join(f'{v:.5g}' for v in row) + (',' if n < last else ''))
    print()


# %%%%%%%%%%%%%%%%%%%%%Step 2：输入参数（可以修改的部分）%%%%%%%%%%%%%%%%%%%%%

# BIG_WHEEL_LEG
g_ac = 9.81

R_w_ac = 0.058
R_l_ac = 0.22

l_c_ac = 0.120

m_w_ac = 0.537
m_l_ac = 1.65
m_b_ac = 11.0

I_w_ac = 0.000516
I_b_ac = 0.025
I_z_ac = 0.380

Ts_ac = 0.001            # 控制周期，s。与固件 chassis_task 的 1 kHz 对应

leso_state_pole = 0.4    # LESO 的 10 个原状态极点（z 域模），带宽 -ln(z)/Ts ≈ 146 Hz
leso_dist_pole = 0.985   # LESO 的 4 个扩张扰动状态极点，带宽 ≈ 2.4 Hz


# 1 = 查 Leg_data 实测表格，0 = 用上交经验公式（Il=0.4l+0.07 等）
USE_MEASURED_TABLE = 0
USE_IDILE_MODEL = 1
# 四列依次为 腿长l、lw、lb、Il，单位 m / m / m / kg*m^2
Leg_data_l = np.array([
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

Leg_data_r = Leg_data_l

#                  s    ds      phi   dphi theta_ll     dtheta_ll    theta_lr    dtheta_lr   theta_b   dtheta_b
lqr_Q = np.diag([100,   50,    1000,   50,     100,          5,           100,        5,        20000,      5]).astype(float)

#        T_wl  T_wr  T_bl  T_br
lqr_R = np.diag([150, 150,  50,   50]).astype(float)

# %%%%%%%%%%%%%%%%%%%%%Step 3：拟合ABK矩阵，得到多项式系数%%%%%%%%%%%%%%%%%%%%%

L0_l = 0.10
L0_r = 0.10

LM = np.arange(0.00, 0.20 + 1e-9, 0.01)
step_max = len(LM)

sample_size = step_max**2

sample_l = np.zeros(sample_size)                              # 每个网格点的 l_l，m
sample_r = np.zeros(sample_size)                              # 每个网格点的 l_r，m
K_sample = np.zeros((sample_size, N_INPUT*N_STATE))           # K  (4x10)  行序 l*10+m
Ad_sample = np.zeros((sample_size, N_STATE*N_STATE))          # Ad (10x10) 行序 i*10+j
Bd_sample = np.zeros((sample_size, N_STATE*N_INPUT))          # Bd (10x4)  行序 i*4+j
L_sample = np.zeros((sample_size, (N_STATE + N_DIST)*N_STATE))  # L (14x10) 行序 i*10+j

for l_step in range(step_max):
    for r_step in range(step_max):

        index = l_step*step_max + r_step

        l_l_ac = L0_l + LM[l_step]
        l_r_ac = L0_r + LM[r_step]

        if USE_MEASURED_TABLE:
            lw_l_ac = interp1(Leg_data_l[:, 0], Leg_data_l[:, 1], l_l_ac)
            lb_l_ac = interp1(Leg_data_l[:, 0], Leg_data_l[:, 2], l_l_ac)
            Il_l_ac = interp1(Leg_data_l[:, 0], Leg_data_l[:, 3], l_l_ac)
            lw_r_ac = interp1(Leg_data_r[:, 0], Leg_data_r[:, 1], l_r_ac)
            lb_r_ac = interp1(Leg_data_r[:, 0], Leg_data_r[:, 2], l_r_ac)
            Il_r_ac = interp1(Leg_data_r[:, 0], Leg_data_r[:, 3], l_r_ac)
        else:
            Il_l_ac = 0.4*l_l_ac + 0.07
            lb_l_ac = 0.218*l_l_ac + 0.075
            lw_l_ac = 0.782*l_l_ac - 0.075
            Il_r_ac = 0.4*l_r_ac + 0.07
            lb_r_ac = 0.218*l_r_ac + 0.075
            lw_r_ac = 0.782*l_r_ac - 0.075

        if USE_IDILE_MODEL:
            I_w_ac = 0.5 * m_w_ac * R_w_ac ** 2
            I_b_ac = m_b_ac*(0.415**2+0.16**2)/12.0
            I_z_ac = m_b_ac*(0.415**2+0.260**2)/12.0
        else:
            I_w_ac = I_w_ac
            I_b_ac = I_b_ac
            I_z_ac = I_z_ac


        A_ac, B_ac = subs_AB(
            (R_w_ac, R_l_ac, l_l_ac, l_r_ac, lw_l_ac, lw_r_ac, lb_l_ac, lb_r_ac,
             l_c_ac, m_w_ac, m_l_ac, m_b_ac, I_w_ac, Il_l_ac, Il_r_ac,
             I_b_ac, I_z_ac, g_ac))

        Ad_ac, Bd_ac = c2d(A_ac, B_ac, Ts_ac)
        K = dlqr(Ad_ac, Bd_ac, lqr_Q, lqr_R)
        _, _, L_obs = leso_gain(Ad_ac, Bd_ac, leso_state_pole, leso_dist_pole)

        sample_l[index] = l_l_ac
        sample_r[index] = l_r_ac
        K_sample[index, :] = K.ravel()
        Ad_sample[index, :] = Ad_ac.ravel()
        Bd_sample[index, :] = Bd_ac.ravel()
        L_sample[index, :] = L_obs.ravel()

K_Fit_Coefficients = fit_poly22(sample_l, sample_r, K_sample)
Ad_Fit_Coefficients = fit_poly22(sample_l, sample_r, Ad_sample)
Bd_Fit_Coefficients = fit_poly22(sample_l, sample_r, Bd_sample)
L_Fit_Coefficients = fit_poly22(sample_l, sample_r, L_sample)

# %%%%%%%%%%%%%%%%%%%%%Step 4：输出可粘贴的系数块%%%%%%%%%%%%%%%%%%%%%
# 每块 6 列，列序 p00 p10 p01 p20 p11 p02，输入 X = l_l、Y = l_r，单位 m。
# B_e = [Bd; 0]、A_e = [[Ad, Bd], [0, I]] 均可由 Ad/Bd 现场拼出，不重复导出。

print_block('K_Fit_Coefficients   K(4x10)   40 行，行序 输出l*10+状态m', K_Fit_Coefficients)
# print_block('Ad_Fit_Coefficients  Ad(10x10) 100 行，行序 i*10+j', Ad_Fit_Coefficients)
# print_block('Bd_Fit_Coefficients  Bd(10x4)  40 行，行序 i*4+j', Bd_Fit_Coefficients)
# print_block('L_Fit_Coefficients   L(14x10)  140 行，行序 i*10+j', L_Fit_Coefficients)

# %%%%%%%%%%%%%%%%%%%%%Step 5：自检%%%%%%%%%%%%%%%%%%%%%

# 取网格中点腿长重算一次 LESO，验证观测器闭环极点落在设计值上
l_mid = L0_l + LM[step_max//2]
Il_mid = 0.4*l_mid + 0.07
lb_mid = 0.218*l_mid + 0.075
lw_mid = 0.782*l_mid - 0.075
A_mid, B_mid = subs_AB(
    (R_w_ac, R_l_ac, l_mid, l_mid, lw_mid, lw_mid, lb_mid, lb_mid,
     l_c_ac, m_w_ac, m_l_ac, m_b_ac, I_w_ac, Il_mid, Il_mid,
     I_b_ac, I_z_ac, g_ac))
Ad_mid, Bd_mid = c2d(A_mid, B_mid, Ts_ac)
A_e_mid, C_e_mid, L_mid = leso_gain(Ad_mid, Bd_mid, leso_state_pole, leso_dist_pole)
obs_eig = np.sort(np.abs(np.linalg.eigvals(A_e_mid - L_mid @ C_e_mid)))

print(f'=== 自检（l_l = l_r = {l_mid:.3f} m，Ts = {Ts_ac} s）===')
print('LESO 闭环极点模（应为 10 个 {:.3f} + 4 个 {:.3f}）：'.format(leso_state_pole, leso_dist_pole))
print('   ' + '  '.join(f'{v:.6f}' for v in obs_eig))
print(f'|L|max = {np.abs(L_mid).max():.4f}')

basis = poly22_basis(sample_l, sample_r)
for title, coefficients, sample in (('K ', K_Fit_Coefficients, K_sample),
                                    ('Ad', Ad_Fit_Coefficients, Ad_sample),
                                    ('Bd', Bd_Fit_Coefficients, Bd_sample),
                                    ('L ', L_Fit_Coefficients, L_sample)):
    residual = np.abs(basis @ coefficients.T - sample)
    print(f'{title} poly22 最大绝对残差 = {residual.max():.3e}'
          f'，最大量级 = {np.abs(sample).max():.3e}')
