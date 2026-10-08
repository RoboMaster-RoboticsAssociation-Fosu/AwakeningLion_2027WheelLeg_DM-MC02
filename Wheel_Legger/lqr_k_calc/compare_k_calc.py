#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compare the actual ABK/HKU models and discrete LQR paths without file output.

Checks all five symbolic equations, independent A/B assembly, discrete gains,
normalized DARE residuals, sampled closed-loop poles, and poly22 conventions.
Continuous CARE gains are reported only as a reference for the design change.
The ABK parameter branches are executed from its source, without its fit loop.
"""
import importlib.util
import os
import sys
import textwrap

# Importing the sibling script must not create or update __pycache__.
sys.dont_write_bytecode = True

import numpy as np
import sympy as sp
from scipy.linalg import solve_continuous_are, solve_discrete_are

HERE = os.path.dirname(os.path.abspath(__file__))
ABK_PATH = os.path.join(os.path.dirname(HERE), 'ABK_LQR.py')
HKU_PATH = os.path.join(HERE, 'hku_lqr_k_calc.py')

with open(ABK_PATH, encoding='utf-8-sig') as source_file:
    abk_source = source_file.read()
parameter_end = abk_source.index('# %%%%%%%%%%%%%%%%%%%%%Step 3')
ns = {'__name__': 'abk_comparison'}
exec(compile(abk_source[:parameter_end], ABK_PATH, 'exec'), ns)

# Read the actual fit grid and the actual mechanical-data selection formulas.
grid_start = abk_source.index('L0_l =', parameter_end)
grid_end = abk_source.index('sample_l =', grid_start)
exec(compile(abk_source[grid_start:grid_end], ABK_PATH, 'exec'), ns)
branch_start = abk_source.index('        if USE_MEASURED_TABLE:', grid_end)
branch_end = abk_source.index('        A_ac, B_ac = subs_AB(', branch_start)
parameter_branch = compile(
    textwrap.dedent(abk_source[branch_start:branch_end]), ABK_PATH, 'exec')

spec = importlib.util.spec_from_file_location('hku', HKU_PATH)
hku = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hku)

# HKU symbols -> ABK symbols. Both state and input orders are unchanged.
smap = {
    hku.R_w: ns['R_w'], hku.R_l: ns['R_l'],
    hku.l_l: ns['l_l'], hku.l_r: ns['l_r'],
    hku.l_wl: ns['lw_l'], hku.l_wr: ns['lw_r'],
    hku.l_bl: ns['lb_l'], hku.l_br: ns['lb_r'],
    hku.l_c: ns['l_c'], hku.m_w: ns['m_w'],
    hku.m_l: ns['m_l'], hku.m_b: ns['m_b'],
    hku.I_w: ns['I_w'], hku.I_ll: ns['Il_l'], hku.I_lr: ns['Il_r'],
    hku.I_b: ns['I_b'], hku.I_z: ns['I_z'], hku.g: ns['g'],
    hku.ddtheta_wl: ns['ddtheta_w_l'], hku.ddtheta_wr: ns['ddtheta_w_r'],
    hku.ddtheta_ll: ns['ddtheta_l_l'], hku.ddtheta_lr: ns['ddtheta_l_r'],
    hku.ddtheta_b: ns['ddtheta_b'],
    hku.theta_ll: ns['theta_l_l'], hku.theta_lr: ns['theta_l_r'],
    hku.theta_b: ns['theta_b'],
    hku.T_wl: ns['T_lw_l'], hku.T_wr: ns['T_lw_r'],
    hku.T_bl: ns['T_bl_l'], hku.T_br: ns['T_bl_r'],
}
mine_eqs = hku.build_equations()
their_eqs = [ns[f'eqn{i}'] for i in range(1, 6)]
print('[1] Symbolic dynamics: HKU - ABK')
for index, (mine, theirs) in enumerate(zip(mine_eqs, their_eqs), 1):
    difference = sp.simplify(mine.subs(smap) - theirs)
    assert difference == 0, f'eqn{index} differs: {difference}'
    print(f'    eqn{index}: identical')

M, G, B_raw = hku.extract_linear_form(mine_eqs)
Mf, Gf, Bf = hku.make_jacobian_functions(M, G, B_raw)


def my_AB(params):
    JA, JB = hku.jacobians_numeric(params, Mf, Gf, Bf)
    return hku.fill_AB(JA, JB, params['R_w'], params['R_l'],
                       params['l_l'], params['l_r'])


def their_AB(params):
    order = ('R_w', 'R_l', 'l_l', 'l_r', 'l_wl', 'l_wr', 'l_bl', 'l_br',
             'l_c', 'm_w', 'm_l', 'm_b', 'I_w', 'I_ll', 'I_lr', 'I_b', 'I_z', 'g')
    return ns['subs_AB'](tuple(params[name] for name in order))


def abk_params(left, right):
    values = dict(ns)
    values.update(l_l_ac=left, l_r_ac=right)
    exec(parameter_branch, values)
    names = {
        'R_w': 'R_w_ac', 'R_l': 'R_l_ac', 'l_l': 'l_l_ac', 'l_r': 'l_r_ac',
        'l_wl': 'lw_l_ac', 'l_wr': 'lw_r_ac', 'l_bl': 'lb_l_ac', 'l_br': 'lb_r_ac',
        'l_c': 'l_c_ac', 'm_w': 'm_w_ac', 'm_l': 'm_l_ac', 'm_b': 'm_b_ac',
        'I_w': 'I_w_ac', 'I_ll': 'Il_l_ac', 'I_lr': 'Il_r_ac',
        'I_b': 'I_b_ac', 'I_z': 'I_z_ac', 'g': 'g_ac',
    }
    return {name: values[source] for name, source in names.items()}


def hku_params(left=None, right=None):
    params = (hku.formula_inertia(hku.PARAMS) if hku.USE_FORMULA_INERTIA
              else dict(hku.PARAMS))
    if left is not None:
        params['l_l'] = left
    if right is not None:
        params['l_r'] = right
    for side in ('l', 'r'):
        lw, lb, inertia = hku.leg_params_at(params[f'l_{side}'])
        params.update({f'l_w{side}': lw, f'l_b{side}': lb, f'I_l{side}': inertia})
    return params


def assert_AB_equal(params):
    mine_A, mine_B = my_AB(params)
    their_A, their_B = their_AB(params)
    np.testing.assert_allclose(mine_A, their_A, rtol=1e-10, atol=1e-10)
    np.testing.assert_allclose(mine_B, their_B, rtol=1e-10, atol=1e-10)
    return mine_A, mine_B, their_A, their_B


Q_abk = ns['lqr_Q']
R_abk = ns['lqr_R']
Ts_abk = ns['Ts_ac']
assert hku.Ts == Ts_abk, 'The two scripts have different sample periods'
Q_hku = hku.Q_LQR
R_hku = hku.R_LQR
abk_left = ns['L0_l'] + ns['LM'][len(ns['LM']) // 2]
abk_right = ns['L0_r'] + ns['LM'][len(ns['LM']) // 2]
cases = (
    ('ABK actual defaults / grid center', abk_params(abk_left, abk_right), Q_abk, R_abk),
    ('HKU actual fixed-leg defaults', hku_params(), Q_hku, R_hku),
)
print('\n[2] Effective parameters, weights, and independent A/B assembly')
print(f'    sample period = {hku.Ts:g} s')
for label, params, Q, R in cases:
    A, B, A_ref, B_ref = assert_AB_equal(params)
    print(f'    {label}:')
    print('      params: ' + ', '.join(f'{key}={value:.6g}' for key, value in params.items()))
    print(f'      Q={np.diag(Q)}, R={np.diag(R)}')
    print(f'      max|dA|={np.max(np.abs(A-A_ref)):.3e}, '
          f'max|dB|={np.max(np.abs(B-B_ref)):.3e}')


def verify_discrete(params, Q, R):
    A, B, A_ref, B_ref = assert_AB_equal(params)
    K, residual_abs, radius = hku.solve_lqr(A, B, Q, R)
    Ad_ref, Bd_ref = ns['c2d'](A_ref, B_ref, Ts_abk)
    K_ref = ns['dlqr'](Ad_ref, Bd_ref, Q, R)
    np.testing.assert_allclose(K, K_ref, rtol=1e-8, atol=1e-8)
    P_ref = solve_discrete_are(Ad_ref, Bd_ref, Q, R)
    gain_rhs = Bd_ref.T @ P_ref @ Ad_ref
    dare_residual = (Ad_ref.T @ P_ref @ Ad_ref - P_ref
                     - Ad_ref.T @ P_ref @ Bd_ref @ np.linalg.solve(
                         R + Bd_ref.T @ P_ref @ Bd_ref, gain_rhs) + Q)
    residual_scale = 1.0 + np.max(np.abs(P_ref))
    normalized = float(residual_abs / residual_scale)
    reference_normalized = float(np.max(np.abs(dare_residual)) / residual_scale)
    assert np.isfinite(normalized) and normalized <= 1e-10, normalized
    assert reference_normalized <= 1e-10, reference_normalized
    radius_ref = float(np.max(np.abs(np.linalg.eigvals(Ad_ref - Bd_ref @ K_ref))))
    assert np.isfinite(radius) and radius < 1.0, radius
    np.testing.assert_allclose(radius, radius_ref, rtol=1e-10, atol=1e-10)
    return K, float(np.max(np.abs(K - K_ref))), normalized, float(radius)


print('\n[3] Discrete LQR agrees with ABK dlqr using the same actual Q/R')
for label, params, Q, R in cases:
    K, difference, residual, radius = verify_discrete(params, Q, R)
    A, B = my_AB(params)
    P_continuous = solve_continuous_are(A, B, Q, R)
    K_continuous = np.linalg.solve(R, B.T @ P_continuous)
    design_difference = float(np.max(np.abs(K - K_continuous)))
    print(f'    {label}: max|dK|={difference:.3e}, '
          f'normalized DARE={residual:.3e}, rho={radius:.9f}')
    print(f'      continuous CARE reference: max|Kd-Kc|={design_difference:.6g}')

# Compare tuning independently of the continuous/discrete design choice.
params = cases[0][1]
K_abk_weights = verify_discrete(params, Q_abk, R_abk)[0]
K_hku_weights = verify_discrete(params, Q_hku, R_hku)[0]
print(f'    Q/R difference on the same model: '
      f'max|dK|={np.max(np.abs(K_abk_weights-K_hku_weights)):.6g}')

grid = hku.leg_grid()
max_gain_difference = 0.0
max_residual = 0.0
max_radius = 0.0
K_samples = []
coordinates = []
for left in grid:
    for right in grid:
        params = hku_params(left[0], right[0])
        K, difference, residual, radius = verify_discrete(params, Q_hku, R_hku)
        max_gain_difference = max(max_gain_difference, difference)
        max_residual = max(max_residual, residual)
        max_radius = max(max_radius, radius)
        K_samples.append(K.ravel())
        coordinates.append((left[0], right[0]))
print(f'    Full HKU grid: {len(grid)} x {len(grid)} = {len(K_samples)} cases PASS')
print(f'      max|dK|={max_gain_difference:.3e}, '
      f'max normalized DARE={max_residual:.3e}, max rho={max_radius:.9f}')

print('\n[4] Poly22 basis, row-major flattening, and least squares')
coordinates = np.asarray(coordinates)
x, y = coordinates[:, 0], coordinates[:, 1]
samples = np.asarray(K_samples)
coeff_abk = ns['fit_poly22'](x, y, samples)
basis = np.column_stack([np.ones_like(x), x, y, x**2, x*y, y**2])
np.testing.assert_allclose(basis, ns['poly22_basis'](x, y), rtol=0.0, atol=0.0)
coeff_hku, residuals = hku.fit_all_coefficients(
    hku_params(), Mf, Gf, Bf, Q_hku, R_hku, grid, want=('K',))['K']
np.testing.assert_allclose(coeff_hku, coeff_abk, rtol=1e-8, atol=1e-8)
np.testing.assert_allclose(residuals, np.abs(basis @ coeff_hku.T - samples),
                           rtol=1e-8, atol=1e-8)
for (left, right), flattened in zip(coordinates, samples):
    expected = np.array([1.0, left, right, left**2, left*right, right**2]) @ coeff_hku.T
    np.testing.assert_allclose(hku.eval_fit(coeff_hku, left, right).ravel(),
                               expected, rtol=1e-10, atol=1e-10)
    assert flattened.shape == (40,)
print(f'    full-grid coefficient max difference = '
      f'{np.max(np.abs(coeff_hku-coeff_abk)):.3e}')
print('    basis = [1, left, right, left^2, left*right, right^2]')
print('    K = (4, 10), C row-major input/state order; all checks PASS')
