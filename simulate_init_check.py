#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模拟无人机3点组合的初始化流程（纯Python版本）"""

import math

# 常量
pi = math.pi
INIT_SIGMA_A = 1.0  # 方位测量噪声（度）
INIT_SIGMA_B = 1.0  # 俯仰测量噪声（度）
INIT_SIGMA_R = 20.0  # 距离测量噪声（m）
TRACK_INIT_TH = 15.0
ANGLE_CHANGE_THRESHOLD = 160.0

# 矩阵运算辅助函数
def mat_vec(matrix, vector):
    result = []
    for row in matrix:
        s = 0.0
        for i in range(len(vector)):
            s += row[i] * vector[i]
        result.append(s)
    return result

def mat_mul(A, B):
    n = len(A)
    m = len(B[0])
    p = len(B)
    C = [[0.0]*m for _ in range(n)]
    for i in range(n):
        for j in range(m):
            s = 0.0
            for k in range(p):
                s += A[i][k] * B[k][j]
            C[i][j] = s
    return C

def mat_add(A, B):
    n = len(A)
    m = len(A[0])
    C = [[A[i][j] + B[i][j] for j in range(m)] for i in range(n)]
    return C

def mat_transpose(A):
    n = len(A)
    m = len(A[0])
    return [[A[i][j] for i in range(n)] for j in range(m)]

def mat_inv_3x3(A):
    """3x3矩阵求逆"""
    a = A[0][0]; b = A[0][1]; c = A[0][2]
    d = A[1][0]; e = A[1][1]; f = A[1][2]
    g = A[2][0]; h = A[2][1]; i = A[2][2]
    
    det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g)
    if abs(det) < 1e-10:
        # 奇异矩阵，返回伪逆
        return [[1e10 if i==j else 0.0 for j in range(3)] for i in range(3)]
    
    inv_det = 1.0 / det
    return [
        [(e*i - f*h)*inv_det, -(b*i - c*h)*inv_det, (b*f - c*e)*inv_det],
        [-(d*i - f*g)*inv_det, (a*i - c*g)*inv_det, -(a*f - c*d)*inv_det],
        [(d*h - e*g)*inv_det, -(a*h - b*g)*inv_det, (a*e - b*d)*inv_det]
    ]

def vec_dot(a, b):
    return sum(x*y for x,y in zip(a,b))

def vec_norm(v):
    return math.sqrt(sum(x*x for x in v))

def vec_sub(a, b):
    return [x-y for x,y in zip(a,b)]

def vec_add(a, b):
    return [x+y for x,y in zip(a,b)]

def vec_scale(v, s):
    return [x*s for x in v]

# 无人机3点数据
points = [
    {'t': 898796, 'beam': 2, 'az': 16.8225, 'el': 5.1668, 'r': 1094.84, 'vr': -11.7406},
    {'t': 898960, 'beam': 3, 'az': 17.3911, 'el': 4.7861, 'r': 1095.84, 'vr': -11.7406},
    {'t': 901584, 'beam': 2, 'az': 16.5781, 'el': 4.6226, 'r': 1124.10, 'vr': -11.7406},
]

def sph2cart(r, az_deg, el_deg):
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    x = r * math.cos(el) * math.cos(az)
    y = r * math.cos(el) * math.sin(az)
    z = r * math.sin(el)
    return [x, y, z]

def cart2sph(x, y, z):
    r = math.sqrt(x*x + y*y + z*z)
    az = math.atan2(y, x) * 180 / pi
    el = math.asin(z / r) * 180 / pi
    return r, az, el

print("="*80)
print("【1】无人机3点数据")
print("="*80)
for i, p in enumerate(points):
    pos = sph2cart(p['r'], p['az'], p['el'])
    print(f"  Point {i} (Frame {p['t']}ms): beam={p['beam']}, Az={p['az']:.4f}°, El={p['el']:.4f}°, R={p['r']:.2f}m, Vr={p['vr']:.4f}m/s")
    print(f"    直角坐标: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")

dt1 = (points[1]['t'] - points[0]['t']) / 1000.0
dt2 = (points[2]['t'] - points[1]['t']) / 1000.0
print(f"\n时间间隔: Frame4→5: {dt1:.3f}s, Frame5→21: {dt2:.3f}s")

print("\n" + "="*80)
print("【2】模拟KF初始化（Frame 4 + Frame 5）")
print("="*80)

pos0 = sph2cart(points[0]['r'], points[0]['az'], points[0]['el'])
pos1 = sph2cart(points[1]['r'], points[1]['az'], points[1]['el'])
pos2 = sph2cart(points[2]['r'], points[2]['az'], points[2]['el'])

# 位置差分速度
v_diff_01 = vec_scale(vec_sub(pos1, pos0), 1.0/dt1)
print(f"\n位置差分速度 (Frame4→5): vx={v_diff_01[0]:.2f}, vy={v_diff_01[1]:.2f}, vz={v_diff_01[2]:.2f} m/s")
spd_diff = vec_norm(v_diff_01)
print(f"  速度大小: {spd_diff:.2f} m/s")

# 用雷达Vr修正速度
r_vr = points[1]['vr']
r_el = math.radians(points[1]['el'])
r_az = math.radians(points[1]['az'])
er_x = math.cos(r_el) * math.cos(r_az)
er_y = math.cos(r_el) * math.sin(r_az)
er_z = math.sin(r_el)

vx_old = v_diff_01[0]
vy_old = v_diff_01[1]
vz_old = v_diff_01[2]
v_radial_old = vx_old * er_x + vy_old * er_y + vz_old * er_z
print(f"\nKF速度在雷达径向方向的投影: {v_radial_old:.2f} m/s")
print(f"  雷达实测Vr: {r_vr:.2f} m/s")

vx_tan = vx_old - v_radial_old * er_x
vy_tan = vy_old - v_radial_old * er_y
vz_tan = vz_old - v_radial_old * er_z
print(f"  切向速度分量: vx_tan={vx_tan:.2f}, vy_tan={vy_tan:.2f}, vz_tan={vz_tan:.2f}")

tan_damp = 0.08
vx_corrected = r_vr * er_x + vx_tan * tan_damp
vy_corrected = r_vr * er_y + vy_tan * tan_damp
vz_corrected = r_vr * er_z + vz_tan * tan_damp

print(f"\n修正后KF速度: vx={vx_corrected:.2f}, vy={vy_corrected:.2f}, vz={vz_corrected:.2f}")
spd_corrected = vec_norm([vx_corrected, vy_corrected, vz_corrected])
print(f"  速度大小: {spd_corrected:.2f} m/s")

# Frame 5的KF状态
X_kf = [pos1[0], vx_corrected, pos1[1], vy_corrected, pos1[2], vz_corrected]
print(f"\nFrame 5 KF状态:")
print(f"  X = [{X_kf[0]:.2f}, {X_kf[1]:.2f}, {X_kf[2]:.2f}, {X_kf[3]:.2f}, {X_kf[4]:.2f}, {X_kf[5]:.2f}]")

# 初始化协方差P
P_kf = [
    [400.0, 0, 0, 0, 0, 0],
    [0, 100.0, 0, 0, 0, 0],
    [0, 0, 400.0, 0, 0, 0],
    [0, 0, 0, 100.0, 0, 0],
    [0, 0, 0, 0, 400.0, 0],
    [0, 0, 0, 0, 0, 100.0],
]
print(f"\n初始协方差P对角线: [{P_kf[i][i]:.1f} for i in range(6)]")

print("\n" + "="*80)
print("【3】预测到Frame 21并计算马氏距离")
print("="*80)

T = dt2  # 2.624s
# 状态转移矩阵F
F = [
    [1, T, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, T, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, T],
    [0, 0, 0, 0, 0, 1],
]

# 过程噪声GQG
sigma_a_local = 10.0
T_nominal = 0.04
T_ratio = min(max(T / T_nominal, 0.5), 5.0)
sa2 = sigma_a_local**2 * T_ratio
print(f"\n过程噪声参数: sigma_a_local={sigma_a_local}, T_ratio={T_ratio}, sa2={sa2}")

T2 = T * T
T3 = T2 * T / 2.0
T4 = T2 * T2 / 4.0

GQG = [[0.0]*6 for _ in range(6)]
GQG[0][0] = sa2 * T4; GQG[0][1] = sa2 * T3
GQG[1][0] = sa2 * T3; GQG[1][1] = sa2 * T2
GQG[2][2] = sa2 * T4; GQG[2][3] = sa2 * T3
GQG[3][2] = sa2 * T3; GQG[3][3] = sa2 * T2
GQG[4][4] = sa2 * T4; GQG[4][5] = sa2 * T3
GQG[5][4] = sa2 * T3; GQG[5][5] = sa2 * T2
print(f"GQG对角线: [{GQG[i][i]:.1f} for i in range(6)]")

# 预测状态
X_predict = mat_vec(F, X_kf)
print(f"\n预测状态 Frame 21:")
print(f"  X_predict = [{X_predict[0]:.2f}, {X_predict[1]:.2f}, {X_predict[2]:.2f}, {X_predict[3]:.2f}, {X_predict[4]:.2f}, {X_predict[5]:.2f}]")

# 预测协方差
F_trans = mat_transpose(F)
FP = mat_mul(F, P_kf)
FPF = mat_mul(FP, F_trans)
P_predict = mat_add(FPF, GQG)
print(f"\n预测协方差P_predict对角线: [{P_predict[i][i]:.1f} for i in range(6)]")

# 实际测量位置
Z_obs = pos2
print(f"\n实际测量 Frame 21: x={Z_obs[0]:.2f}, y={Z_obs[1]:.2f}, z={Z_obs[2]:.2f}")

# 新息
Z_D = vec_sub(Z_obs, [X_predict[0], X_predict[2], X_predict[4]])
print(f"\n残差 Z_D: dx={Z_D[0]:.2f}, dy={Z_D[1]:.2f}, dz={Z_D[2]:.2f}")
print(f"  残差大小: {vec_norm(Z_D):.2f} m")

# 测量噪声
rho, theta, eps = cart2sph(Z_obs[0], Z_obs[1], Z_obs[2])
ct = math.cos(theta); st = math.sin(theta)
ce = math.cos(eps); se = math.sin(eps)
sr = INIT_SIGMA_R
sa = INIT_SIGMA_A * pi / 180
sb = INIT_SIGMA_B * pi / 180

dxdr = ct * ce; dxdt = -rho * st * ce; dxde = -rho * ct * se
dydr = st * ce; dydt = rho * ct * ce; dyde = -rho * st * se
dzdr = se;     dzdt = 0.0;             dzde = rho * ce

R_meas = [[0.0]*3 for _ in range(3)]
R_meas[0][0] = dxdr**2 * sr**2 + dxdt**2 * sa**2 + dxde**2 * sb**2
R_meas[1][1] = dydr**2 * sr**2 + dydt**2 * sa**2 + dyde**2 * sb**2
R_meas[2][2] = dzdr**2 * sr**2 + dzdt**2 * sa**2 + dzde**2 * sb**2
R_meas[0][1] = R_meas[1][0] = dxdr*dydr*sr**2 + dxdt*dydt*sa**2 + dxde*dyde*sb**2
R_meas[0][2] = R_meas[2][0] = dxdr*dzdr*sr**2 + dxdt*dzdt*sa**2 + dxde*dzde*sb**2
R_meas[1][2] = R_meas[2][1] = dydr*dzdr*sr**2 + dydt*dzdt*sa**2 + dyde*dzde*sb**2

print(f"\n测量噪声R对角线: [{R_meas[i][i]:.2f} for i in range(3)]")

# 观测矩阵H
H = [
    [1, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0],
]

# 新息协方差S
H_trans = mat_transpose(H)
HP = mat_mul(H, P_predict)
HPH = mat_mul(HP, H_trans)
S = mat_add(HPH, R_meas)
print(f"\n新息协方差S对角线: [{S[i][i]:.2f} for i in range(3)]")

# 马氏距离
S_inv = mat_inv_3x3(S)
Z_D_trans = [[Z_D[0], Z_D[1], Z_D[2]]]
Z_D_trans_S_inv = mat_mul(Z_D_trans, S_inv)
dist_maha2 = Z_D_trans_S_inv[0][0]*Z_D[0] + Z_D_trans_S_inv[0][1]*Z_D[1] + Z_D_trans_S_inv[0][2]*Z_D[2]
dist_maha = math.sqrt(max(dist_maha2, 0))
print(f"\n马氏距离²: {dist_maha2:.2f}")
print(f"马氏距离: {dist_maha:.2f}")
print(f"TRACK_INIT_TH: {TRACK_INIT_TH}")
print(f"结果: {'通过' if dist_maha < TRACK_INIT_TH else '失败'}")

# 角度变化检查
print("\n" + "="*80)
print("【4】角度变化检查")
print("="*80)

v1 = vec_sub(pos1, pos0)
v2 = vec_sub(pos2, pos1)
cos_angle = vec_dot(v1, v2) / (vec_norm(v1) * vec_norm(v2) + 1e-10)
cos_angle = max(-1.0, min(1.0, cos_angle))
angle = math.acos(cos_angle) * 180 / pi

print(f"Frame4→5 向量: {v1}")
print(f"Frame5→21 向量: {v2}")
print(f"夹角: {angle:.2f}°")
print(f"ANGLE_CHANGE_THRESHOLD: {ANGLE_CHANGE_THRESHOLD}°")
print(f"结果: {'通过' if angle <= ANGLE_CHANGE_THRESHOLD else '失败'}")

# 综合判断
print("\n" + "="*80)
print("【5】综合判断")
print("="*80)
maha_pass = dist_maha < TRACK_INIT_TH
angle_pass = angle <= ANGLE_CHANGE_THRESHOLD
print(f"马氏距离检查: {'通过' if maha_pass else '失败'} ({dist_maha:.2f} < {TRACK_INIT_TH})")
print(f"角度变化检查: {'通过' if angle_pass else '失败'} ({angle:.2f}° ≤ {ANGLE_CHANGE_THRESHOLD}°)")
print(f"最终结果: {'3点组合成功建立' if (maha_pass and angle_pass) else '3点组合失败'}")

# 敏感性分析
print("\n" + "="*80)
print("【6】敏感性分析：速度估计误差对马氏距离的影响")
print("="*80)

for vx_err in [0, 5, 10, 15, 20]:
    X_kf_test = X_kf[:]
    X_kf_test[1] += vx_err
    X_pred_test = mat_vec(F, X_kf_test)
    Z_D_test = [Z_obs[0]-X_pred_test[0], Z_obs[1]-X_pred_test[2], Z_obs[2]-X_pred_test[4]]
    Z_D_trans_test = [[Z_D_test[0], Z_D_test[1], Z_D_test[2]]]
    Z_D_trans_S_inv_test = mat_mul(Z_D_trans_test, S_inv)
    d2_test = sum(Z_D_trans_S_inv_test[0][i]*Z_D_test[i] for i in range(3))
    d_test = math.sqrt(max(d2_test, 0))
    print(f"  速度误差 vx+{vx_err:2d} m/s: 马氏距离={d_test:.2f} {'通过' if d_test < TRACK_INIT_TH else '失败'}")

print("\n" + "="*80)
print("【7】敏感性分析：增大过程噪声的影响")
print("="*80)

for sigma_mult in [1.0, 2.0, 3.0, 5.0, 10.0]:
    sa2_test = sigma_a_local**2 * T_ratio * sigma_mult
    GQG_test = [[0.0]*6 for _ in range(6)]
    GQG_test[0][0] = sa2_test * T4; GQG_test[0][1] = sa2_test * T3
    GQG_test[1][0] = sa2_test * T3; GQG_test[1][1] = sa2_test * T2
    GQG_test[2][2] = sa2_test * T4; GQG_test[2][3] = sa2_test * T3
    GQG_test[3][2] = sa2_test * T3; GQG_test[3][3] = sa2_test * T2
    GQG_test[4][4] = sa2_test * T4; GQG_test[4][5] = sa2_test * T3
    GQG_test[5][4] = sa2_test * T3; GQG_test[5][5] = sa2_test * T2
    
    P_predict_test = mat_add(FPF, GQG_test)
    S_test = mat_add(mat_mul(mat_mul(H, P_predict_test), H_trans), R_meas)
    S_inv_test = mat_inv_3x3(S_test)
    Z_D_trans_test = [[Z_D[0], Z_D[1], Z_D[2]]]
    Z_D_trans_S_inv_test = mat_mul(Z_D_trans_test, S_inv_test)
    d2_test = sum(Z_D_trans_S_inv_test[0][i]*Z_D[i] for i in range(3))
    d_test = math.sqrt(max(d2_test, 0))
    print(f"  sigma_a_local×{sigma_mult:.0f}: 马氏距离={d_test:.2f} {'通过' if d_test < TRACK_INIT_TH else '失败'}")

print("\n" + "="*80)
print("【8】敏感性分析：增大INIT_SIGMA_R的影响")
print("="*80)

for sr_mult in [1, 2, 3, 5, 10]:
    sr_test = INIT_SIGMA_R * sr_mult
    R_test = [[0.0]*3 for _ in range(3)]
    R_test[0][0] = dxdr**2 * sr_test**2 + dxdt**2 * sa**2 + dxde**2 * sb**2
    R_test[1][1] = dydr**2 * sr_test**2 + dydt**2 * sa**2 + dyde**2 * sb**2
    R_test[2][2] = dzdr**2 * sr_test**2 + dzdt**2 * sa**2 + dzde**2 * sb**2
    R_test[0][1] = R_test[1][0] = dxdr*dydr*sr_test**2 + dxdt*dydt*sa**2 + dxde*dyde*sb**2
    R_test[0][2] = R_test[2][0] = dxdr*dzdr*sr_test**2 + dxdt*dzdt*sa**2 + dxde*dzde*sb**2
    R_test[1][2] = R_test[2][1] = dydr*dzdr*sr_test**2 + dydt*dzdt*sa**2 + dyde*dzde*sb**2
    
    S_test = mat_add(HPH, R_test)
    S_inv_test = mat_inv_3x3(S_test)
    Z_D_trans_test = [[Z_D[0], Z_D[1], Z_D[2]]]
    Z_D_trans_S_inv_test = mat_mul(Z_D_trans_test, S_inv_test)
    d2_test = sum(Z_D_trans_S_inv_test[0][i]*Z_D[i] for i in range(3))
    d_test = math.sqrt(max(d2_test, 0))
    print(f"  INIT_SIGMA_R×{sr_mult:2d}: S对角线={[f'{S_test[i][i]:.1f}' for i in range(3)]}, 马氏距离={d_test:.2f} {'通过' if d_test < TRACK_INIT_TH else '失败'}")

print("\n分析完成！")
