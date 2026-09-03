# -*- coding: utf-8 -*-
"""
模拟track_initial.c中get_asso_info2_dist的马氏距离计算
诊断无人机3点组合失败的根因 - 纯Python实现
"""
import math

# 参数定义
INIT_SIGMA_A = 1.0   # 方位测量噪声（度）
INIT_SIGMA_B = 1.0   # 俯仰测量噪声（度）
INIT_SIGMA_R = 20.0  # 距离测量噪声（m）
TRACK_INIT_TH = 15.0
pi = math.pi

# 矩阵运算辅助函数
def mat_mult(A, B):
    """矩阵乘法"""
    n = len(A)
    m = len(B[0])
    p = len(B)
    C = [[0.0]*m for _ in range(n)]
    for i in range(n):
        for j in range(m):
            for k in range(p):
                C[i][j] += A[i][k] * B[k][j]
    return C

def mat_add(A, B):
    """矩阵加法"""
    n = len(A)
    m = len(A[0])
    C = [[0.0]*m for _ in range(n)]
    for i in range(n):
        for j in range(m):
            C[i][j] = A[i][j] + B[i][j]
    return C

def mat_vec_mult(A, x):
    """矩阵乘向量"""
    n = len(A)
    m = len(x)
    y = [0.0]*n
    for i in range(n):
        for j in range(m):
            y[i] += A[i][j] * x[j]
    return y

def vec_dot(a, b):
    """向量点积"""
    s = 0.0
    for i in range(len(a)):
        s += a[i] * b[i]
    return s

def vec_norm(v):
    """向量模长"""
    return math.sqrt(vec_dot(v, v))

def mat_inv_3x3(M):
    """3x3矩阵求逆"""
    a, b, c = M[0]
    d, e, f = M[1]
    g, h, i = M[2]
    
    det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g)
    if abs(det) < 1e-10:
        return None
    
    inv_det = 1.0 / det
    inv = [
        [(e*i - f*h)*inv_det, -(b*i - c*h)*inv_det, (b*f - c*e)*inv_det],
        [-(d*i - f*g)*inv_det, (a*i - c*g)*inv_det, -(a*f - c*d)*inv_det],
        [(d*h - e*g)*inv_det, -(a*h - b*g)*inv_det, (a*e - b*d)*inv_det]
    ]
    return inv

def deg2rad(deg):
    return deg * pi / 180.0

# 无人机点数据
drone_points = [
    {'t': 898796, 'az': 16.82, 'el': 5.17, 'r': 1094.8, 'vr': -11.74, 'beam': 2},
    {'t': 898960, 'az': 17.39, 'el': 4.79, 'r': 1095.8, 'vr': -11.74, 'beam': 3},
    {'t': 901584, 'az': 16.58, 'el': 4.62, 'r': 1124.1, 'vr': -11.74, 'beam': 2},
]

# 球坐标转笛卡尔坐标
def sph2cart(r, az_deg, el_deg):
    az = deg2rad(az_deg)
    el = deg2rad(el_deg)
    x = r * math.cos(el) * math.cos(az)
    y = r * math.cos(el) * math.sin(az)
    z = r * math.sin(el)
    return [x, y, z]

# 计算每个点的笛卡尔坐标
for pt in drone_points:
    pt['xyz'] = sph2cart(pt['r'], pt['az'], pt['el'])

print("=" * 60)
print("无人机3点数据")
print("=" * 60)
for i, pt in enumerate(drone_points):
    print(f"  [{i}] t={pt['t']}ms Az={pt['az']:.2f}° El={pt['el']:.2f}° R={pt['r']:.1f}m Vr={pt['vr']:.2f}m/s Beam={pt['beam']}")
    xyz = pt['xyz']
    print(f"      XYZ=({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f})")

# 时间间隔
dt1 = drone_points[1]['t'] - drone_points[0]['t']  # 164ms
dt2 = drone_points[2]['t'] - drone_points[1]['t']  # 2624ms
print(f"\n时间间隔: Δt1={dt1}ms, Δt2={dt2}ms")

# 使用第2点作为预测基准
pt0 = drone_points[0]  # 第1点
pt1 = drone_points[1]  # 第2点 (tt[1])
pt2 = drone_points[2]  # 第3点 (新点)

Z0 = pt0['xyz']
Z1 = pt1['xyz']
Z2 = pt2['xyz']

print(f"\n第1点 (Z0) 笛卡尔: ({Z0[0]:.1f}, {Z0[1]:.1f}, {Z0[2]:.1f})")
print(f"第2点 (Z1) 笛卡尔: ({Z1[0]:.1f}, {Z1[1]:.1f}, {Z1[2]:.1f})")
print(f"第3点 (Z2) 笛卡尔: ({Z2[0]:.1f}, {Z2[1]:.1f}, {Z2[2]:.1f})")

# 计算位置差分速度
v_diff = [(Z1[i] - Z0[i]) / (dt1 / 1000.0) for i in range(3)]
print(f"\n位置差分速度: ({v_diff[0]:.1f}, {v_diff[1]:.1f}, {v_diff[2]:.1f}) m/s")
print(f"位置差分速度模长: {vec_norm(v_diff):.1f} m/s")

# 计算径向方向向量
er_x = math.cos(deg2rad(pt1['el'])) * math.cos(deg2rad(pt1['az']))
er_y = math.cos(deg2rad(pt1['el'])) * math.sin(deg2rad(pt1['az']))
er_z = math.sin(deg2rad(pt1['el']))
print(f"\n第2点径向方向: ({er_x:.4f}, {er_y:.4f}, {er_z:.4f})")

# 用雷达Vr修正速度
r_vr = pt1['vr']  # -11.74 m/s
v_radial_old = v_diff[0]*er_x + v_diff[1]*er_y + v_diff[2]*er_z
vx_tan = [v_diff[i] - v_radial_old * (er_x if i == 0 else er_y if i == 1 else er_z) for i in range(3)]
tan_damp = 0.15
v_corrected = [r_vr * (er_x if i == 0 else er_y if i == 1 else er_z) + tan_damp * vx_tan[i] for i in range(3)]

print(f"\n修正后速度: ({v_corrected[0]:.2f}, {v_corrected[1]:.2f}, {v_corrected[2]:.2f}) m/s")
print(f"修正后速度模长: {vec_norm(v_corrected):.2f} m/s")

# 构建状态向量X [x, vx, y, vy, z, vz]
X = [Z1[0], v_corrected[0], Z1[1], v_corrected[1], Z1[2], v_corrected[2]]
print(f"\n初始状态X: [{X[0]:.2f}, {X[1]:.2f}, {X[2]:.2f}, {X[3]:.2f}, {X[4]:.2f}, {X[5]:.2f}]")

# 初始化协方差P
sigma_r = INIT_SIGMA_R
sigma_vel = sigma_r / 0.164  # 约122m/s
P_diag = [sigma_r**2, sigma_vel**2, sigma_r**2, sigma_vel**2, sigma_r**2, sigma_vel**2]
print(f"\n初始P对角线: {[f'{v:.1f}' for v in P_diag]}")

# 构建6x6对角协方差矩阵P
P = [[P_diag[i] if i == j else 0.0 for j in range(6)] for i in range(6)]

# 状态转移矩阵F
T_step = dt2 / 1000.0  # 2.624s
F = [
    [1, T_step, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, T_step, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, T_step],
    [0, 0, 0, 0, 0, 1]
]

# 过程噪声GQG
sigma_cv = 1.0
sigma_q = sigma_cv**2

# G矩阵
G = [
    [0.5*T_step**2, 0, 0],
    [T_step, 0, 0],
    [0, 0.5*T_step**2, 0],
    [0, T_step, 0],
    [0, 0, 0.5*T_step**2],
    [0, 0, T_step]
]

# Q矩阵
Q = [[sigma_q if i == j else 0.0 for j in range(3)] for i in range(3)]

# GQG = G @ Q @ G^T (手动计算)
GQ = mat_mult(G, Q)  # 6x3
# GQG[i][j] = sum_k GQ[i][k] * G[j][k]  (因为G^T[k][j] = G[j][k])
GQG = [[0.0]*6 for _ in range(6)]
for i in range(6):
    for j in range(6):
        for k in range(3):
            GQG[i][j] += GQ[i][k] * G[j][k]

print(f"\nGQG对角线: {[f'{GQG[i][i]:.1f}' for i in range(6)]}")

# 预测状态 X_predict = F @ X
X_predict = mat_vec_mult(F, X)
print(f"\n预测状态: [{X_predict[0]:.1f}, {X_predict[1]:.1f}, {X_predict[2]:.1f}, {X_predict[3]:.1f}, {X_predict[4]:.1f}, {X_predict[5]:.1f}]")

# 预测协方差 P_predict = F @ P @ F^T + GQG
FP = mat_mult(F, P)
FPF = mat_mult(FP, [[F[j][i] for j in range(6)] for i in range(6)])
P_predict = mat_add(FPF, GQG)
print(f"预测P对角线: {[f'{P_predict[i][i]:.1f}' for i in range(6)]}")

# 计算测量噪声R矩阵
rho = pt2['r']
theta = deg2rad(pt2['az'])
eps = deg2rad(pt2['el'])

ct = math.cos(theta)
st = math.sin(theta)
ce = math.cos(eps)
se = math.sin(eps)

sr = INIT_SIGMA_R
sa = INIT_SIGMA_A / 180.0 * pi
sb = INIT_SIGMA_B / 180.0 * pi

sr2 = sr**2
sa2 = sa**2
sb2 = sb**2

# 雅可比
dxdr = ct*ce; dxdt = -rho*st*ce; dxde = -rho*ct*se
dydr = st*ce; dydt = rho*ct*ce; dyde = -rho*st*se
dzdr = se; dzdt = 0.0; dzde = rho*ce

# R矩阵 (3x3)
R_mat = [
    [dxdr*dxdr*sr2 + dxdt*dxdt*sa2 + dxde*dxde*sb2, dxdr*dydr*sr2 + dxdt*dydt*sa2 + dxde*dyde*sb2, dxdr*dzdr*sr2 + dxdt*dzdt*sa2 + dxde*dzde*sb2],
    [dxdr*dydr*sr2 + dxdt*dydt*sa2 + dxde*dyde*sb2, dydr*dydr*sr2 + dydt*dydt*sa2 + dyde*dyde*sb2, dydr*dzdr*sr2 + dydt*dzdt*sa2 + dyde*dzde*sb2],
    [dxdr*dzdr*sr2 + dxdt*dzdt*sa2 + dxde*dzde*sb2, dydr*dzdr*sr2 + dydt*dzdt*sa2 + dyde*dzde*sb2, dzdr*dzdr*sr2 + dzdt*dzdt*sa2 + dzde*dzde*sb2]
]

print(f"\n测量噪声R对角线: {[f'{R_mat[i][i]:.2f}' for i in range(3)]}")

# 观测矩阵H (3x6)
H = [
    [1, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0]
]

# 预测位置 Z_predict = H @ X_predict
Z_predict = mat_vec_mult(H, X_predict)
print(f"\n预测位置: ({Z_predict[0]:.1f}, {Z_predict[1]:.1f}, {Z_predict[2]:.1f})")

# 新息 Z_D = Z_obse - Z_predict
Z_D = [Z2[i] - Z_predict[i] for i in range(3)]
print(f"新息Z_D: ({Z_D[0]:.1f}, {Z_D[1]:.1f}, {Z_D[2]:.1f})")
print(f"新息模长: {vec_norm(Z_D):.1f}m")

# 创新协方差 S = H @ P_predict @ H^T + R
HP = mat_mult(H, P_predict)  # 3x6
# HPH = HP @ H^T, 手动计算
HPH = [[0.0]*3 for _ in range(3)]
for i in range(3):
    for j in range(3):
        for k in range(6):
            HPH[i][j] += HP[i][k] * H[j][k]
S = mat_add(HPH, R_mat)
print(f"\n创新协方差S对角线: {[f'{S[i][i]:.2f}' for i in range(3)]}")

# 计算马氏距离
S_inv = mat_inv_3x3(S)
if S_inv:
    # Z_D^T @ S_inv @ Z_D
    Z_D_S_inv = [sum(Z_D[j] * S_inv[i][j] for j in range(3)) for i in range(3)]
    d_maha = sum(Z_D[i] * Z_D_S_inv[i] for i in range(3))
    print(f"\n马氏距离 = {d_maha:.2f}")
    print(f"阈值 TRACK_INIT_TH = {TRACK_INIT_TH}")
    
    print(f"\n{'='*60}")
    if d_maha < TRACK_INIT_TH:
        print("✅ 马氏距离检查通过！")
    else:
        print(f"❌ 马氏距离检查失败！({d_maha:.2f} > {TRACK_INIT_TH})")
        print(f"\n根因分析：")
        print(f"  1. 时间间隔Δt2={dt2}ms过长（跨波位扫描）")
        print(f"  2. 初始协方差P太小，无法覆盖长时间预测不确定性")
        
        # 尝试不同参数
        print(f"\n尝试增大INIT_SIGMA_R:")
        for new_sr in [30, 50, 100]:
            sr2_new = new_sr**2
            R_new = [
                [dxdr*dxdr*sr2_new + dxdt*dxdt*sa2 + dxde*dxde*sb2, dxdr*dydr*sr2_new + dxdt*dydt*sa2 + dxde*dyde*sb2, dxdr*dzdr*sr2_new + dxdt*dzdt*sa2 + dxde*dzde*sb2],
                [dxdr*dydr*sr2_new + dxdt*dydt*sa2 + dxde*dyde*sb2, dydr*dydr*sr2_new + dydt*dydt*sa2 + dyde*dyde*sb2, dydr*dzdr*sr2_new + dydt*dzdt*sa2 + dyde*dzde*sb2],
                [dxdr*dzdr*sr2_new + dxdt*dzdt*sa2 + dxde*dzde*sb2, dydr*dzdr*sr2_new + dydt*dzdt*sa2 + dyde*dzde*sb2, dzdr*dzdr*sr2_new + dzdt*dzdt*sa2 + dzde*dzde*sb2]
            ]
            S_new = mat_add(HPH, R_new)
            S_inv_new = mat_inv_3x3(S_new)
            if S_inv_new:
                Z_D_S_inv_new = [sum(Z_D[j] * S_inv_new[i][j] for j in range(3)) for i in range(3)]
                d_new = sum(Z_D[i] * Z_D_S_inv_new[i] for i in range(3))
                print(f"    INIT_SIGMA_R={new_sr}m: 马氏距离={d_new:.2f} {'PASS' if d_new < TRACK_INIT_TH else 'FAIL'}")
        
        print(f"\n尝试增大sigma_cv（过程噪声）:")
        for new_scv in [2.0, 5.0, 10.0]:
            Q_new = [[new_scv**2 if i == j else 0.0 for j in range(3)] for i in range(3)]
            GQG_new = mat_mult(mat_mult(G, Q_new), [[G[j][i] for j in range(3)] for i in range(6)])
            P_predict_new = mat_add(FPF, GQG_new)
            HP_new = mat_mult(H, P_predict_new)
            HPH_new = mat_mult(HP_new, [[H[j][i] for j in range(6)] for i in range(3)])
            S_new2 = mat_add(HPH_new, R_mat)
            S_inv_new2 = mat_inv_3x3(S_new2)
            if S_inv_new2:
                Z_D_S_inv_new2 = [sum(Z_D[j] * S_inv_new2[i][j] for j in range(3)) for i in range(3)]
                d_new2 = sum(Z_D[i] * Z_D_S_inv_new2[i] for i in range(3))
                print(f"    sigma_cv={new_scv}: 马氏距离={d_new2:.2f} {'PASS' if d_new2 < TRACK_INIT_TH else 'FAIL'}")
