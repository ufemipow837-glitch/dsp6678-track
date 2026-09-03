# -*- coding: utf-8 -*-
"""
完整模拟无人机3点初始化流程
诊断为什么无航迹建立
"""
import math
import sys

pi = 3.141592653589793

# 参数配置
TRACK_INIT_TH = 15.0
TRACK_ASSO_TH = 11.3
VELOCITY_CONSISTENCY_THRESHOLD = 0.5
TRACK_QUALITY_THRESHOLD = 0.7
CONFIDENCE_THRESHOLD = 0.9
INIT_SIGMA_A = 1.0   # 方位角测量噪声(度)
INIT_SIGMA_B = 1.0   # 俯仰角测量噪声(度)  
INIT_SIGMA_R = 20.0  # 距离测量噪声(m)
Vmin = 2.0
Vmax = 1500.0
time_up = 25000.0
time_down = 0.0

# 模拟无人机数据点
# 从data_entry.c提取的真实数据
drone_frames = [
    # (frame_idx, mSecond, azi_deg, el_deg, range_m, velocity, beamNo)
    (4,  898796, 16.8225, 5.1668, 1094.84, -11.7406, 2),
    (5,  898960, 17.3911, 4.7861, 1095.84, -11.7406, 3),  # Frame 5 point 1
    (21, 901584, 16.5781, 4.6226, 1124.10, -11.7406, 2),
    (22, 901748, 17.2821, 4.6089, 1124.83, -11.7406, 3),
    (38, 904372, 18.1500, 4.5000, 1160.00, -11.1945, 2),  # 估算
    (39, 904536, 18.6000, 4.4500, 1175.00, -11.1945, 3),  # 估算
    (55, 907160, 19.2000, 4.3000, 1200.00, -11.4676, 2),  # 估算
    (56, 907324, 19.7000, 4.2500, 1215.00, -11.4676, 3),  # 估算
]

def polar_to_cart(azi_deg, el_deg, r):
    """球坐标→直角坐标"""
    azi = azi_deg / 180.0 * pi
    el = el_deg / 180.0 * pi
    x = r * math.cos(el) * math.cos(azi)
    y = r * math.cos(el) * math.sin(azi)
    z = r * math.sin(el)
    return x, y, z

def cart_to_polar(x, y, z):
    """直角坐标→球坐标"""
    r = math.sqrt(x*x + y*y + z*z)
    azi = math.atan2(y, x) * 180.0 / pi
    el = math.asin(z / r) * 180.0 / pi if r > 0 else 0
    return azi, el, r

def kalman_filter_init(z0, z1, x0, p0):
    """简化的KF初始化：用两点差分计算速度"""
    # z0 = [x0, y0, z0], z1 = [x1, y1, z1]
    # x0 = [x, vx, y, vy, z, vz]
    dt = 0.164  # 假设164ms
    vx = (z1[0] - z0[0]) / dt
    vy = (z1[1] - z0[1]) / dt
    vz = (z1[2] - z0[2]) / dt
    x0_new = [z1[0], vx, z1[1], vy, z1[2], vz]
    # 简化协方差：速度协方差大
    p0_new = [
        [100.0, 0, 0, 0, 0, 0],
        [0, 500.0, 0, 0, 0, 0],
        [0, 0, 100.0, 0, 0, 0],
        [0, 0, 0, 500.0, 0, 0],
        [0, 0, 0, 0, 100.0, 0],
        [0, 0, 0, 0, 0, 500.0],
    ]
    return x0_new, p0_new

def predict_state(x, T):
    """匀速模型预测"""
    x_pred = [0.0] * 6
    x_pred[0] = x[0] + x[1] * T  # x + vx*T
    x_pred[1] = x[1]              # vx不变
    x_pred[2] = x[2] + x[3] * T  # y + vy*T
    x_pred[3] = x[3]              # vy不变
    x_pred[4] = x[4] + x[5] * T  # z + vz*T
    x_pred[5] = x[5]              # vz不变
    return x_pred

def predict_covar(P, T, sigma=5.0):
    """预测协方差（简化）"""
    # F = I (状态转移)
    # Q = G * sigma * G'
    # 这里简化处理
    dt = T
    Q = [
        [dt*dt*sigma*sigma/3, dt*sigma*sigma/2, 0, 0, 0, 0],
        [dt*sigma*sigma/2, sigma*sigma, 0, 0, 0, 0],
        [0, 0, dt*dt*sigma*sigma/3, dt*sigma*sigma/2, 0, 0],
        [0, 0, dt*sigma*sigma/2, sigma*sigma, 0, 0],
        [0, 0, 0, 0, dt*dt*sigma*sigma/3, dt*sigma*sigma/2],
        [0, 0, 0, 0, dt*sigma*sigma/2, sigma*sigma],
    ]
    P_pred = [[P[i][j] + Q[i][j] for j in range(6)] for i in range(6)]
    return P_pred

def compute_measurement_covar(azi_deg, el_deg, r):
    """计算测量噪声协方差R"""
    azi = azi_deg / 180.0 * pi
    el = el_deg / 180.0 * pi
    
    ct = math.cos(azi)
    st = math.sin(azi)
    ce = math.cos(el)
    se = math.sin(el)
    
    sr = INIT_SIGMA_R
    sa = INIT_SIGMA_A / 180.0 * pi
    sb = INIT_SIGMA_B / 180.0 * pi
    
    sr2 = sr * sr
    sa2 = sa * sa
    sb2 = sb * sb
    
    # 雅可比矩阵
    dxdr = ct * ce
    dxdt = -r * st * ce
    dxde = -r * ct * se
    dydr = st * ce
    dydt = r * ct * ce
    dyde = -r * st * se
    dzdr = se
    dzdt = 0.0
    dzde = r * ce
    
    R = [
        [dxdr*dxdr*sr2 + dxdt*dxdt*sa2 + dxde*dxde*sb2,
         dxdr*dydr*sr2 + dxdt*dydt*sa2 + dxde*dyde*sb2,
         dxdr*dzdr*sr2 + dxdt*dzdt*sa2 + dxde*dzde*sb2],
        [dxdr*dydr*sr2 + dxdt*dydt*sa2 + dxde*dyde*sb2,
         dydr*dydr*sr2 + dydt*dydt*sa2 + dyde*dyde*sb2,
         dydr*dzdr*sr2 + dydt*dzdt*sa2 + dyde*dzde*sb2],
        [dxdr*dzdr*sr2 + dxdt*dzdt*sa2 + dxde*dzde*sb2,
         dydr*dzdr*sr2 + dydt*dzdt*sa2 + dyde*dzde*sb2,
         dzdr*dzdr*sr2 + dzdt*dzdt*sa2 + dzde*dzde*sb2],
    ]
    return R

def compute_mahalanobis(x_pred, P_pred, z_meas, R_mat):
    """计算马氏距离"""
    # 创新向量
    nu = [z_meas[i] - x_pred[i*2] for i in range(3)]  # H = [1,0,0,0,0,0; 0,0,1,0,0,0; 0,0,0,0,1,0]
    
    # H*P_pred*H'
    HPH = [[P_pred[i*2][j*2] for j in range(3)] for i in range(3)]
    
    # S = HPH + R
    S = [[HPH[i][j] + R_mat[i][j] for j in range(3)] for i in range(3)]
    
    # 简化：用对角近似
    # d_maha = nu' * inv(S) * nu
    # 用近似：d_maha ≈ sum(nu[i]^2 / S[i][i])
    d_maha = 0.0
    for i in range(3):
        if S[i][i] > 0:
            d_maha += nu[i] * nu[i] / S[i][i]
    
    return d_maha, nu

def calculate_vr_quality(vr0, vr1, vr2):
    """速度质量评估函数"""
    quality = 0.0
    
    def get_sign(v):
        if abs(v) < 0.01:
            return 0
        return 1 if v > 0 else -1
    
    sign0 = get_sign(vr0)
    sign1 = get_sign(vr1)
    sign2 = get_sign(vr2)
    
    n_valid = (sign0 != 0) + (sign1 != 0) + (sign2 != 0)
    
    if n_valid >= 2:
        if sign0 != 0 and sign1 != 0 and sign0 != sign1:
            return 0.0
        if sign0 != 0 and sign2 != 0 and sign0 != sign2:
            return 0.0
        if sign1 != 0 and sign2 != 0 and sign1 != sign2:
            return 0.0
    else:
        return 0.0
    
    vrs = [abs(v) for v in [vr0, vr1, vr2] if abs(v) >= 0.01]
    if not vrs:
        return 0.0
    
    vr_avg_abs = sum(vrs) / len(vrs)
    vr_min_abs = min(vrs)
    vr_max_abs = max(vrs)
    
    if vr_max_abs > 1e-6 and vr_min_abs > 1e-6:
        ratio = vr_min_abs / vr_max_abs
        quality += 0.4 * ratio
    
    if vr_avg_abs >= Vmin and vr_avg_abs <= Vmax:
        quality += 0.3
    elif vr_avg_abs > Vmax:
        quality += 0.0
    else:
        quality += 0.15
    
    if abs(vr_max_abs - vr_min_abs) < 15.0:
        quality += 0.3
    elif abs(vr_max_abs - vr_min_abs) < 30.0:
        quality += 0.15
    
    quality = min(quality, 1.0)
    return quality

def calculate_confidence_for_track(track0, track1, track2):
    """计算航迹置信度 - 基于位置差分速度"""
    T1 = (track1['mSecond'] - track0['mSecond']) / 1000.0
    T2 = (track2['mSecond'] - track1['mSecond']) / 1000.0
    
    if T1 < 0.001 or T2 < 0.001:
        return 0.0
    
    v1x = (track1['x'] - track0['x']) / T1
    v1y = (track1['y'] - track0['y']) / T1
    v1z = (track1['z'] - track0['z']) / T1
    v2x = (track2['x'] - track1['x']) / T2
    v2y = (track2['y'] - track1['y']) / T2
    v2z = (track2['z'] - track1['z']) / T2
    
    v1_norm = math.sqrt(v1x*v1x + v1y*v1y + v1z*v1z)
    v2_norm = math.sqrt(v2x*v2x + v2y*v2y + v2z*v2z)
    
    if v1_norm < 1.0 or v2_norm < 1.0:
        return 0.0
    
    dot_prod = v1x*v2x + v1y*v2y + v1z*v2z
    cos_theta = max(-1.0, min(1.0, dot_prod / (v1_norm * v2_norm + 1e-12)))
    
    angle_consistency = (1.0 + cos_theta) / 2.0
    min_norm = min(v1_norm, v2_norm)
    max_norm = max(v1_norm, v2_norm)
    speed_ratio = min_norm / (max_norm + 1e-12)
    
    v1_valid = 1.0 if Vmin <= v1_norm <= Vmax else 0.3
    v2_valid = 1.0 if Vmin <= v2_norm <= Vmax else 0.3
    
    confidence = (angle_consistency + speed_ratio + v1_valid + v2_valid) / 4.0
    
    # 检查spd
    vx = track2.get('vx', 0)
    vy = track2.get('vy', 0)
    vz = track2.get('vz', 0)
    spd = math.sqrt(vx*vx + vy*vy + vz*vz)
    if spd < Vmin or spd > Vmax:
        confidence *= 0.5
    
    return confidence

# ========== 主模拟 ==========
print("=" * 70)
print("无人机3点初始化完整流程模拟")
print("=" * 70)

# 转换数据为直角坐标
cart_points = []
for fidx, ms, az, el, r, v, beam in drone_frames:
    x, y, z = polar_to_cart(az, el, r)
    cart_points.append({
        'frame': fidx, 'mSecond': ms, 'azi': az, 'ele': el, 'range': r,
        'velocity': v, 'beam': beam, 'x': x, 'y': y, 'z': z
    })
    print(f"Frame {fidx:3d} (beam {beam}): az={az:.2f}° el={el:.2f}° r={r:.1f}m Vr={v:.2f}m/s → x={x:.1f} y={y:.1f} z={z:.1f}")

print("\n" + "=" * 70)
print("STEP 1: 两点初始化 (Frame 4 + Frame 5 point1)")
print("=" * 70)

p0 = cart_points[0]  # Frame 4
p1 = cart_points[1]  # Frame 5

dt_01 = (p1['mSecond'] - p0['mSecond']) / 1000.0
print(f"\n时间差 T1 = {dt_01:.3f}s")
print(f"位置差 Δx = {p1['x']-p0['x']:.1f}m, Δy = {p1['y']-p0['y']:.1f}m, Δz = {p1['z']-p0['z']:.1f}m")

# 距离检查
dist_sq = (p1['x']-p0['x'])**2 + (p1['y']-p0['y'])**2 + (p1['z']-p0['z'])**2
dist = math.sqrt(dist_sq)
gate_1 = 1500.0 * dt_01 + 150.0
print(f"距离 = {dist:.1f}m, 门限 = {gate_1:.1f}m → {'PASS' if dist < gate_1 else 'FAIL!'}")

# 速度门限检查
quality_2 = calculate_vr_quality(p0['velocity'], p1['velocity'], 0.0)
print(f"\n速度质量检查: vr0={p0['velocity']:.2f}, vr1={p1['velocity']:.2f}")
print(f"calculate_vr_quality = {quality_2:.3f} (阈值 {VELOCITY_CONSISTENCY_THRESHOLD}) → {'PASS' if quality_2 >= VELOCITY_CONSISTENCY_THRESHOLD else 'FAIL!'}")

# KF初始化
x0, P0 = kalman_filter_init([p0['x'], p0['y'], p0['z']], [p1['x'], p1['y'], p1['z']], [0]*6, [[0]*6 for _ in range(6)])
print(f"\nKF初始化结果:")
print(f"  x={x0[0]:.1f}, y={x0[2]:.1f}, z={x0[4]:.1f}")
print(f"  vx={x0[1]:.1f}, vy={x0[3]:.1f}, vz={x0[5]:.1f}")
spd_init = math.sqrt(x0[1]**2 + x0[3]**2 + x0[5]**2)
print(f"  速度 = {spd_init:.1f} m/s (真实Vr = {p0['velocity']:.1f} m/s)")
print(f"  !! 位置差分速度 vs 雷达Vr 偏差 = {spd_init - abs(p0['velocity']):.1f} m/s !!")

print("\n" + "=" * 70)
print("STEP 2: 尝试关联第3点 (Frame 21)")
print("=" * 70)

p2 = cart_points[2]  # Frame 21
dt_12 = (p2['mSecond'] - p1['mSecond']) / 1000.0
print(f"\n时间差 T2 = {dt_12:.3f}s")

# 预测
x_pred = predict_state(x0, dt_12)
P_pred = predict_covar(P0, dt_12)
print(f"\n预测位置: x={x_pred[0]:.1f}, y={x_pred[2]:.1f}, z={x_pred[4]:.1f}")
print(f"实测位置: x={p2['x']:.1f}, y={p2['y']:.1f}, z={p2['z']:.1f}")
print(f"位置差: Δx={p2['x']-x_pred[0]:.1f}, Δy={p2['y']-x_pred[2]:.1f}, Δz={p2['z']-x_pred[4]:.1f}")

# 马氏距离
R_mat = compute_measurement_covar(p2['azi'], p2['ele'], p2['range'])
d_maha, nu = compute_mahalanobis(x_pred, P_pred, [p2['x'], p2['y'], p2['z']], R_mat)
print(f"\n创新向量: nu = ({nu[0]:.1f}, {nu[1]:.1f}, {nu[2]:.1f})")
print(f"马氏距离 = {d_maha:.2f} (阈值 TRACK_INIT_TH = {TRACK_INIT_TH})")
print(f"马氏距离检查 → {'PASS' if d_maha < TRACK_INIT_TH else 'FAIL!'}")

# 角度检查
dx = p2['x'] - x0[0]
dy = p2['y'] - x0[2]
dz = p2['z'] - x0[4]
dx0 = x0[0] - p0['x']
dy0 = x0[2] - p0['y']
dz0 = x0[4] - p0['z']
dot_p = dx*dx0 + dy*dy0 + dz*dz0
n1 = math.sqrt(dx*dx + dy*dy + dz*dz)
n0 = math.sqrt(dx0*dx0 + dy0*dy0 + dz0*dz0)
angle = 180.0
if n1 > 0.1 and n0 > 0.1:
    ca = max(-1.0, min(1.0, dot_p / (n1*n0)))
    angle = math.acos(ca) * 180.0 / pi
print(f"\n角度检查: angle={angle:.1f}° (阈值 90°) → {'PASS' if angle <= 90 else 'FAIL!'}")

# 速度质量检查
quality_3 = calculate_vr_quality(p0['velocity'], p1['velocity'], p2['velocity'])
print(f"\n3点速度质量: vr0={p0['velocity']:.2f}, vr1={p1['velocity']:.2f}, vr2={p2['velocity']:.2f}")
print(f"calculate_vr_quality = {quality_3:.3f} (阈值 {TRACK_QUALITY_THRESHOLD}) → {'PASS' if quality_3 >= TRACK_QUALITY_THRESHOLD else 'FAIL!'}")

print("\n" + "=" * 70)
print("STEP 3: new_reliable 置信度检查")
print("=" * 70)

track0 = {'x': p0['x'], 'y': p0['y'], 'z': p0['z'], 'mSecond': p0['mSecond']}
track1 = {'x': x0[0], 'y': x0[2], 'z': x0[4], 'mSecond': p1['mSecond'], 'vx': x0[1], 'vy': x0[3], 'vz': x0[5]}
track2 = {'x': p2['x'], 'y': p2['y'], 'z': p2['z'], 'mSecond': p2['mSecond'], 'vx': x_pred[1], 'vy': x_pred[3], 'vz': x_pred[5]}

# 但实际上track2应该是KF更新后的状态
# 简化：直接用实测点的位置和预测速度
# 真实代码中get_track_asso2会用kalman_filter_init(track1, point2)
x12, P12 = kalman_filter_init([x0[0], x0[2], x0[4]], [p2['x'], p2['y'], p2['z']], [0]*6, [[0]*6 for _ in range(6)])
track2['vx'] = x12[1]
track2['vy'] = x12[3]
track2['vz'] = x12[5]
spd_12 = math.sqrt(x12[1]**2 + x12[3]**2 + x12[5]**2)
print(f"\n3点KF初始化速度: {spd_12:.1f} m/s")

confidence = calculate_confidence_for_track(track0, track1, track2)
print(f"\ncalculate_confidence_for_track = {confidence:.3f} (阈值 {CONFIDENCE_THRESHOLD})")
print(f"置信度检查 → {'PASS' if confidence >= CONFIDENCE_THRESHOLD else 'FAIL!'}")

# 分析速度分量
T1 = (track1['mSecond'] - track0['mSecond']) / 1000.0
T2 = (track2['mSecond'] - track1['mSecond']) / 1000.0
v1x = (track1['x'] - track0['x']) / T1
v1y = (track1['y'] - track0['y']) / T1
v1z = (track1['z'] - track0['z']) / T1
v2x = (track2['x'] - track1['x']) / T2
v2y = (track2['y'] - track1['y']) / T2
v2z = (track2['z'] - track1['z']) / T2
v1_norm = math.sqrt(v1x*v1x + v1y*v1y + v1z*v1z)
v2_norm = math.sqrt(v2x*v2x + v2y*v2y + v2z*v2z)
print(f"\n位置差分速度:")
print(f"  v1 (Frame4→Frame5): {v1_norm:.1f} m/s")
print(f"  v2 (Frame5→Frame21): {v2_norm:.1f} m/s")
print(f"  无人机真实Vr: ~11.74 m/s")
print(f"  !! v1={v1_norm:.1f} vs Vr={abs(p0['velocity']):.1f} → 偏差={abs(v1_norm-abs(p0['velocity'])):.1f} m/s !!")

# 物理一致性检查
print("\n" + "=" * 70)
print("STEP 4: 物理一致性检查 (new_reliable.c)")
print("=" * 70)

vr0, vr1, vr2 = p0['velocity'], p1['velocity'], p2['velocity']
r0, r1, r2 = p0['range'], p1['range'], p2['range']

# ① Vr符号一致性
s0 = 1 if vr0 > 0 else (-1 if vr0 < 0 else 0)
s1 = 1 if vr1 > 0 else (-1 if vr1 < 0 else 0)
s2 = 1 if vr2 > 0 else (-1 if vr2 < 0 else 0)
sign_ok = not (s0 != 0 and s1 != 0 and s2 != 0 and (s0 != s1 or s1 != s2 or s0 != s2))
print(f"\n① Vr符号检查: s0={s0}, s1={s1}, s2={s2} → {'PASS' if sign_ok else 'FAIL!'}")

# ② Vr量级
vr_avg_abs = (abs(vr0) + abs(vr1) + abs(vr2)) / 3.0
print(f"② Vr量级: avg_abs={vr_avg_abs:.1f} m/s → {'PASS' if vr_avg_abs <= 2000 else 'FAIL!'}")

# ③ R/Vr一致性 (仅高速)
dt_s = (p2['mSecond'] - p0['mSecond']) / 1000.0
if vr_avg_abs > 30 and dt_s > 0.01:
    Vr_from_R = (r2 - r0) / dt_s
    rv_check = not ((vr2 > 1 and Vr_from_R < -1) or (vr2 < -1 and Vr_from_R > -1))
    print(f"③ R/Vr一致性: Vr_from_R={Vr_from_R:.1f} vs Vr={vr2:.1f} → {'PASS' if rv_check else 'FAIL!'}")
else:
    print(f"③ R/Vr一致性: 低速目标跳过 (vr_avg_abs={vr_avg_abs:.1f})")

# ④ 远距离检查
dist_check = not (r2 > 5000 and vr_avg_abs < 30)
print(f"④ 远距离检查: r2={r2:.1f}m, vr_avg={vr_avg_abs:.1f} → {'PASS' if dist_check else 'FAIL!'}")

# 方位跨度
az_span = abs(p2['azi'] - p0['azi'])
if az_span > 180: az_span = 360 - az_span
az_check = az_span <= 30
print(f"\n方位跨度: {az_span:.2f}° (阈值30°) → {'PASS' if az_check else 'FAIL!'}")

# 俯仰跨度
el_span = abs(p2['ele'] - p0['ele'])
el_check = el_span <= 25
print(f"俯仰跨度: {el_span:.2f}° (阈值25°) → {'PASS' if el_check else 'FAIL!'}")

# 绝对扇区约束
drone_like = vr_avg_abs <= 80
az_norm = p2['azi']
while az_norm > 180: az_norm -= 360
while az_norm < -180: az_norm += 360
sector_ok = True
if vr_avg_abs < 150:
    if az_norm > 60 or az_norm < -20:
        sector_ok = False
    if p2['ele'] < -15 or p2['ele'] > 75:
        sector_ok = False
    if r2 > 8000:
        sector_ok = False
print(f"扇区约束: az_norm={az_norm:.2f}°, el={p2['ele']:.2f}°, r={r2:.1f}m → {'PASS' if sector_ok else 'FAIL!'}")

print("\n" + "=" * 70)
print("总结与根因分析")
print("=" * 70)

# 综合判断
all_pass = True
if not dist < gate_1:
    all_pass = False
    print("❌ 两点初始化距离检查失败")
if not quality_2 >= VELOCITY_CONSISTENCY_THRESHOLD:
    all_pass = False
    print("❌ 两点初始化速度质量检查失败")
if not (d_maha < TRACK_INIT_TH):
    all_pass = False
    print(f"❌ 三点初始化马氏距离检查失败 (d_maha={d_maha:.2f} > {TRACK_INIT_TH})")
if not (angle <= 90):
    all_pass = False
    print("❌ 角度检查失败")
if not (quality_3 >= TRACK_QUALITY_THRESHOLD):
    all_pass = False
    print("❌ 三点速度质量检查失败")
if not (confidence >= CONFIDENCE_THRESHOLD):
    all_pass = False
    print(f"❌ new_reliable置信度检查失败 (confidence={confidence:.3f} < {CONFIDENCE_THRESHOLD})")
if not sign_ok:
    all_pass = False
    print("❌ Vr符号检查失败")
if not az_check:
    all_pass = False
    print("❌ 方位跨度检查失败")
if not el_check:
    all_pass = False
    print("❌ 俯仰跨度检查失败")
if not sector_ok:
    all_pass = False
    print("❌ 扇区约束检查失败")

if all_pass:
    print("\n✅ 所有检查通过！理论上应该能初始化航迹")
    print("   问题可能在于代码实现细节（如矩阵运算、数据传递等）")
else:
    print("\n❌ 存在检查失败！航迹初始化无法完成")

# 额外分析：问题可能的根源
print("\n" + "=" * 70)
print("深层问题分析")
print("=" * 70)

print(f"""
1. 速度估计偏差:
   位置差分速度 v1 = {v1_norm:.1f} m/s (实际Vr = {abs(p0['velocity']):.1f} m/s)
   偏差原因: 跨波束方位角变化(16.82°→17.39°)产生虚假切向速度
   
2. KF初始化速度:
   spd_init = {spd_init:.1f} m/s (应≈11.7 m/s)
   这导致后续预测位置严重漂移

3. 预测vs实测位置差:
   预测位置: ({x_pred[0]:.1f}, {x_pred[2]:.1f}, {x_pred[4]:.1f})
   实测位置: ({p2['x']:.1f}, {p2['y']:.1f}, {p2['z']:.1f})
   差异: ({p2['x']-x_pred[0]:.1f}, {p2['y']-x_pred[2]:.1f}, {p2['z']-x_pred[4]:.1f})

4. 置信度计算:
   calculate_confidence_for_track = {confidence:.3f}
   位置差分速度导致v1_norm和v2_norm远大于Vr
   v1_norm={v1_norm:.1f} > Vmax={Vmax}? → {'是' if v1_norm > Vmax else '否'}
   v2_norm={v2_norm:.1f} > Vmax={Vmax}? → {'是' if v2_norm > Vmax else '否'}
   
   如果v1_norm > Vmax或v2_norm > Vmax: v1_valid/v2_valid = 0.3
   → confidence被严重压低

5. 关键发现: calculate_confidence_for_track仍使用位置差分速度
   这与我们的修复目标（用Vr替代位置差分）不一致！
   """)