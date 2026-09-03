import math
import sys

print("="*60)
print("无人机3点初始化流程模拟")
print("="*60)

# 无人机数据（来自实测数据）
drone_pts = [
    # frame, t_ms, beam, az_deg, r_m, el_deg, vr_mps
    (3, 898796, 2, 16.82, 1094.8, 5.17, -11.74),
    (4, 898960, 3, 17.39, 1095.8, 4.79, -11.74),
    (20, 901584, 2, 16.58, 1124.1, 4.62, -11.74),
    (21, 901748, 3, 17.28, 1124.8, 4.61, -11.74),
    (37, 904372, 2, 17.39, 1153.1, 4.77, -11.19),
    (38, 904536, 3, 18.39, 1154.6, 3.80, -11.19),
]

def az_el_to_xyz(az_deg, el_deg, r):
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    x = r * math.cos(el) * math.cos(az)
    y = r * math.cos(el) * math.sin(az)
    z = r * math.sin(el)
    return x, y, z

# 模拟3点组合
combinations = [
    (0, 1, 2, "Frame3+Frame4+Frame20"),
    (0, 1, 3, "Frame3+Frame4+Frame21"),
    (1, 2, 3, "Frame4+Frame20+Frame21"),
    (0, 1, 4, "Frame3+Frame4+Frame37"),
    (0, 1, 5, "Frame3+Frame4+Frame38"),
    (2, 3, 4, "Frame20+Frame21+Frame37"),
    (2, 3, 5, "Frame20+Frame21+Frame38"),
]

TRACK_QUALITY_THRESHOLD = 0.8
VELOCITY_CONSISTENCY_THRESHOLD = 0.5
ANGLE_CHANGE_THRESHOLD = 160.0
INIT_SIGMA_A = 1.0  # degrees
INIT_SIGMA_B = 1.0
INIT_SIGMA_R = 20.0

def calculate_vr_quality(vr0, vr1, vr2):
    quality = 0.0
    sign0 = (1 if abs(vr0) > 0.01 else 0) * (1 if vr0 > 0 else -1 if vr0 < 0 else 0)
    sign1 = (1 if abs(vr1) > 0.01 else 0) * (1 if vr1 > 0 else -1 if vr1 < 0 else 0)
    sign2 = (1 if abs(vr2) > 0.01 else 0) * (1 if vr2 > 0 else -1 if vr2 < 0 else 0)
    
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
    
    vrs = [abs(v) for v in [vr0, vr1, vr2] if abs(v) > 0.01]
    vr_avg_abs = sum(vrs) / len(vrs)
    vr_min_abs = min(vrs)
    vr_max_abs = max(vrs)
    
    ratio = vr_min_abs / vr_max_abs if vr_max_abs > 1e-6 else 0.5
    quality = 0.4 * ratio
    
    Vmin, Vmax = 2.0, 1500.0
    if Vmin <= vr_avg_abs <= Vmax:
        quality += 0.3
    elif vr_avg_abs > Vmax:
        quality += 0.0
    else:
        quality += 0.15
    
    if abs(vr_max_abs - vr_min_abs) < 15.0:
        quality += 0.3
    elif abs(vr_max_abs - vr_min_abs) < 30.0:
        quality += 0.15
    
    return min(quality, 1.0)

# 模拟每个组合的3点初始化
for i0, i1, i2, name in combinations:
    p0, p1, p2 = drone_pts[i0], drone_pts[i1], drone_pts[i2]
    t0, t1, t2 = p0[1], p1[1], p2[1]
    dt01 = (t1 - t0) / 1000.0  # seconds
    dt12 = (t2 - t1) / 1000.0
    dt_total = (t2 - t0) / 1000.0
    
    print(f"\n--- {name} ---")
    print(f"  时间间隔: dt01={dt01:.3f}s, dt12={dt12:.3f}s, total={dt_total:.3f}s")
    print(f"  Vr: {p0[6]:.2f}, {p1[6]:.2f}, {p2[6]:.2f}")
    
    # Step 1: Calculate quality
    vr_quality = calculate_vr_quality(p0[6], p1[6], p2[6])
    print(f"  Vr质量检查: {vr_quality:.3f} (阈值={TRACK_QUALITY_THRESHOLD}) {'PASS' if vr_quality >= TRACK_QUALITY_THRESHOLD else 'FAIL'}")
    
    # Step 2: Calculate positions
    x0, y0, z0 = az_el_to_xyz(p0[3], p0[5], p0[4])
    x1, y1, z1 = az_el_to_xyz(p1[3], p1[5], p1[4])
    x2, y2, z2 = az_el_to_xyz(p2[3], p2[5], p2[4])
    
    # Step 3: Estimate velocity from position difference
    vx_est = (x1 - x0) / dt01
    vy_est = (y1 - y0) / dt01
    vz_est = (z1 - z0) / dt01
    spd_est = math.sqrt(vx_est**2 + vy_est**2 + vz_est**2)
    
    # Radial direction of 3rd point
    az2 = math.radians(p2[3])
    el2 = math.radians(p2[5])
    er_x = math.cos(el2) * math.cos(az2)
    er_y = math.cos(el2) * math.sin(az2)
    er_z = math.sin(el2)
    
    # Correct velocity using radar Vr
    vr_radial_old = vx_est * er_x + vy_est * er_y + vz_est * er_z
    vx_tan = vx_est - vr_radial_old * er_x
    vy_tan = vy_est - vr_radial_old * er_y
    vz_tan = vz_est - vr_radial_old * er_z
    tan_damp = 0.15  # get_asso2 uses 0.15
    vx_corr = p2[6] * er_x + vx_tan * tan_damp
    vy_corr = p2[6] * er_y + vy_tan * tan_damp
    vz_corr = p2[6] * er_z + vz_tan * tan_damp
    spd_corr = math.sqrt(vx_corr**2 + vy_corr**2 + vz_corr**2)
    
    print(f"  位置差估算速度: ({vx_est:.1f}, {vy_est:.1f}, {vz_est:.1f}), |V|={spd_est:.1f}m/s")
    print(f"  修正后速度:     ({vx_corr:.1f}, {vy_corr:.1f}, {vz_corr:.1f}), |V|={spd_corr:.1f}m/s")
    print(f"  雷达实测Vr: {p2[6]:.2f}m/s (径向)")
    print(f"  修正后速度径向分量: {vx_corr*er_x + vy_corr*er_y + vz_corr*er_z:.2f}m/s")
    
    # Step 4: Check velocity sign consistency (count==2 stage for matching 3rd point)
    # At count==2, when matching the 3rd point, check Vr sign
    if p0[6] * p1[6] < 0:
        print(f"  ⚠ 前两点Vr符号冲突!")
    elif p0[6] * p2[6] < 0:
        print(f"  ⚠ 第1、3点Vr符号冲突!")
    elif p1[6] * p2[6] < 0:
        print(f"  ⚠ 第2、3点Vr符号冲突!")
    else:
        print(f"  ✓ Vr符号一致")
    
    # Step 5: Angle change check
    dx01 = x1 - x0
    dy01 = y1 - y0
    dz01 = z1 - z0
    dx12 = x2 - x1
    dy12 = y2 - y1
    dz12 = z2 - z1
    n1 = math.sqrt(dx12**2 + dy12**2 + dz12**2)
    n0 = math.sqrt(dx01**2 + dy01**2 + dz01**2)
    if n1 > 0.1 and n0 > 0.1:
        ca = (dx12*dx01 + dy12*dy01 + dz12*dz01) / (n1 * n0)
        ca = max(-1.0, min(1.0, ca))
        angle = math.acos(ca) * 180.0 / math.pi
        angle_ok = angle <= ANGLE_CHANGE_THRESHOLD
        print(f"  角度变化: {angle:.1f}° (阈值={ANGLE_CHANGE_THRESHOLD}°) {'PASS' if angle_ok else 'FAIL'}")
    else:
        print(f"  ⚠ 位置差太小，无法计算角度")
    
    # Step 6: Distance gate (for count==2 matching)
    # gate = 1500 * T + 150
    T_asso = dt12
    gate = 1500.0 * T_asso + 150.0
    dist_3d = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
    print(f"  距离门: gate={gate:.1f}m, 实际距离={dist_3d:.1f}m {'PASS' if dist_3d < gate else 'FAIL'}")

# Check temp_track compaction issue
print("\n" + "="*60)
print("检查temp_track压缩逻辑")
print("="*60)

# Simulate: Frame3 creates temp_track, Frame4 matches, then Frame20 should match
# But compaction happens every frame
# At Frame 5 (t=899128), temp_track last update is Frame4 (t=898960)
# lag_time = 899128 - 898960 = 168ms -> OK
# At Frame 20 (t=901584), temp_track last update is Frame4 (t=898960)  
# lag_time = 901584 - 898960 = 2624ms -> OK (time_up=25000ms)

time_up = 25000  # ms
print(f"time_up = {time_up}ms")
print(f"Frame4->Frame20 gap = {901584-898960}ms -> {'OK' if 901584-898960 < time_up else 'REJECTED'}")
print(f"Frame4->Frame37 gap = {904372-898960}ms -> {'OK' if 904372-898960 < time_up else 'REJECTED'}")

# But what about the vx_tan damp issue for cross-beam initialization
print("\n" + "="*60)
print("关键问题分析：跨波束初始化")
print("="*60)

# Frame3 (beam2) + Frame4 (beam3) -> cross-beam
x3, y3, z3 = az_el_to_xyz(16.82, 5.17, 1094.8)
x4, y4, z4 = az_el_to_xyz(17.39, 4.79, 1095.8)
dt_34 = 0.164  # seconds

vx34 = (x4 - x3) / dt_34
vy34 = (y4 - y3) / dt_34
vz34 = (z4 - z3) / dt_34
spd_34 = math.sqrt(vx34**2 + vy34**2 + vz34**2)

print(f"Frame3→Frame4 (跨波束) 位置差:")
print(f"  dX={x4-x3:.2f}m, dY={y4-y3:.2f}m, dZ={z4-z3:.2f}m")
print(f"  dt={dt_34}s")
print(f"  估算速度: Vx={vx34:.1f}, Vy={vy34:.1f}, Vz={vz34:.1f}, |V|={spd_34:.1f}m/s")
print(f"  实际Vr={-11.74}m/s")

# The problem: cross-beam position differences don't represent radial velocity
# Even with tan_damp=0.15, the tangential error is huge
# vx_tan = vx_old - v_radial_old * er_x
# The v_radial_old is computed using er from the NEW point direction
# This correction should help but might not be enough

# Let's simulate what happens for the 3rd point prediction
# Using the corrected velocity from Frame3+Frame4 initialization
az4 = math.radians(17.39)
el4 = math.radians(4.79)
er_x4 = math.cos(el4) * math.cos(az4)
er_y4 = math.cos(el4) * math.sin(az4)
er_z4 = math.sin(el4)

vr_radial_34 = vx34 * er_x4 + vy34 * er_y4 + vz34 * er_z4
vx_tan_34 = vx34 - vr_radial_34 * er_x4
vy_tan_34 = vy34 - vr_radial_34 * er_y4
vz_tan_34 = vz34 - vr_radial_34 * er_z4

tan_damp = 0.15
vx_corr_34 = (-11.74) * er_x4 + vx_tan_34 * tan_damp
vy_corr_34 = (-11.74) * er_y4 + vy_tan_34 * tan_damp
vz_corr_34 = (-11.74) * er_z4 + vz_tan_34 * tan_damp
spd_corr_34 = math.sqrt(vx_corr_34**2 + vy_corr_34**2 + vz_corr_34**2)

print(f"\n速度修正 (Frame4方向, tan_damp=0.15):")
print(f"  V_radial_old (Frame4方向): {vr_radial_34:.1f}m/s")
print(f"  V_tan: ({vx_tan_34:.1f}, {vy_tan_34:.1f}, {vz_tan_34:.1f})")
print(f"  V_corrected: ({vx_corr_34:.1f}, {vy_corr_34:.1f}, {vz_corr_34:.1f}), |V|={spd_corr_34:.1f}m/s")

# Now predict position at Frame20 (dt=2.624s)
dt_4_20 = 2.624
x_pred = x4 + vx_corr_34 * dt_4_20
y_pred = y4 + vy_corr_34 * dt_4_20
z_pred = z4 + vz_corr_34 * dt_4_20

# Actual position at Frame20
x20, y20, z20 = az_el_to_xyz(16.58, 4.62, 1124.1)

residual = math.sqrt((x_pred-x20)**2 + (y_pred-y20)**2 + (z_pred-z20)**2)
print(f"\n预测位置 (Frame20, dt={dt_4_20}s):")
print(f"  预测: ({x_pred:.1f}, {y_pred:.1f}, {z_pred:.1f})")
print(f"  实际: ({x20:.1f}, {y20:.1f}, {z20:.1f})")
print(f"  残差: {residual:.1f}m")
print(f"  位置变化实际: ({x20-x4:.1f}, {y20-y4:.1f}, {z20-z4:.1f}), |d|={math.sqrt((x20-x4)**2+(y20-y4)**2+(z20-z4)**2):.1f}m")
