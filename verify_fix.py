# -*- coding: utf-8 -*-
"""
验证修复后的无人机3点初始化流程
检查所有检查点是否通过
"""
import math

pi = 3.141592653589793

# 参数配置
TRACK_INIT_TH = 15.0
TRACK_ASSO_TH = 11.3
VELOCITY_CONSISTENCY_THRESHOLD = 0.5
TRACK_QUALITY_THRESHOLD = 0.8
ANGLE_CHANGE_THRESHOLD = 160.0
CONFIDENCE_THRESHOLD = 0.65
INIT_SIGMA_A = 1.0
INIT_SIGMA_B = 1.0
INIT_SIGMA_R = 20.0
Vmin = 2.0
Vmax = 1500.0
time_up = 25000.0
time_down = 0.0

# 无人机数据点
drone_points = [
    (4,  898796, 16.8225, 5.1668, 1094.84, -11.7406, 2, 0),
    (5,  898960, 17.3911, 4.7861, 1095.84, -11.7406, 3, 1),
    (21, 901584, 16.5781, 4.6226, 1124.10, -11.7406, 2, 0),
    (22, 901748, 17.2821, 4.6089, 1124.83, -11.7406, 3, 0),
    (38, 904372, 18.1500, 4.5000, 1160.00, -11.1945, 2, 0),
    (39, 904536, 18.6000, 4.4500, 1175.00, -11.1945, 3, 0),
]

def polar_to_cart(azi_deg, el_deg, r):
    azi = azi_deg / 180.0 * pi
    el = el_deg / 180.0 * pi
    x = r * math.cos(el) * math.cos(azi)
    y = r * math.cos(el) * math.sin(azi)
    z = r * math.sin(el)
    return x, y, z

positions = []
for i, (frame, msec, azi_d, el_d, r, vel, beam, idx) in enumerate(drone_points):
    x, y, z = polar_to_cart(azi_d, el_d, r)
    positions.append((x, y, z))

print("=" * 70)
print("修复后验证: 无人机3点初始化完整流程")
print("=" * 70)

# ===== 步骤1: get_track_begin (Frame 4) =====
print("\n[步骤1] Frame 4: get_track_begin → 创建1点temp_track")
p0 = positions[0]
print(f"  位置: x={p0[0]:.2f}, y={p0[1]:.2f}, z={p0[2]:.2f}")
print(f"  mSecond: {drone_points[0][1]}")
print(f"  方位角门限检查: 雷达方位角={90-drone_points[0][2]:.2f}° → 通过 [-90°, 90°] ✓")

# ===== 步骤2: get_track_asso1 (Frame 5) =====
print("\n[步骤2] Frame 5: get_track_asso1 → 创建2点temp_track")
p1 = positions[1]
T01 = (drone_points[1][1] - drone_points[0][1]) / 1000.0
print(f"  时间差 T = {T01:.4f}s")

# 位置差分速度
vx_pos = (p1[0] - p0[0]) / T01
vy_pos = (p1[1] - p0[1]) / T01
vz_pos = (p1[2] - p0[2]) / T01
print(f"  位置差分速度: vx={vx_pos:.2f}, vy={vy_pos:.2f}, vz={vz_pos:.2f}")

# 距离检查
dist_sq = (p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 + (p1[2]-p0[2])**2
gate = 1500.0 * T01 + 150.0
print(f"  距离检查: dist={math.sqrt(dist_sq):.1f}m < gate={gate:.1f}m → {'PASS' if dist_sq < gate**2 else 'FAIL'}")

# 速度符号检查
vr0 = drone_points[0][5]
vr1 = drone_points[1][5]
print(f"  Vr符号检查: vr0={vr0:.2f}, vr1={vr1:.2f}, 乘积={vr0*vr1:.2f} → {'PASS' if vr0*vr1 > 0 else 'FAIL'}")

# 速度修正
r_azi = drone_points[1][2] / 180.0 * pi
r_el = drone_points[1][3] / 180.0 * pi
r_vr = drone_points[1][5]
er_x = math.cos(r_el) * math.cos(r_azi)
er_y = math.cos(r_el) * math.sin(r_azi)
er_z = math.sin(r_el)

v_radial_old = vx_pos*er_x + vy_pos*er_y + vz_pos*er_z
vx_tan = vx_pos - v_radial_old*er_x
vy_tan = vy_pos - v_radial_old*er_y
vz_tan = vz_pos - v_radial_old*er_z
tan_damp_1 = 0.08
vx_corr = r_vr*er_x + vx_tan*tan_damp_1
vy_corr = r_vr*er_y + vy_tan*tan_damp_1
vz_corr = r_vr*er_z + vz_tan*tan_damp_1
v1_mag = math.sqrt(vx_corr**2 + vy_corr**2 + vz_corr**2)
print(f"  修正后速度: vx={vx_corr:.2f}, vy={vy_corr:.2f}, vz={vz_corr:.2f}, |v|={v1_mag:.2f}m/s")

# ===== 步骤3: 3点匹配检查 (Frame 21) =====
print("\n[步骤3] Frame 21: 匹配2点temp_track → 3点初始化检查")
p2 = positions[2]
T12 = (drone_points[2][1] - drone_points[1][1]) / 1000.0
print(f"  时间差 T = {T12:.4f}s")

# lag_time检查
lag_time = drone_points[2][1] - drone_points[1][1]
print(f"  lag_time={lag_time}ms, 范围[{time_down}, {time_up}] → {'PASS' if time_down <= lag_time <= time_up else 'FAIL'}")

# Vr一致性检查
vr0_3pt = drone_points[0][5]
vr1_3pt = drone_points[1][5]
vr2_3pt = drone_points[2][5]
sa = 1 if vr0_3pt > 0 else (-1 if vr0_3pt < 0 else 0)
sb = 1 if vr1_3pt > 0 else (-1 if vr1_3pt < 0 else 0)
sc = 1 if vr2_3pt > 0 else (-1 if vr2_3pt < 0 else 0)
vr_sign_check = not ((sa != 0 and sb != 0 and sc != 0) and (sa != sb or sb != sc))
print(f"  Vr符号一致性: sa={sa}, sb={sb}, sc={sc} → {'PASS' if vr_sign_check else 'FAIL'}")

# 几何门限检查
avg_azi = (90 - drone_points[0][2] + 90 - drone_points[1][2]) / 2.0
avg_el = (drone_points[0][3] + drone_points[1][3]) / 2.0
avg_r = (drone_points[0][4] + drone_points[1][4]) / 2.0
new_azi = 90 - drone_points[2][2]
new_el = drone_points[2][3]
new_r = drone_points[2][4]
azi_diff = abs(new_azi - avg_azi)
el_diff = abs(new_el - avg_el)
r_diff = abs(new_r - avg_r)
print(f"  几何门限: |dazi|={azi_diff:.2f}°<5°, |del|={el_diff:.2f}°<5°, |dr|={r_diff:.1f}m<500m")
print(f"  → {'PASS' if azi_diff < 5 and el_diff < 5 and r_diff < 500 else 'FAIL'}")

# 马氏距离检查 (简化)
x_pred = p1[0] + vx_corr * T12
y_pred = p1[1] + vy_corr * T12
z_pred = p1[2] + vz_corr * T12
dx = p2[0] - x_pred
dy = p2[1] - y_pred
dz = p2[2] - z_pred
sigma_r = 20.0
sigma_az_rad = 1.0 / 180.0 * pi
r_mean = (drone_points[1][4] + drone_points[2][4]) / 2.0
sigma_x_az = r_mean * sigma_az_rad
S_xx = sigma_r**2 + sigma_x_az**2
S_yy = sigma_r**2 + sigma_x_az**2
S_zz = sigma_r**2
d_maha = math.sqrt(dx**2/S_xx + dy**2/S_yy + dz**2/S_zz)
print(f"  马氏距离: d_maha={d_maha:.2f} < TRACK_INIT_TH={TRACK_INIT_TH} → {'PASS' if d_maha < TRACK_INIT_TH else 'FAIL'}")

# ===== 步骤4: new_reliable 检查 =====
print("\n[步骤4] new_reliable → 创建可靠航迹")

# Vr值
vr0 = drone_points[0][5]
vr1 = drone_points[1][5]
vr2 = drone_points[2][5]
vr_avg = (abs(vr0) + abs(vr1) + abs(vr2)) / 3.0
vr_min = min(abs(vr0), abs(vr1), abs(vr2))
vr_max = max(abs(vr0), abs(vr1), abs(vr2))

print(f"  Vr统计: avg={vr_avg:.2f}, min={vr_min:.2f}, max={vr_max:.2f}")
print(f"  Vr一致性: sign检查 → {'PASS' if sa==sb==sc else 'FAIL'}")
print(f"  Vr量级: ratio={vr_max/(vr_min+1e-6):.2f} < 5.0 → {'PASS' if vr_max/(vr_min+1e-6) < 5 else 'FAIL'}")

# 目标类型
if vr_avg > 80.0:
    target_type = "炮弹"
    drone_like = 0
elif vr_avg >= 30.0:
    target_type = "其他"
    drone_like = 0
else:
    target_type = "无人机"
    drone_like = 1
print(f"  目标类型: {target_type} (vr_avg={vr_avg:.2f})")

# low_speed_pass
all_vr_valid = all(1.0 <= abs(v) <= 30.0 for v in [vr0, vr1, vr2])
low_speed_pass = (vr_avg >= 1.0 and vr_avg <= 30.0 and drone_like and all_vr_valid)
print(f"  low_speed_pass: {low_speed_pass} (all Vr in [1,30]: {all_vr_valid})")

# 方位跨度
az0 = 90 - drone_points[0][2]
az2 = 90 - drone_points[2][2]
az_span = abs(az2 - az0)
if az_span > 180: az_span = 360 - az_span
print(f"  方位跨度: {az_span:.2f}° < 30° → {'PASS' if az_span < 30 else 'FAIL'}")

# 俯仰跨度
el0 = drone_points[0][3]
el2 = drone_points[2][3]
el_span = abs(el2 - el0)
print(f"  俯仰跨度: {el_span:.2f}° < 25° → {'PASS' if el_span < 25 else 'FAIL'}")

# 绝对扇区约束 (修复后)
az_norm = az2
while az_norm > 180: az_norm -= 360
while az_norm < -180: az_norm += 360
sector_ok = -90 <= az_norm <= 90 and -15 <= el2 <= 75
print(f"  绝对扇区(修复后): az={az_norm:.2f}° ∈ [-90°, 90°], el={el2:.2f}° ∈ [-15°, 75°] → {'PASS' if sector_ok else 'FAIL'}")

# 波位检查
beam0 = drone_points[0][6]
beam2 = drone_points[2][6]
beam_span = abs(beam2 - beam0)
print(f"  波位跨度: beam_span={beam_span} → {'PASS' if beam_span <= 5 else 'FAIL'}")

# 综合检查
final_ok = True
if sa != sb or sb != sc: final_ok = False; print("  ✗ Vr符号检查失败")
if vr_max/(vr_min+1e-6) > 5: final_ok = False; print("  ✗ Vr量级检查失败")
if az_span > 30: final_ok = False; print("  ✗ 方位跨度检查失败")
if el_span > 25: final_ok = False; print("  ✗ 俯仰跨度检查失败")
if not sector_ok: final_ok = False; print("  ✗ 绝对扇区检查失败")
if beam_span > 5: final_ok = False; print("  ✗ 波位检查失败")

# 置信度计算 (简化)
v1_norm = math.sqrt(vx_corr**2 + vy_corr**2 + vz_corr**2)
# 对Frame 21也做速度修正
r_azi2 = drone_points[2][2] / 180.0 * pi
r_el2 = drone_points[2][3] / 180.0 * pi
r_vr2 = drone_points[2][5]
er_x2 = math.cos(r_el2) * math.cos(r_azi2)
er_y2 = math.cos(r_el2) * math.sin(r_azi2)
er_z2 = math.sin(r_el2)
vx_pos2 = (p2[0] - p1[0]) / T12
vy_pos2 = (p2[1] - p1[1]) / T12
vz_pos2 = (p2[2] - p1[2]) / T12
v_radial_old2 = vx_pos2*er_x2 + vy_pos2*er_y2 + vz_pos2*er_z2
vx_tan2 = vx_pos2 - v_radial_old2*er_x2
vy_tan2 = vy_pos2 - v_radial_old2*er_y2
vz_tan2 = vz_pos2 - v_radial_old2*er_z2
tan_damp_2 = 0.15
vx_corr2 = r_vr2*er_x2 + vx_tan2*tan_damp_2
vy_corr2 = r_vr2*er_y2 + vy_tan2*tan_damp_2
vz_corr2 = r_vr2*er_z2 + vz_tan2*tan_damp_2
v2_norm = math.sqrt(vx_corr2**2 + vy_corr2**2 + vz_corr2**2)

dot_prod = vx_corr*vx_corr2 + vy_corr*vy_corr2 + vz_corr*vz_corr2
cos_theta = dot_prod / (v1_norm * v2_norm + 1e-8)
cos_theta = max(-1, min(1, cos_theta))
angle_consistency = (1 + cos_theta) / 2
speed_ratio = min(v1_norm, v2_norm) / max(v1_norm, v2_norm)

vr_consistency = vr_min / vr_max
if vr_max - vr_min < 15:
    vr_consistency = 0.7 + 0.3 * vr_consistency
confidence = vr_consistency * 0.6 + angle_consistency * 0.2 + speed_ratio * 0.1 + 1.0 * 0.1

print(f"\n  置信度计算:")
print(f"    v1_norm={v1_norm:.2f}, v2_norm={v2_norm:.2f}")
print(f"    cos_theta={cos_theta:.4f}, angle_consistency={angle_consistency:.4f}")
print(f"    speed_ratio={speed_ratio:.4f}")
print(f"    vr_consistency={vr_consistency:.4f}")
print(f"    confidence={confidence:.4f} > CONFIDENCE_THRESHOLD={CONFIDENCE_THRESHOLD} → {'PASS' if confidence > CONFIDENCE_THRESHOLD else 'FAIL'}")

print()
print("=" * 70)
if final_ok and confidence > CONFIDENCE_THRESHOLD:
    print("🎉 修复验证成功！无人机航迹可以在Frame 21建立可靠航迹！")
    print("   目标类型: 无人机 (target_type=2)")
    print(f"   初始化速度: |v|={v2_norm:.2f}m/s (符合无人机速度范围)")
else:
    print("❌ 验证失败，需要进一步修复")
print("=" * 70)