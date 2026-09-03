# -*- coding: utf-8 -*-
"""
完整追踪无人机3点初始化流程
精确模拟C代码逻辑，定位失败点
"""
import math
import sys

pi = 3.141592653589793

# ===== 参数配置 =====
TRACK_INIT_TH = 15.0       # 马氏距离阈值
TRACK_ASSO_TH = 11.3       # 关联马氏距离阈值  
VELOCITY_CONSISTENCY_THRESHOLD = 0.5
TRACK_QUALITY_THRESHOLD = 0.8
ANGLE_CHANGE_THRESHOLD = 160.0
CONFIDENCE_THRESHOLD = 0.65
INIT_SIGMA_A = 1.0   # 方位角测量噪声(度)
INIT_SIGMA_B = 1.0   # 俯仰角测量噪声(度)
INIT_SIGMA_R = 20.0  # 距离测量噪声(m)
Vmin = 2.0
Vmax = 1500.0
time_up = 25000.0
time_down = 0.0

# ===== 模拟无人机数据点 =====
# 从data_entry.c提取的真实数据
drone_points = [
    # (frame_idx, mSecond, azi_deg, el_deg, range_m, velocity, beamNo, targetNum_index)
    (4,  898796, 16.8225, 5.1668, 1094.84, -11.7406, 2, 0),
    (5,  898960, 17.3911, 4.7861, 1095.84, -11.7406, 3, 1),  # Frame 5 point index 1
    (21, 901584, 16.5781, 4.6226, 1124.10, -11.7406, 2, 0),
    (22, 901748, 17.2821, 4.6089, 1124.83, -11.7406, 3, 0),
    (38, 904372, 18.1500, 4.5000, 1160.00, -11.1945, 2, 0),  # 估算
    (39, 904536, 18.6000, 4.4500, 1175.00, -11.1945, 3, 0),  # 估算
]

def polar_to_cart(azi_deg, el_deg, r):
    """球坐标→直角坐标 (azi为数学方位角: atan2(y/x))"""
    azi = azi_deg / 180.0 * pi
    el = el_deg / 180.0 * pi
    x = r * math.cos(el) * math.cos(azi)
    y = r * math.cos(el) * math.sin(azi)
    z = r * math.sin(el)
    return x, y, z

def cart_to_polar(x, y, z):
    """直角坐标→球坐标 (数学方位角)"""
    r = math.sqrt(x*x + y*y + z*z)
    el = math.asin(z / r) if r > 0 else 0
    azi = math.atan2(y, x)
    azi_deg = azi * 180.0 / pi
    el_deg = el * 180.0 / pi
    return azi_deg, el_deg, r

def radar_az_from_math_azi(math_azi_deg):
    """数学方位角→雷达方位角 (正北=0°, 顺时针)"""
    return 90.0 - math_azi_deg

# ===== 计算位置 =====
print("=" * 70)
print("无人机数据点位置计算")
print("=" * 70)
positions = []
for i, (frame, msec, azi_d, el_d, r, vel, beam, idx) in enumerate(drone_points):
    x, y, z = polar_to_cart(azi_d, el_d, r)
    azi_radar = radar_az_from_math_azi(azi_d)
    positions.append((x, y, z))
    print(f"  Frame {frame:3d}, Beam {beam}, mSecond={msec:.0f}ms")
    print(f"    数学方位角={azi_d:.4f}°, 雷达方位角={azi_radar:.4f}°, 俯仰={el_d:.4f}°, 距离={r:.2f}m")
    print(f"    位置: x={x:.2f}, y={y:.2f}, z={z:.2f}")
    print(f"    速度: Vr={vel:.4f} m/s")
    print()

# ===== 模拟 get_track_asso1 =====
print("=" * 70)
print("模拟 get_track_asso1 (2点初始化)")
print("=" * 70)

p0 = positions[0]  # Frame 4, beam 2
p1 = positions[1]  # Frame 5, beam 3

T01 = (drone_points[1][1] - drone_points[0][1]) / 1000.0
print(f"时间差 T = {T01:.4f}s ({drone_points[1][1]} - {drone_points[0][1]} ms)")

# kalman_filter_init: 速度 = (Z1 - Z0) / T
vx_pos = (p1[0] - p0[0]) / T01
vy_pos = (p1[1] - p0[1]) / T01
vz_pos = (p1[2] - p0[2]) / T01
print(f"位置差分速度: vx={vx_pos:.2f}, vy={vy_pos:.2f}, vz={vz_pos:.2f}")
print(f"位置差分速度大小: |v|={math.sqrt(vx_pos**2+vy_pos**2+vz_pos**2):.2f} m/s")

# 雷达Vr修正
r_azi = drone_points[1][2] / 180.0 * pi  # 数学方位角 (rad)
r_el = drone_points[1][3] / 180.0 * pi   # 俯仰角 (rad)
r_vr = drone_points[1][5]                 # 雷达径向速度

er_x = math.cos(r_el) * math.cos(r_azi)
er_y = math.cos(r_el) * math.sin(r_azi)
er_z = math.sin(r_el)
print(f"径向方向: er_x={er_x:.4f}, er_y={er_y:.4f}, er_z={er_z:.4f}")

v_radial_old = vx_pos*er_x + vy_pos*er_y + vz_pos*er_z
print(f"位置差分速度的径向分量: v_radial_old={v_radial_old:.2f} m/s")
print(f"雷达实测Vr: r_vr={r_vr:.2f} m/s")
print(f"径向分量误差: {abs(v_radial_old - r_vr):.2f} m/s")

vx_tan = vx_pos - v_radial_old * er_x
vy_tan = vy_pos - v_radial_old * er_y
vz_tan = vz_pos - v_radial_old * er_z
print(f"切向分量: vx_tan={vx_tan:.2f}, vy_tan={vy_tan:.2f}, vz_tan={vz_tan:.2f}")

tan_damp_1 = 0.08
vx_corr = r_vr * er_x + vx_tan * tan_damp_1
vy_corr = r_vr * er_y + vy_tan * tan_damp_1
vz_corr = r_vr * er_z + vz_tan * tan_damp_1
print(f"修正后速度 (tan_damp={tan_damp_1}): vx={vx_corr:.2f}, vy={vy_corr:.2f}, vz={vz_corr:.2f}")
print(f"修正后速度大小: |v|={math.sqrt(vx_corr**2+vy_corr**2+vz_corr**2):.2f} m/s")

# ===== 模拟 get_track_asso2 =====
print()
print("=" * 70)
print("模拟 get_track_asso2 (3点初始化)")
print("=" * 70)

# 第3点: Frame 21, beam 2
p2 = positions[2]
T12 = (drone_points[2][1] - drone_points[1][1]) / 1000.0
print(f"时间差 T = {T12:.4f}s ({drone_points[2][1]} - {drone_points[1][1]} ms)")

# kalman_filter_init: Z1是修正后的位置(p1), Z2是新测量(p2)
# 但注意！temp_track中存储的是 CORRECTED STATE，包括修正后的位置
# 修正后的位置就是 p1 (因为X0[0] = Z1[0] = target_data_ptr->x = p1[0])
vx_pos2 = (p2[0] - p1[0]) / T12
vy_pos2 = (p2[1] - p1[1]) / T12
vz_pos2 = (p2[2] - p1[2]) / T12
print(f"位置差分速度(point1→point2): vx={vx_pos2:.2f}, vy={vy_pos2:.2f}, vz={vz_pos2:.2f}")
print(f"位置差分速度大小: |v|={math.sqrt(vx_pos2**2+vy_pos2**2+vz_pos2**2):.2f} m/s")

# 雷达Vr修正 (第3点的Vr)
r_azi2 = drone_points[2][2] / 180.0 * pi
r_el2 = drone_points[2][3] / 180.0 * pi
r_vr2 = drone_points[2][5]

er_x2 = math.cos(r_el2) * math.cos(r_azi2)
er_y2 = math.cos(r_el2) * math.sin(r_azi2)
er_z2 = math.sin(r_el2)
print(f"径向方向(point2): er_x={er_x2:.4f}, er_y={er_y2:.4f}, er_z={er_z2:.4f}")

v_radial_old2 = vx_pos2*er_x2 + vy_pos2*er_y2 + vz_pos2*er_z2
print(f"位置差分速度的径向分量: v_radial_old2={v_radial_old2:.2f} m/s")
print(f"雷达实测Vr: r_vr2={r_vr2:.2f} m/s")

vx_tan2 = vx_pos2 - v_radial_old2 * er_x2
vy_tan2 = vy_pos2 - v_radial_old2 * er_y2
vz_tan2 = vz_pos2 - v_radial_old2 * er_z2

tan_damp_2 = 0.15
vx_corr2 = r_vr2 * er_x2 + vx_tan2 * tan_damp_2
vy_corr2 = r_vr2 * er_y2 + vy_tan2 * tan_damp_2
vz_corr2 = r_vr2 * er_z2 + vz_tan2 * tan_damp_2
print(f"修正后速度 (tan_damp={tan_damp_2}): vx={vx_corr2:.2f}, vy={vy_corr2:.2f}, vz={vz_corr2:.2f}")
print(f"修正后速度大小: |v|={math.sqrt(vx_corr2**2+vy_corr2**2+vz_corr2**2):.2f} m/s")

# ===== 马氏距离检查 =====
print()
print("=" * 70)
print("马氏距离检查 (3点初始化时)")
print("=" * 70)

# 预测位置: 从 point1 的状态(位置+修正后速度) 预测到 point2 时刻
# X_pred = X1 + v_corr2 * T12
x_pred = p1[0] + vx_corr2 * T12
y_pred = p1[1] + vy_corr2 * T12
z_pred = p1[2] + vz_corr2 * T12
print(f"预测位置: x_pred={x_pred:.2f}, y_pred={y_pred:.2f}, z_pred={z_pred:.2f}")
print(f"实际位置: x_meas={p2[0]:.2f}, y_meas={p2[1]:.2f}, z_meas={p2[2]:.2f}")

# 创新 (innovation)
innovation_x = p2[0] - x_pred
innovation_y = p2[1] - y_pred
innovation_z = p2[2] - z_pred
print(f"创新: dx={innovation_x:.2f}, dy={innovation_y:.2f}, dz={innovation_z:.2f}")

# 马氏距离计算
# 测量噪声协方差 R
# R 是3x3矩阵，基于距离、方位角、俯仰角的噪声
# 近似: sigma_r = 20m, sigma_az = 1°*range, sigma_el = 1°*range
sigma_r = INIT_SIGMA_R  # 20m
sigma_az_rad = INIT_SIGMA_A / 180.0 * pi  # 1° in radians
sigma_el_rad = INIT_SIGMA_B / 180.0 * pi  # 1° in radians

# 将角度噪声转换为位置噪声(近似)
r_mean = (drone_points[1][4] + drone_points[2][4]) / 2.0
sigma_x_az = r_mean * sigma_az_rad  # 方位角→横向位置
sigma_y_el = r_mean * sigma_el_rad  # 俯仰角→纵向位置

# 简化的创新协方差 S (对角近似)
S_xx = sigma_r**2 + sigma_x_az**2  # x方向方差
S_yy = sigma_r**2 + sigma_y_el**2  # y方向方差  
S_zz = sigma_r**2  # z方向方差

print(f"测量噪声: sigma_r={sigma_r}m, sigma_az={sigma_az_rad*180/pi:.2f}°, sigma_el={sigma_el_rad*180/pi:.2f}°")
print(f"位置噪声: sigma_x_az={sigma_x_az:.2f}m, sigma_y_el={sigma_y_el:.2f}m")
print(f"创新协方差 S: diag({S_xx:.0f}, {S_yy:.0f}, {S_zz:.0f})")

# 马氏距离
d_maha = math.sqrt(
    innovation_x**2 / S_xx + 
    innovation_y**2 / S_yy + 
    innovation_z**2 / S_zz
)
print(f"马氏距离 d_maha = {d_maha:.2f}")
print(f"阈值 TRACK_INIT_TH = {TRACK_INIT_TH}")
print(f"马氏距离检查: {'PASS' if d_maha < TRACK_INIT_TH else 'FAIL'}!")

# ===== 置信度计算 =====
print()
print("=" * 70)
print("置信度计算")
print("=" * 70)

# 基于Vr的一致性
vr_values = [drone_points[0][5], drone_points[1][5], drone_points[2][5]]
vr_avg = sum(vr_values) / len(vr_values)
vr_avg_abs = sum(abs(v) for v in vr_values) / len(vr_values)
vr_min_abs = min(abs(v) for v in vr_values)
vr_max_abs = max(abs(v) for v in vr_values)

print(f"Vr值: {vr_values}")
print(f"Vr平均: {vr_avg:.4f}, |Vr|平均: {vr_avg_abs:.4f}")
print(f"|Vr|最小: {vr_min_abs:.4f}, |Vr|最大: {vr_max_abs:.4f}")

# 质量计算
quality = 0.0
if vr_max_abs > 0.01 and vr_min_abs > 0.01:
    ratio = vr_min_abs / vr_max_abs
    quality += 0.4 * ratio
    print(f"  一致性比: {ratio:.4f}, quality += {0.4*ratio:.4f}")

if Vmin <= vr_avg_abs <= Vmax:
    quality += 0.3
    print(f"  速度在 [{Vmin}, {Vmax}] 范围内, quality += 0.3")
elif vr_avg_abs > Vmax:
    print(f"  速度超过Vmax, quality += 0.0")
else:
    quality += 0.15
    print(f"  速度低于Vmin, quality += 0.15")

diff = vr_max_abs - vr_min_abs
if diff < 15.0:
    quality += 0.3
    print(f"  速度差 {diff:.4f} < 15m/s, quality += 0.3")
elif diff < 30.0:
    quality += 0.15
    print(f"  速度差 {diff:.4f} < 30m/s, quality += 0.15")

quality = min(quality, 1.0)
print(f"最终质量: {quality:.4f}")
print(f"阈值 TRACK_QUALITY_THRESHOLD = {TRACK_QUALITY_THRESHOLD}")
print(f"质量检查: {'PASS' if quality >= TRACK_QUALITY_THRESHOLD else 'FAIL'}!")

# ===== 综合诊断 =====
print()
print("=" * 70)
print("诊断总结")
print("=" * 70)
print(f"1. 速度修正后大小: {math.sqrt(vx_corr2**2+vy_corr2**2):.2f} m/s (应 <= 20)")
print(f"2. 马氏距离: {d_maha:.2f} (阈值 {TRACK_INIT_TH})")
print(f"3. 速度质量: {quality:.4f} (阈值 {TRACK_QUALITY_THRESHOLD})")
print(f"4. 综合置信度需 > {CONFIDENCE_THRESHOLD}")

# 关键洞察
print()
print("=" * 70)
print("关键洞察: 速度修正算法问题")
print("=" * 70)
print(f"问题1: 位置差分速度的径向分量 v_radial_old = {v_radial_old:.2f} m/s")
print(f"        与雷达Vr = {r_vr:.2f} m/s 符号相反!")
print(f"        这是因为跨波束方位角变化导致的虚假切向速度")
print()
print(f"问题2: 修正公式 v_corrected = Vr*er + v_tan*tan_damp")
print(f"        v_tan 基于错误的径向分解(v_radial_old={v_radial_old:.2f})")
print(f"        导致 v_tan 本身就是错误的")
print()
print(f"问题3: 对于get_track_asso1 (2点):")
print(f"        vx_tan={vx_tan:.2f}, vy_tan={vy_tan:.2f}")
print(f"        tan_damp=0.08, 修正后切向=({vx_tan*tan_damp_1:.2f}, {vy_tan*tan_damp_1:.2f})")
print()
print(f"建议修复:")
print(f"  方案A: 完全使用雷达Vr, 不使用位置差分切向分量")
print(f"         vx = r_vr*er_x, vy = r_vr*er_y, vz = r_vr*er_z")
print(f"  方案B: 使用更小的tan_damp (如0.02)")
print(f"  方案C: 当位置差分速度与Vr方向相反时, 完全丢弃位置差分速度")