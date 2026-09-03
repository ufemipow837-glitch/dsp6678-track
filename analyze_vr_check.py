# -*- coding: utf-8 -*-
"""
分析为什么无人机3点组合无法建立 - 检查Vr符号检查逻辑
在Frame 4+5之后，Frame 6-20的杂波点可能先匹配了无人机的temp_track
"""
import math

pi = math.pi

# 无人机点
drone_points = [
    {'t': 898796, 'az': 16.82, 'el': 5.17, 'r': 1094.8, 'vr': -11.74, 'beam': 2, 'frame': 4},
    {'t': 898960, 'az': 17.39, 'el': 4.79, 'r': 1095.8, 'vr': -11.74, 'beam': 3, 'frame': 5},
    {'t': 901584, 'az': 16.58, 'el': 4.62, 'r': 1124.1, 'vr': -11.74, 'beam': 2, 'frame': 21},
]

# 检查Frame 4和5之后，哪些帧的点可能匹配
# 从实测数据中提取的Frame 6-20的点（部分）
clutter_points = [
    # Frame 6 (t=899124) - 波位4
    {'t': 899124, 'az': 27.80, 'el': -1.66, 'r': 1339.1, 'vr': -0.82, 'beam': 4, 'frame': 6},
    {'t': 899124, 'az': 25.99, 'el': -0.64, 'r': 1458.2, 'vr': -3.82, 'beam': 4, 'frame': 6},
    {'t': 899124, 'az': 26.83, 'el': -0.50, 'r': 2490.1, 'vr': -20.20, 'beam': 4, 'frame': 6},
    {'t': 899124, 'az': 25.87, 'el': 0.39, 'r': 3834.7, 'vr': -31.67, 'beam': 4, 'frame': 6},
    {'t': 899124, 'az': 24.63, 'el': 0.03, 'r': 3931.2, 'vr': -0.55, 'beam': 4, 'frame': 6},
    {'t': 899124, 'az': 26.73, 'el': 0.42, 'r': 4051.0, 'vr': -64.16, 'beam': 4, 'frame': 6},
    {'t': 899124, 'az': 21.50, 'el': 0.32, 'r': 5126.3, 'vr': -8.19, 'beam': 4, 'frame': 6},
    {'t': 899124, 'az': 23.89, 'el': 0.03, 'r': 5879.3, 'vr': -1.91, 'beam': 4, 'frame': 6},
    {'t': 899124, 'az': 23.76, 'el': -0.08, 'r': 5895.1, 'vr': -0.55, 'beam': 4, 'frame': 6},
    # Frame 7 (t=899288) - 无点
    # Frame 8 (t=899452) - 波位6
    {'t': 899452, 'az': 88.51, 'el': 0.19, 'r': 380.6, 'vr': -24.03, 'beam': 6, 'frame': 8},
    # Frame 9 (t=899616) - 波位7
    {'t': 899616, 'az': 39.91, 'el': -1.66, 'r': 1673.1, 'vr': -0.82, 'beam': 7, 'frame': 9},
    {'t': 899616, 'az': 39.45, 'el': 0.40, 'r': 9497.8, 'vr': -1.09, 'beam': 7, 'frame': 9},
    {'t': 899616, 'az': 36.43, 'el': 0.01, 'r': 9112.7, 'vr': -0.55, 'beam': 7, 'frame': 9},
    # Frame 10 (t=899780) - 波位8
    # Frame 11 (t=899944) - 波位9
    {'t': 899944, 'az': 48.78, 'el': 5.89, 'r': 1987.0, 'vr': -9.01, 'beam': 9, 'frame': 11},
    {'t': 899944, 'az': 53.59, 'el': 8.14, 'r': 10147.4, 'vr': -27.03, 'beam': 9, 'frame': 11},
    {'t': 899944, 'az': 50.17, 'el': 20.44, 'r': 1818.9, 'vr': -47.51, 'beam': 9, 'frame': 11},
    {'t': 899944, 'az': 50.17, 'el': 20.83, 'r': 5442.5, 'vr': -135.74, 'beam': 9, 'frame': 11},
    # Frame 12-20 (t=900108-901420)
    {'t': 901584, 'az': 16.58, 'el': 4.62, 'r': 1124.1, 'vr': -11.74, 'beam': 2, 'frame': 21},
]

# 分析Vr符号检查逻辑
print("=" * 60)
print("分析Vr符号检查逻辑（count==2时）")
print("=" * 60)
print(f"无人机点Vr: Frame4={drone_points[0]['vr']}, Frame5={drone_points[1]['vr']}")
print(f"两个点的Vr符号: sa=-1, sb=-1 (都是负的)")
print(f"\n检查Frame 6-20中哪些点会通过Vr符号检查:")
print(f"{'帧号':<8} {'时间':<10} {'方位':<8} {'距离':<10} {'Vr':<10} {'Vr符号':<8} {'能否通过':<10}")
print("-" * 70)

sa = -1  # Frame 4的Vr符号
sb = -1  # Frame 5的Vr符号

for pt in clutter_points:
    vr = pt['vr']
    sc = 0
    if vr > 0.0:
        sc = 1
    elif vr < 0.0:
        sc = -1
    
    # Vr符号检查: (sa != sb) || (sb != sc)
    # 由于sa==sb==-1，只要sc==-1就通过
    vr_symbol_ok = (sa == sb == sc) or (sc == 0) or (sa == 0) or (sb == 0)
    
    status = "通过!" if (sc != 0 and sa != 0 and sb != 0 and vr_symbol_ok) else "拒绝"
    print(f"  F{pt['frame']:<6} {pt['t']:<8}ms {pt['az']:<7.2f}° {pt['r']:<9.1f}m {vr:<9.2f}m/s {sc:<8} {status:<10}")

print(f"\n{'='*60}")
print("问题分析")
print("=" * 60)

# 计算每个点与无人机第二点的距离差
ref_point = drone_points[1]  # Frame 5作为参考
ref_xyz = None

def sph2cart(r, az_deg, el_deg):
    az = az_deg * pi / 180.0
    el = el_deg * pi / 180.0
    x = r * math.cos(el) * math.cos(az)
    y = r * math.cos(el) * math.sin(az)
    z = r * math.sin(el)
    return [x, y, z]

ref_xyz = sph2cart(ref_point['r'], ref_point['az'], ref_point['el'])

print(f"\n以Frame 5为参考点，计算每个候选点的距离差:")
print(f"{'帧号':<8} {'方位':<8} {'距离':<10} {'Vr':<10} {'XYZ距离':<10} {'通过Vr检查':<12} {'门限检查':<10}")
print("-" * 80)

for pt in clutter_points:
    if pt['frame'] <= 5:
        continue
    
    pt_xyz = sph2cart(pt['r'], pt['az'], pt['el'])
    
    # 计算3D距离
    dx = pt_xyz[0] - ref_xyz[0]
    dy = pt_xyz[1] - ref_xyz[1]
    dz = pt_xyz[2] - ref_xyz[2]
    dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    # Vr符号检查
    vr = pt['vr']
    sc = 0
    if vr > 0.0:
        sc = 1
    elif vr < 0.0:
        sc = -1
    vr_symbol_ok = (sa == sb == sc) or (sc == 0) or (sa == 0) or (sb == 0)
    
    # 计算lag_time
    lag_time = pt['t'] - drone_points[1]['t']  # 与Frame 5的时间差
    T = lag_time / 1000.0
    
    # 距离门限（count==2时使用马氏距离，但先看gate）
    gate = 1500.0 * T + 150.0  # count==1时的门限（用于估算）
    gate_sq = gate * gate
    
    passed = "通过!" if vr_symbol_ok else "拒绝"
    
    print(f"  F{pt['frame']:<6} {pt['az']:<7.2f}° {pt['r']:<9.1f}m {vr:<9.2f}m/s {dist_3d:<9.1f}m {passed:<12} gate={gate:.0f}m")

print(f"\n{'='*60}")
print("结论")
print("=" * 60)
print(f"问题：在Frame 5到Frame 21之间，有多个杂波点的Vr为负值，")
print(f"会通过Vr符号检查，消耗无人机的temp_track。")
print(f"\n例如：Frame 6的R=1339m点，Vr=-0.82m/s，距离Frame 5约XXXm，")
print(f"如果马氏距离检查通过，就会消耗掉无人机的temp_track。")
print(f"\n修复方案：")
print(f"  1. 在Vr符号检查后，增加Vr量级相似性检查")
print(f"  2. 要求新点的Vr与已有点的Vr量级相近（比如相差不超过5倍）")
print(f"  3. 对无人机目标（Vr在1-30m/s），设置更严格的Vr门限")
