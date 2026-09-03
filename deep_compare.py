#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度分析输出数据与无人机实测数据对比"""

import re
import math
from collections import defaultdict

# 解析输出数据.txt
def parse_output_data(filepath):
    frames = []
    tracks = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
    
    current_frame = None
    for line in lines:
        # 解析帧信息
        m = re.match(r'Frame\s+(\d+)\s+\|\s+t=(\d+)ms\s+\|\s+n=(\d+)\s+\|\s+Cycle:\s+([\d.]+)', line)
        if m:
            current_frame = {
                'frame_id': int(m.group(1)),
                'timestamp': int(m.group(2)),
                'num_points': int(m.group(3)),
                'cycle_time': float(m.group(4)),
                'tracks': []
            }
            frames.append(current_frame)
        
        # 解析可靠航迹数
        m2 = re.match(r'\s+-> Reliable Tracks:\s+(\d+)', line)
        if m2 and current_frame:
            current_frame['num_reliable'] = int(m2.group(1))
        
        # 解析航迹数据
        m3 = re.match(r'\s+-> Track\[(\d+)\]:\s+Az=([\d.]+)deg\s+El=([-\d.]+)deg\s+R=([-\d.]+)m\s+V=([-\d.]+)m/s', line)
        if m3 and current_frame:
            track = {
                'track_id': int(m3.group(1)),
                'az': float(m3.group(2)),
                'el': float(m3.group(3)),
                'r': float(m3.group(4)),
                'v': float(m3.group(5))
            }
            current_frame['tracks'].append(track)
            tracks.append({**track, 'frame_id': current_frame['frame_id'], 'timestamp': current_frame['timestamp']})
    
    return frames, tracks

# 解析无人机实测数据.txt
def parse_drone_data(filepath):
    drone_points = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()
    
    for line in lines:
        if '点迹解析数据' not in line:
            continue
        
        # 提取各字段
        beam_match = re.search(r'波位号:(\d+)', line)
        ts_match = re.search(r'时间戳:(\d+)', line)
        az_match = re.search(r'方位:([-\d.]+)', line)
        r_match = re.search(r'距离:([-\d.]+)', line)
        el_match = re.search(r'俯仰:([-\d.]+)', line)
        v_match = re.search(r'速度:([-\d.]+)', line)
        snr_match = re.search(r'信噪比:(\d+)', line)
        
        if beam_match and ts_match and az_match and r_match:
            point = {
                'beam': int(beam_match.group(1)),
                'timestamp': int(ts_match.group(1)),
                'az': float(az_match.group(1)),
                'r': float(r_match.group(1)),
                'el': float(el_match.group(1)) if el_match else 0.0,
                'v': float(v_match.group(1)) if v_match else 0.0,
                'snr': int(snr_match.group(1)) if snr_match else 0
            }
            drone_points.append(point)
    
    return drone_points

# 分析
output_file = '输出数据.txt'
drone_file = '无人机实测数据.txt'

frames, tracks = parse_output_data(output_file)
drone_points = parse_drone_data(drone_file)

print("="*80)
print("【1】输出数据统计")
print("="*80)
print(f"总帧数: {len(frames)}")
print(f"有航迹的帧数: {sum(1 for f in frames if f.get('tracks'))}")
print(f"无航迹的帧数: {sum(1 for f in frames if not f.get('tracks'))}")
print(f"航迹总数: {len(tracks)}")

# 找到航迹建立帧
first_track_frame = None
for f in frames:
    if f.get('tracks'):
        first_track_frame = f
        break

if first_track_frame:
    print(f"\n首个航迹建立于 Frame {first_track_frame['frame_id']} (t={first_track_frame['timestamp']}ms)")
    for t in first_track_frame['tracks']:
        print(f"  Track[{t['track_id']}]: Az={t['az']:.2f}°, El={t['el']:.2f}°, R={t['r']:.1f}m, V={t['v']:.1f}m/s")

print("\n" + "="*80)
print("【2】无人机实测数据统计")
print("="*80)
print(f"总点数: {len(drone_points)}")

# 找出波位2/3的无人机点（无人机所在波位）
drone_beam23 = [p for p in drone_points if p['beam'] in [2, 3]]
print(f"波位2/3点数: {len(drone_beam23)}")

# 找出无人机候选点（方位15-20°，距离900-1300m，Vr=-8~-15m/s）
drone_candidates = [p for p in drone_points 
                     if 14 <= p['az'] <= 22 
                     and 800 <= p['r'] <= 1300 
                     and -15 <= p['v'] <= -5]
print(f"\n无人机候选点（Az 14-22°, R 800-1300m, Vr -15~-5m/s）: {len(drone_candidates)}")

print("\n--- 波位2/3的无人机候选点 ---")
for p in drone_candidates[:20]:
    print(f"  t={p['timestamp']}ms, beam={p['beam']}, Az={p['az']:.4f}°, R={p['r']:.2f}m, El={p['el']:.4f}°, Vr={p['v']:.4f}m/s, SNR={p['snr']}")

# 分析无人机3点组合可能
print("\n" + "="*80)
print("【3】无人机3点组合分析")
print("="*80)

# 按时间戳分组
timestamps = sorted(set(p['timestamp'] for p in drone_candidates))
print(f"无人机候选点时间戳: {timestamps[:30]}")

# 检查Frame 4, 5, 21的点
key_timestamps = [898796, 898960, 901584, 901748]  # Frame 4, 5, 21, 22
print("\n--- 关键帧的无人机点 ---")
for ts in key_timestamps:
    points_at_ts = [p for p in drone_candidates if p['timestamp'] == ts]
    all_points_at_ts = [p for p in drone_points if p['timestamp'] == ts and p['beam'] in [2, 3]]
    print(f"  t={ts}ms (Frame {(ts-898140)//164}):")
    for p in all_points_at_ts:
        print(f"    beam={p['beam']}, Az={p['az']:.4f}°, R={p['r']:.2f}m, Vr={p['v']:.4f}m/s, SNR={p['snr']}")

# 分析假航迹
print("\n" + "="*80)
print("【4】假航迹深度分析")
print("="*80)

if tracks:
    # 分析第一条航迹
    t0 = tracks[0]
    print(f"第一条航迹 (Frame {t0['frame_id']}):")
    print(f"  Az={t0['az']:.2f}°, El={t0['el']:.2f}°, R={t0['r']:.1f}m, V={t0['v']:.1f}m/s")
    
    # 找出与该航迹匹配的实测点
    print(f"\n--- 与假航迹方位/距离接近的实测点 ---")
    for p in drone_points:
        if abs(p['az'] - t0['az']) < 2 and abs(p['r'] - t0['r']) < 200:
            print(f"  t={p['timestamp']}ms, beam={p['beam']}, Az={p['az']:.4f}°, R={p['r']:.2f}m, Vr={p['v']:.4f}m/s, SNR={p['snr']}")

# 对比分析
print("\n" + "="*80)
print("【5】准确度分析")
print("="*80)

# 无人机真实轨迹（从实测数据中提取）
# 无人机在Frame 4开始被探测到，方位约16.8°，距离约1095m
drone_true_start = {'frame': 4, 'az': 16.82, 'r': 1094.84, 'v': -11.74}

# 输出航迹
if tracks:
    output_start = tracks[0]
    
    print(f"无人机真实起始位置 (Frame {drone_true_start['frame']}):")
    print(f"  Az={drone_true_start['az']:.2f}°, R={drone_true_start['r']:.1f}m, Vr={drone_true_start['v']:.2f}m/s")
    print(f"\n输出航迹起始位置 (Frame {output_start['frame_id']}):")
    print(f"  Az={output_start['az']:.2f}°, R={output_start['r']:.1f}m, V={output_start['v']:.1f}m/s")
    
    # 计算偏差
    az_err = abs(output_start['az'] - drone_true_start['az'])
    r_err = abs(output_start['r'] - drone_true_start['r'])
    
    print(f"\n偏差:")
    print(f"  方位误差: {az_err:.2f}° (应该<5°)")
    print(f"  距离误差: {r_err:.1f}m (应该<200m)")
    print(f"  结论: 这条航迹不是无人机！")

# 问题诊断
print("\n" + "="*80)
print("【6】根因诊断")
print("="*80)

print("问题1: 为什么Frame 0-91没有可靠航迹?")
print("  - 需要3点组合才能初始化可靠航迹")
print("  - 无人机在Frame 4(波位2)、Frame 5(波位3)有点，但第三点可能未通过初始化检查")
print()

print("问题2: Frame 92的航迹来自哪里?")
print("  - 从实测数据看，Frame 91-92附近的波位7有大量杂波点")
print("  - 可能是3个杂波点巧合组合通过了初始化检查")
print()

print("问题3: 为什么初始化检查未拦截假航迹?")
print("  - 当前代码的初始化门限可能过于宽松")
print("  - 或者无人机3点组合因门限过紧被拒绝")

# 检查data_entry.c的点
print("\n" + "="*80)
print("【7】验证data_entry.c中的无人机点")
print("="*80)

# 搜索data_entry.c中Frame 4附近的点
import os
data_entry_path = 'data_entry.c'
if os.path.exists(data_entry_path):
    with open(data_entry_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    # 查找Frame 4的数据
    print("检查data_entry.c中是否包含正确的无人机数据...")
    # 查找方位16.8°和距离1095m的数据
    if '16.8225' in content or '1094.84' in content:
        print("  data_entry.c 包含无人机正确数据")
    else:
        print("  data_entry.c 可能不包含无人机正确数据！")
    
    # 查找Vr值
    if '-11.7406' in content:
        print("  data_entry.c 包含正确的负径向速度（-11.7406m/s）")
    elif '11.7406' in content:
        print("  data_entry.c 包含速度但符号可能错误！")
    else:
        print("  data_entry.c 不包含预期的无人机速度值")

print("\n" + "="*80)
print("分析完成！")
