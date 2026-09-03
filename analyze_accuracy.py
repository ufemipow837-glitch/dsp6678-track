# -*- coding: utf-8 -*-
"""
分析输出数据.txt与无人机实测数据.txt的准确度对比
"""
import re
import math

# 读取输出数据
with open('输出数据.txt', 'r', encoding='utf-8') as f:
    output_data = f.read()

# 读取无人机实测数据
with open('无人机实测数据.txt', 'r', encoding='utf-8') as f:
    measured_data = f.read()

# 解析输出数据 - 提取航迹信息
output_tracks = []
lines = output_data.split('\n')
for i, line in enumerate(lines):
    track_match = re.search(r'Track\[0\]: Az=([\d.-]+)deg El=([\d.-]+)deg R=([\d.-]+)m V=([\d.-]+)m/s', line)
    if track_match:
        # 查找对应的帧号
        for j in range(i-1, max(0, i-5), -1):
            frame_match = re.search(r'Frame\s+(\d+)\s+\|\s+t=(\d+)ms', lines[j])
            if frame_match:
                frame = int(frame_match.group(1))
                timestamp = int(frame_match.group(2))
                az = float(track_match.group(1))
                el = float(track_match.group(2))
                r = float(track_match.group(3))
                v = float(track_match.group(4))
                output_tracks.append({
                    'frame': frame,
                    't': timestamp,
                    'az': az,
                    'el': el,
                    'r': r,
                    'v': v
                })
                break

# 解析实测数据 - 提取无人机点
drone_points = []
for line in measured_data.split('\n'):
    if '方位:' in line and '速度:' in line:
        az_match = re.search(r'方位:([\d.-]+)', line)
        r_match = re.search(r'距离:([\d.-]+)', line)
        v_match = re.search(r'速度:([\d.-]+)', line)
        time_match = re.search(r'时间戳:(\d+)', line)
        beam_match = re.search(r'波位号:(\d+)', line)
        
        if az_match and r_match and v_match and time_match:
            az = float(az_match.group(1))
            r = float(r_match.group(1))
            v = float(v_match.group(1))
            t = int(time_match.group(1))
            beam = int(beam_match.group(1)) if beam_match else -1
            
            # 筛选无人机点：方位15-20度，距离900-1300m，速度-15到-5m/s
            if 15 <= az <= 20 and 900 <= r <= 1300 and -15 <= v <= -5:
                drone_points.append({
                    't': t, 'az': az, 'el': 0, 'r': r, 'v': v, 'beam': beam
                })

print('=' * 60)
print('输出航迹统计')
print('=' * 60)
print(f'可靠航迹点数: {len(output_tracks)}')

if output_tracks:
    print(f'帧号范围: Frame {output_tracks[0]["frame"]} - Frame {output_tracks[-1]["frame"]}')
    print(f'时间范围: {output_tracks[0]["t"]}ms - {output_tracks[-1]["t"]}ms')
    
    # 统计输出航迹的方位、距离、速度范围
    az_range = f'{min(t["az"] for t in output_tracks):.2f}° - {max(t["az"] for t in output_tracks):.2f}°'
    r_range = f'{min(t["r"] for t in output_tracks):.1f}m - {max(t["r"] for t in output_tracks):.1f}m'
    v_range = f'{min(t["v"] for t in output_tracks):.1f}m/s - {max(t["v"] for t in output_tracks):.1f}m/s'
    print(f'方位范围: {az_range}')
    print(f'距离范围: {r_range}')
    print(f'速度范围: {v_range}')

print('\n' + '=' * 60)
print('无人机实测点统计')
print('=' * 60)
print(f'无人机点数: {len(drone_points)}')

if drone_points:
    print(f'时间范围: {drone_points[0]["t"]}ms - {drone_points[-1]["t"]}ms')
    
    # 统计无人机的方位、距离、速度范围
    az_range = f'{min(p["az"] for p in drone_points):.2f}° - {max(p["az"] for p in drone_points):.2f}°'
    r_range = f'{min(p["r"] for p in drone_points):.1f}m - {max(p["r"] for p in drone_points):.1f}m'
    v_range = f'{min(p["v"] for p in drone_points):.2f}m/s - {max(p["v"] for p in drone_points):.2f}m/s'
    beams = list(set(p["beam"] for p in drone_points))
    print(f'方位范围: {az_range}')
    print(f'距离范围: {r_range}')
    print(f'速度范围: {v_range}')
    print(f'波位号: {beams}')
    
    print(f'\n前10个无人机点:')
    for i, pt in enumerate(drone_points[:10]):
        print(f'  [{i}] t={pt["t"]}ms Az={pt["az"]:.2f}° R={pt["r"]:.1f}m Vr={pt["v"]:.2f}m/s Beam={pt["beam"]}')

print('\n' + '=' * 60)
print('准确度对比分析')
print('=' * 60)

if output_tracks and drone_points:
    # 计算平均值
    avg_output_az = sum(t['az'] for t in output_tracks) / len(output_tracks)
    avg_output_r = sum(t['r'] for t in output_tracks) / len(output_tracks)
    avg_output_v = sum(t['v'] for t in output_tracks) / len(output_tracks)
    
    avg_measured_az = sum(p['az'] for p in drone_points) / len(drone_points)
    avg_measured_r = sum(p['r'] for p in drone_points) / len(drone_points)
    avg_measured_v = sum(p['v'] for p in drone_points) / len(drone_points)  # 实测是负值，表示靠近雷达
    
    print(f'{"指标":<10} {"输出平均":<15} {"实测平均":<15} {"偏差":<15}')
    print('-' * 55)
    print(f'{"方位":<10} {avg_output_az:<15.2f} {avg_measured_az:<15.2f} {avg_output_az - avg_measured_az:<15.2f}°')
    print(f'{"距离":<10} {avg_output_r:<15.1f} {avg_measured_r:<15.1f} {avg_output_r - avg_measured_r:<15.1f}m')
    print(f'{"速度":<10} {avg_output_v:<15.2f} {avg_measured_v:<15.2f} {avg_output_v - avg_measured_v:<15.2f}m/s')
    
    # 判断跟踪目标是否正确
    az_diff = abs(avg_output_az - avg_measured_az)
    r_diff = abs(avg_output_r - avg_measured_r)
    
    print(f'\n{"="*60}')
    if az_diff > 5:
        print(f'⚠️ 严重问题：输出航迹方位与无人机实测偏差 {az_diff:.2f}° (超过5°)')
        print(f'   代码跟踪的是【错误目标（杂波）】，不是无人机！')
        print(f'   输出方位: {avg_output_az:.2f}°  vs  无人机方位: {avg_measured_az:.2f}°')
    elif r_diff > 200:
        print(f'⚠️ 严重问题：输出航迹距离与无人机实测偏差 {r_diff:.1f}m (超过200m)')
        print(f'   代码跟踪的可能是【错误目标】！')
    else:
        print(f'✅ 输出航迹与无人机实测基本匹配')
    
    # 详细对比：找出输出航迹开始帧的位置
    print(f'\n{"="*60}')
    print('初始化分析')
    print('=' * 60)
    print(f'输出航迹开始帧: Frame {output_tracks[0]["frame"]} (t={output_tracks[0]["t"]}ms)')
    
    # 查找此时无人机的位置
    init_time = output_tracks[0]['t']
    nearby_drone = [p for p in drone_points if abs(p['t'] - init_time) < 5000]
    if nearby_drone:
        nearest = min(nearby_drone, key=lambda p: abs(p['t'] - init_time))
        print(f'最近的无人机点: t={nearest["t"]}ms Az={nearest["az"]:.2f}° R={nearest["r"]:.1f}m Vr={nearest["v"]:.2f}m/s')
        print(f'输出航迹初始: t={output_tracks[0]["t"]}ms Az={output_tracks[0]["az"]:.2f}° R={output_tracks[0]["r"]:.1f}m V={output_tracks[0]["v"]:.2f}m/s')
        print(f'方位差: {abs(output_tracks[0]["az"] - nearest["az"]):.2f}°')
        print(f'距离差: {abs(output_tracks[0]["r"] - nearest["r"]):.1f}m')
        
        if abs(output_tracks[0]["az"] - nearest["az"]) > 10:
            print(f'\n⚠️ 确认：航迹初始化时关联了错误的点（方位差超过10°）')
            print(f'   代码未能正确初始化无人机航迹！')
    
    # 分析问题
    print(f'\n{"="*60}')
    print('根因分析')
    print('=' * 60)
    
    # 检查无人机3点组合的时间间隔
    print('无人机点时间序列分析:')
    for i in range(min(10, len(drone_points))):
        pt = drone_points[i]
        prev_t = drone_points[i-1]['t'] if i > 0 else pt['t']
        dt = pt['t'] - prev_t
        print(f'  [{i}] t={pt["t"]}ms (Δt={dt}ms) Az={pt["az"]:.2f}° R={pt["r"]:.1f}m Vr={pt["v"]:.2f}m/s Beam={pt["beam"]}')
    
    if len(drone_points) >= 3:
        # 检查第一个3点组合的时间间隔
        dt1 = drone_points[1]['t'] - drone_points[0]['t']
        dt2 = drone_points[2]['t'] - drone_points[1]['t']
        print(f'\n前3点时间间隔: Δt1={dt1}ms, Δt2={dt2}ms')
        if dt2 > 2000:
            print(f'⚠️ 第2→3点间隔过长 ({dt2}ms > 2000ms)，可能导致马氏距离检查失败')
            print(f'   这是跨波位扫描间隔，需要放宽协方差参数')
