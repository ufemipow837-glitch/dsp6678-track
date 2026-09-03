#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
深度诊断：用完整C代码逻辑模拟，找出无人机3点组合初始化失败的根因
"""
import csv, math, os

PLOT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_parsed.csv')
CPI_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_cpi.csv')

cpi_list = []
with open(CPI_CSV, 'r', encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        cpi_list.append({
            't': int(row['时间戳(ms)']),
            'beam': int(row['波位号']),
            'n': int(row['目标数']),
        })

plots_by_t = {}
with open(PLOT_CSV, 'r', encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        if int(row['是否有效点']) == 1:
            t = int(row['时间戳(ms)'])
            if t not in plots_by_t:
                plots_by_t[t] = []
            plots_by_t[t].append({
                'az': float(row['方位(度)']),
                'el': float(row['俯仰(度)']),
                'r': float(row['距离(m)']),
                'v': float(row['径向速度(m/s)']),
                'beam': int(row['波位号']),
            })

PI = 3.141592653589793
Vmin = 2.0
Vmax = 1500.0
time_up_ms = 25000.0

# 加载所有帧数据
frames_data = []
for i in range(200):
    if i >= len(cpi_list):
        break
    cpi = cpi_list[i]
    t = cpi['t']
    beam = cpi['beam']
    plots = plots_by_t.get(t, [])
    frame_points = []
    for p in plots[:60]:
        az_rad = p['az'] * PI / 180.0
        el_rad = p['el'] * PI / 180.0
        r = p['r']
        v = p['v']
        x = r * math.cos(el_rad) * math.cos(az_rad)
        y = r * math.cos(el_rad) * math.sin(az_rad)
        z = r * math.sin(el_rad)
        frame_points.append({
            'az': az_rad, 'el': el_rad, 'r': r, 'v': v,
            'x': x, 'y': y, 'z': z,
            'az_deg': p['az'], 'el_deg': p['el'],
            'beam': beam, 't': t, 'frame': i
        })
    frames_data.append({
        'frame': i, 't': t, 'beam': beam,
        'n': len(frame_points), 'points': frame_points
    })

def calculate_vr_quality(vr0, vr1, vr2):
    sign0 = 1 if vr0 > 0.01 else (-1 if vr0 < -0.01 else 0)
    sign1 = 1 if vr1 > 0.01 else (-1 if vr1 < -0.01 else 0)
    sign2 = 1 if vr2 > 0.01 else (-1 if vr2 < -0.01 else 0)
    n_valid = (sign0 != 0) + (sign1 != 0) + (sign2 != 0)
    if n_valid >= 2:
        if sign0 != 0 and sign1 != 0 and sign0 != sign1: return 0.0
        if sign0 != 0 and sign2 != 0 and sign0 != sign2: return 0.0
        if sign1 != 0 and sign2 != 0 and sign1 != sign2: return 0.0
    else:
        return 0.0
    valid_vrs = [abs(v) for s, v in [(sign0, vr0), (sign1, vr1), (sign2, vr2)] if s != 0]
    vr_avg_abs = sum(valid_vrs) / len(valid_vrs)
    vr_min_abs = min(valid_vrs)
    vr_max_abs = max(valid_vrs)
    quality = 0.0
    if vr_max_abs > 1e-6 and vr_min_abs > 1e-6:
        quality += 0.4 * vr_min_abs / vr_max_abs
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

# 模拟完整的track_initial.c逻辑
temp_tracks = []  # 每个元素是3个dict的列表
num_temp_max = 200

# 统计
init_log = []  # 记录每帧发生了什么

for fi, frame in enumerate(frames_data):
    points = frame['points']
    if not points:
        continue
    frame_t = frame['t']
    
    # Step 1: 整理temp_tracks (与track_initial.c lines 330-370一致)
    # 跳过lag_time超过time_up的track
    valid_tracks = []
    for tt in temp_tracks:
        count = sum(1 for slot in tt if slot.get('asso_flag') == 1)
        if count == 0:
            continue
        last_t = tt[count-1]['t']
        lag = frame_t - last_t
        if 0 <= lag <= time_up_ms:
            valid_tracks.append(tt)
    temp_tracks = valid_tracks
    
    # Step 2: 为每个点匹配temp_track或创建新track
    tt_used = [False] * len(temp_tracks)
    
    for pi, pt in enumerate(points):
        best_tt_idx = -1
        best_tt_dist = float('inf')
        best_tt_count = 0
        
        for ti, tt in enumerate(temp_tracks):
            if tt_used[ti]:
                continue
            count = sum(1 for slot in tt if slot.get('asso_flag') == 1)
            if count == 0:
                continue
            
            last_t = tt[count-1]['t']
            lag_time = frame_t - last_t
            if lag_time < 0 or lag_time > time_up_ms:
                continue
            
            T = lag_time / 1000.0
            
            if count == 1:
                # Vr符号预筛选
                vr_exist = tt[0]['vr']
                vr_new = pt['v']
                if (vr_exist * vr_new < 0.0) and abs(vr_exist) > 3.0 and abs(vr_new) > 3.0:
                    continue
                
                # 距离检查(笛卡尔)
                dx = pt['x'] - tt[0]['x']
                dy = pt['y'] - tt[0]['y']
                dz = pt['z'] - tt[0]['z']
                dist_sq = dx*dx + dy*dy + dz*dz
                gate = 1500.0 * T + 150.0
                if dist_sq < gate * gate and dist_sq < best_tt_dist:
                    best_tt_dist = dist_sq
                    best_tt_idx = ti
                    best_tt_count = 1
            
            elif count == 2:
                # 3点Vr符号一致性检查
                vr_a = tt[0]['vr']
                vr_b = tt[1]['vr']
                vr_c = pt['v']
                sa = 1 if vr_a > 0 else (-1 if vr_a < 0 else 0)
                sb = 1 if vr_b > 0 else (-1 if vr_b < 0 else 0)
                sc = 1 if vr_c > 0 else (-1 if vr_c < 0 else 0)
                if sa != 0 and sb != 0 and sc != 0:
                    if (sa != sb) or (sb != sc):
                        continue
                
                # 简化马氏距离(实际用get_asso_info2_dist)
                dx = pt['x'] - tt[1]['x']
                dy = pt['y'] - tt[1]['y']
                dz = pt['z'] - tt[1]['z']
                dist_sq = dx*dx + dy*dy + dz*dz
                dist_maha = math.sqrt(dist_sq) / 20.0  # 除以INIT_SIGMA_R
                
                # 角度检查
                dx0 = tt[1]['x'] - tt[0]['x']
                dy0 = tt[1]['y'] - tt[0]['y']
                dz0 = tt[1]['z'] - tt[0]['z']
                n1 = math.sqrt(dist_sq)
                n0 = math.sqrt(dx0*dx0 + dy0*dy0 + dz0*dz0)
                angle = 180.0
                if n1 > 0.1 and n0 > 0.1:
                    ca = (dx*dx0 + dy*dy0 + dz*dz0) / (n1*n0)
                    ca = max(-1.0, min(1.0, ca))
                    angle = math.acos(ca) * 180.0 / PI
                
                if dist_maha < 15.0 and angle <= 160.0 and dist_maha < best_tt_dist:
                    best_tt_dist = dist_maha
                    best_tt_idx = ti
                    best_tt_count = 2
        
        if best_tt_idx >= 0:
            tt_used[best_tt_idx] = True
            tt = temp_tracks[best_tt_idx]
            count = sum(1 for slot in tt if slot.get('asso_flag') == 1)
            
            if count == 1:
                q = calculate_vr_quality(tt[0]['vr'], pt['v'], 0.0)
                if q >= 0.5:
                    dt = max(0.164, (frame_t - tt[0]['t']) / 1000.0)
                    vx_init = (pt['x'] - tt[0]['x']) / dt
                    vy_init = (pt['y'] - tt[0]['y']) / dt
                    vz_init = (pt['z'] - tt[0]['z']) / dt
                    tt[1] = {
                        'asso_flag': 1, 'vr': pt['v'],
                        'x': pt['x'], 'y': pt['y'], 'z': pt['z'],
                        't': frame_t, 'beam': pt['beam'],
                        'az_deg': pt['az_deg'], 'el_deg': pt['el_deg'],
                        'r': pt['r'], 'az': pt['az'], 'el': pt['el'],
                        'vx': vx_init, 'vy': vy_init, 'vz': vz_init
                    }
            elif count == 2:
                q = calculate_vr_quality(tt[0]['vr'], tt[1]['vr'], pt['v'])
                if q >= 0.8:
                    tt[2] = {
                        'asso_flag': 1, 'vr': pt['v'],
                        'x': pt['x'], 'y': pt['y'], 'z': pt['z'],
                        't': frame_t, 'beam': pt['beam'],
                        'az_deg': pt['az_deg'], 'el_deg': pt['el_deg'],
                        'r': pt['r'], 'az': pt['az'], 'el': pt['el']
                    }
                    init_log.append({
                        'frame': fi, 't': frame_t,
                        'vr0': tt[0]['vr'], 'vr1': tt[1]['vr'], 'vr2': pt['v'],
                        'az': pt['az_deg'], 'r': pt['r'],
                        'beams': [tt[0]['beam'], tt[1]['beam'], pt['beam']],
                        'q': q
                    })
        else:
            # 创建新temp_track (get_track_begin)
            temp_tracks.append([
                {
                    'asso_flag': 1, 'vr': pt['v'],
                    'x': pt['x'], 'y': pt['y'], 'z': pt['z'],
                    't': frame_t, 'beam': pt['beam'],
                    'az_deg': pt['az_deg'], 'el_deg': pt['el_deg'],
                    'r': pt['r'], 'az': pt['az'], 'el': pt['el'],
                    'vx': 0, 'vy': 0, 'vz': 0
                },
                {'asso_flag': 0},
                {'asso_flag': 0}
            ])

# 分析结果
print(f"=== 完整模拟结果 ===")
print(f"temp_tracks总数: {len(temp_tracks)}")
print(f"3点组合数: {len(init_log)}")
print()

# 检查3点组合详情
for log in init_log:
    vr_avg = (abs(log['vr0']) + abs(log['vr1']) + abs(log['vr2'])) / 3.0
    beam_span = max(log['beams']) - min(log['beams'])
    print(f"Frame {log['frame']}: Az={log['az']:.2f}°, R={log['r']:.1f}m, Vr_avg={vr_avg:.1f}m/s")
    print(f"  Vr=({log['vr0']:.2f}, {log['vr1']:.2f}, {log['vr2']:.2f}), beams={log['beams']}, q={log['q']:.3f}")
    
    # 判断是否是无人机
    is_drone = (log['vr0'] < -1 and log['vr1'] < -1 and log['vr2'] < -1 and 
                5 < abs(log['vr0']) < 30 and 5 < abs(log['vr1']) < 30 and 5 < abs(log['vr2']) < 30)
    print(f"  疑似无人机: {is_drone}")
    print()

# 特别检查Frame 4-5-21的无人机点
print("=== 无人机点追踪 ===")
drone_frames = [4, 5, 21, 22, 38, 39, 55, 56]
for df in drone_frames:
    if df < len(frames_data):
        fd = frames_data[df]
        drone_pts = [p for p in fd['points'] if p['beam'] in [2, 3] and p['v'] < -1 and 500 < p['r'] < 3000]
        if drone_pts:
            for dp in drone_pts:
                print(f"  Frame {df}: beam={dp['beam']}, Az={dp['az_deg']:.2f}°, R={dp['r']:.1f}m, Vr={dp['v']:.2f}m/s")

print()
print("=== 关键诊断 ===")
print("检查Frame 5的所有点与Frame 4的temp_track匹配情况：")
if len(frames_data) > 5:
    f4 = frames_data[4]
    f5 = frames_data[5]
    print(f"  Frame 4: {len(f4['points'])} 个点")
    for p in f4['points']:
        print(f"    beam={p['beam']}, Az={p['az_deg']:.2f}°, R={p['r']:.1f}m, Vr={p['v']:.2f}m/s, x={p['x']:.1f}, y={p['y']:.1f}, z={p['z']:.1f}")
    print(f"  Frame 5: {len(f5['points'])} 个点")
    for p in f5['points']:
        print(f"    beam={p['beam']}, Az={p['az_deg']:.2f}°, R={p['r']:.1f}m, Vr={p['v']:.2f}m/s, x={p['x']:.1f}, y={p['y']:.1f}, z={p['z']:.1f}")

# 检查Frame 5时存在的temp_tracks
print()
print("=== Frame 5时的temp_tracks ===")
# 重新模拟到Frame 5
temp_tracks2 = []
for fi in range(min(6, len(frames_data))):
    frame = frames_data[fi]
    points = frame['points']
    frame_t = frame['t']
    
    valid_tracks = []
    for tt in temp_tracks2:
        count = sum(1 for slot in tt if slot.get('asso_flag') == 1)
        if count == 0: continue
        last_t = tt[count-1]['t']
        if 0 <= frame_t - last_t <= time_up_ms:
            valid_tracks.append(tt)
    temp_tracks2 = valid_tracks
    
    tt_used = [False] * len(temp_tracks2)
    
    for pi, pt in enumerate(points):
        best_tt_idx = -1
        best_tt_dist = float('inf')
        
        for ti, tt in enumerate(temp_tracks2):
            if tt_used[ti]: continue
            count = sum(1 for slot in tt if slot.get('asso_flag') == 1)
            if count == 0: continue
            last_t = tt[count-1]['t']
            lag_time = frame_t - last_t
            if lag_time < 0 or lag_time > time_up_ms: continue
            T = lag_time / 1000.0
            
            if count == 1:
                vr_exist = tt[0]['vr']
                vr_new = pt['v']
                if (vr_exist * vr_new < 0.0) and abs(vr_exist) > 3.0 and abs(vr_new) > 3.0:
                    continue
                dx = pt['x'] - tt[0]['x']
                dy = pt['y'] - tt[0]['y']
                dz = pt['z'] - tt[0]['z']
                dist_sq = dx*dx + dy*dy + dz*dz
                gate = 1500.0 * T + 150.0
                if dist_sq < gate * gate and dist_sq < best_tt_dist:
                    best_tt_dist = dist_sq
                    best_tt_idx = ti
            
            elif count == 2:
                vr_a = tt[0]['vr']
                vr_b = tt[1]['vr']
                vr_c = pt['v']
                sa = 1 if vr_a > 0 else (-1 if vr_a < 0 else 0)
                sb = 1 if vr_b > 0 else (-1 if vr_b < 0 else 0)
                sc = 1 if vr_c > 0 else (-1 if vr_c < 0 else 0)
                if sa != 0 and sb != 0 and sc != 0:
                    if (sa != sb) or (sb != sc): continue
        
        if best_tt_idx >= 0:
            tt_used[best_tt_idx] = True
            tt = temp_tracks2[best_tt_idx]
            count = sum(1 for slot in tt if slot.get('asso_flag') == 1)
            if count == 1:
                q = calculate_vr_quality(tt[0]['vr'], pt['v'], 0.0)
                if q >= 0.5:
                    dt = max(0.164, (frame_t - tt[0]['t']) / 1000.0)
                    tt[1] = {
                        'asso_flag': 1, 'vr': pt['v'],
                        'x': pt['x'], 'y': pt['y'], 'z': pt['z'],
                        't': frame_t, 'beam': pt['beam'],
                        'az_deg': pt['az_deg'], 'el_deg': pt['el_deg'],
                        'r': pt['r'], 'az': pt['az'], 'el': pt['el'],
                        'vx': (pt['x'] - tt[0]['x'])/dt,
                        'vy': (pt['y'] - tt[0]['y'])/dt,
                        'vz': (pt['z'] - tt[0]['z'])/dt
                    }
        else:
            temp_tracks2.append([
                {
                    'asso_flag': 1, 'vr': pt['v'],
                    'x': pt['x'], 'y': pt['y'], 'z': pt['z'],
                    't': frame_t, 'beam': pt['beam'],
                    'az_deg': pt['az_deg'], 'el_deg': pt['el_deg'],
                    'r': pt['r'], 'az': pt['az'], 'el': pt['el'],
                },
                {'asso_flag': 0},
                {'asso_flag': 0}
            ])
    
    print(f"\nFrame {fi} (t={frame_t}ms)后: {len(temp_tracks2)} 个temp_tracks")
    for ti, tt in enumerate(temp_tracks2):
        count = sum(1 for slot in tt if slot.get('asso_flag') == 1)
        if count >= 1:
            print(f"  tt[{ti}]: count={count}, Vr={tt[0]['vr']:.2f}, beam={tt[0]['beam']}, Az={tt[0]['az_deg']:.2f}°, R={tt[0]['r']:.1f}m")
