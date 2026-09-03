#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re

data_entry_file = r'd:\DSP\6678\track\track_1\data_entry.c'

with open(data_entry_file, 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

pi = 3.141592653589793

# Parse by finding data[N].mSecond = XXX patterns
# Pattern: data[(\d+)\]\.(\w+)\s*=\s*([^;]+);
# We'll do a simpler approach: split by "data[" assignments

frames = {}

# Find all data[k] field assignments
pattern = r'data\[(\d+)\]\.(\w+)\s*=\s*([^;]+);'
for m in re.finditer(pattern, content):
    idx = int(m.group(1))
    field = m.group(2)
    val_str = m.group(3).strip()
    
    if idx not in frames:
        frames[idx] = {'points': []}
    
    if field == 'mSecond':
        frames[idx]['msec'] = float(val_str.replace('f', ''))
    elif field == 'targetNum':
        frames[idx]['tnum'] = int(val_str)
    elif field == 'beamNo':
        frames[idx]['beam'] = int(val_str)
    elif field == 'workMode':
        frames[idx]['wmode'] = int(val_str)
    elif field == 'Month':
        frames[idx]['month'] = int(val_str)
    elif field.startswith('azi['):
        pidx = int(re.search(r'azi\[(\d+)\]', field).group(1))
        val = eval(val_str.replace('f', ''), {'pi': pi})
        while len(frames[idx]['points']) <= pidx:
            frames[idx]['points'].append({})
        frames[idx]['points'][pidx]['azi_deg'] = val * 180.0 / pi
    elif field.startswith('ele['):
        pidx = int(re.search(r'ele\[(\d+)\]', field).group(1))
        val = eval(val_str.replace('f', ''), {'pi': pi})
        while len(frames[idx]['points']) <= pidx:
            frames[idx]['points'].append({})
        frames[idx]['points'][pidx]['ele_deg'] = val * 180.0 / pi
    elif field.startswith('range['):
        pidx = int(re.search(r'range\[(\d+)\]', field).group(1))
        val = float(val_str.replace('f', ''))
        while len(frames[idx]['points']) <= pidx:
            frames[idx]['points'].append({})
        frames[idx]['points'][pidx]['range'] = val
    elif field.startswith('velocity['):
        pidx = int(re.search(r'velocity\[(\d+)\]', field).group(1))
        val = float(val_str.replace('f', ''))
        while len(frames[idx]['points']) <= pidx:
            frames[idx]['points'].append({})
        frames[idx]['points'][pidx]['vel'] = val
    elif field.startswith('Use_Flag_1['):
        pidx = int(re.search(r'Use_Flag_1\[(\d+)\]', field).group(1))
        val = int(val_str)
        while len(frames[idx]['points']) <= pidx:
            frames[idx]['points'].append({})
        frames[idx]['points'][pidx]['valid'] = val

# Build sorted list
frame_list = []
for idx in sorted(frames.keys()):
    f = frames[idx]
    if 'msec' in f and 'tnum' in f:
        frame_list.append(f)

print(f"Parsed {len(frame_list)} frames")
print()

# Look around 946192ms
target_t = 946192
window = 6000  # ±6 seconds

print(f"=== Frames around {target_t}ms (±{window}ms) ===")
print()

drone_count = 0
for f in frame_list:
    t = f.get('msec', 0)
    if abs(t - target_t) <= window:
        beam = f.get('beam', -1)
        tnum = f.get('tnum', 0)
        wmode = f.get('wmode', -1)
        
        # Check for drone points (az 10-25 deg, range 1000-2500m)
        drone_pts = []
        other_pts = []
        for p in f.get('points', []):
            if not p.get('valid', 0):
                continue
            az = p.get('azi_deg', 0)
            r = p.get('range', 0)
            v = abs(p.get('vel', 0))
            if 1000 < r < 2500 and 5 < az < 30 and v < 30:
                drone_pts.append(p)
            else:
                other_pts.append(p)
        
        drone_mark = ""
        if drone_pts:
            drone_mark = f"  <<< DRONE: az={drone_pts[0]['azi_deg']:.1f}°, r={drone_pts[0]['range']:.0f}m, v={drone_pts[0]['vel']:.1f}m/s"
            drone_count += 1
        
        print(f"t={t:.0f}ms  beam={beam:2d}  N={tnum:2d}  wmode={wmode}  other_pts={len(other_pts)}{drone_mark}")

print()
print(f"=== Checking drone detection pattern across all frames ===")
# Find all drone detections
drone_frames = []
for f in frame_list:
    t = f.get('msec', 0)
    beam = f.get('beam', -1)
    for p in f.get('points', []):
        if not p.get('valid', 0):
            continue
        az = p.get('azi_deg', 0)
        r = p.get('range', 0)
        v = abs(p.get('vel', 0))
        if 1000 < r < 2500 and 5 < az < 30 and v < 30:
            drone_frames.append((t, beam, az, r, v))
            break

print(f"Total frames with drone detection: {len(drone_frames)}")
print(f"Drone beam positions: {sorted(set(b for _,b,_,_,_ in drone_frames))}")

# Check gaps between drone detections
print()
print("=== Gaps between drone detections ===")
for i in range(1, len(drone_frames)):
    t_prev = drone_frames[i-1][0]
    t_curr = drone_frames[i][0]
    gap = t_curr - t_prev
    if gap > 3000:  # More than 3 seconds
        print(f"  GAP: {t_prev:.0f}ms(beam{drone_frames[i-1][1]}) -> {t_curr:.0f}ms(beam{drone_frames[i][1]}), gap={gap:.0f}ms ({gap/1000:.2f}s)")
