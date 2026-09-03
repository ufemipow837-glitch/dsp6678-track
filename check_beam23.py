#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re

data_entry_file = r'd:\DSP\6678\track\track_1\data_entry.c'

with open(data_entry_file, 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

pi = 3.141592653589793

frames = {}
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
    elif field.startswith('azi['):
        pidx = int(re.search(r'azi\[(\d+)\]', field).group(1))
        val = eval(val_str.replace('f', ''), {'pi': pi})
        while len(frames[idx]['points']) <= pidx:
            frames[idx]['points'].append({})
        frames[idx]['points'][pidx]['azi_deg'] = val * 180.0 / pi
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

frame_list = []
for idx in sorted(frames.keys()):
    f = frames[idx]
    if 'msec' in f and 'tnum' in f:
        frame_list.append(f)

# Look at beam 2 and beam 3 frames
print("=== Beam 2 and 3 frames around t=940000-950000ms ===")
print()
for f in frame_list:
    t = f.get('msec', 0)
    beam = f.get('beam', -1)
    tnum = f.get('tnum', 0)
    if 938000 < t < 952000 and beam in [2, 3]:
        print(f"t={t:.0f}ms  beam={beam}  N={tnum}")
        for j, p in enumerate(f.get('points', [])):
            if p.get('valid', 0):
                az = p.get('azi_deg', 0)
                r = p.get('range', 0)
                v = abs(p.get('vel', 0))
                # Convert to X,Y to see where this point is
                # Az is measured from what? Let's check...
                # Looking at earlier data, drone should be at ~18° az
                print(f"  [{j}] az={az:.2f}°, r={r:.1f}m, v={v:.2f}m/s")
        print()

# Now let's see: what azimuth range is the drone at? Let's look at first successful track establishment
# According to previous analysis, track was established around Frame 38 (t=904372ms)
print("=== Looking at frames where track was established (around t=904000ms) ===")
for f in frame_list:
    t = f.get('msec', 0)
    beam = f.get('beam', -1)
    tnum = f.get('tnum', 0)
    if 903000 < t < 907000:
        print(f"t={t:.0f}ms  beam={beam}  N={tnum}")
        for j, p in enumerate(f.get('points', [])):
            if p.get('valid', 0):
                az = p.get('azi_deg', 0)
                r = p.get('range', 0)
                v = abs(p.get('vel', 0))
                print(f"  [{j}] az={az:.2f}°, r={r:.1f}m, v={v:.2f}m/s")
