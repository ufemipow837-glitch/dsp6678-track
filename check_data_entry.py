#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re

data_entry_file = r'd:\DSP\6678\track\track_1\data_entry.c'

with open(data_entry_file, 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

# Find all frame data
frames = []
# Pattern: target_data[k].mSecond = XXX;
# target_data[k].targetNum = N;
# and the range/azi/velocity values

# Let's parse frame by frame, looking for mSecond assignments and targetNum
pattern = r'target_data\[(\d+)\]\.mSecond\s*=\s*([\d.]+)f?;'
matches = list(re.finditer(pattern, content))

print(f"Found {len(matches)} frame timestamps")

# For each frame, look at nearby code for targetNum and the point data
frame_data = []
for i, m in enumerate(matches):
    idx = int(m.group(1))
    msec = float(m.group(2))
    
    # Get the text between this timestamp and the next (or end)
    start = m.end()
    if i + 1 < len(matches):
        end = matches[i+1].start()
    else:
        end = len(content)
    
    block = content[start:end]
    
    # Find targetNum
    tnum_match = re.search(r'targetNum\s*=\s*(\d+)', block)
    tnum = int(tnum_match.group(1)) if tnum_match else 0
    
    # Find azimuths, ranges, velocities
    azis = [float(x) for x in re.findall(r'azi\[\d+\]\s*=\s*([-\d.e+]+)f?', block)]
    rngs = [float(x) for x in re.findall(r'range\[\d+\]\s*=\s*([-\d.e+]+)f?', block)]
    vels = [float(x) for x in re.findall(r'velocity\[\d+\]\s*=\s*([-\d.e+]+)f?', block)]
    
    frame_data.append({
        'idx': idx,
        'msec': msec,
        'tnum': tnum,
        'azis': azis,
        'ranges': rngs,
        'velocities': vels
    })

# Sort by msec
frame_data.sort(key=lambda x: x['msec'])

print(f"Frame range: idx={frame_data[0]['idx']}({frame_data[0]['msec']:.0f}ms) - idx={frame_data[-1]['idx']}({frame_data[-1]['msec']:.0f}ms)")
print()

# Look around 946192ms
target_t = 946192
window = 5000
print(f"=== Looking at data_entry frames around {target_t}ms (±{window}ms) ===")
print()

for f in frame_data:
    if abs(f['msec'] - target_t) <= window:
        points_str = ""
        if f['tnum'] > 0:
            pts = []
            for j in range(min(f['tnum'], len(f['azis']), len(f['ranges']), len(f['velocities']))):
                r = f['ranges'][j]
                a = f['azis'][j] * 180.0 / 3.14159265  # rad to deg
                v = f['velocities'][j]
                pts.append(f"[r={r:.0f}m, az={a:.1f}°, v={v:.1f}m/s]")
            points_str = ", ".join(pts)
        else:
            points_str = "(no points)"
        
        print(f"idx={f['idx']:3d}, t={f['msec']:.0f}ms, targetNum={f['tnum']:2d}  {points_str}")

print()

# Check the pattern: find consecutive frames with points and without
print("=== Pattern of targetNum over time (every 5th frame) ===")
for i in range(0, len(frame_data), 5):
    f = frame_data[i]
    has_drone = False
    for r in f['ranges']:
        for a in f['azis']:
            a_deg = a * 180.0 / 3.14159265
            if 1000 < r < 2500 and 10 < a_deg < 25:
                has_drone = True
                break
    drone_mark = " ***DRONE***" if has_drone else ""
    print(f"  idx={f['idx']:3d}, t={f['msec']:.0f}ms, N={f['tnum']:2d}{drone_mark}")
