"""
Generate data_entry.c from REAL measured data (first 500 frames).
This ensures perfect correspondence between simulation and real data.
Split into 10 sub-functions of 50 frames each to avoid C6000 virtual register limit.
"""
import re, math

PI = 3.141592653589793
NUM_FRAMES = 500

# Parse measured data
with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

# Step 1: Collect all unique timestamps and beam mapping
all_ts_set = set()
beam_per_ts = {}
ts_to_frame_idx = {}

for line in content.split('\n'):
    ts_m = re.search(r'时间戳:(\d+)', line)
    bm = re.search(r'波位号:(\d+)', line)
    if ts_m and bm:
        ts = int(ts_m.group(1))
        b = int(bm.group(1))
        all_ts_set.add(ts)
        if ts not in beam_per_ts:
            beam_per_ts[ts] = b

sorted_ts = sorted(all_ts_set)
print(f"Total unique timestamps in measured data: {len(sorted_ts)}")

# Map first 500 timestamps to frame indices
for i, ts in enumerate(sorted_ts[:NUM_FRAMES]):
    ts_to_frame_idx[ts] = i

print(f"Using first {NUM_FRAMES} frames: t=[{sorted_ts[0]}..{sorted_ts[NUM_FRAMES-1]}]ms")

# Step 2: Collect ALL target points per frame
# frame_targets[i] = list of (azi_deg, ele_deg, rng, vel, h) for frame i
frame_targets = [[] for _ in range(NUM_FRAMES)]

for line in content.split('\n'):
    ts_m = re.search(r'时间戳:(\d+)', line)
    if not ts_m:
        continue
    ts = int(ts_m.group(1))
    if ts not in ts_to_frame_idx:
        continue
    
    # Skip empty frame marker lines
    if '目标数:0' in line or '目标数: 0' in line:
        continue
    
    azi_m = re.search(r'方位:([\d.\-]+)', line)
    rng_m = re.search(r'距离:([\d.\-]+)', line)
    ele_m = re.search(r'俯仰:([\d.\-]+)', line)
    vel_m = re.search(r'速度:([\d.\-]+)', line)
    h_m = re.search(r'高度:([\d.\-]+)', line)
    
    if not (azi_m and rng_m and vel_m):
        continue
    
    azi = float(azi_m.group(1))
    rng = float(rng_m.group(1))
    ele = float(ele_m.group(1)) if ele_m else 0.0
    vel = float(vel_m.group(1))
    h = float(h_m.group(1)) if h_m else 0.0
    
    fi = ts_to_frame_idx[ts]
    frame_targets[fi].append((azi, ele, rng, vel, h))

# Step 3: Summary stats
total_targets = sum(len(t) for t in frame_targets)
print(f"Total target points in first {NUM_FRAMES} frames: {total_targets}")

beam2_frames = 0
beam2_with_drone = 0
for fi in range(NUM_FRAMES):
    ts = sorted_ts[fi]
    if beam_per_ts.get(ts) == 2:
        beam2_frames += 1
        # Check if any target looks like drone (H~80-120m, beam 2)
        for azi, ele, rng, vel, h in frame_targets[fi]:
            if 80 < h < 120 and rng > 300:
                beam2_with_drone += 1
                break

print(f"Beam 2 frames: {beam2_frames}, with drone detected: {beam2_with_drone}")

# Step 4: Generate C code
lines = []
lines.append('#include <stdio.h>')
lines.append('#include <c6x.h>')
lines.append('#include <math.h>')
lines.append('#include "struct.h"')
lines.append('#include <stdint.h>')
lines.append('#include <float.h>')
lines.append('/*')
lines.append(' * data_entry.c - 500 frames of SIMULATED radar input')
lines.append(' * Generated from REAL measured data (first 500 frames of 无人机实测数据.txt)')
lines.append(' * Split into 10 sub-functions (50 frames each) for C6000 compiler.')
lines.append(' */')

chunk_size = 50
num_chunks = NUM_FRAMES // chunk_size

for ci in range(num_chunks):
    start_frame = ci * chunk_size
    end_frame = start_frame + chunk_size
    func_name = f'data_entry_fill_{ci:02d}'
    
    lines.append('')
    lines.append(f'static void {func_name}(struct TARGETPIONT_1 (*data)){{')
    
    # Zero-out block (only in first chunk)
    if ci == 0:
        lines.append('    int i, j;')
        lines.append('    /* Zero all 500 frames first */')
        lines.append('    for(i = 0; i < 500; i++){')
        lines.append('        data[i].targetNum = 0;')
        lines.append('        data[i].frameSn = 0;')
        lines.append('        data[i].mSecond = 0.0f;')
        lines.append('        data[i].workMode = 0;')
        lines.append('        data[i].beamNo = 0;')
        lines.append('        data[i].tgtnum = 1;')
        lines.append('        data[i].Year = 0;')
        lines.append('        data[i].Month = 0;')
        lines.append('        data[i].Day = 0;')
        lines.append('        data[i].Hour = 0;')
        lines.append('        data[i].Minute = 0;')
        lines.append('        data[i].Second = 0;')
        lines.append('        for(j = 0; j < 60; j++){')
        lines.append('            data[i].azi[j] = 0.0f;')
        lines.append('            data[i].ele[j] = 0.0f;')
        lines.append('            data[i].range[j] = 0.0f;')
        lines.append('            data[i].velocity[j] = 0.0f;')
        lines.append('            data[i].Use_Flag_1[j] = 0;')
        lines.append('        }')
        lines.append('    }')
        lines.append('')
    
    # Generate each frame
    for fi in range(start_frame, end_frame):
        ts = sorted_ts[fi]
        beam = beam_per_ts.get(ts, 0)
        targets = frame_targets[fi]
        
        lines.append(f'    /* Frame {fi}: t={ts}ms, beam={beam}, n={len(targets)} */')
        lines.append(f'    data[{fi}].targetNum = {len(targets)};')
        lines.append(f'    data[{fi}].frameSn = {fi};')
        lines.append(f'    data[{fi}].mSecond = {float(ts):.1f}f;')
        lines.append(f'    data[{fi}].workMode = 0;')
        lines.append(f'    data[{fi}].beamNo = {beam};')
        lines.append(f'    data[{fi}].Month = 0;')
        
        for ti, (azi_d, ele_d, rng, vel, h) in enumerate(targets):
            azi_rad = azi_d / 180.0 * PI
            ele_rad = ele_d / 180.0 * PI
            lines.append(f'    data[{fi}].azi[{ti}] = {azi_rad:.6f}f;')
            lines.append(f'    data[{fi}].ele[{ti}] = {ele_rad:.6f}f;')
            lines.append(f'    data[{fi}].range[{ti}] = {rng:.2f}f;')
            lines.append(f'    data[{fi}].velocity[{ti}] = {vel:.4f}f;')
            lines.append(f'    data[{fi}].Use_Flag_1[{ti}] = 1;')
        
        lines.append('')
    
    lines.append('}')

# Main entry function
lines.append('')
lines.append('void data_entry(struct TARGETPIONT_1 (*data)){')
for ci in range(num_chunks):
    lines.append(f'    data_entry_fill_{ci:02d}(data);')
lines.append('}')

# Write output
outpath = r'd:\DSP\6678\track\track_1\data_entry.c'
with open(outpath, 'w', encoding='gbk', errors='ignore') as f:
    f.write('\n'.join(lines))
    f.write('\n')

print(f"\nGenerated data_entry.c: {len(lines)} lines")
print(f"File: {outpath}")

# Quick validation
content = open(outpath, 'rb').read().decode('gbk', errors='ignore')
frame_count = len(re.findall(r'data\[(\d+)\]\.targetNum', content))
print(f"Frame entries found: {frame_count}")
assert frame_count == NUM_FRAMES, f"Expected {NUM_FRAMES}, got {frame_count}"

# Verify first/last frame content
print("\nFirst frame check:")
fi = 0
ts = sorted_ts[fi]
beam = beam_per_ts.get(ts, 0)
tgt_n = len(frame_targets[fi])
print(f"  Frame 0: t={ts}ms, beam={beam}, targets={tgt_n}")

print("\nLast frame check:")
fi = NUM_FRAMES - 1
ts = sorted_ts[fi]
beam = beam_per_ts.get(ts, 0)
tgt_n = len(frame_targets[fi])
print(f"  Frame {fi}: t={ts}ms, beam={beam}, targets={tgt_n}")
