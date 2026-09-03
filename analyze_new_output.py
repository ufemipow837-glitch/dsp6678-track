#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re
from collections import defaultdict

output_file = r'd:\DSP\6678\track\track_1\输出数据.txt'

with open(output_file, 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

frames = []
current_frame = None

for line in lines:
    line = line.strip()
    
    frame_match = re.search(r'Frame\s+(\d+)\s+\|\s+t=(\d+)ms', line)
    if frame_match:
        if current_frame:
            frames.append(current_frame)
        current_frame = {
            'frame': int(frame_match.group(1)),
            't': int(frame_match.group(2)),
            'tracks': [],
            'end_tracks': []
        }
        continue
    
    if current_frame is None:
        continue
    
    track_match = re.search(r'Track\[(\d+)\]:\s*Az=([\d.]+)deg\s+El=([-\d.]+)deg\s+R=([\d.]+)m\s+V=([-\d.]+)m/s', line)
    if track_match:
        current_frame['tracks'].append({
            'idx': int(track_match.group(1)),
            'az': float(track_match.group(2)),
            'el': float(track_match.group(3)),
            'r': float(track_match.group(4)),
            'v': float(track_match.group(5))
        })
    
    end_match = re.search(r'TrackEnd\[(\d+)\]:.*Vel=([\d.]+)m/s', line)
    if end_match:
        current_frame['end_tracks'].append({
            'idx': int(end_match.group(1)),
            'v': float(end_match.group(2))
        })

if current_frame:
    frames.append(current_frame)

print(f"Total frames parsed: {len(frames)}")
print(f"Frame range: {frames[0]['frame']} - {frames[-1]['frame']}")
print(f"Time range: {frames[0]['t']}ms - {frames[-1]['t']}ms")
print()

# Look for frames around 946192ms
target_t = 946192
window = 5000  # ±5 seconds
print(f"=== Looking at frames around {target_t}ms (±{window}ms) ===")
print()

relevant_frames = []
for f in frames:
    if abs(f['t'] - target_t) <= window:
        relevant_frames.append(f)

for f in relevant_frames:
    n_tracks = len(f['tracks'])
    n_ends = len(f['end_tracks'])
    track_str = ""
    if n_tracks > 0:
        v_list = [f"{t['v']:.1f}m/s" for t in f['tracks']]
        track_str = f"Tracks: {', '.join(v_list)}"
    else:
        track_str = "NO TRACKS"
    
    end_str = ""
    if n_ends > 0:
        end_str = f", {n_ends} track(s) ended"
    
    print(f"Frame {f['frame']:4d} | t={f['t']:8d}ms | {track_str}{end_str}")

print()
print("=== Checking when track is lost and re-established ===")
# Find frames without tracks
no_track_frames = []
has_track_frames = []
for f in frames:
    if len(f['tracks']) == 0:
        no_track_frames.append(f)
    else:
        has_track_frames.append(f)

if no_track_frames:
    print(f"Frames without any track: {len(no_track_frames)}")
    # Find gaps
    prev_t = None
    gaps = []
    gap_start = None
    for i, f in enumerate(frames):
        has_t = len(f['tracks']) > 0
        if not has_t and gap_start is None:
            gap_start = f
        if has_t and gap_start is not None:
            gaps.append((gap_start, f))
            gap_start = None
    
    print(f"Track gaps (lost→found):")
    for gs, ge in gaps:
        gap_duration = ge['t'] - gs['t']
        print(f"  Lost at Frame {gs['frame']} (t={gs['t']}ms) → Found at Frame {ge['frame']} (t={ge['t']}ms), duration={gap_duration}ms ({gap_duration/1000:.2f}s)")

print()
print("=== Checking velocity over time for anomalies ===")
velocities = []
for f in frames:
    for t in f['tracks']:
        velocities.append((f['frame'], f['t'], t['v']))

if velocities:
    v_vals = [v[2] for v in velocities]
    print(f"Velocity stats: min={min(v_vals):.1f}m/s, max={max(v_vals):.1f}m/s, mean={sum(v_vals)/len(v_vals):.1f}m/s")
    
    # Find velocities > 30m/s
    high_v = [(fr, t, v) for fr, t, v in velocities if v > 30]
    if high_v:
        print(f"\nHigh velocity (>30m/s) events:")
        for fr, t, v in high_v[:20]:
            print(f"  Frame {fr}, t={t}ms, V={v:.1f}m/s")
