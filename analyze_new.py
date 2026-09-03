import re
import math

# Parse output data.txt
tracks = []
with open(r'd:\DSP\6678\track\track_1\输出数据.txt', 'r', encoding='utf-8', errors='replace') as f:
    lines = f.readlines()

current_frame = -1
current_t = -1
for line in lines:
    fm = re.search(r'Frame\s+(\d+)\s+\|\s+t=(\d+)ms', line)
    if fm:
        current_frame = int(fm.group(1))
        current_t = int(fm.group(2))
    
    tm = re.search(r'Track\[0\]:\s*Az=([-\d.]+)deg\s+El=([-\d.]+)deg\s+R=([\d.]+)m\s+V=([-\d.]+)m/s', line)
    if tm and current_frame >= 0:
        az = float(tm.group(1))
        el = float(tm.group(2))
        r = float(tm.group(3))
        v = float(tm.group(4))
        tracks.append({'frame': current_frame, 't': current_t, 'az': az, 'el': el, 'r': r, 'v': v})

print(f"Total track outputs: {len(tracks)} frames")
if tracks:
    print(f"Frame range: {tracks[0]['frame']} - {tracks[-1]['frame']}")
    print(f"Time range: {tracks[0]['t']}ms - {tracks[-1]['t']}ms")
    
    # Print all track data
    print("\n=== Track output (every 5th frame) ===")
    for i, t in enumerate(tracks):
        if i % 5 == 0 or t['v'] > 30:
            print(f"  Frame {t['frame']:3d} (t={t['t']}ms): Az={t['az']:.2f}°, El={t['el']:.2f}°, R={t['r']:.0f}m, V={t['v']:.1f}m/s")
    
    # Find speed anomalies
    print("\n=== Speed analysis ===")
    high_v = [t for t in tracks if t['v'] > 30]
    if high_v:
        print(f"High speed frames (>30m/s): {len(high_v)}")
        for t in high_v[:10]:
            print(f"  Frame {t['frame']} (t={t['t']}ms): V={t['v']:.1f}m/s, Az={t['az']:.2f}°, R={t['r']:.0f}m")
    else:
        print("No speed anomalies (>30m/s) - speed looks good!")
    
    # Check frame continuity
    print("\n=== Frame continuity gaps ===")
    gaps = []
    for i in range(1, len(tracks)):
        gap = tracks[i]['frame'] - tracks[i-1]['frame']
        if gap > 1:
            gaps.append((tracks[i-1]['frame'], tracks[i]['frame'], gap))
    if gaps:
        for prev, curr, g in gaps:
            print(f"  Gap: Frame {prev} -> {curr} (gap={g} frames)")
    else:
        print("No gaps - continuous tracking!")
    
    # Print first 10 and last 10
    print("\n=== First 10 frames ===")
    for t in tracks[:10]:
        print(f"  Frame {t['frame']:3d} (t={t['t']}ms): Az={t['az']:.2f}°, El={t['el']:.2f}°, R={t['r']:.0f}m, V={t['v']:.1f}m/s")
    print("\n=== Last 10 frames ===")
    for t in tracks[-10:]:
        print(f"  Frame {t['frame']:3d} (t={t['t']}ms): Az={t['az']:.2f}°, El={t['el']:.2f}°, R={t['r']:.0f}m, V={t['v']:.1f}m/s")
