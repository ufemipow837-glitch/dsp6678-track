import re, math

with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

# Parse all frames
all_ts = []
beam_per_ts = {}  # ts -> beam
all_points = []   # (ts, beam, azi, rng, ele, vel, h)

for line in content.split('\n'):
    ts_m = re.search(r'时间戳:(\d+)', line)
    bm = re.search(r'波位号:(\d+)', line)
    if not ts_m or not bm:
        continue
    ts = int(ts_m.group(1))
    b = int(bm.group(1))
    
    # Check if this line has target data (has 距离:)
    rng_m = re.search(r'距离:([\d.\-]+)', line)
    azi_m = re.search(r'方位:([\d.\-]+)', line)
    ele_m = re.search(r'俯仰:([\d.\-]+)', line)
    vel_m = re.search(r'速度:([\d.\-]+)', line)
    h_m = re.search(r'高度:([\d.\-]+)', line)
    
    if rng_m and azi_m and vel_m:
        rng = float(rng_m.group(1))
        azi = float(azi_m.group(1))
        ele = float(ele_m.group(1)) if ele_m else 0
        vel = float(vel_m.group(1))
        h = float(h_m.group(1)) if h_m else 0
        all_points.append((ts, b, azi, rng, ele, vel, h))
    
    if ts not in beam_per_ts:
        beam_per_ts[ts] = b

sorted_ts = sorted(beam_per_ts.keys())
print(f"Total unique timestamps: {len(sorted_ts)}")

# Find drone points: beam=2, height ~ 100m (80~120m), and consistent Vr
drone_candidates = []
for ts, b, azi, rng, ele, vel, h in all_points:
    if b == 2 and 70 < h < 130 and rng > 300:
        drone_candidates.append((ts, azi, rng, ele, vel, h))

print(f"\nBeam 2 points with H~100m: {len(drone_candidates)}")
print()

# Now also check: which of these have the same timestamp but different beams (leakage)
# The drone should ONLY appear in beam 2, not beam 1 or 3
print("Drone-like points (H~100m) - checking beam leakage:")
leakage = []
for ts, b, azi, rng, ele, vel, h in all_points:
    if 70 < h < 130 and 300 < rng < 5000 and abs(azi - 17) < 20:
        leakage.append((ts, b, azi, rng, vel, h))

leakage.sort(key=lambda x: (x[0], x[1]))

# Group by timestamp
from collections import defaultdict
by_ts = defaultdict(list)
for ts, b, azi, rng, vel, h in leakage:
    by_ts[ts].append((b, azi, rng, vel, h))

print("Beam leakage (H~100m points in wrong beams):")
for ts in sorted(by_ts.keys())[:30]:
    pts = by_ts[ts]
    beams = [p[0] for p in pts]
    if len(beams) > 1 or beams[0] != 2:
        print(f"  t={ts}: {len(pts)} points, beams={beams}")
        for p in pts:
            print(f"    beam={p[0]}, azi={p[1]:.2f}, R={p[2]:.1f}, Vr={p[3]:.2f}, H={p[4]:.1f}")

print("\n" + "="*80)
print("=== CLEAN DRONE TRAJECTORY (beam 2, H~100m only) ===")
print("="*80)

# Filter out leakage - only keep beam=2, H~80-120, and exclude duplicates per ts
clean_drone = {}
for ts, b, azi, rng, ele, vel, h in all_points:
    if b == 2 and 80 < h < 120:
        if ts not in clean_drone:
            clean_drone[ts] = (azi, rng, ele, vel, h)
        else:
            # Keep the one closer to 100m
            if abs(h - 100) < abs(clean_drone[ts][4] - 100):
                clean_drone[ts] = (azi, rng, ele, vel, h)

print(f"Clean drone frames: {len(clean_drone)}")
print("{:>8} {:>10} {:>10} {:>10} {:>10} {:>10}".format("ts(ms)", "azi(deg)", "R(m)", "ele(deg)", "Vr(m/s)", "H(m)"))
print("-"*65)
for ts in sorted(clean_drone.keys()):
    azi, rng, ele, vel, h = clean_drone[ts]
    print("{:>8} {:>10.2f} {:>10.2f} {:>10.4f} {:>10.4f} {:>10.2f}".format(ts, azi, rng, ele, vel, h))

# Now map to frame indices for first 500 frames
print("\n" + "="*80)
print("=== DRONE TRAJECTORY IN FIRST 500 FRAMES ===")
print("="*80)
frame_idx = {}
for i, ts in enumerate(sorted_ts[:500]):
    frame_idx[ts] = i

print(f"Frame  range: 0~499, t=[{sorted_ts[0]}..{sorted_ts[499]}]ms")
print()
print("{:>6} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}".format("Frame", "t(ms)", "azi(deg)", "R(m)", "ele(deg)", "Vr(m/s)", "H(m)"))
print("-"*75)
count = 0
for i, ts in enumerate(sorted_ts[:500]):
    b = beam_per_ts[ts]
    if b == 2:
        if ts in clean_drone:
            azi, rng, ele, vel, h = clean_drone[ts]
            print("{:>6} {:>10} {:>10.2f} {:>10.2f} {:>10.4f} {:>10.4f} {:>10.2f}".format(i, ts, azi, rng, ele, vel, h))
            count += 1
        else:
            print("{:>6} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}".format(i, ts, "--NO DRONE--", "--", "--", "--", "--"))

print(f"\nDrone visible frames in first 500: {count}")
print(f"Beam 2 frames in first 500: {sum(1 for t in sorted_ts[:500] if beam_per_ts[t]==2)}")
