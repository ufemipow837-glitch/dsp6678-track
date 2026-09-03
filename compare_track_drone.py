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
        tracks.append({
            'frame': current_frame, 't': current_t,
            'az': float(tm.group(1)), 'el': float(tm.group(2)),
            'r': float(tm.group(3)), 'v': float(tm.group(4))
        })

# Parse drone measured data
points = []
with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', 'r', encoding='utf-8', errors='replace') as f:
    for line in f:
        bm = re.search(r'波位号:(\d+)', line)
        ts = re.search(r'时间戳:(\d+)', line)
        az = re.search(r'方位:([-\d.e+]+)', line)
        rng = re.search(r'距离:([-\d.e+]+)', line)
        el = re.search(r'俯仰:([-\d.e+]+)', line)
        vel = re.search(r'速度:([-\d.e+]+)', line)
        snr_m = re.search(r'信噪比:(\d+)', line)
        if ts and az and rng:
            try:
                p = {
                    'beam': int(bm.group(1)) if bm else -1,
                    't': int(ts.group(1)),
                    'az': float(az.group(1)),
                    'r': float(rng.group(1)),
                    'el': float(el.group(1)) if el else 0,
                    'v': abs(float(vel.group(1))) if vel else 0,
                    'snr': int(snr_m.group(1)) if snr_m else 0
                }
                points.append(p)
            except:
                pass

# Drone ground truth: beam 2/3, az=12-20°, r=800-2200m, snr>=10
drone_pts = [p for p in points if p['beam'] in [2,3] and 12 < p['az'] < 20 and 800 < p['r'] < 2200 and p['snr']>=10]
drone_pts.sort(key=lambda x: x['t'])

print(f"Track: {len(tracks)} frames, Frame {tracks[0]['frame']}-{tracks[-1]['frame']}, t={tracks[0]['t']}-{tracks[-1]['t']}ms")
print(f"Drone truth: {len(drone_pts)} beam2/3 detections")

# Build a dictionary of track by timestamp for quick lookup
track_by_t = {t['t']: t for t in tracks}
track_by_frame = {t['frame']: t for t in tracks}

# Compare at beam 2/3 drone detection times
print("\n=== Comparison at actual drone detection times (beam 2/3) ===")
print(f"{'t(ms)':>10} {'beam':>4} {'az_meas':>8} {'az_track':>9} {'az_err':>7} "
      f"{'r_meas':>7} {'r_track':>8} {'r_err':>6} {'v_meas':>7} {'v_track':>8}")
print("-" * 90)

t_start_data = 898140
dt_frame = 164
errors_az = []
errors_r = []
errors_v = []
matched = 0

for dp in drone_pts:
    t = dp['t']
    # Find closest track frame within 200ms
    best_match = None
    best_dt = 999999
    for tr in tracks:
        dt = abs(tr['t'] - t)
        if dt < best_dt and dt <= 500:
            best_dt = dt
            best_match = tr
    
    if best_match:
        az_err = best_match['az'] - dp['az']
        r_err = best_match['r'] - dp['r']
        v_err = best_match['v'] - dp['v']
        errors_az.append(abs(az_err))
        errors_r.append(abs(r_err))
        errors_v.append(abs(v_err))
        matched += 1
        print(f"{t:>10} {dp['beam']:>4} {dp['az']:>8.2f} {best_match['az']:>9.2f} {az_err:>+7.2f} "
              f"{dp['r']:>7.0f} {best_match['r']:>8.0f} {r_err:>+6.0f} "
              f"{dp['v']:>7.1f} {best_match['v']:>8.1f}")

if matched > 0:
    print("-" * 90)
    print(f"\n=== Accuracy Statistics ({matched} matched points) ===")
    print(f"Azimuth:  mean_abs={sum(errors_az)/len(errors_az):.3f}°, RMS={math.sqrt(sum(e*e for e in errors_az)/len(errors_az)):.3f}°")
    print(f"Range:    mean_abs={sum(errors_r)/len(errors_r):.1f}m, RMS={math.sqrt(sum(e*e for e in errors_r)/len(errors_r)):.1f}m")
    print(f"Velocity: mean_abs={sum(errors_v)/len(errors_v):.2f}m/s, RMS={math.sqrt(sum(e*e for e in errors_v)/len(errors_v)):.2f}m/s")

# Show full drone trajectory (distance over time) to see "away then back" pattern
print("\n=== Drone ground truth: distance trend ===")
prev_t = 0
for dp in drone_pts:
    if 898000 < dp['t'] < 970000:
        dt = dp['t'] - prev_t if prev_t else 0
        print(f"  t={dp['t']}ms, beam={dp['beam']}, az={dp['az']:.2f}°, el={dp['el']:.2f}°, r={dp['r']:.0f}m, v={dp['v']:.1f}m/s, dt={dt}ms")
        prev_t = dp['t']

# Check why track ends at 961280ms
print(f"\n=== Track end analysis ===")
last_track = tracks[-1]
print(f"Last track: Frame {last_track['frame']} (t={last_track['t']}ms), R={last_track['r']:.0f}m, V={last_track['v']:.1f}m/s")

# Check drone data after t=961280ms - does it turn back?
print("\n=== Drone data AFTER track ends (t>961280ms) ===")
after = [dp for dp in drone_pts if dp['t'] > 961280]
if after:
    for dp in after[:20]:
        print(f"  t={dp['t']}ms, beam={dp['beam']}, az={dp['az']:.2f}°, r={dp['r']:.0f}m, v={dp['v']:.1f}m/s")
else:
    print("  No drone points after track ends within 400-frame range")

# Check frames 385-400 in output - are they missing?
print("\n=== Output frames after last track (Frames 385-400) ===")
with open(r'd:\DSP\6678\track\track_1\输出数据.txt', 'r', encoding='utf-8', errors='replace') as f:
    content = f.read()
for fn in range(385, 400):
    if f'Frame {fn} ' in content or f'Frame{fn} ' in content:
        # Find lines with this frame
        for line in content.split('\n'):
            if f'Frame {fn} ' in line or f'Frame {fn}|' in line:
                print(f"  {line.strip()[:100]}")
                break
    else:
        pass  # frame not in output = track died
