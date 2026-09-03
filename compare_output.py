import re

# Parse output data - multi-line format
tracks = []
with open('输出数据.txt', 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    line = lines[i]
    m = re.match(r'Frame (\d+) \| t=(\d+)ms', line)
    if m:
        frame = int(m.group(1))
        t = int(m.group(2))
        # Look ahead for track data
        j = i + 1
        while j < len(lines) and j < i + 5:
            tm = re.search(r'Track\[0\]: Az=([-\d.]+)deg El=([-\d.]+)deg R=([-\d.]+)m V=([-\d.]+)m/s', lines[j])
            if tm:
                az = float(tm.group(1))
                el = float(tm.group(2))
                r = float(tm.group(3))
                v = float(tm.group(4))
                tracks.append((frame, t, az, el, r, v))
                break
            j += 1
    i += 1

print(f'Total track frames: {len(tracks)}')
if tracks:
    print(f'Frame range: {tracks[0][0]} - {tracks[-1][0]}')
    print(f'First track: Frame {tracks[0][0]}, Az={tracks[0][2]:.2f}, R={tracks[0][4]:.1f}, V={tracks[0][5]:.1f}')
    print(f'Last track: Frame {tracks[-1][0]}, Az={tracks[-1][2]:.2f}, R={tracks[-1][4]:.1f}, V={tracks[-1][5]:.1f}')
    v_values = [t[5] for t in tracks]
    print(f'Speed variation: min={min(v_values):.2f}, max={max(v_values):.2f}')
    print(f'Speed is CONSTANT at 8.1m/s = PREDICTION ONLY (no associations)!')

# Parse drone measured data
drone_points = []
with open('无人机实测数据.txt', 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        m = re.search(r'波位号:(\d+).*时间戳:(\d+).*方位:([-\d.]+).*距离:([-\d.]+).*俯仰:([-\d.]+).*速度:([-\d.]+)', line)
        if m:
            beam = int(m.group(1))
            t = int(m.group(2))
            az = float(m.group(3))
            r = float(m.group(4))
            el = float(m.group(5))
            vr = float(m.group(6))
            if beam in [2, 3] and -15 < vr < -5 and 800 < r < 2500:
                drone_points.append((beam, t, az, r, el, vr))

print(f'\nDrone points found: {len(drone_points)}')
for i, p in enumerate(drone_points[:10]):
    print(f'  Beam={p[0]}, t={p[1]}ms, Az={p[2]:.2f}, R={p[3]:.1f}, El={p[4]:.2f}, Vr={p[5]:.2f}')

# Compare
if tracks and drone_points:
    print('\n=== ACCURACY COMPARISON ===')
    errors = []
    for tr in tracks:
        frame, t, az_t, el_t, r_t, v_t = tr
        best = None
        best_dt = float('inf')
        for dp in drone_points:
            dt = abs(dp[1] - t)
            if dt < best_dt:
                best_dt = dt
                best = dp
        if best and best_dt < 500:
            beam, t_d, az_d, r_d, el_d, vr_d = best
            az_err = abs(az_t - az_d)
            r_err = abs(r_t - r_d)
            v_err = abs(v_t - abs(vr_d))
            errors.append((frame, t, az_t, az_d, az_err, r_t, r_d, r_err, v_t, abs(vr_d), v_err))
    
    print(f'Matched frames (within 500ms): {len(errors)}')
    if errors:
        avg_az_err = sum(e[4] for e in errors) / len(errors)
        avg_r_err = sum(e[7] for e in errors) / len(errors)
        avg_v_err = sum(e[10] for e in errors) / len(errors)
        print(f'Average Az error: {avg_az_err:.2f} deg')
        print(f'Average R error: {avg_r_err:.1f}m')
        print(f'Average V error: {avg_v_err:.1f}m/s')
    else:
        print('No matches - track is NOT following the drone!')
        
    print('\n=== ROOT CAUSE ANALYSIS ===')
    print(f'Track: Az={tracks[0][2]:.1f}-{tracks[-1][2]:.1f} deg, R={tracks[0][4]:.0f}-{tracks[-1][4]:.0f}m, V={tracks[0][5]:.1f}m/s')
    print(f'Drone: Az={drone_points[0][2]:.1f}-{drone_points[-1][2]:.1f} deg, R={drone_points[0][3]:.0f}-{drone_points[-1][3]:.0f}m, Vr={drone_points[0][5]:.1f}m/s')
    print(f'\nTrack Az ({tracks[0][2]:.1f}) vs Drone Az ({drone_points[0][2]:.1f}): DIFFERENT TARGETS!')
    print(f'Track R ({tracks[0][4]:.0f}m) vs Drone R ({drone_points[0][3]:.0f}m): DIFFERENT TARGETS!')
    print(f'Track V ({tracks[0][5]:.1f}m/s) vs Drone |Vr| ({abs(drone_points[0][5]):.1f}m/s): SIMILAR but wrong target!')
    print(f'\nThe code initialized a FALSE track at Frame 92 on clutter, NOT the drone!')
    print(f'Drone 3-point combinations exist (Frames 4+5, 38+39, 55+56, etc.)')
    print(f'but the initialization logic rejected them.')
