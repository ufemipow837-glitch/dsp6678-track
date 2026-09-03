import re, sys, math

# Parse output data
output = {}
with open('输出数据.txt', 'r', encoding='utf-8') as f:
    for line in f:
        m = re.match(r'Frame\s+(\d+)\s+\|\s+t=(\d+)ms\s+\|\s+n=(\d+)', line)
        if m:
            frame = int(m.group(1))
            t = int(m.group(2))
            output[frame] = {'t': t, 'tracks': []}
        m2 = re.match(r'\s+-> Track\[\d+\]: Az=([\d.]+)deg El=([\d.]+)deg R=([\d.]+)m V=([\d.]+)m/s', line)
        if m2:
            if frame in output:
                output[frame]['tracks'].append({
                    'az': float(m2.group(1)), 'el': float(m2.group(2)),
                    'r': float(m2.group(3)), 'v': float(m2.group(4))})

first_track_frame = None
for frm in sorted(output.keys()):
    if output[frm]['tracks']:
        first_track_frame = frm
        break

print('=== OUTPUT DATA ANALYSIS ===')
print('First reliable track at Frame %d' % first_track_frame)
if first_track_frame:
    trks = output[first_track_frame]['tracks']
    for t in trks:
        print('  Track: Az=%.2f El=%.2f R=%.1f V=%.1f' % (t['az'], t['el'], t['r'], t['v']))
    t_92 = output[first_track_frame]['t']
    print('  Timestamp: %dms' % t_92)

# Parse drone measured data
drone = {}
with open('无人机实测数据.txt', 'r', encoding='utf-8') as f:
    for line in f:
        m = re.match(r'点迹解析数据： 波位号:(\d+).*?时间戳:(\d+).*?方位:([\d.]+).*?距离:([\d.]+).*?俯仰:([\d.-]+).*?速度:([\d.-]+)', line)
        if m:
            beam = int(m.group(1))
            t = int(m.group(2))
            az = float(m.group(3))
            r = float(m.group(4))
            el = float(m.group(5))
            v = float(m.group(6))
            if t not in drone:
                drone[t] = []
            drone[t].append({'beam': beam, 'az': az, 'r': r, 'el': el, 'v': v})

# Find all points near the false track location
if first_track_frame:
    print('\n=== POINTS NEAR FALSE TRACK LOCATION (Az~31.4, R~787m) ===')
    for t in sorted(drone.keys()):
        for pt in drone[t]:
            if abs(pt['az'] - 31.4) < 3.0 and abs(pt['r'] - 787) < 50:
                fr = int((t - 898140 + 82) / 164)
                print('  Frame %d, t=%dms, beam=%d, az=%.2f, r=%.1f, v=%.2f' % (fr, t, pt['beam'], pt['az'], pt['r'], pt['v']))

# Show drone trajectory
print('\n=== DRONE TRAJECTORY (Vr < -3 m/s) ===')
for t in sorted(drone.keys()):
    for pt in drone[t]:
        if pt['v'] < -3.0:
            fr = int((t - 898140 + 82) / 164)
            print('  Frame %d, t=%dms, beam=%d, az=%.2f, r=%.1f, v=%.2f' % (fr, t, pt['beam'], pt['az'], pt['r'], pt['v']))

# Check what 3-point combination could form at Frame 92
if first_track_frame:
    print('\n=== SIMULATING 3-POINT COMBINATIONS FOR FRAME %d ===' % first_track_frame)
    t_target = output[first_track_frame]['t']
    
    # Collect all points in frames 0 to first_track_frame
    all_points = []
    for t in sorted(drone.keys()):
        if t <= t_target:
            for pt in drone[t]:
                fr = int((t - 898140 + 82) / 164)
                all_points.append({'t': t, 'frame': fr, 'az': pt['az'], 'r': pt['r'], 
                                  'el': pt['el'], 'v': pt['v'], 'beam': pt['beam']})
    
    # Find all 3-point combinations where the 3rd point is at Frame first_track_frame
    target_frame = first_track_frame
    target_points = [p for p in all_points if p['frame'] == target_frame]
    
    # Look for 2-point combos in earlier frames that could match
    for tp in target_points:
        print('\nTarget point at Frame %d: az=%.2f, r=%.1f, v=%.2f, beam=%d' % (tp['frame'], tp['az'], tp['r'], tp['v'], tp['beam']))
        
        # Check what previous temp tracks might exist
        # Look for points at similar az/r in frames 90, 91
        for prev_f in [target_frame - 2, target_frame - 1]:
            prev_pts = [p for p in all_points if p['frame'] == prev_f]
            for pp in prev_pts:
                dist_az = abs(pp['az'] - tp['az'])
                dist_r = abs(pp['r'] - tp['r'])
                if dist_az < 5.0 and dist_r < 500:
                    print('  Possible prev point Frame %d: az=%.2f, r=%.1f, v=%.2f, beam=%d (dAz=%.1f, dR=%.1f)' % 
                          (pp['frame'], pp['az'], pp['r'], pp['v'], pp['beam'], dist_az, dist_r))

# Now analyze: what happens with the drone's 3-point combination?
print('\n=== DRONE 3-POINT COMBINATION ANALYSIS ===')
drone_pts = []
for t in sorted(drone.keys()):
    for pt in drone[t]:
        if pt['v'] < -5.0:
            fr = int((t - 898140 + 82) / 164)
            drone_pts.append({'frame': fr, 't': t, 'az': pt['az'], 'r': pt['r'], 'el': pt['el'], 'v': pt['v'], 'beam': pt['beam']})

if len(drone_pts) >= 3:
    # First 3 drone points
    p0, p1, p2 = drone_pts[0], drone_pts[1], drone_pts[2]
    print('Drone 3-point combination:')
    print('  Pt0: Frame %d, az=%.2f, r=%.1f, v=%.2f, beam=%d' % (p0['frame'], p0['az'], p0['r'], p0['v'], p0['beam']))
    print('  Pt1: Frame %d, az=%.2f, r=%.1f, v=%.2f, beam=%d' % (p1['frame'], p1['az'], p1['r'], p1['v'], p1['beam']))
    print('  Pt2: Frame %d, az=%.2f, r=%.1f, v=%.2f, beam=%d' % (p2['frame'], p2['az'], p2['r'], p2['v'], p2['beam']))
    
    t01 = (p1['t'] - p0['t']) / 1000.0
    t12 = (p2['t'] - p1['t']) / 1000.0
    print('  Time gap 0->1: %.3fs, 1->2: %.3fs' % (t01, t12))
    
    # Check velocity consistency
    print('  Vr values: %.2f, %.2f, %.2f' % (p0['v'], p1['v'], p2['v']))
    print('  Vr signs: %d, %d, %d' % (1 if p0['v'] > 0 else -1, 1 if p1['v'] > 0 else -1, 1 if p2['v'] > 0 else -1))
    
    # Az span
    az0_rad = p0['az'] * math.pi / 180.0
    az2_rad = p2['az'] * math.pi / 180.0
    az_span = abs(p2['az'] - p0['az'])
    if az_span > 180: az_span = 360 - az_span
    print('  Az span: %.2f deg' % az_span)
    
    # El span
    el_span = abs(p2['el'] - p0['el'])
    print('  El span: %.2f deg' % el_span)
    
    # R monotonicity
    r0, r1, r2 = p0['r'], p1['r'], p2['r']
    increasing = r1 > r0 and r2 > r1
    decreasing = r1 < r0 and r2 < r1
    print('  R values: %.1f, %.1f, %.1f (inc=%d, dec=%d)' % (r0, r1, r2, increasing, decreasing))
    
    # Low speed check
    vr_avg_abs = (abs(p0['v']) + abs(p1['v']) + abs(p2['v'])) / 3.0
    all_vr_1_30 = all(1.0 <= abs(p['v']) <= 30.0 for p in [p0, p1, p2])
    print('  Vr avg abs: %.2f, all in 1-30: %d' % (vr_avg_abs, all_vr_1_30))
    print('  low_speed_pass would be: %d' % (1 if (1 <= vr_avg_abs <= 30 and all_vr_1_30) else 0))
    
    # What is the Mahalanobis distance likely?
    # With T = t12 = 2.624s, the predicted position uncertainty would be huge
    # Let's calculate approximate position change
    v_radial_avg = -11.74  # m/s negative = approaching
    dr_predicted = v_radial_avg * t12  # negative = R decreasing
    print('  Predicted R change in %.3fs: %.1f m' % (t12, dr_predicted))
    print('  Actual R change: %.1f m' % (r2 - r1))
    print('  NOTE: Large time gap %.3fs between pt1 and pt2 causes large Mahalanobis distance' % t12)

print('\n=== ANALYSIS COMPLETE ===')
