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

# Get all track frames
track_frames = []
for frm in sorted(output.keys()):
    if output[frm]['tracks']:
        track_frames.append((frm, output[frm]['t'], output[frm]['tracks']))

print('=== ALL RELIABLE TRACKS IN OUTPUT ===')
for frm, t, trks in track_frames[:10]:
    for t2 in trks:
        print('  Frame %d (t=%dms): Az=%.2f El=%.2f R=%.1f V=%.1f' % (frm, t, t2['az'], t2['el'], t2['r'], t2['v']))

# Parse all data from drone measured file
all_data = []
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
            # Calculate frame number from timestamp
            frame = int((t - 898140 + 82) / 164)
            all_data.append({'frame': frame, 't': t, 'beam': beam, 'az': az, 'r': r, 'el': el, 'v': v})

# Identify REAL drone points: beam 2/3, Vr ~ -11.74, Az 12-22, R 900-1400
drone_pts = [p for p in all_data 
             if p['beam'] in [2, 3] 
             and 12 <= p['az'] <= 22 
             and 900 <= p['r'] <= 1400
             and -15 <= p['v'] <= -8]

print('\n=== REAL DRONE TRAJECTORY ===')
for p in drone_pts[:15]:
    print('  Frame %d, t=%dms, beam=%d, az=%.2f, r=%.1f, el=%.2f, v=%.2f' % 
          (p['frame'], p['t'], p['beam'], p['az'], p['r'], p['el'], p['v']))

# Now analyze: what 3-point combo creates the first reliable track?
if track_frames:
    first_frame, first_t, first_trks = track_frames[0]
    print('\n=== ANALYZING FIRST RELIABLE TRACK ===')
    print('Frame %d (t=%dms): %s' % (first_frame, first_t, first_trks))
    
    t_target = first_t
    
    # Find all points near the track location
    for trk in first_trks:
        print('\nSearching for points near Az=%.2f, R=%.1f:' % (trk['az'], trk['r']))
        for p in all_data:
            if abs(p['az'] - trk['az']) < 3.0 and abs(p['r'] - trk['r']) < 100:
                print('  Frame %d, t=%dms, beam=%d, az=%.2f, r=%.1f, v=%.2f' % 
                      (p['frame'], p['t'], p['beam'], p['az'], p['r'], p['v']))

# Now simulate: why does the drone 3-point combination fail?
if len(drone_pts) >= 3:
    print('\n=== DRONE 3-POINT COMBINATION CHECK ===')
    
    # The first 3 drone points
    p0, p1, p2 = drone_pts[0], drone_pts[1], drone_pts[2]
    
    print('3-Drone points:')
    print('  Pt0: Frame %d, az=%.2f, r=%.1f, v=%.2f, beam=%d' % (p0['frame'], p0['az'], p0['r'], p0['v'], p0['beam']))
    print('  Pt1: Frame %d, az=%.2f, r=%.1f, v=%.2f, beam=%d' % (p1['frame'], p1['az'], p1['r'], p1['v'], p1['beam']))
    print('  Pt2: Frame %d, az=%.2f, r=%.1f, v=%.2f, beam=%d' % (p2['frame'], p2['az'], p2['r'], p2['v'], p2['beam']))
    
    t01 = (p1['t'] - p0['t']) / 1000.0
    t12 = (p2['t'] - p1['t']) / 1000.0
    print('  Time gaps: 0->1=%.3fs, 1->2=%.3fs' % (t01, t12))
    
    # Check Vr consistency
    vr0, vr1, vr2 = p0['v'], p1['v'], p2['v']
    print('  Vr: %.2f, %.2f, %.2f' % (vr0, vr1, vr2))
    
    # Check low_speed conditions
    vr_avg_abs = (abs(vr0) + abs(vr1) + abs(vr2)) / 3.0
    all_vr_in_range = all(1.0 <= abs(v) <= 30.0 for v in [vr0, vr1, vr2])
    low_speed_pass = (1.0 <= vr_avg_abs <= 30.0) and all_vr_in_range
    
    print('  Vr avg abs: %.2f' % vr_avg_abs)
    print('  All Vr in 1-30: %d' % all_vr_in_range)
    print('  low_speed_pass: %d' % low_speed_pass)
    
    # Check Az span
    az_span = abs(p2['az'] - p0['az'])
    if az_span > 180: az_span = 360 - az_span
    print('  Az span: %.2f deg (limit 30)' % az_span)
    
    # Check beam span
    beam_span = abs(p2['beam'] - p0['beam'])
    print('  Beam span: %d (limit 5)' % beam_span)
    
    # Check R monotonicity
    r0, r1, r2 = p0['r'], p1['r'], p2['r']
    increasing = r1 > r0 and r2 > r1
    decreasing = r1 < r0 and r2 < r1
    stable = abs(r1-r0) < 5 and abs(r2-r1) < 5
    print('  R: %.1f, %.1f, %.1f (inc=%d, dec=%d, stable=%d)' % (r0, r1, r2, increasing, decreasing, stable))
    
    # The key issue: the time gap between pt1 and pt2 is 2.624s
    # The Mahalanobis distance for this large T will be huge
    # Let's simulate the get_asso_info2_dist function
    
    # Initial covariance after 2-point initialization (Frame 4 and Frame 5)
    # Initial sigma: R=20m, A=1deg, B=1deg
    # The Kalman filter after 2 close points (T=0.164s) has some uncertainty
    # After propagating T=2.624s, the predicted position uncertainty is huge
    
    # Let me check: what would the predicted position be?
    # Rough estimate: velocity from position difference
    vx_init = (p1['r'] - p0['r']) / t01  # radial velocity from R change
    # Actual Vr is -11.74, but position-based estimate:
    vx_init_actual = (p1['r'] - p0['r']) / t01
    print('  Position-based radial velocity: %.2f m/s' % vx_init_actual)
    print('  Actual radar Vr: %.2f m/s' % vr0)
    print('  NOTE: Position-based velocity may differ from Vr')
    
    # Predict position at pt2 time
    r_predicted = p1['r'] + vx_init_actual * t12
    print('  Predicted R at Frame %d: %.1f (actual: %.1f)' % (p2['frame'], r_predicted, p2['r']))
    print('  Prediction error: %.1f m' % abs(p2['r'] - r_predicted))
    
    # Check if this error fits within the gate
    # With INIT_SIGMA_R=20 and propagated covariance
    # The Mahalanobis distance would be error^2 / sigma_pred^2
    # sigma_pred grows with T, but so does the error
    
    print('\n=== KEY INSIGHT ===')
    print('The drone 3-point combination has a %.3fs gap between pt1 and pt2' % t12)
    print('During this time, the radar does NOT detect the drone (it scans other beams)')
    print('The propagated covariance grows, but the position prediction may still have large errors')
    print('The TRACK_INIT_TH=15.0 gate may reject this combination')
    
# Find what ACTUALLY forms the first track
if track_frames:
    print('\n=== WHAT FORMS THE FIRST TRACK AT FRAME %d ===' % track_frames[0][0])
    
    first_frm = track_frames[0][0]
    first_t = track_frames[0][1]
    
    # Collect all points in frames up to first_frm
    points_before = [p for p in all_data if p['frame'] <= first_frm]
    
    # Look for 3-point combinations ending at first_frm
    target_pts = [p for p in points_before if p['frame'] == first_frm]
    
    # Find all points at similar az/r in previous frames
    for tp in target_pts:
        if abs(tp['r'] - 787) < 100:  # The track R is ~787
            print('\nTarget point: Frame %d, az=%.2f, r=%.1f, v=%.2f, beam=%d' % 
                  (tp['frame'], tp['az'], tp['r'], tp['v'], tp['beam']))
            
            # Check frames 90, 91 (the two previous frames)
            for prev_f in range(first_frm - 3, first_frm):
                prev_pts = [p for p in points_before if p['frame'] == prev_f]
                for pp in prev_pts:
                    d_az = abs(pp['az'] - tp['az'])
                    d_r = abs(pp['r'] - tp['r'])
                    if d_az < 5.0 and d_r < 500:
                        print('  Prev point Frame %d: az=%.2f, r=%.1f, v=%.2f, beam=%d (dAz=%.1f, dR=%.1f)' % 
                              (pp['frame'], pp['az'], pp['r'], pp['v'], pp['beam'], d_az, d_r))
            
            # Check even earlier frames (85-89)
            for prev_f in range(max(0, first_frm - 7), first_frm - 3):
                prev_pts = [p for p in points_before if p['frame'] == prev_f]
                for pp in prev_pts:
                    d_az = abs(pp['az'] - tp['az'])
                    d_r = abs(pp['r'] - tp['r'])
                    if d_az < 5.0 and d_r < 500:
                        print('  Early point Frame %d: az=%.2f, r=%.1f, v=%.2f, beam=%d (dAz=%.1f, dR=%.1f)' % 
                              (pp['frame'], pp['az'], pp['r'], pp['v'], pp['beam'], d_az, d_r))

print('\n=== COMPLETE ===')
