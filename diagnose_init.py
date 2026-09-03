# -*- coding: utf-8 -*-
"""
Simulate new_reliable.c initialization checks on candidate 3-point combinations.
Goal: diagnose why real drone (Frame 4, 5, 61) was rejected,
      and why the false target (Az=31.4, R=786) was accepted at Frame 92.
"""
import math
import os

pi = math.pi

# ---------------------------------------------------------------------------
# 1. Real drone candidate (known from data_entry.c)
#    Frame 4 (t=898796ms, beamNo=2): az=16.8225 deg, el=5.1668 deg, R=1094.84, Vr=-11.7406
#    Frame 5 (t=898960ms, beamNo=3): az=17.3911 deg, el=4.7861 deg, R=1095.84, Vr=-11.7406
#    Frame 61 (t=901584ms, beamNo=2): az=16.5781 deg, el=4.6226 deg, R=1124.10, Vr=-11.7406
# ---------------------------------------------------------------------------
real_drone = {
    'frame0': 4, 't0': 898796, 'beam0': 2,
    'az0': 16.8225, 'el0': 5.1668, 'r0': 1094.84, 'vr0': -11.7406,
    'frame1': 5, 't1': 898960, 'beam1': 3,
    'az1': 17.3911, 'el1': 4.7861, 'r1': 1095.84, 'vr1': -11.7406,
    'frame2': 61, 't2': 901584, 'beam2': 2,
    'az2': 16.5781, 'el2': 4.6226, 'r2': 1124.10, 'vr2': -11.7406,
}

# ---------------------------------------------------------------------------
# 2. False target candidate (guessed from output_data.txt)
#    Frame 92 (t=913228ms) established track: Az=31.41deg, R=786.7m, V=8.2m/s
#    We need to find 3 points around Frame 92 that could form this.
#    Let's scan drone_data_parsed.csv for points near Frame 92 (t=913228ms)
# ---------------------------------------------------------------------------

def simulate_checks(p, label):
    """
    p: dict with keys: t0, t2, az0, az2, el0, el2, r0, r1, r2, vr0, vr1, vr2, beam0, beam2
    """
    final_ok = 1
    reasons = []
    
    # Extract
    vr0, vr1, vr2 = p['vr0'], p['vr1'], p['vr2']
    r0, r1, r2 = p['r0'], p['r1'], p['r2']
    az0, az1, az2 = p['az0'], p.get('az1', p['az0']), p['az2']
    el0, el1, el2 = p['el0'], p.get('el1', p['el0']), p['el2']
    beam0, beam1, beam2 = p['beam0'], p.get('beam1', p['beam0']), p['beam2']
    t0, t2 = p['t0'], p['t2']
    
    print(f"\n{'='*60}")
    print(f"Checking candidate: {label}")
    print(f"{'='*60}")
    print(f"Point 0: az={az0:.2f}°, el={el0:.2f}°, r={r0:.1f}m, vr={vr0:.2f}m/s, beam={beam0}")
    print(f"Point 1: az={az1:.2f}°, el={el1:.2f}°, r={r1:.1f}m, vr={vr1:.2f}m/s, beam={beam1}")
    print(f"Point 2: az={az2:.2f}°, el={el2:.2f}°, r={r2:.1f}m, vr={vr2:.2f}m/s, beam={beam2}")
    
    # ① Vr sign consistency
    s0 = 1 if vr0 > 0 else (-1 if vr0 < 0 else 0)
    s1 = 1 if vr1 > 0 else (-1 if vr1 < 0 else 0)
    s2 = 1 if vr2 > 0 else (-1 if vr2 < 0 else 0)
    print(f"\n--- Check 1: Vr Sign Consistency ---")
    print(f"  s0={s0}, s1={s1}, s2={s2}")
    if s0 != 0 and s1 != 0 and s2 != 0:
        if (s0 != s1) or (s1 != s2) or (s0 != s2):
            final_ok = 0
            reasons.append("Vr sign inconsistency")
            print(f"  FAIL: Vr directions are mixed")
        else:
            print(f"  PASS")
    else:
        print(f"  SKIP (some vr=0)")
    
    # Vr magnitude ratio
    vr_min = min(vr0, vr1, vr2)
    vr_max = max(vr0, vr1, vr2)
    vr_min_abs = min(abs(vr0), abs(vr1), abs(vr2))
    vr_max_abs = max(abs(vr0), abs(vr1), abs(vr2))
    vr_avg_abs = (abs(vr0) + abs(vr1) + abs(vr2)) / 3.0
    print(f"\n  Vr avg_abs={vr_avg_abs:.2f}, min_abs={vr_min_abs:.2f}, max_abs={vr_max_abs:.2f}")
    
    if vr_max_abs > 0.01 and vr_min_abs > 0.01:
        vr_ratio = vr_max_abs / (vr_min_abs + 1e-6)
        print(f"  Vr ratio (max/min) = {vr_ratio:.2f}")
        if vr_ratio > 5.0:
            final_ok = 0
            reasons.append(f"Vr magnitude ratio too high ({vr_ratio:.1f} > 5)")
            print(f"  FAIL: Vr ratio > 5")
        else:
            print(f"  PASS")
    
    # ② Target type classification
    print(f"\n--- Check 2: Target Type Classification ---")
    drone_like = 1 if vr_avg_abs <= 80.0 else 0
    print(f"  vr_avg_abs={vr_avg_abs:.2f} -> {'drone' if drone_like else 'projectile'}")
    
    # Low-speed pass
    low_speed_pass = 0
    if 1.0 <= vr_avg_abs <= 30.0 and drone_like:
        all_valid = all(1.0 <= abs(vr) <= 30.0 for vr in [vr0, vr1, vr2])
        if all_valid:
            low_speed_pass = 1
            print(f"  LOW_SPEED_PASS = 1 (all Vr in 1-30 m/s)")
    
    # ③ Distance monotonicity
    print(f"\n--- Check 3: Distance Monotonicity ---")
    if not low_speed_pass:
        inc = (r1 > r0) and (r2 > r1)
        dec = (r1 < r0) and (r2 < r1)
        stable = abs(r1 - r0) < 5.0 and abs(r2 - r1) < 5.0
        print(f"  inc={inc}, dec={dec}, stable={stable}")
        if not inc and not dec and not stable:
            if drone_like and vr_avg_abs < 15.0:
                print(f"  PASS (drone low-speed, R fluctuation allowed)")
            else:
                final_ok = 0
                reasons.append("R not monotonic and not stable")
                print(f"  FAIL")
        else:
            print(f"  PASS")
    else:
        print(f"  SKIP (low_speed_pass=1)")
    
    delta_r = r2 - r0
    
    # ④ Azimuth span
    print(f"\n--- Check 4: Azimuth Span ---")
    az_span = abs(az2 - az0)
    if az_span > 180.0:
        az_span = 360.0 - az_span
    print(f"  az_span = {az_span:.2f}°")
    if az_span > 30.0:
        final_ok = 0
        reasons.append(f"Azimuth span too large ({az_span:.1f}° > 30°)")
        print(f"  FAIL")
    else:
        print(f"  PASS")
    
    # ⑤ Elevation span
    print(f"\n--- Check 5: Elevation Span ---")
    el_span = abs(el2 - el0)
    print(f"  el_span = {el_span:.2f}°")
    if el_span > 25.0:
        final_ok = 0
        reasons.append(f"Elevation span too large ({el_span:.1f}° > 25°)")
        print(f"  FAIL")
    else:
        print(f"  PASS")
    
    # ⑥ Absolute sector constraints
    print(f"\n--- Check 6: Absolute Sector Constraints ---")
    if not drone_like or vr_avg_abs < 600.0:
        az_norm = az2
        while az_norm > 180.0:
            az_norm -= 360.0
        while az_norm < -180.0:
            az_norm += 360.0
        print(f"  az_norm = {az_norm:.2f}°, el2 = {el2:.2f}°, r2 = {r2:.1f}m")
        
        if vr_avg_abs < 150.0:
            if az_norm > 60.0 or az_norm < -20.0:
                final_ok = 0
                reasons.append(f"Azimuth out of sector ({az_norm:.1f}° not in [-20°, 60°])")
                print(f"  FAIL: azimuth out of allowed sector")
            else:
                print(f"  PASS: azimuth in sector")
            
            if el2 < -15.0 or el2 > 75.0:
                final_ok = 0
                reasons.append(f"Elevation out of bounds ({el2:.1f}° not in [-15°, 75°])")
                print(f"  FAIL: elevation out of bounds")
            else:
                print(f"  PASS: elevation in bounds")
            
            if r2 > 8000.0 or r0 > 8000.0:
                final_ok = 0
                reasons.append(f"Distance too large (r0={r0:.1f}, r2={r2:.1f} > 8000m)")
                print(f"  FAIL: distance too large")
            else:
                print(f"  PASS: distance OK")
    
    # ⑦ Beam reasonableness
    print(f"\n--- Check 7: Beam Reasonableness ---")
    beam_span = abs(beam2 - beam0)
    print(f"  beam_span = {beam_span}")
    if beam_span > 5.0:
        if az_span > 15.0:
            final_ok = 0
            reasons.append(f"Beam span too large ({beam_span} > 5) with large azimuth span ({az_span:.1f}° > 15°)")
            print(f"  FAIL")
        else:
            print(f"  PASS (beam span large but azimuth span OK)")
    else:
        print(f"  PASS")
    
    # ⑧ Cross-validation
    print(f"\n--- Check 8: Cross-validation (Vr vs R trend) ---")
    if not low_speed_pass:
        dt_s = (t2 - t0) / 1000.0
        if dt_s > 0.01:
            Vr_from_R = delta_r / dt_s
            R_ref = max(r0, r2)
            rel_delta = abs(delta_r) / (R_ref + 1.0)
            print(f"  dt={dt_s:.2f}s, delta_r={delta_r:.1f}m, Vr_from_R={Vr_from_R:.2f}m/s")
            print(f"  rel_delta={rel_delta:.4f} ({rel_delta*100:.1f}%)")
            
            if vr2 < -2.0:
                print(f"  vr2 < -2.0 (approaching), delta_r = {delta_r:.1f}m")
                if delta_r > 0 and rel_delta > 0.20:
                    final_ok = 0
                    reasons.append(f"Approaching but distance increasing significantly")
                    print(f"  FAIL: approaching but R increases by {rel_delta*100:.1f}%")
                else:
                    print(f"  PASS")
            elif vr2 > 2.0:
                print(f"  vr2 > 2.0 (receding), delta_r = {delta_r:.1f}m")
                if delta_r < 0 and rel_delta > 0.20:
                    final_ok = 0
                    reasons.append(f"Receding but distance decreasing significantly")
                    print(f"  FAIL: receding but R decreases by {rel_delta*100:.1f}%")
                else:
                    print(f"  PASS")
            
            if (vr2 > 5.0 and Vr_from_R < -10.0) or (vr2 < -5.0 and Vr_from_R > 10.0):
                final_ok = 0
                reasons.append(f"Vr and R change direction completely opposite")
                print(f"  FAIL: Vr and R direction conflict")
    else:
        print(f"  SKIP (low_speed_pass=1)")
    
    print(f"\n{'='*60}")
    print(f"FINAL RESULT: {'PASS' if final_ok else 'FAIL'}")
    if not final_ok:
        print(f"Reasons:")
        for r in reasons:
            print(f"  - {r}")
    print(f"{'='*60}")
    
    return final_ok, reasons


# Test 1: Real drone
print("="*70)
print("TEST 1: REAL DRONE CANDIDATE")
print("="*70)
ok, reasons = simulate_checks(real_drone, "Real Drone (Frame 4,5,61)")

# Test 2: Try a candidate that matches the false track output (Az≈31.4°, R≈787m)
# We need to find 3 points from drone_data_parsed.csv around Frame 92
# Let's read the CSV and find candidates

import csv

csv_path = r"d:\DSP\6678\track\track_1\drone_data_parsed.csv"
pts_near_frame92 = []

with open(csv_path, 'r', encoding='utf-8-sig') as f:
    reader = csv.DictReader(f)
    for row in reader:
        t_ms = float(row['时间戳(ms)'])
        if 912000 <= t_ms <= 915000:  # around Frame 92 (t=913228)
            pts_near_frame92.append({
                't': t_ms,
                'az': float(row['方位(度)']),
                'el': float(row['俯仰(度)']),
                'r': float(row['距离(m)']),
                'vr': float(row['径向速度(m/s)']),
                'beam': int(row['波位号']),
            })

print("\n" + "="*70)
print("POINTS NEAR FRAME 92 (t=912000-915000ms)")
print("="*70)
for p in pts_near_frame92:
    print(f"  t={p['t']:.0f}ms, beam={p['beam']}, az={p['az']:.2f}°, el={p['el']:.2f}°, r={p['r']:.1f}m, vr={p['vr']:.2f}m/s")

# Now try to form 3-point combinations from these points that could produce Az≈31.4°
# The false track at Frame 92 has Az≈31.4°, R≈787m
# Look for points with Az around 31° and R around 787m
print("\n" + "="*70)
print("SEARCHING FOR FALSE TARGET CANDIDATES")
print("="*70)

# Get all points from Frame 85 to 95 (t around 911000-915000)
all_pts = []
with open(csv_path, 'r', encoding='utf-8-sig') as f:
    reader = csv.DictReader(f)
    for row in reader:
        t_ms = float(row['时间戳(ms)'])
        if 909000 <= t_ms <= 916000:
            all_pts.append({
                't': t_ms,
                'az': float(row['方位(度)']),
                'el': float(row['俯仰(度)']),
                'r': float(row['距离(m)']),
                'vr': float(row['径向速度(m/s)']),
                'beam': int(row['波位号']),
            })

# Find points near Az=31°, R=787m
candidates_near_false = []
for p in all_pts:
    if abs(p['az'] - 31.4) < 15.0 and abs(p['r'] - 787) < 500:
        candidates_near_false.append(p)
        print(f"  NEAR FALSE TARGET: t={p['t']:.0f}ms, beam={p['beam']}, az={p['az']:.2f}°, el={p['el']:.2f}°, r={p['r']:.1f}m, vr={p['vr']:.2f}m/s")

# Try 3-point combinations from points around Az 28-34°
print("\n" + "="*70)
print("TESTING FALSE TARGET 3-POINT COMBINATIONS")
print("="*70)

from itertools import combinations

# Filter points with Az between 28-35°
filtered_pts = [p for p in all_pts if 28.0 <= p['az'] <= 35.0]
print(f"Found {len(filtered_pts)} points with Az 28-35°")
for p in filtered_pts:
    print(f"  t={p['t']:.0f}ms, beam={p['beam']}, az={p['az']:.2f}°, el={p['el']:.2f}°, r={p['r']:.1f}m, vr={p['vr']:.2f}m/s")

# Test combinations
for c in combinations(range(len(filtered_pts)), 3):
    p0, p1, p2 = filtered_pts[c[0]], filtered_pts[c[1]], filtered_pts[c[2]]
    cand = {
        't0': p0['t'], 't2': p2['t'],
        'az0': p0['az'], 'az1': p1['az'], 'az2': p2['az'],
        'el0': p0['el'], 'el1': p1['el'], 'el2': p2['el'],
        'r0': p0['r'], 'r1': p1['r'], 'r2': p2['r'],
        'vr0': p0['vr'], 'vr1': p1['vr'], 'vr2': p2['vr'],
        'beam0': p0['beam'], 'beam1': p1['beam'], 'beam2': p2['beam'],
    }
    ok, reasons = simulate_checks(cand, f"False Candidate [{c[0]},{c[1]},{c[2]}]")
    if ok:
        print(f"\n!!! THIS COMBINATION PASSED ALL CHECKS !!!")
        print(f"     p0: t={p0['t']:.0f}ms, az={p0['az']:.2f}°, r={p0['r']:.1f}m")
        print(f"     p1: t={p1['t']:.0f}ms, az={p1['az']:.2f}°, r={p1['r']:.1f}m")
        print(f"     p2: t={p2['t']:.0f}ms, az={p2['az']:.2f}°, r={p2['r']:.1f}m")
        break
