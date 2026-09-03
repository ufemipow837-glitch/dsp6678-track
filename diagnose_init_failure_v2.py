import re
import math

def parse_data_entry_v2(filename):
    """Parse data_entry.c - new format with data[i].field[j] assignments"""
    with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    frames = []
    
    # Find all data[N].xxx assignments
    # Pattern: data[N].field[M] = value;
    
    # First find all frame-level metadata
    frame_meta = {}
    for match in re.finditer(r'data\[(\d+)\]\.targetNum\s*=\s*(\d+)', content):
        idx = int(match.group(1))
        frame_meta[idx] = {'targetNum': int(match.group(2))}
    
    for match in re.finditer(r'data\[(\d+)\]\.frameSn\s*=\s*(\d+)', content):
        idx = int(match.group(1))
        if idx in frame_meta:
            frame_meta[idx]['frameSn'] = int(match.group(2))
    
    for match in re.finditer(r'data\[(\d+)\]\.mSecond\s*=\s*([\d.]+)f?', content):
        idx = int(match.group(1))
        if idx in frame_meta:
            frame_meta[idx]['mSecond'] = float(match.group(2))
    
    for match in re.finditer(r'data\[(\d+)\]\.beamNo\s*=\s*(\d+)', content):
        idx = int(match.group(1))
        if idx in frame_meta:
            frame_meta[idx]['beamNo'] = int(match.group(2))
    
    for match in re.finditer(r'data\[(\d+)\]\.Month\s*=\s*(\d+)', content):
        idx = int(match.group(1))
        if idx in frame_meta:
            frame_meta[idx]['Month'] = int(match.group(2))
    
    # Now find all point-level data: azi[j], ele[j], range[j], velocity[j]
    # Build per-frame per-point data
    frame_points = {}  # {frame_idx: [{azi, ele, range, velocity}, ...]}
    
    for match in re.finditer(r'data\[(\d+)\]\.azi\[(\d+)\]\s*=\s*([-\d.]+)/180\.0f\*pi', content):
        fi = int(match.group(1))
        pi_idx = int(match.group(2))
        azi_deg = float(match.group(3))
        if fi not in frame_points:
            frame_points[fi] = {}
        if pi_idx not in frame_points[fi]:
            frame_points[fi][pi_idx] = {}
        frame_points[fi][pi_idx]['azi_deg'] = azi_deg
        frame_points[fi][pi_idx]['azi_rad'] = math.radians(azi_deg)
    
    for match in re.finditer(r'data\[(\d+)\]\.ele\[(\d+)\]\s*=\s*([-\d.]+)/180\.0f\*pi', content):
        fi = int(match.group(1))
        pi_idx = int(match.group(2))
        el_deg = float(match.group(3))
        if fi not in frame_points:
            frame_points[fi] = {}
        if pi_idx not in frame_points[fi]:
            frame_points[fi][pi_idx] = {}
        frame_points[fi][pi_idx]['el_deg'] = el_deg
        frame_points[fi][pi_idx]['el_rad'] = math.radians(el_deg)
    
    for match in re.finditer(r'data\[(\d+)\]\.range\[(\d+)\]\s*=\s*([-\d.]+)f?', content):
        fi = int(match.group(1))
        pi_idx = int(match.group(2))
        rng = float(match.group(3))
        if fi not in frame_points:
            frame_points[fi] = {}
        if pi_idx not in frame_points[fi]:
            frame_points[fi][pi_idx] = {}
        frame_points[fi][pi_idx]['range'] = rng
    
    for match in re.finditer(r'data\[(\d+)\]\.velocity\[(\d+)\]\s*=\s*([-\d.]+)f?', content):
        fi = int(match.group(1))
        pi_idx = int(match.group(2))
        vel = float(match.group(3))
        if fi not in frame_points:
            frame_points[fi] = {}
        if pi_idx not in frame_points[fi]:
            frame_points[fi][pi_idx] = {}
        frame_points[fi][pi_idx]['velocity'] = vel
    
    for match in re.finditer(r'data\[(\d+)\]\.Use_Flag_1\[(\d+)\]\s*=\s*(\d+)', content):
        fi = int(match.group(1))
        pi_idx = int(match.group(2))
        flag = int(match.group(3))
        if fi not in frame_points:
            frame_points[fi] = {}
        if pi_idx not in frame_points[fi]:
            frame_points[fi][pi_idx] = {}
        frame_points[fi][pi_idx]['flag'] = flag
    
    # Build complete frame data
    all_frames = []
    for idx in sorted(frame_meta.keys()):
        meta = frame_meta[idx]
        points = []
        if idx in frame_points:
            for pi_idx in sorted(frame_points[idx].keys()):
                pt = frame_points[idx][pi_idx]
                if pt.get('flag', 0) == 1:
                    pt['point_idx'] = pi_idx
                    pt['frame_idx'] = idx
                    pt['beamNo'] = meta.get('beamNo', -1)
                    pt['frameSn'] = meta.get('frameSn', idx)
                    pt['mSecond'] = meta.get('mSecond', 0)
                    points.append(pt)
        if points:
            all_frames.extend(points)
    
    return all_frames

def simulate_full_init_checks(points_combo, Vmin=2.0, Vmax=1500.0):
    """Simulate ALL initialization checks from new_reliable.c"""
    p0, p1, p2 = points_combo
    
    results = []
    final_ok = 1
    failures = []
    
    # Extract 3-point data
    vr0, vr1, vr2 = p0['velocity'], p1['velocity'], p2['velocity']
    r0, r1, r2 = p0['range'], p1['range'], p2['range']
    az0, az1, az2 = p0['azi_deg'], p1['azi_deg'], p2['azi_deg']
    el0, el1, el2 = p0['el_deg'], p1['el_deg'], p2['el_deg']
    beam0, beam1, beam2 = p0['beamNo'], p1['beamNo'], p2['beamNo']
    t0, t1, t2 = p0['mSecond'], p1['mSecond'], p2['mSecond']
    f0, f1, f2 = p0['frame_idx'], p1['frame_idx'], p2['frame_idx']
    
    results.append(f"=== 3点组合: Frame{f0}(b{beam0}) -> Frame{f1}(b{beam1}) -> Frame{f2}(b{beam2}) ===")
    results.append(f"  Vr: [{vr0:+.2f}, {vr1:+.2f}, {vr2:+.2f}] m/s")
    results.append(f"  R:  [{r0:.1f}, {r1:.1f}, {r2:.1f}] m")
    results.append(f"  Az: [{az0:.2f}, {az1:.2f}, {az2:.2f}]°")
    results.append(f"  El: [{el0:.2f}, {el1:.2f}, {el2:.2f}]°")
    results.append(f"  dt: {(t2-t0):.0f}ms ({(t2-t0)/1000:.2f}s)")
    
    # === ① Vr符号一致性 ===
    sign0 = 1 if vr0 > 0 else (-1 if vr0 < 0 else 0)
    sign1 = 1 if vr1 > 0 else (-1 if vr1 < 0 else 0)
    sign2 = 1 if vr2 > 0 else (-1 if vr2 < 0 else 0)
    
    s_ok = True
    if sign0 != 0 and sign1 != 0 and sign2 != 0:
        if (sign0 != sign1) or (sign1 != sign2) or (sign0 != sign2):
            s_ok = False
            final_ok = 0
            failures.append(f"① Vr方向不一致 ({sign0},{sign1},{sign2})")
    if s_ok:
        results.append(f"① Vr符号一致 ✓ ({sign0},{sign1},{sign2})")
    
    # Vr统计
    vr_min_s = min(vr0, vr1, vr2)
    vr_max_s = max(vr0, vr1, vr2)
    vr_avg_abs = (abs(vr0) + abs(vr1) + abs(vr2)) / 3.0
    
    # Vr量级差异
    if vr_max_s > 0.0 and vr_min_s < 0.0:
        final_ok = 0
        failures.append(f"①b Vr正负混合")
    elif abs(vr_max_s) > 0.01 and abs(vr_min_s) > 0.01:
        vr_ratio = abs(vr_max_s) / (abs(vr_min_s) + 1e-6)
        if vr_ratio > 5.0:
            final_ok = 0
            failures.append(f"①c Vr差异过大 ratio={vr_ratio:.2f}>5")
    
    results.append(f"  Vr统计: min={vr_min_s:.2f}, max={vr_max_s:.2f}, avg_abs={vr_avg_abs:.2f}")
    
    # === ② 目标类型判断 ===
    drone_like = 1 if vr_avg_abs <= 80.0 else 0
    results.append(f"② 目标类型: {'无人机' if drone_like else '炮弹'} (vr_avg_abs={vr_avg_abs:.1f})")
    
    # === 低速标志 ===
    low_speed_pass = 0
    if 1.0 <= vr_avg_abs <= 30.0 and drone_like:
        all_vr_valid = all(1.0 <= abs(vr) <= 30.0 for vr in [vr0, vr1, vr2])
        if all_vr_valid:
            low_speed_pass = 1
            results.append(f"②b low_speed_pass=1 ✓")
    
    # === ③ 距离单调性 ===
    if not low_speed_pass:
        inc = (r1 > r0) and (r2 > r1)
        dec = (r1 < r0) and (r2 < r1)
        stable = abs(r1 - r0) < 5.0 and abs(r2 - r1) < 5.0
        if not inc and not dec and not stable:
            if drone_like and vr_avg_abs < 15.0:
                results.append(f"③ R非单调但低速无人机，放行")
            else:
                final_ok = 0
                failures.append(f"③ R非单调: {r0:.0f}->{r1:.0f}->{r2:.0f}")
    
    # === ④ 方位跨度 ===
    az_span = abs(az2 - az0)
    if az_span > 180.0:
        az_span = 360.0 - az_span
    results.append(f"④ 方位跨度: {az_span:.2f}° (阈值30°)")
    if az_span > 30.0:
        final_ok = 0
        failures.append(f"④ 方位跨度过大: {az_span:.2f}°>30°")
    
    # === ⑤ 俯仰跨度 ===
    el_span = abs(el2 - el0)
    results.append(f"⑤ 俯仰跨度: {el_span:.2f}° (阈值25°)")
    if el_span > 25.0:
        final_ok = 0
        failures.append(f"⑤ 俯仰跨度过大: {el_span:.2f}°>25°")
    
    # === ⑥ 绝对扇区约束 ===
    if not drone_like or vr_avg_abs < 600.0:
        az_norm = az2
        while az_norm > 180.0:
            az_norm -= 360.0
        while az_norm < -180.0:
            az_norm += 360.0
        
        if vr_avg_abs < 150.0:
            if az_norm > 60.0 or az_norm < -20.0:
                final_ok = 0
                failures.append(f"⑥a az_norm={az_norm:.2f}° 不在[-20°,60°]")
            if el2 < -15.0 or el2 > 75.0:
                final_ok = 0
                failures.append(f"⑥b el2={el2:.2f}° 不在[-15°,75°]")
            if r2 > 8000.0 or r0 > 8000.0:
                final_ok = 0
                failures.append(f"⑥c R={r0:.0f}/{r2:.0f}m > 8000m")
    
    # === ⑦ 波位合理性 ===
    beam_span = abs(beam2 - beam0)
    if beam_span > 5.0 and az_span > 15.0:
        final_ok = 0
        failures.append(f"⑦ 波位跨度过大: beam_span={beam_span}, az_span={az_span:.1f}°")
    
    # === ⑧ 交叉验证 ===
    if not low_speed_pass:
        dt_s = (t2 - t0) / 1000.0
        if dt_s > 0.01:
            delta_r = r2 - r0
            Vr_from_R = delta_r / dt_s
            R_ref = max(r0, r2)
            rel_delta = abs(delta_r) / (R_ref + 1.0)
            
            if vr2 < -2.0:
                if delta_r > 0 and rel_delta > 0.20:
                    final_ok = 0
                    failures.append(f"⑧a 靠近却远离: vr2={vr2:.1f}, delta_r={delta_r:.1f}, rel={rel_delta:.3f}")
            elif vr2 > 2.0:
                if delta_r < 0 and rel_delta > 0.20:
                    final_ok = 0
                    failures.append(f"⑧b 远离却靠近: vr2={vr2:.1f}, delta_r={delta_r:.1f}, rel={rel_delta:.3f}")
            
            if (vr2 > 5.0 and Vr_from_R < -10.0) or (vr2 < -5.0 and Vr_from_R > 10.0):
                final_ok = 0
                failures.append(f"⑧c Vr与R变化率反向: vr2={vr2:.1f}, Vr_from_R={Vr_from_R:.1f}")
    
    results.append(f"\n{'✅ PASS' if final_ok else '❌ FAIL'}")
    for f in failures:
        results.append(f"   ↳ {f}")
    
    return results, final_ok

# === Main ===
print("=" * 70)
print("解析 data_entry.c — 提取所有有效点")
print("=" * 70)

all_points = parse_data_entry_v2('data_entry.c')
print(f"\n总共解析了 {len(all_points)} 个有效点")

# Filter drone candidates: beam 2 or 3, |Vr| between 3-25 m/s
drone_candidates = [p for p in all_points 
                    if p['beamNo'] in [2, 3] 
                    and 3.0 <= abs(p['velocity']) <= 25.0]
print(f"无人机候选点 (beam 2/3, |Vr| 3-25m/s): {len(drone_candidates)} 个")

if drone_candidates:
    print("\n前10个候选点:")
    for p in drone_candidates[:10]:
        print(f"  Frame{p['frame_idx']:3d} | beam={p['beamNo']} | Vr={p['velocity']:+7.2f} | R={p['range']:8.1f} | Az={p['azi_deg']:6.2f}° | El={p['el_deg']:5.2f}° | t={p['mSecond']:.0f}ms")

# Now try to simulate 3-point initialization
# The key challenge: 3 points must be from DIFFERENT frames (track_initial.c logic)
# Let's group points by frame and find valid combinations
print("\n" + "=" * 70)
print("模拟初始化检查（搜索有效的3点组合）")
print("=" * 70)

# Group drone candidates by frame
frame_groups = {}
for p in drone_candidates:
    fi = p['frame_idx']
    if fi not in frame_groups:
        frame_groups[fi] = []
    frame_groups[fi].append(p)

# Sort frames
sorted_frames = sorted(frame_groups.keys())
print(f"无人机候选点分布在 {len(sorted_frames)} 个帧中")

# Show frame distribution
print("\n帧分布 (前30帧):")
for fi in sorted_frames[:30]:
    pts = frame_groups[fi]
    for p in pts:
        print(f"  Frame{fi:3d}: beam={p['beamNo']}, Vr={p['velocity']:+7.2f}, R={p['range']:8.1f}, Az={p['azi_deg']:6.2f}°, El={p['el_deg']:5.2f}°")

# Try 3-point combinations (using first point from each of 3 consecutive frames with drone candidates)
passed = 0
failed = 0
failure_reasons = {}

for i in range(min(len(sorted_frames) - 2, 50)):
    f0, f1, f2 = sorted_frames[i], sorted_frames[i+1], sorted_frames[i+2]
    
    # Take first drone point from each frame
    p0 = frame_groups[f0][0]
    p1 = frame_groups[f1][0]
    p2 = frame_groups[f2][0]
    
    dt = p2['mSecond'] - p0['mSecond']
    if dt < 0:
        continue
    
    results, ok = simulate_full_init_checks([p0, p1, p2])
    
    if ok:
        passed += 1
        print(f"\n{'✅' * 3} 找到有效3点组合!")
        for r in results:
            print(f"  {r}")
        break
    else:
        failed += 1
        # Collect failure reasons
        for r in results:
            if r.startswith('   ↳ '):
                reason = r.replace('   ↳ ', '')
                failure_reasons[reason] = failure_reasons.get(reason, 0) + 1
        
        if i < 3:  # Show first 3 failures in detail
            print(f"\n--- 组合 {i} (frames {f0},{f1},{f2}) ---")
            for r in results:
                print(f"  {r}")

print(f"\n统计: 通过={passed}, 失败={failed}")
if failure_reasons:
    print("\n失败原因统计:")
    for reason, count in sorted(failure_reasons.items(), key=lambda x: -x[1]):
        print(f"  [{count}次] {reason}")

# Also check: what about beam 1/4 points?
print("\n" + "=" * 70)
print("扩展搜索：所有beam的低速候选点")
print("=" * 70)
all_low_speed = [p for p in all_points 
                 if 1.0 <= abs(p['velocity']) <= 30.0 
                 and p['range'] > 500.0]
print(f"所有低速候选点 (|Vr| 1-30m/s, R>500m): {len(all_low_speed)} 个")
frames_with_low = sorted(set(p['frame_idx'] for p in all_low_speed))
print(f"分布在 {len(frames_with_low)} 个帧中")

# Show first few
for fi in frames_with_low[:10]:
    for p in [x for x in all_low_speed if x['frame_idx'] == fi]:
        print(f"  Frame{p['frame_idx']:3d} | beam={p['beamNo']} | Vr={p['velocity']:+7.2f} | R={p['range']:8.1f} | Az={p['azi_deg']:6.2f}° | El={p['el_deg']:5.2f}°")

print("\n分析完成！")
