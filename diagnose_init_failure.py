import re
import math

def parse_data_entry(filename):
    """Parse data_entry.c to extract point data per frame"""
    with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    frames = {}
    
    # Match the fill_frames function pattern
    # Each point has: x, y, z, azi, ele, range, velocity, mSecond, beamNo
    # Pattern: target_data[X] = { ... };
    
    # Find all target_data assignments
    pattern = r'target_data\[(\d+)\]\s*=\s*\{([^}]+)\}'
    matches = re.findall(pattern, content)
    
    for idx, data_str in matches:
        idx = int(idx)
        # Parse the data fields
        fields = [f.strip() for f in data_str.split(',') if f.strip()]
        
        point = {}
        for field in fields:
            if '=' in field:
                key, val = field.split('=', 1)
                key = key.strip()
                val = val.strip()
                try:
                    if key in ['azi', 'ele']:
                        point[key] = float(val)  # radians in data
                    elif key == 'range':
                        point[key] = float(val)
                    elif key == 'velocity':
                        point[key] = float(val)
                    elif key in ['x', 'y', 'z']:
                        point[key] = float(val)
                    elif key == 'mSecond':
                        point[key] = float(val)
                    elif key == 'beamNo':
                        point[key] = int(val)
                except:
                    pass
        
        if point:
            frames[idx] = point
    
    return frames

def extract_drone_candidates(frames_data):
    """Extract potential drone points based on beam position and velocity"""
    # Drone is detected on beams 2 and 3 (with some 1 and 4)
    candidates = []
    
    for idx, point in sorted(frames_data.items()):
        beam = point.get('beamNo', -1)
        vel = point.get('velocity', 0)
        r = point.get('range', 0)
        
        # Drone points typically have:
        # beam 2 or 3
        # velocity around -10 to -15 m/s (approaching radar)
        # range starting around 1200-2000m
        if beam in [2, 3] and abs(vel) > 3.0 and abs(vel) < 25.0:
            candidates.append({
                'frame': idx,
                'beam': beam,
                'vel': vel,
                'range': r,
                'azi_rad': point.get('azi', 0),
                'ele_rad': point.get('ele', 0),
                'x': point.get('x', 0),
                'y': point.get('y', 0),
                'z': point.get('z', 0),
                'mSecond': point.get('mSecond', 0),
            })
    
    return candidates

def simulate_init_checks(points):
    """Simulate the initialization checks in new_reliable.c"""
    if len(points) < 3:
        return None, "Not enough points"
    
    # Get 3 consecutive points (simplified - in reality they need to be from different frames)
    p0, p1, p2 = points[0], points[1], points[2]
    
    results = []
    
    # Extract radar data
    vr0, vr1, vr2 = p0['vel'], p1['vel'], p2['vel']
    r0, r1, r2 = p0['range'], p1['range'], p2['range']
    az0 = p0['azi_rad'] * 180.0 / math.pi
    az1 = p1['azi_rad'] * 180.0 / math.pi
    az2 = p2['azi_rad'] * 180.0 / math.pi
    el0 = p0['ele_rad'] * 180.0 / math.pi
    el1 = p1['ele_rad'] * 180.0 / math.pi
    el2 = p2['ele_rad'] * 180.0 / math.pi
    beam0, beam1, beam2 = p0['beam'], p1['beam'], p2['beam']
    t0, t1, t2 = p0['mSecond'], p1['mSecond'], p2['mSecond']
    
    results.append(f"3点数据: Frame{p0['frame']}(b{beam0})->Frame{p1['frame']}(b{beam1})->Frame{p2['frame']}(b{beam2})")
    results.append(f"  Vr: {vr0:.2f}, {vr1:.2f}, {vr2:.2f} (符号: {'-' if vr0<0 else '+'}, {'-' if vr1<0 else '+'}, {'-' if vr2<0 else '+'})")
    results.append(f"  R:  {r0:.1f}, {r1:.1f}, {r2:.1f}")
    results.append(f"  Az: {az0:.2f}°, {az1:.2f}°, {az2:.2f}°")
    results.append(f"  El: {el0:.2f}°, {el1:.2f}°, {el2:.2f}°")
    results.append(f"  dt: {t2-t0:.0f}ms ({(t2-t0)/1000:.2f}s)")
    
    final_ok = 1
    failure_reasons = []
    
    # === ① Vr符号一致性检查 ===
    sign0 = 1 if vr0 > 0 else (-1 if vr0 < 0 else 0)
    sign1 = 1 if vr1 > 0 else (-1 if vr1 < 0 else 0)
    sign2 = 1 if vr2 > 0 else (-1 if vr2 < 0 else 0)
    
    if sign0 != 0 and sign1 != 0 and sign2 != 0:
        if (sign0 != sign1) or (sign1 != sign2) or (sign0 != sign2):
            final_ok = 0
            failure_reasons.append(f"① Vr方向不一致: sign({sign0},{sign1},{sign2})")
        else:
            results.append(f"① Vr符号一致 ✓")
    else:
        results.append(f"① Vr符号检查跳过（有Vr≈0的点）")
    
    # === Vr统计量 ===
    vr_min = min(vr0, vr1, vr2)
    vr_max = max(vr0, vr1, vr2)
    vr_avg = (abs(vr0) + abs(vr1) + abs(vr2)) / 3.0
    
    results.append(f"  Vr统计: min={vr_min:.2f}, max={vr_max:.2f}, avg_abs={vr_avg:.2f}")
    
    # Vr量级差异检查
    if vr_max > 0.0 and vr_min < 0.0:
        final_ok = 0
        failure_reasons.append(f"①b Vr正负混合: max={vr_max:.2f}>0, min={vr_min:.2f}<0")
    elif abs(vr_max) > 0.01 and abs(vr_min) > 0.01:
        vr_ratio = abs(vr_max) / (abs(vr_min) + 1e-6)
        results.append(f"  Vr比值: {vr_ratio:.2f}")
        if vr_ratio > 5.0:
            final_ok = 0
            failure_reasons.append(f"①c Vr量级差异过大: ratio={vr_ratio:.2f}>5")
    
    # === ② 目标类型判断 ===
    drone_like = 0
    if vr_avg > 80.0:
        drone_like = 0
        results.append(f"② 目标类型: 炮弹 (vr_avg={vr_avg:.1f}>80)")
    else:
        drone_like = 1
        results.append(f"② 目标类型: 无人机 (vr_avg={vr_avg:.1f}≤80)")
    
    # === 低速目标检查 ===
    low_speed_pass = 0
    if 1.0 <= vr_avg <= 30.0 and drone_like:
        all_vr_valid = all(1.0 <= abs(vr) <= 30.0 for vr in [vr0, vr1, vr2])
        if all_vr_valid:
            low_speed_pass = 1
            results.append(f"②b 低速目标直接通过标志: low_speed_pass=1 ✓")
    
    # === ③ 距离单调性检查 ===
    if not low_speed_pass:
        inc = (r1 > r0) and (r2 > r1)
        dec = (r1 < r0) and (r2 < r1)
        stable = abs(r1 - r0) < 5.0 and abs(r2 - r1) < 5.0
        
        if not inc and not dec and not stable:
            if drone_like and vr_avg < 15.0:
                results.append(f"③ R非单调但低速无人机，放行")
            else:
                final_ok = 0
                failure_reasons.append(f"③ R非单调: R={r0:.1f}->{r1:.1f}->{r2:.1f}")
    
    # === ④ 方位跨度检查 ===
    az_span = abs(az2 - az0)
    if az_span > 180.0:
        az_span = 360.0 - az_span
    results.append(f"④ 方位跨度: {az_span:.2f}°")
    if az_span > 30.0:
        final_ok = 0
        failure_reasons.append(f"④ 方位跨度过大: {az_span:.2f}°>30°")
    
    # === ⑤ 俯仰跨度检查 ===
    el_span = abs(el2 - el0)
    results.append(f"⑤ 俯仰跨度: {el_span:.2f}°")
    if el_span > 25.0:
        final_ok = 0
        failure_reasons.append(f"⑤ 俯仰跨度过大: {el_span:.2f}°>25°")
    
    # === ⑥ 绝对扇区约束 ===
    if not drone_like or vr_avg < 600.0:
        az_norm = az2
        while az_norm > 180.0:
            az_norm -= 360.0
        while az_norm < -180.0:
            az_norm += 360.0
        
        results.append(f"⑥ 绝对扇区: az_norm={az_norm:.2f}°, el2={el2:.2f}°, r2={r2:.1f}m")
        
        if vr_avg < 150.0:
            if az_norm > 60.0 or az_norm < -20.0:
                final_ok = 0
                failure_reasons.append(f"⑥a 方位超出扇区: az_norm={az_norm:.2f}°不在[-20°,60°]")
            if el2 < -15.0 or el2 > 75.0:
                final_ok = 0
                failure_reasons.append(f"⑥b 俯仰超出范围: el2={el2:.2f}°不在[-15°,75°]")
            if r2 > 8000.0 or r0 > 8000.0:
                final_ok = 0
                failure_reasons.append(f"⑥c 距离超出范围: r0={r0:.1f}, r2={r2:.1f} > 8000m")
    
    # === ⑦ 波位合理性检查 ===
    beam_span = abs(beam2 - beam0)
    results.append(f"⑦ 波位跨度: {beam_span} (beam0={beam0}, beam2={beam2})")
    if beam_span > 5.0 and az_span > 15.0:
        final_ok = 0
        failure_reasons.append(f"⑦ 波位跨度过大且方位跨度大: beam_span={beam_span}, az_span={az_span:.1f}°")
    
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
                    failure_reasons.append(f"⑧a 靠近却远离: vr2={vr2:.1f}<0, delta_r={delta_r:.1f}>0, rel_delta={rel_delta:.3f}>0.2")
            elif vr2 > 2.0:
                if delta_r < 0 and rel_delta > 0.20:
                    final_ok = 0
                    failure_reasons.append(f"⑧b 远离却靠近: vr2={vr2:.1f}>0, delta_r={delta_r:.1f}<0, rel_delta={rel_delta:.3f}>0.2")
            
            if (vr2 > 5.0 and Vr_from_R < -10.0) or (vr2 < -5.0 and Vr_from_R > 10.0):
                final_ok = 0
                failure_reasons.append(f"⑧c Vr与R变化率方向相反: vr2={vr2:.1f}, Vr_from_R={Vr_from_R:.1f}")
    
    results.append(f"\n{'=== 最终结果 ==='}")
    if final_ok:
        results.append("✅ 通过所有检查！可以建立可靠航迹")
    else:
        results.append("❌ 初始化被拦截！")
        for reason in failure_reasons:
            results.append(f"   - {reason}")
    
    return results, final_ok

# Main execution
print("=" * 60)
print("分析 data_entry.c 中的无人机候选点")
print("=" * 60)

frames_data = parse_data_entry('data_entry.c')
print(f"\n总共解析了 {len(frames_data)} 个点")

candidates = extract_drone_candidates(frames_data)
print(f"无人机候选点（beam 2/3, |Vr| 3-25m/s）: {len(candidates)} 个")

if candidates:
    print("\n候选点列表:")
    for c in candidates[:20]:  # Show first 20
        print(f"  Frame{c['frame']:3d} | beam={c['beam']} | Vr={c['vel']:7.2f} | R={c['range']:8.1f} | Az={c['azi_rad']*180/math.pi:6.2f}° | El={c['ele_rad']*180/math.pi:5.2f}° | t={c['mSecond']:.0f}ms")
    
    # Try to find valid 3-point combinations
    print("\n" + "=" * 60)
    print("模拟3点初始化检查（遍历所有连续3点组合）")
    print("=" * 60)
    
    passed_count = 0
    failed_count = 0
    
    for i in range(min(len(candidates) - 2, 30)):  # Check first 30 combinations
        combo = candidates[i:i+3]
        
        # Check time gap
        dt = combo[2]['mSecond'] - combo[0]['mSecond']
        if dt < 0:
            continue
        
        results, passed = simulate_init_checks(combo)
        
        if passed:
            passed_count += 1
            print(f"\n{'✅' * 3} 组合 {i} PASS!")
            for r in results:
                print(f"  {r}")
            break  # Stop at first successful combo
        else:
            failed_count += 1
            if i < 5 or '失败' in str(results[-1]):
                print(f"\n--- 组合 {i} 失败分析 ---")
                for r in results:
                    print(f"  {r}")
    
    print(f"\n统计: 通过={passed_count}, 失败={failed_count}")
    
    # Check the specific issue: cross-beam angle span
    print("\n" + "=" * 60)
    print("关键诊断：跨波束角度跨度分析")
    print("=" * 60)
    
    for i in range(min(len(candidates) - 2, 10)):
        combo = candidates[i:i+3]
        azs = [c['azi_rad'] * 180/math.pi for c in combo]
        els = [c['ele_rad'] * 180/math.pi for c in combo]
        beams = [c['beam'] for c in combo]
        az_span = abs(azs[2] - azs[0])
        el_span = abs(els[2] - els[0])
        
        print(f"Combo{i}: beams={beams}, az_span={az_span:.1f}°, el_span={el_span:.1f}°, "
              f"az0={azs[0]:.1f}°, az2={azs[2]:.1f}°, el2={els[2]:.1f}°, r2={combo[2]['range']:.0f}m")
        
        if az_span > 30 or el_span > 25:
            print(f"  ❌ 跨度超限！az_span={az_span:.1f} > 30 or el_span={el_span:.1f} > 25")
else:
    print("没有找到无人机候选点！")
    print("请检查data_entry.c中的Vr符号是否正确...")

print("\n完成分析。")
