# -*- coding: utf-8 -*-
"""
分析关键点：Frame 38前后（初始化）、Frame 199（原航迹删除）、Frame 233（新航迹建立）
对应的data_entry.c点迹数据，找根因
"""
import re
import sys

def parse_data_entry():
    with open(r'd:\DSP\6678\track\track_1\data_entry.c', 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    # 找所有fill_frames_*函数中设置target_data的行
    # 匹配模式：target_data[N].xxx = val;
    frames = {}  # frame_idx -> list of {idx, x,y,z,velocity,beamNo,mSecond,range}
    
    # 先找每个frame循环中的赋值
    # 模式如：target_data[0].Month = 921312; （frame号在函数名里，或看循环下标）
    
    # 更简单的方法：找所有设置mSecond的点，反推frame
    dot_pattern = re.compile(
        r'target_data\[(\d+)\]\.mSecond\s*=\s*(\d+);.*?'
        r'target_data\[\1\]\.beamNo\s*=\s*(\d+);.*?'
        r'target_data\[\1\]\.range\s*=\s*([\-\d\.]+)f;.*?'
        r'target_data\[\1\]\.velocity\s*=\s*([\-\d\.]+)f;.*?'
        r'target_data\[\1\]\.az\s*=\s*([\-\d\.]+)f;.*?'
        r'target_data\[\1\]\.el\s*=\s*([\-\d\.]+)f;.*?'
        r'target_data\[\1\]\.x\s*=\s*([\-\d\.]+)f;.*?'
        r'target_data\[\1\]\.y\s*=\s*([\-\d\.]+)f;.*?'
        r'target_data\[\1\]\.z\s*=\s*([\-\d\.]+)f;',
        re.DOTALL
    )
    
    # 按mSecond分组（同帧=同时间戳）
    dots_by_time = {}
    for m in dot_pattern.finditer(content):
        idx = int(m.group(1))
        ms = int(m.group(2))
        beam = int(m.group(3))
        rng = float(m.group(4))
        vel = float(m.group(5))
        az = float(m.group(6))
        el = float(m.group(7))
        x = float(m.group(8))
        y = float(m.group(9))
        z = float(m.group(10))
        
        if ms not in dots_by_time:
            dots_by_time[ms] = []
        dots_by_time[ms].append({
            'idx': idx, 'mSecond': ms, 'beamNo': beam,
            'range': rng, 'velocity': vel,
            'az': az, 'el': el,
            'x': x, 'y': y, 'z': z
        })
    
    # 按时间排序，得到frame号
    timestamps = sorted(dots_by_time.keys())
    
    print("="*80)
    print(f"总帧数: {len(timestamps)}, 时间戳范围: {timestamps[0]}ms ~ {timestamps[-1]}ms")
    print("="*80)
    
    # === 找Frame 38 (t=904372ms) 前后的点 ===
    print("\n" + "="*80)
    print("【第1部分】Frame 36-40（初始化附近）的点迹，检查3点建航迹的速度方向")
    print("="*80)
    for fidx in range(36, 41):
        if fidx < len(timestamps):
            t = timestamps[fidx]
            dots = dots_by_time[t]
            print(f"\n  Frame {fidx:03d} | t={t}ms | n={len(dots)}")
            for d in dots:
                print(f"    dot[{d['idx']}] beam={d['beamNo']} Az={d['az']:.2f}° El={d['el']:.2f}° R={d['range']:.1f}m Vr={d['velocity']:.2f}m/s  xyz=({d['x']:.1f},{d['y']:.1f},{d['z']:.1f})")
    
    # 找建航迹的3个点（Frame 38附近的连续3个无人机点）
    # 从Frame 36开始，找beam=2/3、Vr≈-11m/s的点
    print("\n" + "-"*60)
    print("【分析】从Frame 0-60中找可能用于初始化的连续3个无人机点（beam2/3, |Vr|<30）:")
    drone_candidates = []
    for fidx in range(0, 60):
        if fidx < len(timestamps):
            t = timestamps[fidx]
            dots = dots_by_time[t]
            for d in dots:
                if d['beamNo'] in [2,3] and abs(d['velocity']) < 30 and d['range'] < 2000:
                    drone_candidates.append((fidx, t, d))
                    print(f"  候选 F{fidx:03d} beam={d['beamNo']} R={d['range']:.1f}m Vr={d['velocity']:.2f}m/s Az={d['az']:.2f}°")
    
    if len(drone_candidates) >= 3:
        print("\n  ~~~ 假设用前3个候选点初始化，计算速度方向 ~~~")
        for i in range(min(3, len(drone_candidates)-2)):
            f0, t0, d0 = drone_candidates[i]
            f1, t1, d1 = drone_candidates[i+1]
            f2, t2, d2 = drone_candidates[i+2]
            T1 = (t1 - t0) / 1000.0
            T2 = (t2 - t1) / 1000.0
            if T1 > 0 and T2 > 0:
                v1x = (d1['x'] - d0['x']) / T1
                v1y = (d1['y'] - d0['y']) / T1
                v1z = (d1['z'] - d0['z']) / T1
                v2x = (d2['x'] - d1['x']) / T2
                v2y = (d2['y'] - d1['y']) / T2
                v2z = (d2['z'] - d1['z']) / T2
                v1 = (v1x**2+v1y**2+v1z**2)**0.5
                v2 = (v2x**2+v2y**2+v2z**2)**0.5
                # 径向速度（位置点投影）
                r0 = (d0['x']**2+d0['y']**2+d0['z']**2)**0.5
                vr1_calc = (d1['range'] - d0['range']) / T1
                vr2_calc = (d2['range'] - d1['range']) / T2
                print(f"  组合 F{f0:03d}-F{f1:03d}-F{f2:03d}:")
                print(f"    合速度 v1={v1:.2f}m/s v2={v2:.2f}m/s")
                print(f"    距离变化 R0-R1-R2: {d0['range']:.1f}→{d1['range']:.1f}→{d2['range']:.1f}")
                print(f"    位置差估算径向速度: ΔR/ΔT1={vr1_calc:.2f} ΔR/ΔT2={vr2_calc:.2f}")
                print(f"    雷达实测径向速度: Vr0={d0['velocity']:.2f} Vr1={d1['velocity']:.2f} Vr2={d2['velocity']:.2f}")
                print(f"    → 距离递增={d2['range']>d0['range']} → 远离雷达  |  距离递减={d2['range']<d0['range']} → 靠近雷达")
    
    # === 找Frame 199前后（航迹删除）的点 ===
    print("\n" + "="*80)
    print("【第2部分】Frame 197-201（原无人机航迹被删除前后）的点迹")
    print("="*80)
    for fidx in range(197, 202):
        if fidx < len(timestamps):
            t = timestamps[fidx]
            dots = dots_by_time[t]
            print(f"\n  Frame {fidx:03d} | t={t}ms | n={len(dots)}")
            for d in dots:
                print(f"    dot[{d['idx']}] beam={d['beamNo']} Az={d['az']:.2f}° El={d['el']:.2f}° R={d['range']:.1f}m Vr={d['velocity']:.2f}m/s")
    
    # === 找Frame 233前后（新航迹建立V=153m/s）的点 ===
    print("\n" + "="*80)
    print("【第3部分】Frame 228-235（新假航迹建立）的点迹，找3个高速杂波点")
    print("="*80)
    for fidx in range(228, 236):
        if fidx < len(timestamps):
            t = timestamps[fidx]
            dots = dots_by_time[t]
            print(f"\n  Frame {fidx:03d} | t={t}ms | n={len(dots)}")
            for d in dots:
                print(f"    dot[{d['idx']}] beam={d['beamNo']} Az={d['az']:.2f}° El={d['el']:.2f}° R={d['range']:.1f}m Vr={d['velocity']:.2f}m/s")
    
    # 在Frame 228-235附近找连续3个高速点（用于初始化假航迹）
    print("\n" + "-"*60)
    print("【分析】Frame 220-240中找Vr>100m/s的连续3个点（假航迹初始化源）:")
    high_speed = []
    for fidx in range(220, 241):
        if fidx < len(timestamps):
            t = timestamps[fidx]
            dots = dots_by_time[t]
            for d in dots:
                if abs(d['velocity']) > 100:
                    high_speed.append((fidx, t, d))
                    print(f"  高速点 F{fidx:03d} beam={d['beamNo']} R={d['range']:.1f}m Vr={d['velocity']:.2f}m/s Az={d['az']:.2f}°")
                    # 计算合速度（如果有前一个高速点的话）
                    if len(high_speed) >= 2:
                        pf, pt, pd = high_speed[-2]
                        T = (t - pt) / 1000.0
                        if T > 0:
                            vx = (d['x']-pd['x'])/T
                            vy = (d['y']-pd['y'])/T
                            vz = (d['z']-pd['z'])/T
                            v = (vx*vx+vy*vy+vz*vz)**0.5
                            print(f"    → 与F{pf:03d}合速度估算: {v:.2f}m/s (ΔT={T:.3f}s)")
    
    # === 找Frame 55/56/72（应该有无人机点但是没关联上） ===
    print("\n" + "="*80)
    print("【第4部分】Frame 54-58 和 Frame 71-74（应该关联但实际没关联的帧）")
    print("="*80)
    for rng_name, (start, end) in [("F54-58", (54,59)), ("F71-74", (71,75))]:
        print(f"\n  -- {rng_name} --")
        for fidx in range(start, end):
            if fidx < len(timestamps):
                t = timestamps[fidx]
                dots = dots_by_time[t]
                print(f"    Frame {fidx:03d} | t={t}ms | n={len(dots)}")
                for d in dots:
                    flag_drone = d['beamNo'] in [2,3] and abs(d['velocity'])<30 and 16<d['az']<19
                    print(f"      dot beam={d['beamNo']} Az={d['az']:.2f}° El={d['el']:.2f}° R={d['range']:.1f}m Vr={d['velocity']:.2f}m/s  {'←疑似无人机!' if flag_drone else ''}")
    
    # === 分析速度门bug：原航迹预测Vr方向 vs 点迹实测Vr方向 ===
    print("\n" + "="*80)
    print("【第5部分】速度门致命BUG验证：原无人机航迹（F38）预测Vr方向 vs 无人机点实测Vr")
    print("="*80)
    # F38初始化位置：Az≈17.39°, El≈4.77°, R≈1153m, V=10.9m/s
    # 纯预测时速度方向=初始化时估算的方向（3点位置差/时间）
    # 如果初始化时距离递增（远离），则速度方向向外，vr_pred>0
    # 但无人机实际在靠近（Vr≈-11.47m/s<0），两者方向相反！
    if len(drone_candidates) >= 3:
        f0, t0, d0 = drone_candidates[0]
        f2, t2, d2 = drone_candidates[2]
        init_vel_away = d2['range'] > d0['range']  # 初始化估算速度方向：远离?
        real_vr_away = d0['velocity'] > 0           # 实测径向速度：远离?
        print(f"  初始化3点: R0={d0['range']:.1f}m → R2={d2['range']:.1f}m → 位置差估算{'远离' if init_vel_away else '靠近'} (vr_pred>0={init_vel_away})")
        print(f"  实测Vr0={d0['velocity']:.2f}m/s → 雷达实测{'远离' if real_vr_away else '靠近'} (Vr>0={real_vr_away})")
        print(f"  初始化估算方向 vs 实测方向: {'一致 ✓' if init_vel_away == real_vr_away else '❌ 相反！速度门会把正确点全拒！'}")
        if init_vel_away != real_vr_away:
            print("  【致命BUG】初始化时用位置差(R2-R1)/T算合速度，方向为径向投影，但如果3点采样时无人机有横向移动，")
            print("           合速度投影到径向可能与雷达实测径向速度Vr符号相反！")
            print("           速度门第6条: vr_pred * vr_dot < 0 且都>5 → 直接continue拒绝！")
            print("           结果：从F38开始所有正确无人机点都被拒，predict_flag疯涨，最终航迹被删！")
    
    print("\n" + "="*80)
    print("【第6部分】Frame 233 假航迹初始化时target_type值推算")
    print("="*80)
    if len(high_speed) >= 3:
        f0,t0,d0 = high_speed[0]
        f1,t1,d1 = high_speed[1]
        f2,t2,d2 = high_speed[2]
        T1=(t1-t0)/1000.0; T2=(t2-t1)/1000.0
        if T1>0 and T2>0:
            v2x=(d2['x']-d1['x'])/T2
            v2y=(d2['y']-d1['y'])/T2
            v2z=(d2['z']-d1['z'])/T2
            spd=(v2x*v2x+v2y*v2y+v2z*v2z)**0.5
            print(f"  假航迹初始化速度(合速度): spd={spd:.2f}m/s")
            if spd > 200:
                print(f"    spd>200 → init_hint=1 → target_type=1 (炮弹)")
            elif spd < 60:
                print(f"    spd<60 → init_hint=2 → target_type=2 (无人机)")
            else:
                print(f"    60≤spd≤200 → init_hint=0 → target_type=0 (未知) ← 走宽松约束！")
            print(f"  target_type=0时，track_asso.c L509 prev_vel={spd:.1f}{'<' if spd<60 else '>'}60 → {'无人机约束' if spd<60 else '走炮弹Vmax约束（完全放开）'}")
    
    print("\n【总结根因链】")
    print("  1. 初始化时3点位置差估算的速度径向投影 vs 雷达实测Vr → 符号相反")
    print("  2. 速度门第6条 (vr_pred*vr_dot<0 且都>5) → 正确无人机点全部被拒")
    print("  3. 原无人机航迹 (target_type=2) 纯预测160帧 → 被track_die删除")
    print("  4. Frame233附近3个高速杂波初始化新航迹 → spd≈150∈[60,200] → target_type=0 (未知)")
    print("  5. target_type=0且prev_vel>60 → 走炮弹Vmax约束（速度几百m/s都放行）→ 假航迹持续跟踪高速杂波")
    print("  → 用户看到：速度从10.9m/s突然跳到153.2m/s，方位从18°跳到55°")
