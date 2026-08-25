#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析雷达扫描模式和点迹在波位上的分布"""
import csv, os, math
from collections import defaultdict, Counter

PLOT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_parsed.csv')

def load_plots():
    plots = []
    with open(PLOT_CSV, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if int(row['是否有效点']) == 1:
                plots.append({
                    't': int(row['时间戳(ms)']),
                    'beam': int(row['波位号']),
                    'az': float(row['方位(度)']),
                    'r': float(row['距离(m)']),
                    'el': float(row['俯仰(度)']),
                    'alt': float(row['高度(m)']),
                    'v': float(row['径向速度(m/s)']),
                    'snr': int(row['信噪比']),
                })
    return plots

def main():
    plots = load_plots()
    plots_sorted = sorted(plots, key=lambda p: p['t'])

    print("=" * 70)
    print(" 雷达扫描模式分析")
    print("=" * 70)

    # 1. 波位号vs方位角
    print("\n[1] 波位号与方位角对应关系(前200个点)")
    beam_az = defaultdict(list)
    for p in plots_sorted[:500]:
        beam_az[p['beam']].append(p['az'])
    for b in sorted(beam_az.keys()):
        azs = beam_az[b]
        print(f"  波位{b:>2}: 方位{min(azs):>6.1f}°~{max(azs):>6.1f}° (平均{sum(azs)/len(azs):>6.1f}°), {len(azs)}个点")

    # 2. 扫描周期分析: 波位0连续出现的时间间隔
    print("\n[2] 扫描周期估计(波位0相邻出现的时间间隔)")
    beam0_times = [p['t'] for p in plots_sorted if p['beam'] == 0]
    if len(beam0_times) > 2:
        intervals = [beam0_times[i+1]-beam0_times[i] for i in range(min(50,len(beam0_times)-1))]
        ic = Counter(intervals)
        for iv, c in sorted(ic.items()):
            print(f"  {iv}ms: {c}次")
        scan_period = max(intervals, key=lambda x: ic[x])
        print(f"  估计扫描周期: ~{scan_period}ms ({scan_period/1000:.2f}s)")
        print(f"  转速: ~{60000/scan_period:.1f} 转/分钟")

    # 3. 看连续点的波位变化
    print("\n[3] 波位序列(前30个点)")
    for p in plots_sorted[:30]:
        print(f"  t={p['t']}ms, beam={p['beam']:>2}, az={p['az']:>6.2f}°, r={p['r']:>7.1f}m, el={p['el']:>6.2f}°, alt={p['alt']:>7.1f}m, v={p['v']:>7.1f}m/s")

    # 4. 检查同一方位角上是否有连续点迹(即无人机可能被多个相邻波位观测到)
    print("\n[4] 按距离-方位网格找稳定目标(寻找无人机候选区域)")
    # 按时间窗口(每圈扫描)分组
    if len(beam0_times) > 2:
        scan_period = 2788  # 17*164
        scans = []
        for i, t0 in enumerate(beam0_times):
            t1 = t0 + scan_period
            scan_plots = [p for p in plots_sorted if t0 <= p['t'] < t1]
            scans.append((t0, scan_plots))
        print(f"  总扫描圈数: {len(scans)}")

        # 找距离2000-5000m,俯仰>0,正高度,|v|<40的稳定出现点
        print(f"\n  每圈扫描中满足条件(2-6km,俯仰>-1°,高度>0m,|v|<50,SNR>=10)的点数:")
        stable_points = []
        for si, (t0, sp) in enumerate(scans):
            candidates = [p for p in sp if 2000 <= p['r'] <= 6000 and p['el'] > -1.0
                         and p['alt'] > 0 and abs(p['v']) < 50 and p['snr'] >= 10]
            if candidates:
                # 按距离聚类找最稳定的
                for p in candidates:
                    stable_points.append((si, p))
            if si < 30:
                print(f"    第{si:>3}圈(t={t0}ms): {len(candidates)}个候选点")

        # 检查stable_points中是否有连贯轨迹(相邻圈的距离变化<500m)
        print(f"\n[5] 跨圈连续性分析(相邻圈距离变化<300m视为同一目标)")
        scans_candidates = defaultdict(list)
        for si, p in stable_points:
            scans_candidates[si].append(p)

        # 简单多目标关联
        tracks = []
        for si in sorted(scans_candidates.keys()):
            cands = scans_candidates[si]
            for p in cands:
                best_tid = -1
                best_d = 1e18
                for tid, trk in enumerate(tracks):
                    if trk['last_scan'] == si - 1:
                        lp = trk['points'][-1]
                        # 近似距离变化
                        d = abs(p['r'] - lp['r'])
                        # 方位也应接近
                        az_diff = abs(p['az'] - lp['az'])
                        az_diff = min(az_diff, 360-az_diff)
                        if d < 800 and az_diff < 15 and d < best_d:
                            best_d = d
                            best_tid = tid
                if best_tid >= 0:
                    tracks[best_tid]['points'].append(p)
                    tracks[best_tid]['last_scan'] = si
                else:
                    tracks.append({'points': [p], 'last_scan': si})

        long_trks = [t for t in tracks if len(t['points']) >= 5]
        long_trks.sort(key=lambda x: -len(x['points']))
        print(f"  跨圈连续轨迹(>=5圈): {len(long_trks)}条")
        for i, trk in enumerate(long_trks[:10]):
            pts = trk['points']
            rs = [p['r'] for p in pts]
            azs = [p['az'] for p in pts]
            als = [p['alt'] for p in pts]
            vs = [p['v'] for p in pts]
            t0 = pts[0]['t']; t1 = pts[-1]['t']
            print(f"  轨迹{i}: {len(pts)}圈, ({(t1-t0)/1000:.0f}s), "
                  f"距{min(rs):.0f}~{max(rs):.0f}m, 方位{min(azs):.1f}~{max(azs):.1f}°, "
                  f"高{min(als):.0f}~{max(als):.0f}m, 速度{min(vs):.1f}~{max(vs):.1f}m/s")

if __name__ == '__main__':
    main()
