#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机数据时空聚类分析
在球坐标系下直接寻找连续移动的目标轨迹
"""
import csv, math, os
from collections import defaultdict

PLOT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_parsed.csv')
BEAM_TIME = 0.164  # s

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
                    'rcs': float(row['RCS']),
                })
    return plots

def sph_dist_deg(p0, p1):
    """球坐标系下两点距离(近似: 用笛卡尔距离)"""
    az0 = math.radians(p0['az']); el0 = math.radians(p0['el'])
    az1 = math.radians(p1['az']); el1 = math.radians(p1['el'])
    x0 = p0['r']*math.cos(el0)*math.sin(az0)
    y0 = p0['r']*math.cos(el0)*math.cos(az0)
    z0 = p0['r']*math.sin(el0)
    x1 = p1['r']*math.cos(el1)*math.sin(az1)
    y1 = p1['r']*math.cos(el1)*math.cos(az1)
    z1 = p1['r']*math.sin(el1)
    return math.sqrt((x1-x0)**2+(y1-y0)**2+(z1-z0)**2)

def cluster_tracks(plots, vel_max=80.0, max_gap=3):
    """简单贪心聚类: 按时间排序,每点关联到最近的已有轨迹"""
    plots_sorted = sorted(plots, key=lambda p: p['t'])
    tracks = []

    for p in plots_sorted:
        best_tid = -1
        best_d = 1e18
        for tid, trk in enumerate(tracks):
            last = trk[-1]
            dt = (p['t'] - last['t']) / 1000.0
            if dt <= 0 or dt > max_gap * BEAM_TIME * 1.5:
                continue
            d = sph_dist_deg(last, p)
            v_est = d / dt
            if v_est < vel_max and d < best_d:
                best_d = d
                best_tid = tid
        if best_tid >= 0:
            tracks[best_tid].append(p)
        else:
            tracks.append([p])
    return tracks

def main():
    print("加载数据...")
    plots = load_plots()
    print(f"总点数: {len(plots)}")

    # 先筛选: 排除明显的地杂波(俯仰<-1度 或 高度<0), 排除高速目标
    filtered = [p for p in plots if p['el'] >= -1.0 and p['alt'] >= -10
                and abs(p['v']) <= 80 and p['r'] <= 20000 and p['snr'] >= 8]
    print(f"预筛选后(俯仰>=-1°, 高度>=-10m, |v|<=80, r<=20km, SNR>=8): {len(filtered)}")

    tracks = cluster_tracks(filtered, vel_max=60.0, max_gap=3)
    print(f"\n聚类得到轨迹数: {len(tracks)}")

    long_tracks = [(i, t) for i, t in enumerate(tracks) if len(t) >= 20]
    long_tracks.sort(key=lambda x: -len(x[1]))
    print(f"长度>=20点的轨迹: {len(long_tracks)}")

    print(f"\n{'ID':>4} {'点数':>5} {'起始T':>10} {'终止T':>10} {'时长(s)':>7} "
          f"{'距min':>7} {'距max':>7} {'高min':>7} {'高max':>7} "
          f"{'方位范围':>14} {'俯仰范围':>12} {'v范围':>12} {'SNR':>4}")
    print("-" * 120)
    for tid, trk in long_tracks[:20]:
        t0 = trk[0]['t']; t1 = trk[-1]['t']
        rs = [p['r'] for p in trk]; als = [p['alt'] for p in trk]
        azs = [p['az'] for p in trk]; els = [p['el'] for p in trk]
        vs = [p['v'] for p in trk]; snrs = [p['snr'] for p in trk]
        dur = (t1-t0)/1000.0
        print(f"{tid:>4} {len(trk):>5} {t0:>10} {t1:>10} {dur:>7.1f} "
              f"{min(rs):>7.0f} {max(rs):>7.0f} {min(als):>7.0f} {max(als):>7.0f} "
              f"{min(azs):>6.1f}~{max(azs):>6.1f} {min(els):>5.1f}~{max(els):>5.1f} "
              f"{min(vs):>5.1f}~{max(vs):>5.1f} {sum(snrs)/len(snrs):>4.0f}")

    # 导出最长的几条轨迹到CSV,方便验证
    out_csv = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_tracks_found.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['轨迹ID','点序号','时间(ms)','波位','方位(度)','距离(m)','俯仰(度)','高度(m)','径向速度(m/s)','SNR','RCS'])
        for tid, trk in long_tracks[:10]:
            for idx, p in enumerate(trk):
                w.writerow([tid, idx, p['t'], p['beam'], f"{p['az']:.4f}", f"{p['r']:.2f}",
                           f"{p['el']:.4f}", f"{p['alt']:.2f}", f"{p['v']:.4f}", p['snr'], f"{p['rcs']:.6e}"])
    print(f"\n轨迹点已导出到: {out_csv}")

    # 分析最长轨迹的时间连续性
    if long_tracks:
        tid, longest = long_tracks[0]
        print(f"\n最长轨迹(ID={tid}, {len(longest)}点) 的帧连续性分析:")
        gaps = []
        for i in range(1, len(longest)):
            dt = longest[i]['t'] - longest[i-1]['t']
            gaps.append(dt)
        from collections import Counter
        gc = Counter(gaps)
        for g, c in sorted(gc.items()):
            print(f"  帧间隔{g}ms: {c}次")
        miss = sum(1 for g in gaps if g > 200)
        print(f"  漏帧数(>200ms间隔): {miss}")

if __name__ == '__main__':
    main()
