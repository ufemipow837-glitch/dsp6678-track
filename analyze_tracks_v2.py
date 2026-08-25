#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析无人机点迹在扫描圈上的分布，找连续轨迹"""
import csv, os, math
from collections import defaultdict, Counter

PLOT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_parsed.csv')
BEAM_TIME = 0.164
BEAMS_PER_SCAN = 17
SCAN_PERIOD = BEAM_TIME * BEAMS_PER_SCAN * 1000  # ~2788ms

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

def to_cart(p):
    az = math.radians(p['az']); el = math.radians(p['el'])
    return (p['r']*math.cos(el)*math.sin(az),
            p['r']*math.cos(el)*math.cos(az),
            p['r']*math.sin(el))

def euc(p0, p1):
    c0, c1 = to_cart(p0), to_cart(p1)
    return math.sqrt((c0[0]-c1[0])**2+(c0[1]-c1[1])**2+(c0[2]-c1[2])**2)

def main():
    plots = load_plots()
    plots_sorted = sorted(plots, key=lambda p: p['t'])

    print("=" * 70)
    print(f" 雷达扫描模式: {BEAMS_PER_SCAN}波位, 每波位{BEAM_TIME*1000:.0f}ms, 扫描周期{SCAN_PERIOD:.0f}ms")
    print("=" * 70)

    # 找无人机候选区域：正俯仰, 正高度, SNR>=10, 距离1-8km, |径向速度|<60
    candidates = [p for p in plots_sorted if p['el'] >= -0.5 and p['alt'] >= 0
                  and p['snr'] >= 10 and 1000 <= p['r'] <= 8000 and abs(p['v']) <= 60]
    print(f"\n无人机候选点(俯仰>=-0.5°,高度>=0,SNR>=10,距1-8km,|v|<=60): {len(candidates)}")

    # 按距离-方位初步聚类
    print("\n[1] 按距离-方位网格分析候选点分布")
    grid = defaultdict(list)
    for p in candidates:
        ra = int(p['r']/500)
        aa = int(p['az']/5)
        grid[(ra,aa)].append(p)
    large_grids = [(g, len(ps)) for g, ps in grid.items() if len(ps) >= 10]
    large_grids.sort(key=lambda x: -x[1])
    print(f"  候选网格数(每网格500m×5°): {len(grid)}, 含>=10点的网格: {len(large_grids)}")
    for g, cnt in large_grids[:15]:
        ra, aa = g
        ps = grid[g]
        rs = [p['r'] for p in ps]; azs = [p['az'] for p in ps]
        print(f"    距{ra*500}-{(ra+1)*500}m, 方位{aa*5}-{(aa+1)*5}°: {cnt}点, "
              f"r范围{min(rs):.0f}-{max(rs):.0f}m, az范围{min(azs):.1f}-{max(azs):.1f}°")

    # 在每个候选网格附近做跨圈关联
    print("\n[2] 跨圈轨迹关联(同一圈的点先凝聚, 相邻圈距离<600m视为同一目标)")
    # 按扫描圈分组
    t_start = plots_sorted[0]['t']
    scan_groups = defaultdict(list)
    for p in candidates:
        scan_idx = int((p['t'] - t_start) / SCAN_PERIOD)
        scan_groups[scan_idx].append(p)

    # 每圈内先做简单凝聚(同一圈距离<200m的点合并,取SNR最高的)
    def coalesce(points, d_th=200.0):
        if not points: return []
        ps = sorted(points, key=lambda p: -p['snr'])
        kept = []
        for p in ps:
            if not any(euc(p, k) < d_th for k in kept):
                kept.append(p)
        return kept

    scans_coalesced = {}
    for si, ps in scan_groups.items():
        cc = coalesce(ps, 200.0)
        if cc:
            scans_coalesced[si] = cc

    print(f"  有效扫描圈数: {len(scans_coalesced)}")

    # 跨圈多目标关联
    tracks = []
    for si in sorted(scans_coalesced.keys()):
        for p in scans_coalesced[si]:
            best_tid = -1
            best_d = 600.0  # 600m门限
            for tid, trk in enumerate(tracks):
                if trk['last_scan'] < si - 2: continue  # 最多允许隔1个扫描圈
                lp = trk['points'][-1]
                d = euc(p, lp)
                if d < best_d:
                    best_d = d
                    best_tid = tid
            if best_tid >= 0:
                tracks[best_tid]['points'].append(p)
                tracks[best_tid]['last_scan'] = si
            else:
                tracks.append({'points':[p], 'last_scan':si})

    long_trks = [t for t in tracks if len(t['points']) >= 8]
    long_trks.sort(key=lambda x: -len(x['points']))
    print(f"  跨圈轨迹总数: {len(tracks)}, 长度>=8圈: {len(long_trks)}")

    print(f"\n  {'ID':>3} {'圈数':>4} {'点数':>4} {'起始T':>10} {'终止T':>10} {'时长(s)':>7} "
          f"{'距min':>7} {'距max':>7} {'高min':>6} {'高max':>6} "
          f"{'方位min':>7} {'方位max':>7} {'vmin':>6} {'vmax':>6} {'meanSNR':>7}")
    print("  " + "-" * 120)
    for i, trk in enumerate(long_trks[:15]):
        pts = trk['points']
        rs = [p['r'] for p in pts]; als = [p['alt'] for p in pts]
        azs = [p['az'] for p in pts]; vs = [p['v'] for p in pts]
        snrs = [p['snr'] for p in pts]
        t0 = pts[0]['t']; t1 = pts[-1]['t']
        print(f"  {i:>3} {len(set(p['t'] for p in pts)):>4} {len(pts):>4} {t0:>10} {t1:>10} "
              f"{(t1-t0)/1000:>7.0f} {min(rs):>7.0f} {max(rs):>7.0f} {min(als):>6.0f} "
              f"{max(als):>6.0f} {min(azs):>7.1f} {max(azs):>7.1f} {min(vs):>6.1f} "
              f"{max(vs):>6.1f} {sum(snrs)/len(snrs):>7.0f}")

    # 导出最可能的无人机轨迹
    out_csv = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_tracks_verified.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['轨迹ID','圈号','时间(ms)','波位','方位(度)','距离(m)','俯仰(度)',
                   '高度(m)','径向速度(m/s)','SNR','RCS','X(m)','Y(m)','Z(m)'])
        for tid, trk in enumerate(long_trks[:5]):
            for p in trk['points']:
                x,y,z = to_cart(p)
                scan_idx = int((p['t'] - t_start) / SCAN_PERIOD)
                w.writerow([tid, scan_idx, p['t'], p['beam'], f"{p['az']:.4f}", f"{p['r']:.2f}",
                           f"{p['el']:.4f}", f"{p['alt']:.2f}", f"{p['v']:.4f}", p['snr'],
                           f"{p['rcs']:.6e}", f"{x:.1f}", f"{y:.1f}", f"{z:.1f}"])
    print(f"\n  无人机轨迹已导出到: {out_csv}")

    # 检查点迹时间间隔分布(对最长轨迹)
    if long_trks:
        longest = long_trks[0]['points']
        longest.sort(key=lambda p: p['t'])
        print(f"\n[3] 最长轨迹(0号)的点迹时间间隔:")
        dts = [longest[i+1]['t']-longest[i]['t'] for i in range(len(longest)-1)]
        dc = Counter(dts)
        for dt_, c in sorted(dc.items()):
            print(f"    {dt_}ms: {c}次")
        print(f"    最大间隔: {max(dts)}ms, 最小间隔: {min(dts)}ms, 平均: {sum(dts)/len(dts):.0f}ms")

        # 计算三维速度
        print(f"\n[4] 最长轨迹的三维运动参数:")
        for i in range(1, len(longest)):
            p0 = longest[i-1]; p1 = longest[i]
            dt = (p1['t']-p0['t'])/1000.0
            d = euc(p0, p1)
            v3d = d/dt
            dr = p1['r'] - p0['r']
            print(f"    t={p1['t']}ms: d={d:.0f}m, dt={dt*1000:.0f}ms, "
                  f"v3d={v3d:.1f}m/s, 径向距离差={dr:.0f}m, beam={p1['beam']}")

if __name__ == '__main__':
    main()
