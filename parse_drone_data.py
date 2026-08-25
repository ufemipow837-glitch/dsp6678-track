#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机实测数据解析提取工具
从 无人机实测数据.txt 中提取点迹数据，按波位(CPI)分组，输出统计信息和结构化CSV
"""

import re
import csv
import os
from collections import defaultdict

INPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '无人机实测数据.txt')
OUTPUT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_parsed.csv')
OUTPUT_CPI_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_cpi.csv')

def parse_line(line):
    """解析一行点迹数据，返回字典或None"""
    line = line.strip()
    if not line:
        return None

    fields = {}

    m_beam = re.search(r'波位号:(-?\d+)', line)
    m_bdtime = re.search(r'北斗时间戳:(\d+)', line)
    m_time = re.search(r'时间戳:(\d+)', line)
    m_targets = re.search(r'目标数:(\d+)', line)

    if not m_time:
        return None

    fields['beam'] = int(m_beam.group(1)) if m_beam else -1
    fields['bd_time'] = int(m_bdtime.group(1)) if m_bdtime else 0
    fields['mSecond'] = int(m_time.group(1))
    fields['target_num'] = int(m_targets.group(1)) if m_targets else 0

    if fields['target_num'] > 0:
        m_azi = re.search(r'方位:(-?[\d.]+)', line)
        m_rng = re.search(r'距离:(-?[\d.]+)', line)
        m_ele = re.search(r'俯仰:(-?[\d.]+)', line)
        m_alt = re.search(r'高度:(-?[\d.]+)', line)
        m_vel = re.search(r'速度:(-?[\d.e+-]+)', line)
        m_snr = re.search(r'信噪比:(\d+)', line)
        m_rcs = re.search(r'RCS:(-?[\d.e+-]+)', line)
        m_wide = re.search(r'宽窄脉冲标志:(\d+)', line)
        m_pbeam = re.search(r'俯仰波束:(\d+)', line)

        fields['azi_deg'] = float(m_azi.group(1)) if m_azi else 0.0
        fields['range_m'] = float(m_rng.group(1)) if m_rng else 0.0
        fields['ele_deg'] = float(m_ele.group(1)) if m_ele else 0.0
        fields['alt_m'] = float(m_alt.group(1)) if m_alt else 0.0
        fields['vel_mps'] = float(m_vel.group(1)) if m_vel else 0.0
        fields['snr'] = int(m_snr.group(1)) if m_snr else 0
        fields['rcs'] = float(m_rcs.group(1)) if m_rcs else 0.0
        fields['wide_narrow'] = int(m_wide.group(1)) if m_wide else 0
        fields['pitch_beam'] = int(m_pbeam.group(1)) if m_pbeam else 0
        fields['has_plot'] = True
    else:
        fields['azi_deg'] = 0.0
        fields['range_m'] = 0.0
        fields['ele_deg'] = 0.0
        fields['alt_m'] = 0.0
        fields['vel_mps'] = 0.0
        fields['snr'] = 0
        fields['rcs'] = 0.0
        fields['wide_narrow'] = 0
        fields['pitch_beam'] = 0
        fields['has_plot'] = False

    return fields


def main():
    print("=" * 70)
    print("  无人机实测数据解析工具")
    print("=" * 70)

    if not os.path.exists(INPUT_FILE):
        print(f"错误：找不到输入文件 {INPUT_FILE}")
        return

    all_plots = []
    with open(INPUT_FILE, 'r', encoding='utf-8') as f:
        for line in f:
            parsed = parse_line(line)
            if parsed:
                all_plots.append(parsed)

    print(f"\n[1] 原始数据统计")
    print(f"    总行数（点迹+空帧标记）: {len(all_plots)}")

    plots = [p for p in all_plots if p['has_plot']]
    empty_marks = [p for p in all_plots if not p['has_plot']]
    print(f"    有效点迹数: {len(plots)}")
    print(f"    空帧标记行数: {len(empty_marks)}")

    cpi_dict = defaultdict(list)
    for p in all_plots:
        cpi_dict[p['mSecond']].append(p)

    cpi_times = sorted(cpi_dict.keys())
    print(f"\n[2] CPI(波位)统计")
    print(f"    总CPI数: {len(cpi_times)}")

    cpi_plot_counts = [len(cpi_dict[t]) for t in cpi_times]
    empty_cpis = [t for t in cpi_times if len(cpi_dict[t]) == 1 and not cpi_dict[t][0]['has_plot']]
    nonempty_cpis = [t for t in cpi_times if t not in set(empty_cpis)]

    print(f"    有目标CPI数: {len(nonempty_cpis)}")
    print(f"    空CPI数(目标数=0): {len(empty_cpis)}")
    print(f"    每波位最多点迹数: {max(cpi_plot_counts)}")
    print(f"    每波位最少点迹数: {min(cpi_plot_counts)}")
    print(f"    平均每波位点迹数: {sum(cpi_plot_counts)/len(cpi_plot_counts):.1f}")

    if len(cpi_times) > 1:
        intervals = [cpi_times[i+1] - cpi_times[i] for i in range(len(cpi_times)-1)]
        from collections import Counter
        interval_counts = Counter(intervals)
        print(f"\n[3] 时间间隔统计(ms)")
        for iv, cnt in sorted(interval_counts.items()):
            pct = cnt / len(intervals) * 100
            print(f"    {iv}ms: {cnt}次 ({pct:.1f}%)")
        print(f"    时间范围: {cpi_times[0]}ms ~ {cpi_times[-1]}ms ({(cpi_times[-1]-cpi_times[0])/1000:.1f}s)")

    print(f"\n[4] 点迹特征统计（全部{len(plots)}个有效点）")
    vels = [p['vel_mps'] for p in plots]
    alts = [p['alt_m'] for p in plots]
    ranges = [p['range_m'] for p in plots]
    eles = [p['ele_deg'] for p in plots]
    azis = [p['azi_deg'] for p in plots]
    snrs = [p['snr'] for p in plots]
    print(f"    径向速度: min={min(vels):.1f} m/s, max={max(vels):.1f} m/s, mean={sum(vels)/len(vels):.1f} m/s")
    print(f"    高度:     min={min(alts):.1f} m, max={max(alts):.1f} m")
    print(f"    距离:     min={min(ranges):.1f} m, max={max(ranges):.1f} m")
    print(f"    俯仰角:   min={min(eles):.2f} deg, max={max(eles):.2f} deg")
    print(f"    方位角:   min={min(azis):.2f} deg, max={max(azis):.2f} deg")
    print(f"    信噪比:   min={min(snrs)}, max={max(snrs)}, mean={sum(snrs)/len(snrs):.1f}")

    ground_clutter = [p for p in plots if p['ele_deg'] < 0 or p['alt_m'] < 0]
    possible_targets = [p for p in plots if p['ele_deg'] >= 0 and p['alt_m'] >= 0]
    print(f"\n[5] 杂波/目标初步分类")
    print(f"    地杂波(俯仰<0或高度<0): {len(ground_clutter)} 个 ({len(ground_clutter)/len(plots)*100:.1f}%)")
    print(f"    潜在目标(俯仰>=0且高度>=0): {len(possible_targets)} 个 ({len(possible_targets)/len(plots)*100:.1f}%)")

    drone_like = [p for p in plots if p['ele_deg'] >= 0 and p['alt_m'] >= 50
                  and abs(p['vel_mps']) <= 80 and p['range_m'] <= 20000 and p['snr'] >= 8]
    print(f"    无人机候选(高度>=50m,|v|<=80m/s,距<20km,SNR>=8): {len(drone_like)} 个")

    wide_pulse = [p for p in plots if p['wide_narrow'] == 0]
    narrow_pulse = [p for p in plots if p['wide_narrow'] == 1]
    print(f"\n[6] 脉冲类型")
    print(f"    宽脉冲(宽窄=0): {len(wide_pulse)} 个")
    print(f"    窄脉冲(宽窄=1): {len(narrow_pulse)} 个")

    print(f"\n[7] 俯仰波束分布")
    pbeam_counts = defaultdict(int)
    for p in plots:
        pbeam_counts[p['pitch_beam']] += 1
    for pb in sorted(pbeam_counts.keys()):
        print(f"    俯仰波束{pb}: {pbeam_counts[pb]} 个")

    print(f"\n[8] 写入CSV文件...")

    with open(OUTPUT_CSV, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['序号', '波位号', '北斗时间戳', '时间戳(ms)', '方位(度)', '距离(m)',
                         '俯仰(度)', '高度(m)', '径向速度(m/s)', '信噪比', 'RCS',
                         '宽窄脉冲(0宽1窄)', '俯仰波束', '本波位目标数', '是否有效点'])
        for idx, p in enumerate(all_plots, 1):
            writer.writerow([
                idx, p['beam'], p['bd_time'], p['mSecond'],
                f"{p['azi_deg']:.4f}", f"{p['range_m']:.2f}", f"{p['ele_deg']:.4f}",
                f"{p['alt_m']:.2f}", f"{p['vel_mps']:.4f}", p['snr'],
                f"{p['rcs']:.6e}", p['wide_narrow'], p['pitch_beam'],
                p['target_num'], 1 if p['has_plot'] else 0
            ])
    print(f"    -> {OUTPUT_CSV}")

    with open(OUTPUT_CPI_CSV, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['CPI序号', '波位号', '时间戳(ms)', '帧间隔(ms)', '目标数',
                         '有效点迹数', '宽脉冲数', '窄脉冲数',
                         '最小距离(m)', '最大距离(m)',
                         '最小速度(m/s)', '最大速度(m/s)',
                         '正高度点数', '负高度点数(地杂波)'])
        prev_t = None
        for cpi_idx, t in enumerate(cpi_times, 1):
            cpi_plots = cpi_dict[t]
            real_plots = [p for p in cpi_plots if p['has_plot']]
            n_targets = cpi_plots[0]['target_num'] if cpi_plots else 0
            beam = cpi_plots[0]['beam'] if cpi_plots else -1
            interval = t - prev_t if prev_t is not None else 0
            prev_t = t

            wp = sum(1 for p in real_plots if p['wide_narrow'] == 0)
            np_ = sum(1 for p in real_plots if p['wide_narrow'] == 1)

            if real_plots:
                r_min = min(p['range_m'] for p in real_plots)
                r_max = max(p['range_m'] for p in real_plots)
                v_min = min(p['vel_mps'] for p in real_plots)
                v_max = max(p['vel_mps'] for p in real_plots)
                pos_alt = sum(1 for p in real_plots if p['alt_m'] >= 0)
                neg_alt = sum(1 for p in real_plots if p['alt_m'] < 0)
            else:
                r_min = r_max = v_min = v_max = 0
                pos_alt = neg_alt = 0

            writer.writerow([
                cpi_idx, beam, t, interval, n_targets, len(real_plots),
                wp, np_,
                f"{r_min:.1f}", f"{r_max:.1f}",
                f"{v_min:.2f}", f"{v_max:.2f}",
                pos_alt, neg_alt
            ])
    print(f"    -> {OUTPUT_CPI_CSV}")

    print(f"\n[9] 前30个CPI数据预览（用于验证算法输入）")
    print("-" * 100)
    print(f"{'CPI':>4} {'波位':>4} {'时间(ms)':>10} {'间隔':>5} {'点数':>4} {'宽':>3} {'窄':>3} "
          f"{'距min':>8} {'距max':>8} {'vmin':>7} {'vmax':>7} {'正高':>4} {'负高':>4}")
    print("-" * 100)
    prev_t = None
    for cpi_idx, t in enumerate(cpi_times[:30], 1):
        cpi_plots = cpi_dict[t]
        real_plots = [p for p in cpi_plots if p['has_plot']]
        n_targets = cpi_plots[0]['target_num']
        beam = cpi_plots[0]['beam']
        interval = t - prev_t if prev_t is not None else 0
        prev_t = t
        wp = sum(1 for p in real_plots if p['wide_narrow'] == 0)
        np_ = sum(1 for p in real_plots if p['wide_narrow'] == 1)
        if real_plots:
            r_min = min(p['range_m'] for p in real_plots)
            r_max = max(p['range_m'] for p in real_plots)
            v_min = min(p['vel_mps'] for p in real_plots)
            v_max = max(p['vel_mps'] for p in real_plots)
            pos_alt = sum(1 for p in real_plots if p['alt_m'] >= 0)
            neg_alt = sum(1 for p in real_plots if p['alt_m'] < 0)
        else:
            r_min = r_max = v_min = v_max = 0
            pos_alt = neg_alt = 0
        print(f"{cpi_idx:>4} {beam:>4} {t:>10} {interval:>5} {len(real_plots):>4} {wp:>3} {np_:>3} "
              f"{r_min:>8.1f} {r_max:>8.1f} {v_min:>7.1f} {v_max:>7.1f} {pos_alt:>4} {neg_alt:>4}")

    print(f"\n[10] 无人机候选点迹样例（按高度>=50m, |v|<=50m/s, SNR>=10筛选，取前20个）")
    print("-" * 110)
    print(f"{'序号':>4} {'波位':>4} {'时间(ms)':>10} {'方位':>8} {'距离':>9} {'俯仰':>8} {'高度':>9} "
          f"{'速度':>8} {'SNR':>4} {'RCS':>10} {'脉冲':>4}")
    print("-" * 110)
    drone_sorted = sorted(drone_like, key=lambda p: p['mSecond'])
    for idx, p in enumerate(drone_sorted[:20], 1):
        print(f"{idx:>4} {p['beam']:>4} {p['mSecond']:>10} {p['azi_deg']:>8.2f} {p['range_m']:>9.1f} "
              f"{p['ele_deg']:>8.3f} {p['alt_m']:>9.1f} {p['vel_mps']:>8.2f} {p['snr']:>4} "
              f"{p['rcs']:>10.3e} {'宽' if p['wide_narrow']==0 else '窄':>4}")

    print(f"\n解析完成！输出文件：")
    print(f"  1. 点迹级CSV: {OUTPUT_CSV}")
    print(f"  2. CPI级CSV:  {OUTPUT_CPI_CSV}")


if __name__ == '__main__':
    main()
