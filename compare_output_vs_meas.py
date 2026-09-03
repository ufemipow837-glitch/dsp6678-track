#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对比代码输出数据.txt与无人机实测数据.txt的跟踪精度
"""
import re
import math
import statistics
from collections import defaultdict

OUTPUT_FILE = '输出数据.txt'
MEAS_FILE = '无人机实测数据.txt'

def parse_output(filename):
    """解析DSP输出日志，提取可靠航迹数据"""
    tracks = []
    current_frame = None
    current_t = None
    
    with open(filename, 'r', encoding='gbk', errors='ignore') as f:
        for line in f:
            line = line.strip()
            
            # 匹配Frame行: "Frame 038 | t=904396ms | n=3 | Cycle: 2.728 ms"
            frame_match = re.match(r'Frame\s+(\d+)\s*\|\s*t=(\d+)ms\s*\|\s*n=(\d+)', line)
            if frame_match:
                current_frame = int(frame_match.group(1))
                current_t = int(frame_match.group(2))
                continue
            
            # 匹配Track行: "-> Track[0]: Az=16.07deg El=2.64deg R=1271.2m V=17.3m/s"
            track_match = re.match(r'->\s*Track\[(\d+)\]:\s*Az=([-\d.]+)deg\s+El=([-\d.]+)deg\s+R=([-\d.]+)m\s+V=([-\d.]+)m/s', line)
            if track_match and current_frame is not None:
                track_idx = int(track_match.group(1))
                az = float(track_match.group(2))
                el = float(track_match.group(3))
                r = float(track_match.group(4))
                v = float(track_match.group(5))
                tracks.append({
                    'frame': current_frame,
                    't': current_t,
                    'track_idx': track_idx,
                    'az': az,
                    'el': el,
                    'r': r,
                    'v': v
                })
    return tracks

def parse_meas(filename):
    """解析实测数据，提取所有点迹"""
    plots_by_t = defaultdict(list)
    all_plots = []
    
    with open(filename, 'r', encoding='gbk', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line.startswith('点迹解析数据'):
                continue
            
            # 提取关键字段
            t_match = re.search(r'时间戳:(\d+)', line)
            az_match = re.search(r'方位:([-\d.]+)', line)
            el_match = re.search(r'俯仰:([-\d.]+)', line)
            r_match = re.search(r'距离:([-\d.]+)', line)
            v_match = re.search(r'速度:([-\d.]+)', line)
            snr_match = re.search(r'信噪比:(\d+)', line)
            beam_match = re.search(r'波位号:(\d+)', line)
            n_match = re.search(r'目标数:(\d+)', line)
            
            if t_match and az_match and r_match:
                t = int(t_match.group(1))
                az = float(az_match.group(1))
                el = float(el_match.group(1)) if el_match else 0.0
                r = float(r_match.group(1))
                v = float(v_match.group(1)) if v_match else 0.0
                snr = int(snr_match.group(1)) if snr_match else 0
                beam = int(beam_match.group(1)) if beam_match else -1
                n = int(n_match.group(1)) if n_match else 0
                
                plot = {
                    't': t,
                    'az': az,
                    'el': el,
                    'r': r,
                    'v': abs(v),
                    'v_raw': v,
                    'snr': snr,
                    'beam': beam,
                    'n': n
                }
                plots_by_t[t].append(plot)
                all_plots.append(plot)
    
    return plots_by_t, all_plots

def find_drone_plot(plots, az_est, el_est, r_est, az_tol=3.0, r_tol=200.0):
    """在一堆点中找到最像无人机的那个点（距离预测位置最近、SNR高、速度约10-20m/s）"""
    candidates = []
    for p in plots:
        az_diff = abs(p['az'] - az_est)
        r_diff = abs(p['r'] - r_est)
        # 用方位+距离作为匹配准则，SNR高的优先
        if az_diff < az_tol and r_diff < r_tol:
            score = az_diff * 10 + r_diff / 10 - p['snr'] * 0.5
            candidates.append((score, p))
    
    if not candidates:
        # 如果严格波门内没有，放宽到更大范围找最近的
        for p in plots:
            az_diff = abs(p['az'] - az_est)
            r_diff = abs(p['r'] - r_est)
            score = az_diff * 5 + r_diff / 20
            candidates.append((score, p))
    
    candidates.sort(key=lambda x: x[0])
    return candidates[0][1] if candidates else None

# 解析数据
print("解析数据中...")
tracks = parse_output(OUTPUT_FILE)
plots_by_t, all_plots = parse_meas(MEAS_FILE)

print(f"输出数据: {len(tracks)} 个可靠航迹点")
print(f"实测数据: {len(all_plots)} 个点迹")

# 找出可靠航迹建立的起始帧
if tracks:
    first_frame = tracks[0]['frame']
    print(f"可靠航迹起始帧: Frame {first_frame} (t={tracks[0]['t']}ms)")

# 对比分析
print("\n" + "=" * 120)
print("跟踪结果 vs 实测数据 逐帧对比")
print("=" * 120)
print(f"{'Frame':>5} {'t(ms)':>8} | {'Az_DSP':>8} {'Az_Meas':>8} {'Az_err':>8} | "
      f"{'El_DSP':>8} {'El_Meas':>8} {'El_err':>8} | {'R_DSP':>9} {'R_Meas':>9} {'R_err':>8} | "
      f"{'V_DSP':>6} {'V_Meas':>7} {'V_err':>6} | 匹配状态")
print("-" * 120)

az_errors = []
el_errors = []
r_errors = []
v_errors = []
matched = []
missed = []

# 用于计算无人机实际速度（连续两帧距离差/时间差）
prev_r_meas = None
prev_t = None

for trk in tracks:
    t = trk['t']
    plots = plots_by_t.get(t, [])
    
    if not plots:
        missed.append((trk, '该时刻无实测点'))
        print(f"{trk['frame']:>5} {t:>8} | {trk['az']:>8.2f} {'--':>8} {'':>8} | "
              f"{trk['el']:>8.2f} {'--':>8} {'':>8} | {trk['r']:>9.1f} {'--':>9} {'':>8} | "
              f"{trk['v']:>6.1f} {'--':>7} {'':>6} | 无实测点")
        continue
    
    # 找匹配的无人机点
    matched_plot = find_drone_plot(plots, trk['az'], trk['el'], trk['r'])
    
    if matched_plot is None:
        missed.append((trk, '未找到匹配点'))
        print(f"{trk['frame']:>5} {t:>8} | {trk['az']:>8.2f} {'??':>8} {'':>8} | "
              f"{trk['el']:>8.2f} {'??':>8} {'':>8} | {trk['r']:>9.1f} {'??':>9} {'':>8} | "
              f"{trk['v']:>6.1f} {'??':>7} {'':>6} | 无匹配点")
        continue
    
    az_err = trk['az'] - matched_plot['az']
    el_err = trk['el'] - matched_plot['el']
    r_err = trk['r'] - matched_plot['r']
    v_err = trk['v'] - matched_plot['v']
    
    az_errors.append(abs(az_err))
    el_errors.append(abs(el_err))
    r_errors.append(abs(r_err))
    v_errors.append(abs(v_err))
    
    # 计算真实速度（距离差/时间差）
    if prev_r_meas is not None and prev_t is not None:
        dt = (t - prev_t) / 1000.0
        if dt > 0:
            v_real = (matched_plot['r'] - prev_r_meas) / dt
            v_err_real = trk['v'] - abs(v_real)
        else:
            v_real = None
            v_err_real = None
    else:
        v_real = None
        v_err_real = None
    
    matched.append({
        'trk': trk,
        'meas': matched_plot,
        'az_err': az_err,
        'el_err': el_err,
        'r_err': r_err,
        'v_err': v_err,
        'v_real': v_real
    })
    
    # 标记大误差
    mark = ''
    if abs(az_err) > 1.0 or abs(r_err) > 30 or abs(v_err) > 5:
        mark = ' *'
    
    print(f"{trk['frame']:>5} {t:>8} | {trk['az']:>8.2f} {matched_plot['az']:>8.2f} {az_err:>+7.2f}° | "
          f"{trk['el']:>8.2f} {matched_plot['el']:>8.2f} {el_err:>+7.2f}° | "
          f"{trk['r']:>9.1f} {matched_plot['r']:>9.1f} {r_err:>+7.1f}m | "
          f"{trk['v']:>6.1f} {matched_plot['v']:>7.2f} {v_err:>+5.1f}{mark}")
    
    prev_r_meas = matched_plot['r']
    prev_t = t

print("-" * 120)

# 统计结果
print(f"\n统计汇总:")
print(f"  可靠航迹总帧数: {len(tracks)}")
print(f"  成功匹配帧数: {len(matched)}")
print(f"  未匹配帧数: {len(missed)}")
print(f"  匹配率: {len(matched)/len(tracks)*100:.1f}%")

if az_errors:
    print("\n" + "=" * 80)
    print("📊 跟踪误差统计 (相对于实测点):")
    print("-" * 80)
    print(f"  方位角误差:")
    print(f"    RMS = {math.sqrt(sum(e**2 for e in az_errors)/len(az_errors)):.3f}°")
    print(f"    平均 = {statistics.mean(az_errors):.3f}°")
    print(f"    最大 = {max(az_errors):.3f}°")
    print(f"  俯仰角误差:")
    print(f"    RMS = {math.sqrt(sum(e**2 for e in el_errors)/len(el_errors)):.3f}°")
    print(f"    平均 = {statistics.mean(el_errors):.3f}°")
    print(f"    最大 = {max(el_errors):.3f}°")
    print(f"  距离误差:")
    print(f"    RMS = {math.sqrt(sum(e**2 for e in r_errors)/len(r_errors)):.2f}m")
    print(f"    平均 = {statistics.mean(r_errors):.2f}m")
    print(f"    最大 = {max(r_errors):.2f}m")
    print(f"  速度误差 (DSP速度 vs 实测径向速度绝对值):")
    print(f"    RMS = {math.sqrt(sum(e**2 for e in v_errors)/len(v_errors)):.2f}m/s")
    print(f"    平均 = {statistics.mean(v_errors):.2f}m/s")
    print(f"    最大 = {max(v_errors):.2f}m/s")
    
    # 分析速度误差：DSP输出的是合速度，实测是径向速度，两者有本质区别
    print("\n" + "=" * 80)
    print("💡 重要说明：")
    print("-" * 80)
    print("  DSP输出的 V 是三维合速度 (sqrt(vx²+vy²+vz²))，约12~20m/s")
    print("  实测数据中的速度是径向速度 (沿雷达视线方向)，约±12m/s")
    print("  两者物理意义不同，不能直接对比！")
    print("  应该用位置误差来判断跟踪精度。")
    
    # 距离漂移分析
    print("\n" + "=" * 80)
    print("📈 扫描周期预测精度 (波束扫走后再回来的位置保持能力):")
    print("-" * 80)
    
    # 找出连续匹配帧之间的间隔
    scan_cycles = []
    for i in range(1, len(matched)):
        frame_gap = matched[i]['trk']['frame'] - matched[i-1]['trk']['frame']
        if frame_gap > 3:  # 间隔大于3帧说明波束离开了
            r_pred_drift = matched[i]['trk']['r'] - matched[i-1]['trk']['r']
            r_meas_drift = matched[i]['meas']['r'] - matched[i-1]['meas']['r']
            pred_err_at_return = matched[i]['r_err']
            scan_cycles.append({
                'gap': frame_gap,
                'r_pred_drift': r_pred_drift,
                'r_meas_drift': r_meas_drift,
                'pred_err': pred_err_at_return
            })
    
    if scan_cycles:
        pred_errs = [c['pred_err'] for c in scan_cycles]
        print(f"  共 {len(scan_cycles)} 个扫描周期，波束离开平均 {statistics.mean([c['gap'] for c in scan_cycles]):.0f} 帧")
        print(f"  回到波位时的预测距离误差:")
        print(f"    RMS = {math.sqrt(sum(e**2 for e in pred_errs)/len(pred_errs)):.2f}m")
        print(f"    平均 = {statistics.mean(pred_errs):.2f}m")
        print(f"    最大 = {max(abs(e) for e in pred_errs):.2f}m")

print("\n" + "=" * 80)
print("✅ 结论:")
print("=" * 80)
