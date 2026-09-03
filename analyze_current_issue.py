# -*- coding: utf-8 -*-
"""
分析当前问题：
1. 找出输出数据.txt中速度跳变帧
2. 诊断为什么航迹初始化后一次关联都没成功（速度一直10.9m/s）
"""
import re
import sys

def parse_output_data(filepath):
    """解析输出数据.txt，提取每帧航迹信息"""
    frames = []
    with open(filepath, 'r', encoding='gbk', errors='ignore') as f:
        content = f.read()
    
    # 匹配Frame行和Track行
    frame_pattern = r'Frame (\d+) \| t=(\d+)ms \| n=(\d+)'
    track_pattern = r'-> Track\[(\d+)\]: Az=([\d.]+)deg El=([\d.]+)deg R=([\d.]+)m V=([\d.]+)m/s'
    
    lines = content.split('\n')
    current_frame = None
    
    for line in lines:
        fm = re.match(frame_pattern, line.strip())
        if fm:
            current_frame = {
                'frame_num': int(fm.group(1)),
                'timestamp': int(fm.group(2)),
                'n_points': int(fm.group(3)),
                'tracks': []
            }
            frames.append(current_frame)
            continue
        
        tm = re.search(track_pattern, line)
        if tm and current_frame is not None:
            track = {
                'track_id': int(tm.group(1)),
                'az': float(tm.group(2)),
                'el': float(tm.group(3)),
                'r': float(tm.group(4)),
                'v': float(tm.group(5))
            }
            current_frame['tracks'].append(track)
    
    return frames

def analyze_speed_stability(frames):
    """分析速度稳定性：找出速度跳变点和纯预测持续时间"""
    print("=" * 80)
    print("【分析1】速度稳定性和跳变点")
    print("=" * 80)
    
    prev_v = None
    pure_prediction_start = None
    speed_jumps = []
    
    for frame in frames:
        if not frame['tracks']:
            if prev_v is not None:
                print(f"  Frame {frame['frame_num']:4d}: 航迹丢失！(t={frame['timestamp']}ms)")
                prev_v = None
            continue
        
        for tr in frame['tracks']:
            cur_v = tr['v']
            
            if prev_v is None:
                print(f"  Frame {frame['frame_num']:4d}: 航迹建立 V={cur_v:.1f}m/s")
                pure_prediction_start = frame['frame_num']
                prev_v = cur_v
                continue
            
            delta_v = cur_v - prev_v
            
            # 速度突变检测（>5m/s变化）
            if abs(delta_v) > 5.0:
                speed_jumps.append({
                    'frame': frame['frame_num'],
                    'timestamp': frame['timestamp'],
                    'prev_v': prev_v,
                    'new_v': cur_v,
                    'delta': delta_v,
                    'az': tr['az'],
                    'el': tr['el'],
                    'r': tr['r']
                })
                pure_prediction_start = frame['frame_num']  # 重置
            
            prev_v = cur_v
    
    if speed_jumps:
        print(f"\n  发现 {len(speed_jumps)} 次速度跳变：")
        for jmp in speed_jumps:
            print(f"    Frame {jmp['frame']:4d} (t={jmp['timestamp']}ms): "
                  f"V={jmp['prev_v']:.1f} → {jmp['new_v']:.1f} m/s (Δ={jmp['delta']:+.1f})")
            print(f"      Az={jmp['az']:.2f}°, El={jmp['el']:.2f}°, R={jmp['r']:.1f}m")
    else:
        print("  未发现速度跳变")
    
    return speed_jumps

def check_association_pattern(frames):
    """检查关联模式：速度不变=纯预测，速度微变=关联成功"""
    print("\n" + "=" * 80)
    print("【分析2】关联成功情况（通过速度变化判断）")
    print("=" * 80)
    
    last_v = None
    consecutive_pure_pred = 0
    max_pure_pred = 0
    max_pure_pred_frame = 0
    assoc_count = 0
    
    for frame in frames:
        if not frame['tracks']:
            if consecutive_pure_pred > max_pure_pred:
                max_pure_pred = consecutive_pure_pred
                max_pure_pred_frame = frame['frame_num']
            consecutive_pure_pred = 0
            last_v = None
            continue
        
        for tr in frame['tracks']:
            cur_v = tr['v']
            
            if last_v is None:
                last_v = cur_v
                consecutive_pure_pred = 0
                continue
            
            if abs(cur_v - last_v) < 0.01:  # 速度完全不变=纯预测
                consecutive_pure_pred += 1
                if consecutive_pure_pred == 20:  # 每20帧报告一次
                    print(f"  Frame {frame['frame_num']-19:4d}-{frame['frame_num']:4d}: "
                          f"连续20帧纯预测（速度固定V={cur_v:.1f}m/s）")
            else:  # 速度有变化=关联成功
                if consecutive_pure_pred > max_pure_pred:
                    max_pure_pred = consecutive_pure_pred
                    max_pure_pred_frame = frame['frame_num']
                consecutive_pure_pred = 0
                assoc_count += 1
            
            last_v = cur_v
    
    if consecutive_pure_pred > max_pure_pred:
        max_pure_pred = consecutive_pure_pred
    
    print(f"\n  统计结果：")
    print(f"    纯预测最长连续帧数：{max_pure_pred} (约Frame {max_pure_pred_frame-max_pure_pred}-{max_pure_pred_frame})")
    print(f"    关联成功总次数：{assoc_count}")
    
    return assoc_count, max_pure_pred

def main():
    output_path = r'd:\DSP\6678\track\track_1\输出数据.txt'
    
    try:
        frames = parse_output_data(output_path)
    except Exception as e:
        print(f"读取输出数据.txt失败: {e}")
        return
    
    print(f"共解析 {len(frames)} 帧数据\n")
    
    jumps = analyze_speed_stability(frames)
    assoc, max_pp = check_association_pattern(frames)
    
    print("\n" + "=" * 80)
    print("【诊断结论】")
    print("=" * 80)
    
    if assoc == 0:
        print("  ❌ 严重问题：航迹建立后【0次关联成功】，全程纯预测！")
        print("     原因：所有门限检查都在拒绝正确无人机点")
        print("     后果：predict_flag持续增长 → 超过阈值后航迹被删除 →")
        print("           新高速杂波点建立假航迹(target_type=1炮弹) → 速度飙升")
    elif max_pp > 100:
        print(f"  ⚠️  警告：存在长达{max_pp}帧的纯预测区间，远超beam扫描间隔15帧")
        print("     可能导致航迹漂移后误关联")
    
    if jumps:
        print(f"\n  ⚠️  速度跳变根因分析：")
        for jmp in jumps:
            if jmp['new_v'] > 50 and jmp['prev_v'] < 20:
                print(f"    Frame {jmp['frame']}: 无人机速度({jmp['prev_v']:.1f}m/s) → "
                      f"炮弹速度({jmp['new_v']:.1f}m/s)")
                print(f"    → 原航迹已被删除，新假航迹用高速杂波初始化(target_type=1)")
                print(f"    → 炮弹约束宽松(Vmax=几百m/s)，后续杂波全部放行！")

if __name__ == '__main__':
    main()
