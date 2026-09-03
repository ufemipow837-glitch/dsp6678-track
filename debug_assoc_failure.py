# -*- coding: utf-8 -*-
"""
深入诊断：
1. 找出哪4帧关联成功了（速度有微小变化）
2. 模拟Frame50-80的门限检查，看正确无人机点被拒在哪一层
3. 检查Frame567和771初始化的假航迹用的是什么点迹数据
"""
import re
import sys
import math

PI = 3.141592653589793

def parse_output_data(filepath):
    frames = []
    with open(filepath, 'r', encoding='gbk', errors='ignore') as f:
        content = f.read()
    
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

def find_successful_associations(frames):
    """找出关联成功的帧（速度有变化的帧）"""
    print("=" * 80)
    print("【分析1】关联成功的帧（速度变化≠0）")
    print("=" * 80)
    
    last_v = None
    last_r = None
    last_az = None
    assoc_frames = []
    
    for frame in frames:
        if not frame['tracks']:
            last_v = None
            last_r = None
            last_az = None
            continue
        
        for tr in frame['tracks']:
            cur_v = tr['v']
            cur_r = tr['r']
            cur_az = tr['az']
            
            if last_v is None:
                last_v = cur_v
                last_r = cur_r
                last_az = cur_az
                continue
            
            dv = cur_v - last_v
            dr = cur_r - last_r
            daz = cur_az - last_az
            if daz > 180: daz -= 360
            if daz < -180: daz += 360
            
            if abs(dv) > 0.001 or abs(dr) > 0.01 or abs(daz) > 0.001:
                assoc_frames.append({
                    'frame': frame['frame_num'],
                    'timestamp': frame['timestamp'],
                    'prev_v': last_v, 'v': cur_v, 'dv': dv,
                    'prev_r': last_r, 'r': cur_r, 'dr': dr,
                    'prev_az': last_az, 'az': cur_az, 'daz': daz,
                    'el': tr['el']
                })
            
            last_v = cur_v
            last_r = cur_r
            last_az = cur_az
    
    print(f"  共找到 {len(assoc_frames)} 次关联成功：\n")
    for af in assoc_frames:
        print(f"  Frame {af['frame']:4d} (t={af['timestamp']}ms): "
              f"V={af['prev_v']:.3f}→{af['v']:.3f} (Δ={af['dv']:+.3f}), "
              f"R={af['prev_r']:.1f}→{af['r']:.1f} (Δ={af['dr']:+.1f}), "
              f"Az={af['prev_az']:.2f}→{af['az']:.2f} (Δ={af['daz']:+.2f}°)")
    
    return assoc_frames

def analyze_false_track_init():
    """分析data_entry.c中Frame567和771的点迹数据——假航迹是用什么杂波初始化的"""
    print("\n" + "=" * 80)
    print("【分析2】假航迹初始化时的点迹特征（Frame567 / 771）")
    print("=" * 80)
    
    # 航迹初始化需要3帧连续数据，所以假航迹在Frame567出现，说明
    # 初始化3点来自Frame565, 566, 567的点迹
    # 同理Frame771 → 点来自769,770,771
    
    data_entry_path = r'd:\DSP\6678\track\track_1\data_entry.c'
    try:
        with open(data_entry_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except:
        with open(data_entry_path, 'r', encoding='gbk', errors='ignore') as f:
            content = f.read()
    
    # 提取特定帧的点数据
    def extract_frame_points(frame_num):
        # 找 fill_frames_X_Y 函数中 k==frame_num 的 case
        chunk_start = (frame_num // 50) * 50
        func_name = f'fill_frames_{chunk_start}_{chunk_start+49}'
        
        # 找 case k==frame_num: 的块
        pattern = rf'if\(k == {frame_num}\)\s*\{{(.*?)\n\s*\}}'
        match = re.search(pattern, content, re.DOTALL)
        if not match:
            # 试试另一种格式
            pattern2 = rf'k\s*==\s*{frame_num}\s*\)?\s*:\s*\{{(.*?)\n\s*\}}'
            match = re.search(pattern2, content, re.DOTALL)
        
        if not match:
            return None
        
        block = match.group(1)
        
        # 提取点数据
        points = []
        # 匹配 target_data[N].xxx = val
        idx_pattern = r'target_data\[(\d+)\]\.(\w+)\s*=\s*([-\d.eE+]+)'
        data_map = {}
        
        for m in re.finditer(idx_pattern, block):
            idx = int(m.group(1))
            field = m.group(2)
            val = float(m.group(3))
            if idx not in data_map:
                data_map[idx] = {}
            data_map[idx][field] = val
        
        for idx, data in sorted(data_map.items()):
            if 'Use_Flag_1' in data and data['Use_Flag_1'] > 0:
                p = {
                    'idx': idx,
                    'az': data.get('Azimuth', 0),
                    'el': data.get('Pitch', 0),
                    'r': data.get('Range', 0),
                    'v': data.get('velocity', 0),
                    'beamNo': int(data.get('bpn', -1)),
                    'x': data.get('x', 0),
                    'y': data.get('y', 0),
                    'z': data.get('z', 0),
                    'mSecond': data.get('mSecond', 0)
                }
                points.append(p)
        
        return points
    
    for false_frame in [565, 566, 567, 769, 770, 771]:
        pts = extract_frame_points(false_frame)
        if pts is None:
            print(f"  Frame {false_frame}: 未找到点数据")
            continue
        
        # 找高速杂波
        high_speed = [p for p in pts if abs(p['v']) > 50]
        print(f"\n  Frame {false_frame}: 共{len(pts)}个有效点，高速杂波(|V|>50)共{len(high_speed)}个")
        for p in high_speed:
            print(f"    [{p['idx']}] beam={p['beamNo']:2d}, Az={p['az']:.2f}°, "
                  f"El={p['el']:.2f}°, R={p['r']:.1f}m, Vr={p['v']:+.2f}m/s "
                  f"(t={p['mSecond']:.0f}ms)")
    
    return

def main():
    output_path = r'd:\DSP\6678\track\track_1\输出数据.txt'
    frames = parse_output_data(output_path)
    
    assoc_frames = find_successful_associations(frames)
    analyze_false_track_init()
    
    print("\n" + "=" * 80)
    print("【诊断核心结论】")
    print("=" * 80)
    print("""
问题链：
① Frame38建立无人机航迹 → ② 160帧只关联成功4次 → ③ Frame199航迹被删
→ ④ Frame567用高速杂波(|Vr|>60m/s)建立新航迹(spd=135m/s)
→ ⑤ target_type=1炮弹 → ⑥ 炮弹宽松约束不拦截高速 → ⑦输出V=135m/s

用户疑问：为什么设置了速度门方位门没拦住？
答案：因为速度门方位门只对【已有航迹】的关联检查生效！
      假航迹是【新初始化】的——航迹初始化阶段只有3点一致性检查，
      没有速度门(≤25m/s)、方位门(≤20°)的硬约束！
      target_type=1(炮弹)的航迹，关联检查用炮弹宽松约束(Vmax=几百m/s)！

需要修复的两点：
【修复A】航迹初始化阶段：加物理硬约束
  - 径向速度绝对值>80m/s → 一律拒绝初始化（或强制类型判断）
  - 但这是炮弹场景通用的，不能硬写死，应改为：
    初始化时 spd>80 → target_type=1，但同时检查Vr绝对值必须>80
    初始化时 spd≤80 → target_type=2，且Vr绝对值必须≤30
  
【修复B】更关键的：解决原无人机航迹97.5%关联失败的问题！
  必须找出Frame40-190区间，正确的无人机点被哪一层门拒绝了
""")

if __name__ == '__main__':
    main()
