import re
import math

# 手动解析data_entry.c
frames_data = []
current_frame = None

with open('data_entry.c', 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    line = lines[i]
    
    # 检查帧开始
    m = re.match(r'\s*data\[(\d+)\]\.targetNum\s*=\s*(\d+)', line)
    if m:
        if current_frame is not None:
            frames_data.append(current_frame)
        current_frame = {
            'idx': int(m.group(1)),
            'targetNum': int(m.group(2)),
            'beamNo': -1,
            'mSecond': 0,
            'points': []
        }
    elif current_frame is not None:
        m = re.match(r'\s*data\[(\d+)\]\.beamNo\s*=\s*(\d+)', line)
        if m:
            current_frame['beamNo'] = int(m.group(2))
        
        m = re.match(r'\s*data\[(\d+)\]\.mSecond\s*=\s*([\d.]+)', line)
        if m:
            current_frame['mSecond'] = float(m.group(2))
        
        # 检查点数据
        for ptype in ['azi', 'ele', 'range', 'velocity']:
            for idx in range(60):
                pat = rf'\s*data\[\d+\]\.{ptype}\[{idx}\]\s*=\s*([^;]+);'
                m = re.search(pat, line)
                if m:
                    val_str = m.group(1).strip()
                    if 'pi' in val_str and ('*' in val_str or '/' in val_str):
                        # 如: 79.2044/180.0f*pi
                        val_str = val_str.replace('pi', str(math.pi))
                        val_str = val_str.replace('f', '')
                        try:
                            val = eval(val_str)
                        except:
                            val = 0.0
                    else:
                        val_str = val_str.rstrip('f')
                        try:
                            val = float(val_str)
                        except:
                            val = 0.0
                    
                    while len(current_frame['points']) <= idx:
                        current_frame['points'].append({'azi': 0, 'ele': 0, 'range': 0, 'velocity': 0})
                    current_frame['points'][idx][ptype] = val
    
    i += 1

if current_frame is not None:
    frames_data.append(current_frame)

print(f'Total frames: {len(frames_data)}')

# 显示Frame 4, 5, 6的数据
for f in frames_data:
    if f['idx'] in [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]:
        print(f'\nFrame {f["idx"]} (beam={f["beamNo"]}, t={f["mSecond"]:.0f}ms, n={f["targetNum"]}):')
        for pi, p in enumerate(f['points'][:f['targetNum']]):
            az_deg = p['azi'] * 180 / math.pi
            el_deg = p['ele'] * 180 / math.pi
            print(f'  Point {pi}: Az={az_deg:7.2f}° El={el_deg:6.2f}° R={p["range"]:8.1f}m Vr={p["velocity"]:8.2f}m/s')

# 找无人机点（Vr在8-15m/s，距离500-2000m）
print('\n=== 无人机候选点 (Vr 8-15m/s, R 500-2000m) ===')
for f in frames_data:
    for p in f['points'][:f['targetNum']]:
        vr = p['velocity']
        r = p['range']
        if 8.0 <= abs(vr) <= 15.0 and 500 <= r <= 2000:
            az_deg = p['azi'] * 180 / math.pi
            el_deg = p['ele'] * 180 / math.pi
            print(f'  Frame {f["idx"]:3d} | beam={f["beamNo"]:2d} | t={f["mSecond"]:.0f}ms | Az={az_deg:7.2f}° | El={el_deg:6.2f}° | R={r:8.1f}m | Vr={vr:8.2f}m/s')
