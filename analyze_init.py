import re
import math

# 读取data_entry.c，提取帧数据
with open('data_entry.c', 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

# 解析帧数据
frames = []
current_frame = None

lines = content.split('\n')
for line in lines:
    # 匹配帧开始
    match = re.match(r'data\[(\d+)\]\.targetNum\s*=\s*(\d+)', line)
    if match:
        if current_frame is not None:
            frames.append(current_frame)
        current_frame = {
            'frame_idx': int(match.group(1)),
            'targetNum': int(match.group(2)),
            'beamNo': 0,
            'mSecond': 0,
            'azi': [],
            'ele': [],
            'range': [],
            'velocity': []
        }
    elif current_frame is not None:
        match = re.match(r'data\[(\d+)\]\.beamNo\s*=\s*(\d+)', line)
        if match:
            current_frame['beamNo'] = int(match.group(2))
        match = re.match(r'data\[(\d+)\]\.mSecond\s*=\s*([\d.]+)', line)
        if match:
            current_frame['mSecond'] = float(match.group(2))
        
        # 提取azi
        for i in range(60):
            pat = f'data\\[\\d+\\]\\.azi\\[{i}\\]\\s*=\\s*([^;]+);'
            m = re.search(pat, line)
            if m:
                val_str = m.group(1).strip()
                if 'pi' in val_str.lower():
                    val_str = val_str.replace('pi', str(math.pi))
                    val_str = val_str.replace('/180.0f*pi', ' * 180.0 / ' + str(math.pi))
                    val_str = val_str.replace('/180*pi', ' * 180.0 / ' + str(math.pi))
                try:
                    val = eval(val_str)
                except:
                    val = 0.0
                while len(current_frame['azi']) <= i:
                    current_frame['azi'].append(0.0)
                current_frame['azi'][i] = val
        
        # 提取ele
        for i in range(60):
            pat = f'data\\[\\d+\\]\\.ele\\[{i}\\]\\s*=\\s*([^;]+);'
            m = re.search(pat, line)
            if m:
                val_str = m.group(1).strip()
                if 'pi' in val_str.lower():
                    val_str = val_str.replace('pi', str(math.pi))
                    val_str = val_str.replace('/180.0f*pi', ' * 180.0 / ' + str(math.pi))
                    val_str = val_str.replace('/180*pi', ' * 180.0 / ' + str(math.pi))
                try:
                    val = eval(val_str)
                except:
                    val = 0.0
                while len(current_frame['ele']) <= i:
                    current_frame['ele'].append(0.0)
                current_frame['ele'][i] = val

        # 提取range
        for i in range(60):
            pat = f'data\\[\\d+\\]\\.range\\[{i}\\]\\s*=\\s*([^;]+);'
            m = re.search(pat, line)
            if m:
                val_str = m.group(1).strip().rstrip('f')
                try:
                    val = float(val_str)
                except:
                    val = 0.0
                while len(current_frame['range']) <= i:
                    current_frame['range'].append(0.0)
                current_frame['range'][i] = val

        # 提取velocity
        for i in range(60):
            pat = f'data\\[\\d+\\]\\.velocity\\[{i}\\]\\s*=\\s*([^;]+);'
            m = re.search(pat, line)
            if m:
                val_str = m.group(1).strip().rstrip('f')
                try:
                    val = float(val_str)
                except:
                    val = 0.0
                while len(current_frame['velocity']) <= i:
                    current_frame['velocity'].append(0.0)
                current_frame['velocity'][i] = val

if current_frame is not None:
    frames.append(current_frame)

print(f'Total frames parsed: {len(frames)}')
if frames:
    f0 = frames[0]
    print(f'Frame 0: beamNo={f0["beamNo"]}, mSecond={f0["mSecond"]}, targetNum={f0["targetNum"]}')
    for i in range(min(f0["targetNum"], len(f0["azi"]))):
        az = f0["azi"][i] * 180 / math.pi
        el = f0["ele"][i] * 180 / math.pi
        r = f0["range"][i] if i < len(f0["range"]) else 0
        v = f0["velocity"][i] if i < len(f0["velocity"]) else 0
        print(f'  Point {i}: Az={az:.2f}° El={el:.2f}° R={r:.1f}m Vr={v:.2f}m/s')

# 找出无人机候选点（Vr在1-30m/s）
print('\n=== 无人机候选点 (Vr 1-30m/s) ===')
drone_candidates = []
for f in frames:
    for i in range(min(f['targetNum'], len(f['velocity']))):
        vr = f['velocity'][i]
        if 1.0 <= abs(vr) <= 30.0:
            r = f['range'][i] if i < len(f['range']) else 0
            az = f['azi'][i] * 180 / math.pi if i < len(f['azi']) else 0
            el = f['ele'][i] * 180 / math.pi if i < len(f['ele']) else 0
            drone_candidates.append({
                'frame': f['frame_idx'],
                'beam': f['beamNo'],
                'time': f['mSecond'],
                'pt_idx': i,
                'azi_deg': az,
                'el_deg': el,
                'range': r,
                'vr': vr
            })

for dc in drone_candidates[:40]:
    print(f'  Frame {dc["frame"]:3d} | beam={dc["beam"]:2d} | t={dc["time"]:.0f}ms | Az={dc["azi_deg"]:7.2f}° | El={dc["el_deg"]:6.2f}° | R={dc["range"]:8.1f}m | Vr={dc["vr"]:7.2f}m/s')

print(f'\nTotal drone candidates: {len(drone_candidates)}')
