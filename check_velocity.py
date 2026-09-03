import re

# 解析输出数据.txt
track_data = []
with open(r'd:\DSP\6678\track\track_1\输出数据.txt', 'r', encoding='utf-8', errors='ignore') as f:
    current_frame = -1
    current_t = -1
    for line in f:
        frame_match = re.search(r'Frame\s+(\d+)\s+\|\s+t=(\d+)ms', line)
        if frame_match:
            current_frame = int(frame_match.group(1))
            current_t = int(frame_match.group(2))
        
        track_match = re.search(r'Track\[0\]:\s*Az=([-\d.]+)deg\s+El=([-\d.]+)deg\s+R=([-\d.]+)m\s+V=([-\d.]+)m/s', line)
        if track_match and current_frame >= 0:
            az = float(track_match.group(1))
            el = float(track_match.group(2))
            r = float(track_match.group(3))
            v = float(track_match.group(4))
            track_data.append({
                'frame': current_frame,
                't': current_t,
                'az': az, 'el': el, 'r': r, 'v': v
            })

print(f'输出数据总航迹点: {len(track_data)}')
if track_data:
    print(f'帧范围: {track_data[0]["frame"]} - {track_data[-1]["frame"]}')
    print(f'时间范围: {track_data[0]["t"]}ms - {track_data[-1]["t"]}ms')

# 找速度异常点(速度>30m/s 或 速度<0, 正常无人机约10-15m/s)
print('\n=== 速度异常点 (V>25m/s 或 V<0, 正常无人机10~15m/s) ===')
abnormal = []
for i, d in enumerate(track_data):
    if abs(d['v']) > 25 or d['v'] < 0:
        abnormal.append((i, d))
        print(f'  索引{i}: Frame={d["frame"]}, t={d["t"]}ms, V={d["v"]:.1f}m/s, Az={d["az"]:.2f}°, R={d["r"]:.1f}m')

# 打印异常前后各5个点
if abnormal:
    first_bad_idx = abnormal[0][0]
    print(f'\n=== 第一个异常点前后10个点 ===')
    start = max(0, first_bad_idx - 10)
    end = min(len(track_data), first_bad_idx + 10)
    for i in range(start, end):
        d = track_data[i]
        mark = ' <-- 异常' if abs(d['v']) > 25 else ''
        print(f'  [{i}] Frame={d["frame"]}, t={d["t"]}ms, V={d["v"]:.1f}m/s, Az={d["az"]:.2f}°, R={d["r"]:.1f}m{mark}')

# 打印末尾20个点看看后期情况
print(f'\n=== 最后20个航迹点 ===')
for d in track_data[-20:]:
    mark = ' <-- 异常' if abs(d['v']) > 25 else ''
    print(f'  Frame={d["frame"]}, t={d["t"]}ms, V={d["v"]:.1f}m/s, Az={d["az"]:.2f}°, R={d["r"]:.1f}m{mark}')
