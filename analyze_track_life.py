import re

# 分析输出数据.txt中每帧的Reliable Tracks数量
with open(r'd:\DSP\6678\track\track_1\输出数据.txt', 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

results = []
current_frame = -1
current_t = -1
rel_num = -1
tracks = []

for line in lines:
    fm = re.search(r'Frame\s+(\d+)\s+\|\s+t=(\d+)ms', line)
    if fm:
        if current_frame >= 0:
            results.append({
                'frame': current_frame, 't': current_t,
                'rel': rel_num, 'tracks': tracks
            })
        current_frame = int(fm.group(1))
        current_t = int(fm.group(2))
        rel_num = 0
        tracks = []
    
    rm = re.search(r'Reliable Tracks:\s*(\d+)', line)
    if rm:
        rel_num = int(rm.group(1))
    
    tm = re.search(r'Track\[0\]:\s*Az=([-\d.]+)deg\s+El=([-\d.]+)deg\s+R=([-\d.]+)m\s+V=([-\d.]+)m/s', line)
    if tm:
        tracks.append({
            'az': float(tm.group(1)), 'el': float(tm.group(2)),
            'r': float(tm.group(3)), 'v': float(tm.group(4))
        })

if current_frame >= 0:
    results.append({
        'frame': current_frame, 't': current_t,
        'rel': rel_num, 'tracks': tracks
    })

print(f'总帧数: {len(results)}')

# 找出reliable_track_num变化的地方
print('\n=== 航迹状态变化 (Frame 250-320) ===')
prev_rel = -1
for r in results:
    if 250 <= r['frame'] <= 320:
        v_str = ''
        if r['tracks']:
            t = r['tracks'][0]
            v_str = f"V={t['v']:.1f}, Az={t['az']:.1f}°, R={t['r']:.0f}m"
        rel_change = ' <-- CHANGED' if r['rel'] != prev_rel else ''
        print(f"Frame {r['frame']:3d}: rel={r['rel']} {v_str}{rel_change}")
        prev_rel = r['rel']

# 找出航迹丢失和重新起始的点
print('\n=== 航迹丢失/重建事件 ===')
prev_rel = 0
for i, r in enumerate(results):
    if r['rel'] > prev_rel:
        print(f"Frame {r['frame']}: 航迹起始/恢复 (rel {prev_rel}->{r['rel']})")
    elif r['rel'] < prev_rel:
        print(f"Frame {r['frame']}: 航迹丢失/删除 (rel {prev_rel}->{r['rel']})")
    prev_rel = r['rel']
