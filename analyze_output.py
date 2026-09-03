# -*- coding: utf-8 -*-
import re, math
from collections import defaultdict, Counter

path = r'd:\DSP\6678\track\track_1\输出数据.txt'
with open(path, 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

fr_pat = re.compile(r'Frame (\d+).*?beam=(\d+)')
rel_pat = re.compile(r'Reliable Tracks:\s*(\d+)')
tr_pat = re.compile(r'Track\[(\d+)\]:\s*Az=\s*([-\d.]+)deg\s+El=\s*([-\d.]+)deg\s+R=\s*([-\d.]+)m\s+V=\s*([-\d.]+)m/s\s*\(type=(\d+)\s+batch=(\d+)\)')

batches = defaultdict(lambda: {'frames':[], 'az':[], 'el':[], 'r':[], 'v':[], 'type':[], 'beam':[]})
rel_frames = []
cur_frame = -1
cur_beam = -1
rel_counts = []

for ln in lines:
    m = fr_pat.search(ln)
    if m:
        cur_frame = int(m.group(1))
        cur_beam = int(m.group(2))
    m = rel_pat.search(ln)
    if m:
        rn = int(m.group(1))
        rel_counts.append(rn)
        if rn > 0:
            rel_frames.append(cur_frame)
    m = tr_pat.search(ln)
    if m:
        az=float(m.group(2)); el=float(m.group(3)); r=float(m.group(4))
        v=float(m.group(5)); ttype=int(m.group(6)); batch=int(m.group(7))
        b = batches[batch]
        b['frames'].append(cur_frame); b['az'].append(az); b['el'].append(el)
        b['r'].append(r); b['v'].append(v); b['type'].append(ttype); b['beam'].append(cur_beam)

print(f'总帧数(可靠记录): {len(rel_counts)}')
print(f'有可靠航迹的帧数: {len(rel_frames)}')
if rel_frames:
    rf_set = set(rel_frames)
    mn, mx = min(rel_frames), max(rel_frames)
    print(f'  可靠帧范围: {mn} ~ {mx}  覆盖率={100*len(rel_frames)/(mx-mn+1):.1f}%')
    misses = [f for f in range(mn, mx+1) if f not in rf_set]
    print(f'  中间丢失帧数: {len(misses)}  前30丢失帧: {misses[:30]}')
print()
print('===== 航迹批详情 =====')
for batch in sorted(batches.keys()):
    b = batches[batch]
    n = len(b['frames'])
    if n == 0: continue
    f1, fl = b['frames'][0], b['frames'][-1]
    lifespan = fl - f1 + 1
    tc = Counter(b['type'])
    type_str = ' '.join(f't{k}x{v}' for k,v in sorted(tc.items()))
    bc = Counter(b['beam'])
    beam_str = ' '.join(f'b{k}x{v}' for k,v in sorted(bc.items(), key=lambda x:-x[1])[:6])
    avg_az = sum(b['az'])/n; avg_el = sum(b['el'])/n
    r_min, r_max = min(b['r']), max(b['r'])
    v_avg = sum(b['v'])/n; v_maxabs = max(abs(v) for v in b['v'])
    hts = [r*math.sin(math.radians(el)) for r,el in zip(b['r'], b['el'])]
    h_min, h_max, h_avg = min(hts), max(hts), sum(hts)/n
    if v_maxabs < 30 and 40 < h_avg < 160 and r_max < 7000:
        tag = '★无人机候选'
    elif v_maxabs > 80:
        tag = '♦炮弹类'
    elif r_min > 7000 or h_avg < 20 or h_avg > 5000:
        tag = '▲杂波'
    elif v_maxabs < 80:
        tag = '○其他低速'
    else:
        tag = '?'
    life_warn = '  ⚠硬TTL死!' if lifespan <= 12 else ''
    print(f'批{batch:>3} {tag}: {n:>3}帧 f{f1:>3}→{fl:>3} 寿命{lifespan:>3}  类型:{type_str}  {beam_str}{life_warn}')
    print(f'       Az≈{avg_az:>6.1f}° El≈{avg_el:>5.1f}° R=[{r_min:>6.0f},{r_max:>6.0f}] |V|max={v_maxabs:>5.1f} avgV={v_avg:>5.1f}  H≈{h_avg:>5.0f}m[{h_min:>4.0f},{h_max:>4.0f}]')
print()
print(f'每帧可靠数分布: {dict(sorted(Counter(rel_counts).items()))}')
print(f'同时最大可靠: {max(rel_counts) if rel_counts else 0}')
