import re
from collections import defaultdict

frames = []
cur_frame = None
with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', 'r', encoding='utf-8') as f:
    for line in f:
        m = re.match(r'点迹解析数据： 波位号:(\d+) .*?时间戳:(\d+) .*?方位:([-\d.]+) 距离:([-\d.]+) .*?俯仰:([-\d.]+) 高度:([-\d.]+) 速度:([-\d.]+)', line)
        if m:
            bn = int(m.group(1))
            ts = int(m.group(2))
            az = float(m.group(3))
            r = float(m.group(4))
            h = float(m.group(6))
            v = float(m.group(7))
            if cur_frame is None or cur_frame['ts'] != ts:
                cur_frame = {'ts': ts, 'beams': defaultdict(list), 'drone_points': []}
                frames.append(cur_frame)
            cur_frame['beams'][bn].append({'az': az, 'r': r, 'h': h, 'v': v})
            if abs(v) > 7 and abs(v) < 18 and h > 50 and h < 200 and r > 800 and r < 5000:
                cur_frame['drone_points'].append({'bn': bn, 'az': az, 'r': r, 'v': v, 'h': h})

print(f"总帧数: {len(frames)}")
t0 = frames[0]['ts']
t1 = frames[-1]['ts']
print(f"时间范围: {t0} ~ {t1} ({(t1-t0)/1000:.1f}s)")

print("\n=== 每帧概览 (t=0~180s) ===")
for i, fr in enumerate(frames):
    t = (fr['ts'] - t0) / 1000.0
    if t > 180:
        break
    drones = [p for p in fr['drone_points'] if (p['v'] < -7 and p['v'] > -18) or (p['v'] > 7 and p['v'] < 18)]
    beams_hit = sorted(fr['beams'].keys())
    main_bn = max(fr['beams'].keys(), key=lambda b: len(fr['beams'][b])) if fr['beams'] else -1
    tag = ''
    if drones:
        bns = [p['bn'] for p in drones]
        vrs = [round(p['v'], 1) for p in drones]
        rs = [round(p['r'], 0) for p in drones]
        tag = f'<<DRONE>> BW={bns} Vr={vrs} R={rs}'
    if tag or t < 15 or (i % 20 == 0):
        print(f'  Frame {i:3d} t={t:6.1f}s ts={fr["ts"]} beams={beams_hit} main={main_bn} {tag}')

print("\n=== 无人机出现连续段 ===")
segments = []
seg_start = None
prev_has = False
for i, fr in enumerate(frames):
    drones = [p for p in fr['drone_points'] if (p['v'] < -7 and p['v'] > -18) or (p['v'] > 7 and p['v'] < 18)]
    has = len(drones) > 0
    if has and not prev_has:
        seg_start = i
    elif not has and prev_has and seg_start is not None:
        segments.append((seg_start, i-1))
        seg_start = None
    prev_has = has
if seg_start is not None:
    segments.append((seg_start, len(frames)-1))

for idx, (s, e) in enumerate(segments):
    ts_s = (frames[s]['ts'] - t0) / 1000.0
    ts_e = (frames[e]['ts'] - t0) / 1000.0
    gap_b = (frames[s]['ts'] - frames[s-1]['ts']) / 1000.0 if s > 0 else 0
    gap_a = (frames[e+1]['ts'] - frames[e]['ts']) / 1000.0 if e+1 < len(frames) else 0
    durs = []
    for k in range(s, e+1):
        for p in frames[k]['drone_points']:
            if abs(p['v']) > 7 and abs(p['v']) < 18:
                durs.append(p['v'])
    avg_vr = sum(durs)/len(durs) if durs else 0
    direction = '远离(Vr<0)' if avg_vr < 0 else '靠近(Vr>0)'
    print(f'  段{idx+1}: Frame {s:3d}~{e:3d} | t={ts_s:5.1f}s~{ts_e:5.1f}s | 持续 {ts_e-ts_s:.1f}s | 前间隙 {gap_b:.1f}s 后间隙 {gap_a:.1f}s | {direction} avgVr={avg_vr:.1f}m/s')

print("\n=== DSP 输出解析 ===")
dsp_frames = []
with open(r'd:\DSP\6678\track\track_1\输出数据.txt', 'r', encoding='utf-8') as f:
    content = f.read()

frame_blocks = re.split(r'Frame (\d+) \| t=(\d+)ms', content)
i = 1
while i < len(frame_blocks):
    fnum = int(frame_blocks[i])
    fts = int(frame_blocks[i+1])
    rest = frame_blocks[i+2] if i+2 < len(frame_blocks) else ''
    n_reliable = 0
    m = re.search(r'Reliable Tracks:\s*(\d+)', rest)
    if m:
        n_reliable = int(m.group(1))
    tracks = []
    for tm in re.finditer(r'Track\[(\d+)\]:\s*Az=([\-\d.]+)deg\s+El=([\-\d.]+)deg\s+R=([\-\d.]+)m\s+V=([\-\d.]+)m/s', rest):
        tracks.append({'idx': int(tm.group(1)), 'az': float(tm.group(2)), 'r': float(tm.group(4)), 'v': float(tm.group(5))})
    dsp_frames.append({'fnum': fnum, 'ts': fts, 'n_reliable': n_reliable, 'tracks': tracks})
    i += 3

t0_dsp = dsp_frames[0]['ts'] if dsp_frames else 0
print(f'DSP 总帧: {len(dsp_frames)} | 时间 {t0_dsp}~{dsp_frames[-1]["ts"]} ({(dsp_frames[-1]["ts"]-t0_dsp)/1000:.1f}s)')

first_track = None
for df in dsp_frames:
    if df['n_reliable'] > 0:
        t = (df['ts']-t0_dsp)/1000.0
        print(f'首次出可靠航迹: Frame {df["fnum"]} t={t:.1f}s n_tracks={df["n_reliable"]} {df["tracks"]}')
        first_track = df
        break

print("\n=== DSP 航迹跳变检测 ===")
prev_tracks = {}
for df in dsp_frames:
    t = (df['ts'] - t0_dsp) / 1000.0
    for tk in df['tracks']:
        key = tk['idx']
        if key in prev_tracks:
            prev = prev_tracks[key]
            dr = abs(tk['r'] - prev['r'])
            daz = abs(tk['az'] - prev['az'])
            if dr > 500 or daz > 10:
                print(f'  跳变! Frame {df["fnum"]} t={t:.1f}s Track[{key}]: Az {prev["az"]:.1f}->{tk["az"]:.1f} R {prev["r"]:.0f}->{tk["r"]:.0f} dR={dr:.0f} dAz={daz:.1f}')
        prev_tracks[key] = tk.copy()
