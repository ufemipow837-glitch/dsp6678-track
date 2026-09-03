import re, math

with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', encoding='utf-8') as f:
    text = f.read()

lines = text.splitlines()

CN_ts  = '\u65f6\u95f4\u6233'
CN_bw  = '\u6ce2\u4f4d\u53f7'
CN_az  = '\u65b9\u4f4d'
CN_r   = '\u8ddd\u79bb'
CN_el  = '\u4fef\u4ef0'
CN_v   = '\u901f\u5ea6'

frame_data = {}
skipped_lines = 0
missing_az = 0
missing_r = 0
invalid_r = 0

for line in lines:
    if not line.strip():
        continue
    ts_m = re.search(CN_ts + r'[:：]\s*(\d+)', line)
    if not ts_m:
        continue
    ts = int(ts_m.group(1))
    
    az = None
    r  = None
    el = None
    vr = None
    
    az_m = re.search(CN_az + r'[:：]\s*([-\d.eE+]+)', line)
    r_m  = re.search(CN_r  + r'[:：]\s*([-\d.eE+]+)', line)
    el_m = re.search(CN_el + r'[:：]\s*([-\d.eE+]+)', line)
    v_m  = re.search(CN_v  + r'[:：]\s*([-\d.eE+]+)', line)
    
    if az_m: az = float(az_m.group(1))
    else: missing_az += 1
    if r_m: r = float(r_m.group(1))
    else: missing_r += 1
    if el_m: el = float(el_m.group(1))
    if v_m: vr = float(v_m.group(1))
    
    if az is None or r is None:
        continue
    
    if not (50 < r < 30000):
        invalid_r += 1
        continue
    
    bw = int(re.search(CN_bw + r'[:：]\s*(\d+)', line).group(1)) if re.search(CN_bw + r'[:：]\s*(\d+)', line) else 0
    el_val = el if el is not None else 0.0
    vr_val = vr if vr is not None else 0.0
    
    frame_data.setdefault(ts, []).append(
        (bw, math.radians(az), r, math.radians(el_val), vr_val)
    )

frame_ts = sorted(frame_data.keys())
print('With data: %d frames' % len(frame_ts))
print('Total unique timestamps in file: 4385')
print('Skipped lines (no az/r): missing_az=%d, missing_r=%d, invalid_r=%d' % (missing_az, missing_r, invalid_r))
print()

# 对比前 60 个时间戳
print('First 60 timestamps comparison:')
with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', encoding='utf-8') as f:
    all_ts = sorted(set(int(x) for x in re.findall(CN_ts + r'[:：](\d+)', f.read())))

with_data_set = set(frame_ts[:1300])
full_set = set(all_ts[:1300])
missing = full_set - with_data_set
extra = with_data_set - full_set
print('Timestamps in file but NOT with data:', sorted(missing))
print('Timestamps with data but NOT in first 1300 file timestamps:', sorted(extra))

# 所以我们选的 selected_ts 不是连续的
# 应该直接用 all_ts[:1300] 作为帧索引
# 如果某个时间戳没数据，就让那个帧 targetNum = 0
print()
print('=== Solution: use all_ts[:1300] as frame basis ===')
print('Then data[idx] corresponds to all_ts[idx], with data from frame_data.get(all_ts[idx], [])')
