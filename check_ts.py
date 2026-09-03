import re

with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', encoding='utf-8') as f:
    text = f.read()

lines = text.splitlines()

# 按行提取时间戳 + 验证唯一性
ts_seen = {}
for line in lines:
    m = re.search(r'时间戳[:：](\d+)', line)
    if m:
        ts = int(m.group(1))
        if ts not in ts_seen:
            ts_seen[ts] = 0
        ts_seen[ts] += 1

frame_ts = sorted(ts_seen.keys())
print('Unique timestamps:', len(frame_ts))
print('First 10 ts:', frame_ts[:10])
print('First 10 intervals:', [frame_ts[i+1]-frame_ts[i] for i in range(9)])
print()

# 对比：frame_ts[50] 应该 = frame_ts[0] + 50*164
expected_50 = frame_ts[0] + 50 * 164
actual_50 = frame_ts[50]
print('Frame 0 : %d ms' % frame_ts[0])
print('Frame 50 expected: %d ms (t0 + 50*164)' % expected_50)
print('Frame 50 actual  : %d ms' % actual_50)
print('Match: %s' % (expected_50 == actual_50))

# 那生成的 data_entry.c 为什么不对？
# 因为我用 selected_ts = frame_ts[:1300]，但然后 idx=0 对应 selected_ts[0]
# 也就是 data[0].mSecond = frame_ts[0] = 898140
# data[50].mSecond = frame_ts[50] = 898140 + 50*164 = 906340

print()
print('selected_ts[0:3]:', frame_ts[:3])
print('selected_ts[48:52]:', frame_ts[48:52])

# 再确认生成的 data_entry.c
import os
c_new = open(r'd:\DSP\6678\track\track_1\data_entry.c', encoding='ascii').read()
pat = re.compile(r'data\[\s*(\d+)\s*\]\.mSecond\s*=\s*([-\d.]+)f')
ts_new = [(int(fi), float(ts)) for fi, ts in pat.findall(c_new)]
ts_new.sort()
print()
print('Generated Frame 0: mSecond=%.0f' % ts_new[0][1])
print('Generated Frame 1: mSecond=%.0f' % ts_new[1][1])
print('Generated Frame 49: mSecond=%.0f' % ts_new[49][1])
print('Generated Frame 50: mSecond=%.0f' % ts_new[50][1])
print('Expected Frame 50:  %.0f' % (frame_ts[0] + 50*164))
