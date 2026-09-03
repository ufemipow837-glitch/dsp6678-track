import re
from collections import Counter

with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', encoding='utf-8') as f:
    text = f.read()

# 提取所有时间戳
ts_all = sorted(set(int(x) for x in re.findall(r'时间戳[:：](\d+)', text)))
print('Total timestamps:', len(ts_all))
print('Range: %d ~ %d ms' % (ts_all[0], ts_all[-1]))

# Frame intervals
intervals = [ts_all[i+1]-ts_all[i] for i in range(len(ts_all)-1)]
print()
print('First 20 intervals:', intervals[:20])
print()
print('Interval distribution:')
dist = Counter(intervals)
for k, v in sorted(dist.items()):
    print('  %dms: %d times' % (k, v))

print()
print('Mean interval: %.1f ms' % (sum(intervals)/len(intervals)))
print('Backup fixed interval: 164ms')
