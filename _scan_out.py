# -*- coding: utf-8 -*-
import re
lines = open('输出数据.txt', encoding='utf-8', errors='replace').read().split('\n')
cur_frame = ''
for i, ln in enumerate(lines):
    m = re.match(r'Frame (\d+) \| t=(\d+)ms \| n=(\d+) \| Cycle=([\d.]+)ms \| beam=(\d+)', ln)
    if m:
        cur_frame = 'F%s t=%s n=%s cyc=%s beam=%s' % m.groups()
        continue
    if 'Reliable Tracks:' in ln:
        print(cur_frame, '||', ln.strip())
    if '-> Track[' in ln:
        print('     ', ln.strip())
    if 'found_any=1' in ln:
        print('     !! found_any=1 in', cur_frame, '::', ln.strip())
