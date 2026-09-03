# -*- coding: utf-8 -*-
import re
lines = open('输出数据.txt', encoding='utf-8', errors='replace').read().split('\n')
cur = ''
out = []
for ln in lines:
    m = re.match(r'Frame (\d+) \| t=(\d+)ms \| n=(\d+) \| Cycle=([\d.]+)ms \| beam=(\d+)', ln)
    if m:
        cur = 'F%s t=%s n=%s beam=%s' % (m.group(1), m.group(2), m.group(3), m.group(5))
        continue
    if 'Reliable Tracks:' in ln:
        mm = re.search(r'Reliable Tracks: (\d+)\s+\[Sn=(-?\d+) init_cand=(-?\d+) temp_before=(-?\d+) temp_after=(-?\d+)', ln)
        if mm:
            out.append('%s | rel=%s Sn=%s cand=%s tB=%s tA=%s' % (cur, mm.group(1), mm.group(2), mm.group(3), mm.group(4), mm.group(5)))
    if '-> Track[' in ln:
        out.append('      ' + ln.strip())
    if 'wrote_back=' in ln:
        mm = re.search(r'wrote_back=(\d+) temp_after=(\d+)', ln)
        if mm and (int(mm.group(1)) != 1 or True):
            out.append('      wb=%s tA=%s' % (mm.group(1), mm.group(2)))
# print only frame summary lines + track lines
for o in out:
    if not o.startswith('      wb='):
        print(o)
