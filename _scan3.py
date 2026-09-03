# -*- coding: utf-8 -*-
import re
lines = open('输出数据.txt', encoding='utf-8', errors='replace').read().split('\n')
capture = False
buf = []
for ln in lines:
    m = re.match(r'Frame (\d+)', ln)
    if m:
        f = int(m.group(1))
        capture = (18 <= f <= 23)
    if capture:
        buf.append(ln)
print('\n'.join(buf[:200]))
