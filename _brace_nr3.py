# -*- coding: utf-8 -*-
import re
lines = open('new_reliable.c', encoding='utf-8', errors='replace').read().split('\n')
depth = 0
keypoints = set([98,112,115,125,132,234,236,249,503,506,524,553,556,568,596,609,612,619,620,621,622,630,640,643,645,646,647,657,767,773,785,787])
for n, ln in enumerate(lines, 1):
    s = ln.split('//')[0]
    s = re.sub(r'"[^"]*"', '', s)
    s = re.sub(r"'[^']*'", '', s)
    opens = s.count('{')
    closes = s.count('}')
    before = depth
    depth += opens - closes
    if n in keypoints or 'while' in ln or 'reliable_reject_rollback' in ln:
        print('L%-4d before=%-2d after=%-2d  %s' % (n, before, depth, ln.strip()[:95]))
print('FINAL DEPTH =', depth)
