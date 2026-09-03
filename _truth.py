# -*- coding: utf-8 -*-
import re
lines = open('无人机实测数据.txt', encoding='utf-8', errors='replace').read().split('\n')
pat = re.compile(r'波位号:(-?\d+).*?时间戳:(-?\d+).*?方位:(-?[\d.]+)\s+距离:(-?[\d.]+)\s+俯仰:(-?[\d.]+)\s+高度:(-?[\d.]+)\s+速度:(-?[\d.]+)')
pts = []
for l in lines:
    m = pat.search(l)
    if not m:
        continue
    beam = int(m.group(1)); t = int(m.group(2)); az = float(m.group(3))
    r = float(m.group(4)); el = float(m.group(5)); h = float(m.group(6)); v = float(m.group(7))
    pts.append((beam, t, az, r, el, h, v))
print('total parsed points:', len(pts))
drone = [p for p in pts if 40 <= p[5] <= 180 and 4 <= abs(p[6]) <= 26 and 500 <= p[3] <= 6000]
print('drone-like points:', len(drone))
print('=== Drone trajectory beam in 1..3 (first 60) ===')
cnt = 0
for p in drone:
    beam, t, az, r, el, h, v = p
    if beam in (1, 2, 3):
        print('beam=%2d t=%d az=%6.2f R=%7.1f el=%5.2f h=%6.1f V=%7.2f' % (beam, t, az, r, el, h, v))
        cnt += 1
        if cnt >= 60:
            break
b2 = sorted(drone, key=lambda x: x[1])
print('=== drone-like time span t=%d..%d  R %.0f..%.0f ===' % (b2[0][1], b2[-1][1], min(x[3] for x in b2), max(x[3] for x in b2)))
by_r = sorted(drone, key=lambda x: -x[3])
print('=== Max-R drone points (turnaround) ===')
for p in by_r[:10]:
    print('beam=%2d t=%d az=%6.2f R=%7.1f h=%6.1f V=%7.2f' % (p[0], p[1], p[2], p[3], p[5], p[6]))
