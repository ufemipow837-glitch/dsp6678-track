# -*- coding: utf-8 -*-
import re, io
ls = io.open('无人机实测数据.txt', encoding='utf-8', errors='replace').readlines()
pat = re.compile(r'波位号:(\S+)\s+北斗时间戳:(\d+)\s+时间戳:(\d+)\s+方位:(\S+)\s+距离:(\S+)\s+俯仰:(\S+)\s+高度:(\S+)\s+速度:(\S+)')
pts = []
for l in ls:
    m = pat.search(l)
    if m:
        beam=int(float(m.group(1))); t=int(m.group(3)); az=float(m.group(4)); r=float(m.group(5))
        el=float(m.group(6)); h=float(m.group(7)); v=float(m.group(8))
        pts.append((beam,t,az,r,el,h,v))
print('total points:', len(pts))
# drone: az in [10,25], h in [40,200], r<8000
drone = [p for p in pts if 10<=p[2]<=25 and 40<=p[5]<=200 and p[3]<8000]
print('drone-like points:', len(drone))
for p in drone[:40]:
    print('beam=%-2d t=%d az=%.2f r=%.1f el=%.2f h=%.1f vr=%.2f' % p)
