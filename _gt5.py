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

# 找建航后第1-5轮每帧beam=2的点 (beam2间隔=2788ms)
# 第1轮beam2=898796, 第2轮=901584(建航Frame021), 第3轮=904372
# 搜索t=904372±200ms的点
print('=== 搜索 t=904200~904600 内所有实测点 ===')
for p in pts:
    if 904000 <= p[1] <= 904700:
        print('beam=%-2d t=%d az=%.2f r=%.1f el=%.2f h=%.1f vr=%.2f' % p)

print('\n=== 搜索 t=907000~907500 内所有实测点 (第4轮beam2) ===')
for p in pts:
    if 906900 <= p[1] <= 907500:
        print('beam=%-2d t=%d az=%.2f r=%.1f el=%.2f h=%.1f vr=%.2f' % p)

print('\n=== 搜索 t=909700~910300 内所有实测点 (第5轮beam2) ===')
for p in pts:
    if 909600 <= p[1] <= 910300:
        print('beam=%-2d t=%d az=%.2f r=%.1f el=%.2f h=%.1f vr=%.2f' % p)
        
print('\n=== 搜索 t=912500~913100 内所有实测点 (第6轮beam2) ===')
for p in pts:
    if 912400 <= p[1] <= 913100:
        print('beam=%-2d t=%d az=%.2f r=%.1f el=%.2f h=%.1f vr=%.2f' % p)

print('\n=== 搜索 t=915300~915900 内所有实测点 (第7轮beam2) ===')
for p in pts:
    if 915200 <= p[1] <= 915900:
        print('beam=%-2d t=%d az=%.2f r=%.1f el=%.2f h=%.1f vr=%.2f' % p)
        
# 找所有beam=2且t>=901584的无人机点 (az<25)
print('\n=== 建航后所有beam=2无人机点 (az<=25, r<=5000) ===')
for p in pts:
    if p[0]==2 and p[1]>=901584 and p[2]<=25 and p[3]<=5000 and p[5]>=40:
        print('t=%d az=%.2f r=%.1f el=%.2f h=%.1f vr=%.2f' % (p[1],p[2],p[3],p[4],p[5],p[6]))
