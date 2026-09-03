# -*- coding: utf-8 -*-
import io, os
cands=[]
for f in os.listdir("."):
    if f.endswith(".txt"):
        try:
            n=sum(1 for _ in io.open(f,encoding="gbk",errors="ignore"))
            cands.append((n,f))
        except: pass
cands.sort(reverse=True)
fname=cands[0][1]
rows=[]
with io.open(fname,encoding="gbk",errors="ignore") as fh:
    for ln in fh:
        p=ln.split()
        if len(p)<10: continue
        def v(t): return t.split(u":")[-1]
        try:
            beam=int(v(p[1])); t=int(v(p[3])); az=float(v(p[4])); r=float(v(p[5]))
            el=float(v(p[6])); h=float(v(p[7])); vv=float(v(p[8])); snr=int(float(v(p[9])))
        except: continue
        rows.append((t,beam,az,r,el,h,vv,snr))
# early window: first 60 frames => t < 898140+60*164
T0=898140; DT=164
print("=== ALL points beam1-4, az 8..22, r 700..5000, first 140 frames ===")
for (t,beam,az,r,el,h,vv,snr) in rows:
    fr=(t-T0)/DT
    if fr>140: break
    if beam in (1,2,3,4) and 8<=az<=22 and 700<=r<=5000:
        print("fr=%5.1f t=%d bm=%d az=%.2f r=%.1f el=%.2f h=%.1f v=%.2f snr=%d"%(fr,t,beam,az,r,el,h,vv,snr))
