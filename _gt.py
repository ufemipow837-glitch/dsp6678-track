# -*- coding: utf-8 -*-
import os, io
# pick the largest .txt by line count = drone measured file
cands = []
for f in os.listdir(u"." if False else "."):
    if f.endswith(".txt"):
        try:
            n = sum(1 for _ in io.open(f, encoding="gbk", errors="ignore"))
            cands.append((n, f))
        except Exception:
            pass
cands.sort(reverse=True)
fname = cands[0][1]
import sys
DRONE = []
ALL = []
with io.open(fname, encoding="gbk", errors="ignore") as fh:
    for ln in fh:
        p = ln.split()
        if len(p) < 10:
            continue
        def val(tok):
            return tok.split(u":")[-1]
        try:
            beam = int(val(p[1])); t = int(val(p[3])); az = float(val(p[4]))
            r = float(val(p[5])); el = float(val(p[6])); h = float(val(p[7]))
            v = float(val(p[8])); snr = int(float(val(p[9])))
        except Exception:
            continue
        ALL.append((t, beam, az, r, el, h, v, snr))
        if snr >= 30 and abs(abs(v)-11.74) < 2.5 and 60 < h < 160:
            DRONE.append((t, beam, az, r, el, h, v, snr))
print("file:", fname.encode("ascii","replace").decode())
print("total points:", len(ALL), " drone-like:", len(DRONE))
from collections import Counter
print("beam dist:", dict(Counter(d[1] for d in DRONE)))
rs=[d[3] for d in DRONE]; ts=[d[0] for d in DRONE]
print("R min/max: %.1f / %.1f" % (min(rs), max(rs)))
print("t: %d..%d span=%.1fs" % (ts[0], ts[-1], (ts[-1]-ts[0])/1000.0))
print("=== drone dwells (t beam az r el h v snr) ===")
for d in DRONE:
    print("%d %d %.2f %.1f %.2f %.1f %.2f %d" % d)
