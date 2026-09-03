# -*- coding: utf-8 -*-
import re
lines = open("new_reliable.c", encoding="utf-8", errors="replace").read().split("\n")
depth = 0
stack = []
pairs = []
for n, ln in enumerate(lines, 1):
    s = re.sub(r'"[^"]*"', "", ln)
    s = re.sub(r"//.*", "", s)
    for ch in s:
        if ch == "{":
            stack.append((n, depth, ln.strip()[:70]))
            depth += 1
        elif ch == "}":
            depth -= 1
            if stack:
                op = stack.pop()
                pairs.append((op[0], n, op[1], op[2]))
for op_n, cl_n, op_d, txt in pairs:
    if op_n in (112,132,137,152,158,172,217,236,249,483,502,552,608,626,630,639) or (615 <= cl_n <= 650) or (615 <= op_n <= 650):
        print("L%4d (d%d) %-55s <==> L%4d" % (op_n, op_d, txt[:55], cl_n))
print("---- live depth 600-650 ----")
depth = 0
for n, ln in enumerate(lines, 1):
    s = re.sub(r'"[^"]*"', "", ln)
    s = re.sub(r"//.*", "", s)
    o = s.count("{"); c = s.count("}")
    if 600 <= n <= 650:
        print("L%4d d%2d->%2d  %s" % (n, depth, depth+o-c, ln.rstrip()[:95]))
    depth += o - c
