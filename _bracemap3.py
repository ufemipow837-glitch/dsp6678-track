# -*- coding: utf-8 -*-
import re
path = r'd:\DSP\6678\track\track_1\new_reliable.c'
lines = open(path, encoding='utf-8', errors='replace').read().split('\n')
depth = 0
stack = []
pairs = []
for ln, raw in enumerate(lines, 1):
    s = re.sub(r'//.*', '', raw)
    s = re.sub(r'"(\\.|[^"\\])*"', '""', s)
    s = re.sub(r"'(\\.|[^'\\])*'", "''", s)
    for ch in s:
        if ch == '{':
            stack.append((ln, depth)); depth += 1
        elif ch == '}':
            depth -= 1
            op_ln, op_dep = stack.pop()
            pairs.append((op_ln, op_dep, ln))
print('final depth:', depth)
watch_open = set([98,112,132,137,152,158,172,176,217,236,249,281,391,432,445,483,502,520,552,592,608,626,639,653,700,721,754,758,760])
watch_close = set([615,618,636,641,642,643,794,796])
for op_ln, op_dep, cl_ln in pairs:
    if op_ln in watch_open or cl_ln in watch_close:
        text = lines[op_ln-1].strip()[:60]
        print('L%-4d (d%d) {  ->  } L%-4d   | %s' % (op_ln, op_dep, cl_ln, text))
