# -*- coding: utf-8 -*-
import re
path = r'd:\DSP\6678\track\track_1\new_reliable.c'
lines = open(path, encoding='utf-8').read().split('\n')
depth = 0
for ln, s in enumerate(lines, 1):
    s2 = re.sub(r'//.*', '', s)
    s2 = re.sub(r'"[^"]*"', '""', s2)
    s2 = re.sub(r"'[^']*'", "''", s2)
    opens = s2.count('{')
    closes = s2.count('}')
    if opens or closes:
        note = ''
        if 'while' in s2: note += ' <WHILE>'
        if re.search(r'\bif\b', s2): note += ' [if]'
        if re.search(r'\bfor\b', s2): note += ' [for]'
        if 'goto' in s2: note += ' [GOTO]'
        if re.search(r'^\s*[A-Za-z_]\w*\s*:', s2): note += ' [LABEL]'
        print('L%4d d=%2d +%d-%d%s | %s' % (ln, depth, opens, closes, note, s.strip()[:70]))
        depth += opens - closes
print('FINAL DEPTH =', depth)
