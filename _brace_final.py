# -*- coding: utf-8 -*-
import re, sys

def strip_code(src):
    out = []
    i = 0
    n = len(src)
    while i < n:
        c = src[i]
        if c == '/' and i+1 < n and src[i+1] == '/':
            while i < n and src[i] != '\n':
                i += 1
            continue
        if c == '/' and i+1 < n and src[i+1] == '*':
            i += 2
            while i+1 < n and not (src[i] == '*' and src[i+1] == '/'):
                i += 1
            i += 2
            continue
        if c == '"':
            i += 1
            while i < n and src[i] != '"':
                if src[i] == '\\':
                    i += 2
                    continue
                i += 1
            i += 1
            continue
        if c == "'":
            i += 1
            while i < n and src[i] != "'":
                if src[i] == '\\':
                    i += 2
                    continue
                i += 1
            i += 1
            continue
        out.append(c)
        i += 1
    return ''.join(out)

for fn in sys.argv[1:]:
    src = open(fn, encoding='utf-8').read()
    code = strip_code(src)
    lines = code.split('\n')
    depth = 0
    print('==== %s ====' % fn)
    for ln, line in enumerate(lines, 1):
        before = depth
        depth += line.count('{') - line.count('}')
        if depth < 0:
            print('  NEGATIVE at L%d: %s' % (ln, line.strip()[:80]))
        if re.search(r'while\(candidates_left|track_asso_num\) > 0|if\(best_idx >= 0|if\(final_ok\)|reliable_reject_rollback|writeback_start|after_while', line):
            print('  L%-4d depth %2d->%2d : %s' % (ln, before, depth, line.strip()[:85]))
    print('  FINAL depth:', depth)
