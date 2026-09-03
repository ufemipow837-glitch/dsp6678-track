# -*- coding: utf-8 -*-
import re
src = open('new_reliable.c', encoding='utf-8').read()
lines = src.split('\n')
depth = 0
in_block_comment = False
for ln, line in enumerate(lines, 1):
    i = 0
    n = len(line)
    line_depth_before = depth
    while i < n:
        c = line[i]
        if in_block_comment:
            if c == '*' and i+1 < n and line[i+1] == '/':
                in_block_comment = False
                i += 2
                continue
            i += 1
            continue
        if c == '/' and i+1 < n and line[i+1] == '*':
            in_block_comment = True
            i += 2
            continue
        if c == '/' and i+1 < n and line[i+1] == '/':
            break
        if c == '"':
            i += 1
            while i < n and line[i] != '"':
                if line[i] == '\\': i += 2; continue
                i += 1
            i += 1
            continue
        if c == "'":
            i += 1
            while i < n and line[i] != "'":
                if line[i] == '\\': i += 2; continue
                i += 1
            i += 1
            continue
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
        i += 1
    # print landmarks and every line between 230 and 260, 495-650
    s = line.strip()
    if (re.search(r'(while\(candidates_left|track_asso_num\) > 0|if\(best_idx|if\(final_ok|reliable_reject_rollback|writeback_start|after_while|end while|END while|for\(i = 0; i < num_candidates)', s)
            or (245 <= ln <= 255) or (498 <= ln <= 510) or (615 <= ln <= 650)):
        print('L%-4d d%2d->%2d : %s' % (ln, line_depth_before, depth, s[:88]))
print('FINAL depth:', depth)
