# -*- coding: utf-8 -*-
import re
src = open('new_reliable.c', encoding='utf-8').read()
lines = src.split('\n')
depth = 0
stack = []
for ln, line in enumerate(lines, 1):
    s = re.sub(r'"[^"]*"', '', line)
    s = re.sub(r'//.*', '', s)
    opens = s.count('{')
    closes = s.count('}')
    tag = ''
    if 'while(candidates_left' in s: tag = 'WHILE-OPEN'
    if 'if((*track_asso_num) > 0' in s: tag = 'IF-GUARD'
    if 'for(i = 0; i < num_candidates' in s: tag = 'FOR-SCAN'
    if 'reliable_reject_rollback:' in s: tag = 'LABEL'
    if 'best_idx >= 0 &&' in s: tag = 'IF-BEST'
    if opens or closes or tag:
        print('L%4d d=%2d +%d -%d %-12s %s' % (ln, depth, opens, closes, tag, line.strip()[:70]))
    for _ in range(opens): stack.append(ln)
    for _ in range(closes):
        if stack: stack.pop()
    depth += opens - closes
print('FINAL depth =', depth, ' unclosed:', stack)
