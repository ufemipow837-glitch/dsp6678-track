# -*- coding: utf-8 -*-
import io, re

def check(path):
    s = io.open(path, 'r', encoding='utf-8', errors='replace').read()
    # strip comments and strings crudely for brace count
    t = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
    t = re.sub(r'//[^\n]*', '', t)
    t = re.sub(r'"(\\.|[^"\\])*"', '""', t)
    t = re.sub(r"'(\\.|[^'\\])*'", "''", t)
    bal = 0; line_bal = {}
    for ln, line in enumerate(t.split('\n'), 1):
        bal += line.count('{') - line.count('}')
        line_bal[ln] = bal
    print('%s: brace_balance=%d  imm_miss=%d  FALLBACK=%d' % (
        path.split('\\')[-1], bal, s.count('imm_miss'), s.count('FALLBACK')))
    if bal != 0:
        # print lines where balance goes negative or near end
        for ln in sorted(line_bal):
            if line_bal[ln] < 0:
                print('  NEGATIVE at line', ln)
    return bal

for p in [r'd:\DSP\6678\track\track_1\track_asso.c',
          r'd:\DSP\6678\track\track_1\new_reliable.c',
          r'd:\DSP\6678\track\track_1\track_initial.c',
          r'd:\DSP\6678\track\track_1\track_predict.c']:
    check(p)
