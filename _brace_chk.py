import re, sys
fn = sys.argv[1] if len(sys.argv) > 1 else 'new_reliable.c'
lines = open(fn, encoding='utf-8', errors='replace').read().split('\n')
depth = 0
out = []
for n, l in enumerate(lines, 1):
    s = re.sub(r'//.*', '', l)
    s = re.sub(r'"[^"]*"', '""', s)
    s = re.sub(r'/\*.*?\*/', '', s)
    for ch in s:
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
    tag = ''
    if 'while' in l and 'candidates_left' in l: tag = 'WHILE'
    if 'end_of_scan' in l: tag = 'end_of_scan'
    if 'if(!found_any)' in l: tag = 'if(!found_any)break'
    if 'reliable_reject_rollback' in l and ':' in l and 'goto' not in l: tag = 'LABEL'
    if 'after_while' in l and 'printf' in l: tag = 'after_while'
    if 'writeback_start' in l: tag = 'writeback_start'
    if 'BUG21' in l: tag = 'BUG21-close'
    if 'void new_reliable' in l: tag = 'FUNC-OPEN'
    if depth < 0: tag += '  <<< NEGATIVE'
    if tag:
        out.append('L%-4d depth=%-3d %s | %s' % (n, depth, tag, l.strip()[:70]))
print('FINAL DEPTH =', depth)
print('\n'.join(out))
