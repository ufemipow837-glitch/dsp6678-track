p = r'd:\DSP\6678\track\track_1\global_variable.c'
c = open(p, encoding='gbk', errors='ignore').read()

lines = c.splitlines()
new_lines = []
skip_next_pragma = False
for i, l in enumerate(lines):
    if skip_next_pragma and 'DATA_SECTION(target_data' in l:
        print(f'Removing duplicate pragma at L{i+1}: {l}')
        skip_next_pragma = False
        continue
    # 如果看到 pragma target_data，标记下一个不要加（如果接下来还有一个 pragma target_data）
    if 'DATA_SECTION(target_data' in l:
        # 看下一行是不是也是 pragma target_data
        if i+1 < len(lines) and 'DATA_SECTION(target_data' in lines[i+1]:
            skip_next_pragma = True
            print(f'Found double pragma at L{i+1}, will skip next one')
        # 也检查这行有没有分号
        if not l.rstrip().endswith(';'):
            print(f'Adding missing semicolon at L{i+1}: {l}')
            l = l.rstrip() + ';'
    new_lines.append(l)

open(p, 'w', encoding='gbk', errors='ignore').write('\n'.join(new_lines) + '\n')
print('\nDone. Verify:')
c2 = open(p, encoding='gbk', errors='ignore').read()
for i, l in enumerate(c2.splitlines(), 1):
    if 'target_data' in l or 'DATA_SECTION' in l:
        print(f'  L{i}: {l}')
