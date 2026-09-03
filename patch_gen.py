"""Patch gen_data_entry.py to output angle expressions instead of radians."""
p = r'd:\DSP\6678\track\track_1\gen_data_entry.py'
c = open(p, encoding='utf-8').read()

# Change azi output from:
#   data[NNNN].azi[NN] = X.XXXXXXf;
# to:
#   data[NNNN].azi[NN] = X.XXXXf / 180.0f * pi;
old_azi = "out.append('    data[{:4d}].azi[{:2d}] = {:10.6f}f;'.format(idx, pj, azi_r))"
new_azi = "out.append('    data[{:4d}].azi[{:2d}] = {:10.4f}f / 180.0f * pi;'.format(idx, pj, math.degrees(azi_r)))"

old_ele = "out.append('    data[{:4d}].ele[{:2d}] = {:10.6f}f;'.format(idx, pj, ele_r))"
new_ele = "out.append('    data[{:4d}].ele[{:2d}] = {:10.4f}f / 180.0f * pi;'.format(idx, pj, math.degrees(ele_r)))"

if old_azi in c:
    c = c.replace(old_azi, new_azi, 1)
    print('OK: azi -> degrees/180*pi')
else:
    print('FAIL: azi pattern not found')
    for i, l in enumerate(c.splitlines(), 1):
        if 'azi[' in l and 'format(idx' in l:
            print('  L%d: %s' % (i, l.strip()))

if old_ele in c:
    c = c.replace(old_ele, new_ele, 1)
    print('OK: ele -> degrees/180*pi')
else:
    print('FAIL: ele pattern not found')
    for i, l in enumerate(c.splitlines(), 1):
        if 'ele[' in l and 'format(idx' in l:
            print('  L%d: %s' % (i, l.strip()))

open(p, 'w', encoding='utf-8').write(c)
print('gen_data_entry.py updated successfully!')
