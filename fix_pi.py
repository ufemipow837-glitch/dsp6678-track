p = r'd:\DSP\6678\track\track_1\gen_data_entry.py'
c = open(p, encoding='utf-8').read()

# Replace 'pi' with literal '3.1415926535f'
old = '* pi;'
new = '* 3.1415926535f;'

count = c.count(old)
c = c.replace(old, new)

open(p, 'w', encoding='utf-8').write(c)
print('Replaced %d occurrences of "pi" with "3.1415926535f"' % count)

# Verify
c2 = open(p, encoding='utf-8').read()
print('Remaining "pi" occurrences:', c2.count('* pi;'))
