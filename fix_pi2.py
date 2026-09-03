p = r'd:\DSP\6678\track\track_1\gen_data_entry.py'
c = open(p, encoding='utf-8').read()

# 1. Change back from literal 3.1415926535f to pi in expressions
c = c.replace('f / 180.0f * 3.1415926535f;', 'f / 180.0f * pi;')

# 2. Add #define pi at the top of the generated C file
# Find the "#include \"struct.h\"" line and add after it
old_include = '#include "struct.h"'
new_include = '#include "struct.h"\n\n#ifndef pi\n#define pi 3.1415926535f\n#endif'

if old_include in c:
    # We need to modify the OUTPUT strings, not the Python code itself
    # Actually, let's change what gets written to data_entry.c
    # The header generation part uses out.append() calls
    pass

# The #define needs to be in the generated C file output, not in gen_data_entry.py
# Let's modify the header section in gen_data_entry.py
# Find: out.append('#include "struct.h"')
old_header = "out.append('#include \"struct.h\"')"
new_header = """out.append('#include "struct.h"')
out.append('')
out.append('#ifndef pi')
out.append('#define pi 3.1415926535f')
out.append('#endif')"""

if old_header in c:
    c = c.replace(old_header, new_header, 1)
    print('OK: added #define pi to generated header')
else:
    print('FAIL: header pattern not found')

open(p, 'w', encoding='utf-8').write(c)
print('gen_data_entry.py updated!')

# Verify
c2 = open(p, encoding='utf-8').read()
print()
print('Checking:')
print('  pi occurrences in output strings:', c2.count('* pi;'))
print('  3.1415926535f occurrences (should be 1 in define):', c2.count('3.1415926535'))
