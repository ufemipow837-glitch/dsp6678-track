p = r'd:\DSP\6678\track\track_1\main.c'
c = open(p, encoding='gbk', errors='ignore').read()

# Find the line with printf("----...") and time_p
idx_printf = c.find('printf("--------')
idx_timep = c.find('time_p[(i)/8]')

print('printf line offset:', idx_printf)
print('time_p line offset:', idx_timep)
print()
print('Context between them:')
print(repr(c[idx_printf:idx_timep+60]))
print()

# Insert fflush(stdout) between them
old_section = c[idx_printf:idx_timep]
new_section = old_section + '\n        fflush(stdout);\n'

c2 = c.replace(old_section, new_section, 1)

# Verify
if 'fflush(stdout)' in c2:
    open(p, 'w', encoding='gbk', errors='ignore').write(c2)
    print('OK! fflush(stdout) added after Frame printf block')
else:
    print('FAIL')

# Also try increasing buffer size via setvbuf at start of main
idx_main = c2.find('int main(')
idx_open = c2.find('{', idx_main)
# Add after first line of main body
main_body_start = idx_open + 1

buf_setup = '\n    /* Increase printf buffer to avoid overflow */\n'
buf_setup += '    static char printf_buf[8192];\n'
buf_setup += '    setvbuf(stdout, printf_buf, _IOFBF, sizeof(printf_buf));\n'

# Only add once
if 'setvbuf' not in c2:
    c2 = c2[:main_body_start] + buf_setup + c2[main_body_start:]
    open(p, 'w', encoding='gbk', errors='ignore').write(c2)
    print('OK! setvbuf buffer setup added at start of main()')
else:
    print('setvbuf already present')

print()
print('Summary:')
print('  1. fflush(stdout) after every frame (forces flush)')
print('  2. setvbuf to 8KB buffer (reduces flush frequency)')
