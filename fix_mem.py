import os

p = r'd:\DSP\6678\track\track_1\global_variable.c'
c = open(p, encoding='gbk', errors='ignore').read()

# 找 target_data 行
import re
m = re.search(r'struct TARGETPIONT_1\s+target_data\[(\d+)\]\s*=\s*\{0\}', c)
if m:
    old_line = m.group(0)
    size = m.group(1)
    pragma = '#pragma DATA_SECTION(target_data, ".far:DDR")\n'
    new_line = pragma + old_line
    c = c.replace(old_line, new_line, 1)
    open(p, 'w', encoding='gbk', errors='ignore').write(c)
    print('OK: target_data[%s] -> DDR3 via .far:DDR section' % size)
else:
    print('FAIL: pattern not found')
    # 打印附近
    idx = c.find('TARGETPIONT_1')
    if idx >= 0:
        print(repr(c[idx-30:idx+100]))
