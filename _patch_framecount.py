import os

root = r'd:\DSP\6678\track\track_1'

# 1. main.c
fpath = os.path.join(root, 'main.c')
with open(fpath, 'rb') as f:
    c = f.read().decode('gbk', errors='ignore')
c = c.replace('#define TOTAL_FRAMES 100', '#define TOTAL_FRAMES 500')
with open(fpath, 'wb') as f:
    f.write(c.encode('gbk', errors='ignore'))
print('main.c: TOTAL_FRAMES 100->500 OK')

# 2. global_variable.c
fpath = os.path.join(root, 'global_variable.c')
with open(fpath, 'rb') as f:
    c = f.read().decode('gbk', errors='ignore')
c = c.replace('int asso_info1[100]', 'int asso_info1[500]')
c = c.replace('int asso_info2[100]', 'int asso_info2[500]')
c = c.replace('struct TARGETPIONT_1 target_data[100]', 'struct TARGETPIONT_1 target_data[500]')
with open(fpath, 'wb') as f:
    f.write(c.encode('gbk', errors='ignore'))
print('global_variable.c: arrays [100]->[500] OK')

# 3. include/global_variable.h
fpath = os.path.join(root, 'include', 'global_variable.h')
with open(fpath, 'rb') as f:
    c = f.read().decode('gbk', errors='ignore')
c = c.replace('extern int asso_info1[100]', 'extern int asso_info1[500]')
c = c.replace('extern int asso_info2[100]', 'extern int asso_info2[500]')
with open(fpath, 'wb') as f:
    f.write(c.encode('gbk', errors='ignore'))
print('include/global_variable.h: arrays [100]->[500] OK')

print('All basic frame count changes done.')
