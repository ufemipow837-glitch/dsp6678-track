import re
files = ['new_reliable.c','track_asso.c','track_initial.c','track_predict.c','track_die.c','data_process_func.c','track.c','main.c','imm.c','dot_coh.c']
def strip(s):
    s = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
    s = re.sub(r'//[^\n]*', '', s)
    return s
for fn in files:
    s = strip(open(fn, encoding='utf-8', errors='replace').read())
    print('%-20s {=%3d }=%3d (=%3d )=%3d' % (fn, s.count('{'), s.count('}'), s.count('('), s.count(')')))
