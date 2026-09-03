import os

BASE = r'd:\DSP\6678\track\track_1'

for fn in ['main.c', 'track.c', 'imm.c', 'new_reliable.c', 'track_die.c', 'track_asso.c', 'data_process_func.c']:
    fp = os.path.join(BASE, fn)
    try:
        c = open(fp, encoding='gbk', errors='ignore').read()
        hits = []
        for i, l in enumerate(c.splitlines(), 1):
            if 'mSecond' in l or ('dt' in l.lower() and ('predict' in l.lower() or 'update' in l.lower() or 'filter' in l.lower())):
                hits.append('  L%d: %s' % (i, l.strip()[:130]))
        if hits:
            print('=== %s ===' % fn)
            for h in hits[:10]:
                print(h)
            print()
    except Exception as e:
        print('%s: %s' % (fn, e))
