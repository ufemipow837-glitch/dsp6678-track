import re, sys
files = ['new_reliable.c','track_asso.c','track_initial.c','track_die.c','data_process_func.c','track.c','track_predict.c','main.c']
def strip_cc(src):
    out=[]; i=0; n=len(src); state='code'
    while i<n:
        c=src[i]
        if state=='code':
            if src.startswith('//',i):
                j=src.find('\n',i); 
                if j<0: j=n
                i=j; continue
            if src.startswith('/*',i):
                j=src.find('*/',i+2)
                if j<0: j=n
                else: j+=2
                i=j; continue
            if c=='"': state='str'; out.append(' '); i+=1; continue
            if c=="'": state='char'; out.append(' '); i+=1; continue
            out.append(c); i+=1
        elif state=='str':
            if c=='\\': out.append('  '); i+=2; continue
            if c=='"': state='code'; out.append(' '); i+=1; continue
            out.append('\n' if c=='\n' else ' '); i+=1
        elif state=='char':
            if c=='\\': out.append('  '); i+=2; continue
            if c=="'": state='code'; out.append(' '); i+=1; continue
            out.append('\n' if c=='\n' else ' '); i+=1
    return ''.join(out)
for fn in files:
    src=open(fn,encoding='utf-8',errors='replace').read()
    code=strip_cc(src)
    lines=code.split('\n')
    b=p=0; err=None
    for li,line in enumerate(lines,1):
        b+=line.count('{')-line.count('}')
        p+=line.count('(')-line.count(')')
        if b<0 or p<0:
            err=(li,b,p,line.strip()[:80]); break
    if err or b!=0 or p!=0:
        print(f"{fn}: BRACES={b} PARENS={p}  NEG@ {err}")
    else:
        print(f"{fn}: OK (lines={len(lines)})")
