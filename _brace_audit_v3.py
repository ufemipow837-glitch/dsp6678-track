import re
def strip(s):
    s = re.sub(r"//[^\n]*", "", s)
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    s = re.sub(r'"(\\.|[^"\\])*"', '""', s)
    s = re.sub(r"'(\\.|[^'\\])*'", "''", s)
    return s
for fn in ["new_reliable.c","track_asso.c","track_initial.c","data_process_func.c","track.c","track_die.c","main.c"]:
    s = strip(open(fn,encoding="utf-8",errors="replace").read())
    line=1; out=[]
    for op,cl,nm in [("{","}","brace"),("(",")","paren")]:
        depth=0; mind=0; ml=0; line=1
        for c in s:
            if c=="\n": line+=1
            elif c==op: depth+=1
            elif c==cl:
                depth-=1
                if depth<mind: mind=depth; ml=line
        out.append(f"{nm}:final={depth},min={mind}@L{ml}")
    print(f"{fn:24s} " + " | ".join(out))
