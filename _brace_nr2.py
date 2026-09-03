import re, sys
fn = sys.argv[1] if len(sys.argv) > 1 else "new_reliable.c"
lines = open(fn, encoding="utf-8", errors="replace").read().split("\n")
depth = 0
watch = {97,112,132,236,249,503,506,556,568,620,622,630,640,643,646,647,648,657,767,773,781,784,785,787}
for idx, raw in enumerate(lines, 1):
    s = re.sub(r"//.*", "", raw)
    s = re.sub(r"\"(\\.|[^\"\\])*\"", "\"\"", s)
    s = re.sub(r"'(\\.|[^'\\])*'", "''", s)
    o = s.count("{"); c = s.count("}")
    depth += o - c
    if o or c or idx in watch:
        print("L%-4d d=%-3d o=%d c=%d | %s" % (idx, depth, o, c, raw.strip()[:78]))
print("FINAL DEPTH =", depth)
