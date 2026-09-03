"""Generate data_entry.c from measured radar data (UTF-8)
Produces N_FRAMES frames split into sub-functions.
ALL output is ASCII-safe for C6000 compiler.
"""
import re, math, os, sys

BASE = r'd:\DSP\6678\track\track_1'
INPUT = os.path.join(BASE, '无人机实测数据.txt')
OUTPUT = os.path.join(BASE, 'data_entry.c')
N_FRAMES = 1300
FRAMES_PER_FUNC = 50

# Correct Unicode for Chinese field names (from actual file bytes)
CN_ts  = '\u65f6\u95f4\u6233'   # 时间戳
CN_bw  = '\u6ce2\u4f4d\u53f7'   # 波位号
CN_az  = '\u65b9\u4f4d'         # 方位
CN_r   = '\u8ddd\u79bb'         # 距离
CN_el  = '\u4fef\u4ef0'         # 俯仰 (CORRECT: 0x4FEF 0x4EF0)
CN_v   = '\u901f\u5ea6'         # 速度

# More specific: must match field boundaries (not prefix of longer fields like "俯仰差幅度")
# Pattern: field name followed by : or ：, then optional spaces, then value
FIELD_PATTERNS = [
    (CN_ts,  re.compile(re.escape(CN_ts) + r'[:：]\s*(\d+)')),
    (CN_bw,  re.compile(re.escape(CN_bw) + r'[:：]\s*(\d+)')),
    (CN_az,  re.compile(re.escape(CN_az) + r'[:：]\s*([-\d.eE+]+)')),
    (CN_r,   re.compile(re.escape(CN_r) + r'[:：]\s*([-\d.eE+]+)')),
    (CN_el,  re.compile(re.escape(CN_el) + r'[:：]\s*([-\d.eE+]+)')),
    (CN_v,   re.compile(re.escape(CN_v) + r'[:：]\s*([-\d.eE+]+)')),
]

def find_first_match(patterns, line):
    """Find the EARLIEST match among all patterns in the line."""
    best_pos = -1
    best_val = None
    for name, pat in patterns:
        m = pat.search(line)
        if m:
            pos = m.start()
            if best_pos == -1 or pos < best_pos:
                best_pos = pos
                best_val = m.group(1)
    return best_val

# Parse measured data
print(f'Reading {INPUT}')
with open(INPUT, encoding='utf-8') as f:
    text = f.read()

lines = text.splitlines()
print(f'Total lines: {len(lines)}')

# Parse points and group by timestamp (frame)
frame_data = {}  # ts -> list of (beamNo, azi_rad, range, ele_rad, vr)
skipped = 0

for line in lines:
    if not line.strip():
        continue
    
    ts_m = re.search(CN_ts + r'[:：]\s*(\d+)', line)
    if not ts_m:
        continue
    ts = int(ts_m.group(1))
    
    # Find each field by position (earliest match wins for each field type)
    # But we need position-aware matching to avoid "俯仰差幅度" matching before "俯仰"
    # Better: extract ALL name:value pairs, then pick the right ones
    
    # Strategy: split by known field names, take what comes after each
    # For azimuth/distance/elevation/velocity, search with negative lookbehind
    # to avoid matching field name that's a prefix of another
    
    # Actually, the simplest robust approach: 
    # Search for each field name with position tracking
    # Use word-boundary-like approach: field name must be followed immediately by : or ：
    
    def get_field(name, line, is_num=True):
        """Get field value for given Chinese field name."""
        # Use pattern: exact field name followed by colon
        pat = re.compile(re.escape(name) + r'[:：]\s*([-\d.eE+]+)')
        m = pat.search(line)
        if m:
            return float(m.group(1)) if is_num else m.group(1)
        return None
    
    az = get_field(CN_az, line)
    r  = get_field(CN_r, line)
    if az is None or r is None:
        continue
    
    bw = get_field(CN_bw, line)
    bw = int(bw) if bw is not None else 0
    
    el = get_field(CN_el, line)
    el_val = el if el is not None else 0.0
    
    vr = get_field(CN_v, line)
    vr_val = vr if vr is not None else 0.0
    
    if 50 < r < 30000 and ts > 0:
        frame_data.setdefault(ts, []).append(
            (bw, math.radians(az), r, math.radians(el_val), vr_val)
        )

frame_ts = sorted(frame_data.keys())
print(f'Valid frames: {len(frame_ts)}')
print(f'Time range: {frame_ts[0]}~{frame_ts[-1]}ms = {(frame_ts[-1]-frame_ts[0])/1000:.1f}s')

# Get ALL timestamps from file (including frames with no valid data)
import re as _re
all_ts = sorted(set(int(x) for x in _re.findall(CN_ts + r'[:：](\d+)', text)))
print(f'All unique timestamps in file: {len(all_ts)}')

if len(all_ts) < N_FRAMES:
    print(f'WARNING: only {len(all_ts)} timestamps, using ALL')
    selected_ts = all_ts
else:
    selected_ts = all_ts[:N_FRAMES]
    print(f'Using first {N_FRAMES} frames (all timestamps): {selected_ts[0]}~{selected_ts[-1]}ms = {(selected_ts[-1]-selected_ts[0])/1000:.1f}s')

# Verify: every selected_ts should have an entry in frame_data (even if empty list)
no_data = [t for t in selected_ts if t not in frame_data]
print(f'Frames with NO valid data (targetNum=0): {len(no_data)}')
# For empty frames, ensure frame_data has empty list
for t in no_data:
    frame_data[t] = []

pts_counts = [len(frame_data[t]) for t in selected_ts]
print(f'Points/frame: min={min(pts_counts)}, max={max(pts_counts)}, avg={sum(pts_counts)/len(pts_counts):.1f}')

# Quick sanity check on Frame 0
ts0 = selected_ts[0]
pts0 = frame_data[ts0]
print(f'\nSanity Frame 0 (t={ts0}ms):')
for i, (bw, az_r, rng, el_r, vr) in enumerate(pts0[:4]):
    print(f'  pt[{i}]: bw={bw}, azi={az_r:.6f}rad={math.degrees(az_r):.4f}deg, ele={el_r:.6f}rad={math.degrees(el_r):.4f}deg, r={rng:.1f}, v={vr:.4f}')

# Compare with backup
bak_path = os.path.join(BASE, 'data_entry_500frames.c.bak')
if os.path.exists(bak_path):
    c_bak = open(bak_path, encoding='gbk', errors='ignore').read()
    import re as _re
    bak_ele0 = _re.search(r'data\[0\]\.ele\[0\] = ([-\d.]+)f', c_bak)
    if bak_ele0:
        print(f'  backup ele[0] = {bak_ele0.group(1)} rad')
        if pts0:
            print(f'  new    ele[0] = {pts0[0][3]:.6f} rad')
            match = abs(float(bak_ele0.group(1)) - pts0[0][3]) < 0.001
            print(f'  MATCH: {match} {"YES" if match else "NO - STILL WRONG!"}')

# Build C code (ASCII-only)
out = []
out.append('/*')
out.append(f' * data_entry.c - {len(selected_ts)} frames of simulated radar input (auto-generated)')
out.append(' * Source: measured radar data (UTF-8)')
out.append(f' * Time span: {(selected_ts[-1]-selected_ts[0])/1000:.1f}s ({selected_ts[0]}~{selected_ts[-1]}ms)')
out.append(f' * Split into sub-functions ({FRAMES_PER_FUNC} frames each) for C6000 compiler')
out.append(' */')
out.append('')
out.append('#include <stdio.h>')
out.append('#include <c6x.h>')
out.append('#include <math.h>')
out.append('#include "struct.h"')
out.append('')
out.append('#ifndef pi')
out.append('#define pi 3.1415926535f')
out.append('#endif')
out.append('')

n_funcs = (len(selected_ts) + FRAMES_PER_FUNC - 1) // FRAMES_PER_FUNC

for fi in range(n_funcs):
    start = fi * FRAMES_PER_FUNC
    end = min(start + FRAMES_PER_FUNC, len(selected_ts))
    
    out.append('static void data_entry_fill_{:02d}(struct TARGETPIONT_1 (*data)){{'.format(fi))
    
    if fi == 0:
        out.append('    int i, j;')
        out.append('    /* Zero all {} frames first */'.format(len(selected_ts)))
        out.append('    for(i = 0; i < {}; i++){{'.format(len(selected_ts)))
        out.append('        data[i].targetNum = 0;')
        out.append('        data[i].frameSn = 0;')
        out.append('        data[i].mSecond = 0.0f;')
        out.append('        data[i].workMode = 0;')
        out.append('        data[i].beamNo = 0;')
        out.append('        data[i].tgtnum = 1;')
        out.append('        data[i].Year = 0;')
        out.append('        data[i].Month = 0;')
        out.append('        data[i].Day = 0;')
        out.append('        data[i].Hour = 0;')
        out.append('        data[i].Minute = 0;')
        out.append('        data[i].Second = 0;')
        out.append('        for(j = 0; j < 60; j++){')
        out.append('            data[i].azi[j] = 0.0f;')
        out.append('            data[i].ele[j] = 0.0f;')
        out.append('            data[i].range[j] = 0.0f;')
        out.append('            data[i].velocity[j] = 0.0f;')
        out.append('            data[i].Use_Flag_1[j] = 0;')
        out.append('        }')
        out.append('    }')
    
    out.append('')
    
    for idx in range(start, end):
        ts = selected_ts[idx]
        pts = frame_data[ts][:60]
        n = len(pts)
        bn = pts[0][0] if pts else 0
        
        out.append('    /* Frame {:4d}  t={:7d}ms  beamNo={:2d}  n={:2d} */'.format(idx, ts, bn, n))
        out.append('    data[{:4d}].targetNum = {:2d};'.format(idx, n))
        out.append('    data[{:4d}].frameSn = {:4d};'.format(idx, idx))
        out.append('    data[{:4d}].mSecond = {:7d}.0f;'.format(idx, ts))
        out.append('    data[{:4d}].workMode = 0;'.format(idx))
        out.append('    data[{:4d}].beamNo = {:2d};'.format(idx, bn))
        out.append('    data[{:4d}].Month = 0;'.format(idx))
        
        for pj, (bw, azi_r, rng, ele_r, vr) in enumerate(pts):
            out.append('    data[{:4d}].azi[{:2d}] = {:10.4f}f / 180.0f * pi;'.format(idx, pj, math.degrees(azi_r)))
            out.append('    data[{:4d}].ele[{:2d}] = {:10.4f}f / 180.0f * pi;'.format(idx, pj, math.degrees(ele_r)))
            out.append('    data[{:4d}].range[{:2d}] = {:10.2f}f;'.format(idx, pj, rng))
            out.append('    data[{:4d}].velocity[{:2d}] = {:10.4f}f;'.format(idx, pj, vr))
            out.append('    data[{:4d}].Use_Flag_1[{:2d}] = 1;'.format(idx, pj))
        
        out.append('')
    
    out.append('}')
    out.append('')

out.append('void data_entry(struct TARGETPIONT_1 (*data)){')
for fi in range(n_funcs):
    out.append('    data_entry_fill_{:02d}(data);'.format(fi))
out.append('}')

result = '\n'.join(out) + '\n'

# Verify ASCII-only
bad = [(i, c) for i, c in enumerate(result) if ord(c) > 127]
if bad:
    print(f'\nERROR: {len(bad)} non-ASCII chars!')
    for pos, ch in bad[:3]:
        ctx = result[max(0,pos-30):pos+30]
        print(f'  pos={pos}, code={hex(ord(ch))}, ctx={repr(ctx)}')
    sys.exit(1)

with open(OUTPUT, 'w', encoding='ascii', newline='') as f:
    f.write(result)

size = os.path.getsize(OUTPUT)
print(f'\n=== SUCCESS ===')
print(f'Output: {OUTPUT}')
print(f'Size: {size} bytes ({size/1024:.1f} KB)')
print(f'Functions: {n_funcs}')
print(f'Frames: 0 ~ {len(selected_ts)-1}')
print(f'Time: {(selected_ts[-1]-selected_ts[0])/1000:.1f}s')
