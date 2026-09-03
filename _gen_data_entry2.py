import re, math

DRONE_AZI_DEG = 17.0
DRONE_HEIGHT = 100.0
V_DRONE_AWAY = 17.0
V_DRONE_BACK = -15.0
FRAME_MS = 164
BASE_TIME = 898140.0
PI = 3.141592653589793
NUM_FRAMES = 500
NUM_BEAMS = 17
BEAM0 = 15

def frame_beam(f):
    return (BEAM0 + f) % NUM_BEAMS

def frame_time(f):
    return BASE_TIME + f * FRAME_MS

def drone_R_and_Vr(frame):
    t = frame * 0.164
    if frame <= 250:
        Vr = V_DRONE_AWAY
        R = 1100 + Vr * t
        if R > 2500: R = 2500
    else:
        t_peak = 250 * 0.164
        R_peak = 1100 + V_DRONE_AWAY * t_peak
        Vr = V_DRONE_BACK
        R = R_peak + Vr * (t - t_peak)
        if R < 1050: R = 1050
    return R, Vr

def drone_azi_ele(frame, R):
    azi = DRONE_AZI_DEG + 0.5 * (frame % 37) / 37.0 - 0.25
    ele_deg = math.degrees(math.atan2(DRONE_HEIGHT, R))
    ele_deg = max(2.0, min(8.0, ele_deg))
    return azi, ele_deg

def is_drone_visible(frame):
    if frame_beam(frame) != 2:
        return False
    R, _ = drone_R_and_Vr(frame)
    if R > 2300:
        return (frame * 7 + 13) % 10 != 0
    return True

def gen_clutter_targets(frame):
    random_state = (frame * 1103515245 + 12345) & 0x7FFFFFFF
    def rand():
        nonlocal random_state
        random_state = (random_state * 1103515245 + 12345) & 0x7FFFFFFF
        return random_state / 0x7FFFFFFF
    beam = frame_beam(frame)
    if beam in [4, 7]:
        count = 10 + int(rand() * 9)
    elif beam in [6, 8, 9, 10, 12]:
        count = 2 + int(rand() * 5)
    else:
        count = int(rand() * 4)
    targets = []
    for i in range(count):
        azi_deg = beam * 360.0 / NUM_BEAMS + (rand() - 0.5) * 6.0
        ele_deg = -2 + rand() * 14
        r = 300 + rand() * 11700
        vr = -20 + rand() * 70
        targets.append((azi_deg, ele_deg, r, vr))
    return targets

def gen_frame_lines(f):
    beam = frame_beam(f)
    t = frame_time(f)
    targets = []
    if beam == 2 and is_drone_visible(f):
        R, Vr = drone_R_and_Vr(f)
        azi_deg, ele_deg = drone_azi_ele(f, R)
        targets.append((azi_deg, ele_deg, R, Vr))
    else:
        clutter = gen_clutter_targets(f)
        for c in clutter:
            targets.append((c[0], c[1], c[2], c[3]))

    lines = []
    lines.append(f'    /* Frame {f}: t={int(t)}ms, beam={beam}, targets={len(targets)} */')
    lines.append(f'    data[{f}].targetNum = {len(targets)};')
    lines.append(f'    data[{f}].frameSn = {f};')
    lines.append(f'    data[{f}].mSecond = {t:.1f}f;')
    lines.append(f'    data[{f}].workMode = 0;')
    lines.append(f'    data[{f}].beamNo = {beam};')
    lines.append(f'    data[{f}].Month = 0;')
    for ti, (azi_d, ele_d, r, vr) in enumerate(targets):
        azi_rad = azi_d / 180.0 * PI
        ele_rad = ele_d / 180.0 * PI
        lines.append(f'    data[{f}].azi[{ti}] = {azi_rad:.6f}f;')
        lines.append(f'    data[{f}].ele[{ti}] = {ele_rad:.6f}f;')
        lines.append(f'    data[{f}].range[{ti}] = {r:.2f}f;')
        lines.append(f'    data[{f}].velocity[{ti}] = {vr:.4f}f;')
        lines.append(f'    data[{f}].Use_Flag_1[{ti}] = 1;')
    lines.append('')
    return lines

lines = []
lines.append('#include <stdio.h>')
lines.append('#include <c6x.h>')
lines.append('#include <math.h>')
lines.append('#include "struct.h"')
lines.append('#include <stdint.h>')
lines.append('#include <float.h>')
lines.append('/*')
lines.append(' * data_entry.c - 500 frames simulated radar input')
lines.append(' * Split into 10 sub-functions (50 frames each) to avoid C6000')
lines.append(' * compiler virtual register limit in single large functions.')
lines.append(' */')

chunk_size = 50
num_chunks = NUM_FRAMES // chunk_size

for ci in range(num_chunks):
    start = ci * chunk_size
    end = start + chunk_size
    func_name = f'data_entry_fill_{ci:02d}'
    lines.append('')
    lines.append(f'static void {func_name}(struct TARGETPIONT_1 (*data)){{')
    
    if ci == 0:
        lines.append('    int i, j;')
        lines.append('    /* Zero all 500 frames first */')
        lines.append('    for(i = 0; i < 500; i++){')
        lines.append('        data[i].targetNum = 0;')
        lines.append('        data[i].frameSn = 0;')
        lines.append('        data[i].mSecond = 0.0f;')
        lines.append('        data[i].workMode = 0;')
        lines.append('        data[i].beamNo = 0;')
        lines.append('        data[i].tgtnum = 1;')
        lines.append('        data[i].Year = 0;')
        lines.append('        data[i].Month = 0;')
        lines.append('        data[i].Day = 0;')
        lines.append('        data[i].Hour = 0;')
        lines.append('        data[i].Minute = 0;')
        lines.append('        data[i].Second = 0;')
        lines.append('        for(j = 0; j < 60; j++){')
        lines.append('            data[i].azi[j] = 0.0f;')
        lines.append('            data[i].ele[j] = 0.0f;')
        lines.append('            data[i].range[j] = 0.0f;')
        lines.append('            data[i].velocity[j] = 0.0f;')
        lines.append('            data[i].Use_Flag_1[j] = 0;')
        lines.append('        }')
        lines.append('    }')
        lines.append('')
    
    for f in range(start, end):
        lines.extend(gen_frame_lines(f))
    
    lines.append('}')

lines.append('')
lines.append('void data_entry(struct TARGETPIONT_1 (*data)){')
for ci in range(num_chunks):
    lines.append(f'    data_entry_fill_{ci:02d}(data);')
lines.append('}')

outpath = r'd:\DSP\6678\track\track_1\data_entry.c'
with open(outpath, 'w', encoding='gbk', errors='ignore') as f:
    f.write('\n'.join(lines))
    f.write('\n')

print(f'Generated data_entry.c: {len(lines)} lines, 500 frames in {num_chunks} chunks')

content = open(outpath, 'rb').read().decode('gbk', errors='ignore')
frames = re.findall(r'data\[(\d+)\]\.targetNum', content)
print(f'Total frame entries: {len(frames)} [{frames[0]}..{frames[-1]}]')

funcs = re.findall(r'static void (data_entry_fill_\d+)', content)
print(f'Sub-functions: {len(funcs)}')

# Check data_entry.h
hpath = r'd:\DSP\6678\track\track_1\data_entry.h'
with open(hpath, 'w') as f:
    f.write('#ifndef DATA_ENTRY_H\n#define DATA_ENTRY_H\n\n')
    f.write('#include "struct.h"\n\n')
    f.write('void data_entry(struct TARGETPIONT_1 (*data));\n\n')
    f.write('#endif\n')
print('data_entry.h updated')
