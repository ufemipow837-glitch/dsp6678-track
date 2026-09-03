import re, sys
import math

# 1. 解析输出数据.txt
track_data = []
with open('输出数据.txt', 'r', encoding='utf-8', errors='replace') as f:
    lines = f.readlines()

current_frame = -1
current_t = -1
update_flag = {}
for line in lines:
    if 'Update:' in line:
        um = re.search(r'Update:\s*(\d+)', line)
        if um:
            update_flag[current_frame] = int(um.group(1))
    frame_match = re.search(r'Frame\s+(\d+)\s+\|\s+t=(\d+)ms', line)
    if frame_match:
        current_frame = int(frame_match.group(1))
        current_t = int(frame_match.group(2))
    track_match = re.search(r'Track\[(\d+)\]:\s*Az=([\d.]+)deg\s+El=([-\d.]+)deg\s+R=([\d.]+)m\s+V=([-\d.]+)m/s', line)
    if track_match and current_frame >= 0:
        tid = int(track_match.group(1))
        az = float(track_match.group(2))
        el = float(track_match.group(3))
        r = float(track_match.group(4))
        v = float(track_match.group(5))
        upd = update_flag.get(current_frame, -1)
        track_data.append((current_frame, current_t, az, el, r, v, tid, upd))

print(f"=== 输出数据.txt 分析 ===")
print(f"航迹点总数: {len(track_data)}")
if not track_data:
    print("ERROR: 没有航迹数据！"); sys.exit(0)
frames = [f for f,t,_,_,_,_,_,_ in track_data]
print(f"Frame范围: {min(frames)} ~ {max(frames)}")
ts = [t for f,t,_,_,_,_,_,_ in track_data]
print(f"时间范围: {min(ts)}ms ~ {max(ts)}ms (时长{(max(ts)-min(ts))/1000:.1f}s)")
# 看航迹速度分段
vs = [v for _,_,_,_,_,v,_,_ in track_data]
print(f"速度: 最小={min(vs):.1f}, 最大={max(vs):.1f}, 均值={sum(vs)/len(vs):.1f}")
print(f"  速度>30m/s的点数: {sum(1 for v in vs if v>30)}")
print(f"  速度>60m/s的点数: {sum(1 for v in vs if v>60)}")
# 航迹起始值
f0,t0,az0,el0,r0,v0,_,_ = track_data[0]
print(f"  起始帧Frame{f0}: Az={az0:.2f}, El={el0:.2f}, R={r0:.1f}, V={v0:.1f}")

# 2. 解析无人机实测数据.txt
meas_pts = {}  # t -> list[(az,el,r,v,bw)]  同一波位可能多点
with open('无人机实测数据.txt', 'r', encoding='utf-8', errors='replace') as f:
    lines2 = f.readlines()
for line in lines2:
    # 格式: 点迹解析数据： 波位号:2 北斗时间戳:900000 时间戳:900000 方位:16.5 距离:1200.0 俯仰:3.2 高度:67.0 速度:12.5
    m = re.search(r'波位号:(\d+).*?时间戳:(\d+).*?方位:([-\d.]+).*?距离:([\d.]+).*?俯仰:([-\d.]+).*?速度:([-\d.]+)', line)
    if m:
        bw = int(m.group(1))
        t = int(m.group(2))
        az = float(m.group(3))
        r = float(m.group(4))
        el = float(m.group(5))
        v = abs(float(m.group(6)))
        if t not in meas_pts:
            meas_pts[t] = []
        meas_pts[t].append((az, el, r, v, bw))

print(f"\n=== 无人机实测数据.txt 分析 ===")
print(f"点迹时间戳数量: {len(meas_pts)}")
total_pts = sum(len(v) for v in meas_pts.values())
print(f"点迹总数: {total_pts}")
mtimes = sorted(meas_pts.keys())
print(f"时间范围: {min(mtimes)}ms ~ {max(mtimes)}ms (时长{(max(mtimes)-min(mtimes))/1000:.1f}s)")
# 看波位2/3的数据
bw23 = [p for t,arr in meas_pts.items() for p in arr if p[4] in (2,3)]
print(f"波位2/3的点数: {len(bw23)}")
if bw23:
    azs = [p[0] for p in bw23]
    els = [p[1] for p in bw23]
    rs = [p[2] for p in bw23]
    vs = [p[3] for p in bw23]
    print(f"  无人机波位2/3范围: Az[{min(azs):.2f}~{max(azs):.2f}], El[{min(els):.2f}~{max(els):.2f}], R[{min(rs):.0f}~{max(rs):.0f}]m, V[{min(vs):.1f}~{max(vs):.1f}]m/s")
    # 看距离变化趋势（找转弯点）
    if len(rs) > 10:
        idx_max = rs.index(max(rs))
        print(f"  最远距离点: R={max(rs):.0f}m, 对应前{idx_max}/{len(rs)}点")

# 3. 逐帧对比：找每个航迹点t附近波位2/3的最优点（50ms容差）
print(f"\n=== 逐帧匹配（波位2/3优先, 50ms容差） ===")

def find_best_match(t_tr, az_tr, el_tr, r_tr, tol=50):
    """在t±tol范围内，优先找波位2/3，选3D距离最近的点"""
    best = None; best_e3d = 999999
    cand_ts = [mt for mt in mtimes if abs(mt - t_tr) <= tol]
    for mt in cand_ts:
        for (maz,mel,mr,mv,mbw) in meas_pts[mt]:
            # 优先波位2/3
            if mbw not in (2,3):
                continue
            daz = az_tr - maz
            del_ = el_tr - mel
            dr = r_tr - mr
            e3d = math.sqrt(dr*dr + (r_tr*math.pi/180*daz)**2 + (r_tr*math.pi/180*del_)**2)
            if e3d < best_e3d:
                best_e3d = e3d
                best = (mt, maz, mel, mr, mv, mbw, abs(mt-t_tr), daz, del_, dr)
    # 找不到2/3则看其他波位(作为参考)
    if best is None:
        for mt in cand_ts:
            for (maz,mel,mr,mv,mbw) in meas_pts[mt]:
                daz = az_tr - maz; del_ = el_tr - mel; dr = r_tr - mr
                e3d = math.sqrt(dr*dr + (r_tr*math.pi/180*daz)**2 + (r_tr*math.pi/180*del_)**2)
                if e3d < best_e3d:
                    best_e3d = e3d
                    best = (mt, maz, mel, mr, mv, mbw, abs(mt-t_tr), daz, del_, dr)
    return best

matches = []
for f,t,az,el,r,v,tid,upd in track_data:
    best = find_best_match(t, az, el, r)
    if best:
        mt, maz, mel, mr, mv, mbw, tdiff, daz, del_, dr = best
        dv = v - mv
        e3d = math.sqrt(dr*dr + (r*math.pi/180*daz)**2 + (r*math.pi/180*del_)**2)
        upd_str = {0:'Predict',1:'Update',-1:'?'}.get(upd, '?')
        matches.append((f,t,upd_str,az,el,r,v,mt,maz,mel,mr,mv,mbw,tdiff,daz,del_,dr,dv,e3d))

print(f"匹配到的帧数: {len(matches)}/{len(track_data)}")

if matches:
    # 总体统计
    dazs = [m[14] for m in matches]
    dels = [m[15] for m in matches]
    drs = [m[16] for m in matches]
    dvs = [m[17] for m in matches]
    e3ds = [m[18] for m in matches]
    vs_tr = [m[6] for m in matches]
    def stat(arr, name):
        if arr:
            a = [abs(x) for x in arr]
            print(f"  {name}: |Δ|均值={sum(a)/len(a):.3f}, RMS={math.sqrt(sum(x*x for x in arr)/len(arr)):.3f}, |Δ|max={max(a):.3f}")
    print("\n--- 总体误差统计 ---")
    stat(dazs,"方位差(deg)")
    stat(dels,"俯仰差(deg)")
    stat(drs, "距离差(m)")
    stat(dvs, "速度差(m/s)")
    if e3ds:
        a = [abs(x) for x in e3ds]
        print(f"  3D位置误差: 均值={sum(a)/len(a):.2f}m, RMS={math.sqrt(sum(x*x for x in e3ds)/len(e3ds)):.2f}m, max={max(a):.2f}m")

    # 分段统计
    n = len(matches)
    segs = [(0, n//3, "前1/3"), (n//3, 2*n//3, "中1/3"), (2*n//3, n, "后1/3")]
    for s,e,name in segs:
        seg = matches[s:e]
        e3d_seg = [m[18] for m in seg]
        v_seg = [m[6] for m in seg]
        dv_seg = [m[17] for m in seg]
        if e3d_seg:
            print(f"\n  {name}(Frame{seg[0][0]}-{seg[-1][0]}): 3D_RMS={math.sqrt(sum(x*x for x in e3d_seg)/len(e3d_seg)):.2f}m, 平均V={sum(v_seg)/len(v_seg):.1f}m/s, V误差RMS={math.sqrt(sum(x*x for x in dv_seg)/len(dv_seg)):.2f}m/s")

    # 逐帧明细
    print(f"\n=== 逐帧明细（前30 + 异常）===")
    hdr = f"{'Frame':>5} {'t(ms)':>8} {'Mode':>7} | {'AzT':>5} {'AzM':>5} {'ΔAz':>6} | {'ElT':>5} {'ElM':>5} {'ΔEl':>6} | {'RT':>6} {'RM':>6} {'ΔR':>7} | {'VT':>5} {'VM':>5} {'ΔV':>6} | {'3D':>7} BW"
    print(hdr); print("-"*len(hdr))
    for i,m in enumerate(matches):
        f,t,upd,az,el,r,v,mt,maz,mel,mr,mv,mbw,tdiff,daz,del_,dr,dv,e3d = m
        abn = (abs(v)>30) or (e3d>150)
        if i<30 or abn:
            mark = "  ***HIGH V" if abs(v)>30 else ("  ***LARGE ERR" if e3d>150 else "")
            print(f"{f:5d} {t:8d} {upd:>7} | {az:5.2f} {maz:5.2f} {daz:+6.3f} | {el:5.2f} {mel:5.2f} {del_:+6.3f} | {r:6.1f} {mr:6.1f} {dr:+7.1f} | {v:5.1f} {mv:5.1f} {dv:+6.2f} | {e3d:7.1f} {mbw}{mark}")

    # 全部速度异常点
    print(f"\n=== 速度>30m/s的异常点(共{sum(1 for m in matches if abs(m[6])>30)}个) ===")
    for m in matches:
        if abs(m[6])>30:
            f,t,upd,az,el,r,v,mt,maz,mel,mr,mv,mbw,tdiff,daz,del_,dr,dv,e3d = m
            print(f"  F{f:4d} t={t}ms T[{az:.2f},{el:.2f},{r:.0f}m,{v:.1f}m/s] M[bw{mbw},{maz:.2f},{mel:.2f},{mr:.0f}m,{mv:.1f}] 3Derr={e3d:.0f}m  tdiff={tdiff}")
else:
    print("无法匹配！列出航迹前30帧:")
    for i in range(min(30,len(track_data))):
        f,t,az,el,r,v,_,_ = track_data[i]
        print(f"  F{f:4d} t={t}ms Az={az:.2f} El={el:.2f} R={r:.1f} V={v:.1f}")
