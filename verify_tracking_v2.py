#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机跟踪算法功能验证 V2
修正参数: time_up=4500ms, MAX_MISSING=35帧, 方位门限放宽至±85°(模拟正确配置)
"""
import csv, math, os
from collections import defaultdict

PLOT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_parsed.csv')

Vmin = 5.0
Vmax = 1500.0
time_up = 4500.0
time_down = 0.0
beam_time = 0.164
range_down = 100.0
range_up = 50000.0
ele_down_deg = -5.0
ele_up_deg = 60.0
azi_down_deg = -85.0   # 放宽至±85°(模拟配置为匹配扫描扇区)
azi_up_deg = 85.0

INIT_MAH_TH = 15.0
ANGLE_CHANGE_TH = 90.0
MAX_NEW_PER_FRAME = 20
MAX_MISS = 35
PRE_GATE = 150.0
ASSO_MAH_TH = 25.0
ASSO_VEL_TH = 20.0

PI = 3.14159265

def deg2rad(d): return d*PI/180.0
def rad2deg(r): return r*180.0/PI

# 坐标转换(与track.c一致,无姿态旋转)
def sph2cart_track(az_deg, el_deg, r):
    """FPGA球坐标→track内部笛卡尔坐标
    x = r*sin(az)*cos(el)  (FPGA az=0→x=0; az=90→x=r)
    y = r*cos(az)*cos(el)  (FPGA az=0→y=r; az=90→y=0)
    z = r*sin(el)
    azi_track = atan2(y,x)  (内部方位角)
    """
    az = deg2rad(az_deg); el = deg2rad(el_deg)
    x = r*math.sin(az)*math.cos(el)
    y = r*math.cos(az)*math.cos(el)
    z = r*math.sin(el)
    return (x,y,z)

def track_azi_deg(x,y):
    return rad2deg(math.atan2(y,x))

class KF_CV:
    def __init__(self):
        self.X = [0.0]*6
        self.P = [[0.0]*6 for _ in range(6)]
        self.initialized = False
        self.sig_a = 8.0
    def init2(self, p0, p1, dt):
        self.X = [p1[0], (p1[0]-p0[0])/dt,
                  p1[1], (p1[1]-p0[1])/dt,
                  p1[2], (p1[2]-p0[2])/dt]
        for i in range(6):
            for j in range(6):
                self.P[i][j] = 0.0
        for i in range(6):
            self.P[i][i] = 400.0 if i%2==0 else 400.0
        self.initialized = True
    def predict(self, T):
        if not self.initialized: return None
        F = [[0]*6 for _ in range(6)]
        F[0][0]=1; F[0][1]=T
        F[1][1]=1
        F[2][2]=1; F[2][3]=T
        F[3][3]=1
        F[4][4]=1; F[4][5]=T
        F[5][5]=1
        q = self.sig_a**2
        t2=T*T; t3=t2*T/2; t4=t2*t2/4
        Q = [[0]*6 for _ in range(6)]
        Q[0][0]=q*t4; Q[0][1]=q*t3
        Q[1][0]=q*t3; Q[1][1]=q*t2
        Q[2][2]=q*t4; Q[2][3]=q*t3
        Q[3][2]=q*t3; Q[3][3]=q*t2
        Q[4][4]=q*t4; Q[4][5]=q*t3
        Q[5][4]=q*t3; Q[5][5]=q*t2
        Xn = [0.0]*6
        for i in range(6):
            for j in range(6):
                Xn[i] += F[i][j]*self.X[j]
        Pn = [[0.0]*6 for _ in range(6)]
        for i in range(6):
            for j in range(6):
                for k in range(6):
                    Pn[i][j] += F[i][k]*self.P[k][j]*F[j][k]
                Pn[i][j] += Q[i][j]
        self.X = Xn; self.P = Pn
        return (self.X[0], self.X[2], self.X[4])
    def update(self, z):
        if not self.initialized: return 0.0
        # 简化: 对角R, 基于距离估计量测噪声
        r = math.sqrt(z[0]**2+z[1]**2+z[2]**2)+1.0
        sigma_r = 8.0; sigma_a = deg2rad(0.3)
        R_diag = [max((sigma_r)**2, 25.0)]*3
        HPH = [self.P[0][0]+R_diag[0], self.P[2][2]+R_diag[1], self.P[4][4]+R_diag[2]]
        nu = [z[i] - [self.X[0],self.X[2],self.X[4]][i] for i in range(3)]
        maha = sum(nu[i]**2/max(HPH[i],1.0) for i in range(3))
        alpha = 0.35
        self.X[0] += alpha*nu[0]; self.X[2] += alpha*nu[1]; self.X[4] += alpha*nu[2]
        self.X[1] += 0.15*nu[0]/max(beam_time,0.001)
        self.X[3] += 0.15*nu[1]/max(beam_time,0.001)
        self.X[5] += 0.15*nu[2]/max(beam_time,0.001)
        # P衰减
        for i in range(6):
            self.P[i][i] *= 0.9
        return maha
    def velocity(self):
        return math.sqrt(self.X[1]**2+self.X[3]**2+self.X[5]**2)
    def pos(self):
        return (self.X[0], self.X[2], self.X[4])

class TempTrack:
    def __init__(self, p, xyz, idx):
        self.p0 = xyz; self.p1 = None; self.p2 = None
        self.kf = None
        self.id = idx
        self.state = 1  # 1:1点, 2:2点
        self.miss_cnt = 0
        self.last_t = p['t']
        self.last_xyz = xyz
        self.last_range = p['r']
    def try_add_2(self, xyz, dt):
        v = math.sqrt(sum((xyz[i]-self.p0[i])**2 for i in range(3)))/dt
        if v < Vmin or v > Vmax: return False
        self.p1 = xyz
        self.state = 2
        self.kf = KF_CV()
        self.kf.init2(self.p0, xyz, dt)
        self.last_t += dt*1000
        self.last_xyz = xyz
        self.miss_cnt = 0
        return True
    def try_add_3(self, xyz, dt):
        pred = tuple(self.p1[i] + dt*(self.p1[i]-self.p0[i])/max(dt,beam_time) for i in range(3))
        # 角度约束
        v1 = tuple(xyz[i]-self.p1[i] for i in range(3))
        v0 = tuple(self.p1[i]-self.p0[i] for i in range(3))
        n1=math.sqrt(sum(x*x for x in v1)); n0=math.sqrt(sum(x*x for x in v0))
        if n1>0.1 and n0>0.1:
            dot=sum(v1[i]*v0[i] for i in range(3))
            ca=max(-1,min(1,dot/(n1*n0)))
            angle=math.acos(ca)*180/PI
            if angle>ANGLE_CHANGE_TH: return False
        # 速度比
        v12 = n0/max(dt,0.001); v23 = n1/max(dt,0.001)
        if v12 < Vmin or v12 > Vmax: return False
        if v23 < Vmin or v23 > Vmax: return False
        vn=min(v12,v23)/max(v12,v23)
        if vn < 0.3: return False
        # quality = ( (1+cos)/2 + speed_ratio ) / 2 >= 0.7
        quality = ((1+max(-1,min(1,dot/(n1*n0))))/2 + vn)/2
        if quality < 0.7: return False
        # 马氏距离(简化)
        P_diag=[400]*3
        md=sum((xyz[i]-pred[i])**2/P_diag[i] for i in range(3))
        if md > INIT_MAH_TH: return False
        self.p2 = xyz
        self.last_xyz = xyz
        self.last_t += dt*1000
        self.miss_cnt = 0
        return True

class RelTrack:
    def __init__(self, tt, idx):
        self.kf = KF_CV()
        dt0 = beam_time
        self.kf.init2(tt.p0, tt.p2, max((tt.last_t - (tt.last_t-dt0*1000))/1000, beam_time))
        self.id = idx
        self.miss_cnt = 0
        self.last_t = tt.last_t
        self.update_flag = True
        self.plot_count = 3
        self.start_t = tt.last_t - dt0*1000*2
        self.last_range = tt.last_range

def euc(a,b):
    return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))

def main():
    print("="*70)
    print(" 无人机跟踪算法验证 V2 (修正后参数)")
    print("="*70)
    print(f" 参数: Vmin={Vmin}, time_up={time_up}ms, MAX_MISS={MAX_MISS}")
    print(f"       azi=[{azi_down_deg}°,{azi_up_deg}°], ele=[{ele_down_deg}°,{ele_up_deg}°]")

    # 加载数据,按时间分组
    all_plots = []
    with open(PLOT_CSV, 'r', encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            if int(row['是否有效点'])==1:
                p = {
                    't': int(row['时间戳(ms)']),
                    'beam': int(row['波位号']),
                    'az': float(row['方位(度)']),
                    'r': float(row['距离(m)']),
                    'el': float(row['俯仰(度)']),
                    'alt': float(row['高度(m)']),
                    'v': float(row['径向速度(m/s)']),
                    'snr': int(row['信噪比']),
                }
                xyz = sph2cart_track(p['az'], p['el'], p['r'])
                p['xyz'] = xyz
                p['tazi'] = track_azi_deg(xyz[0], xyz[1])
                all_plots.append(p)

    # 按时间戳分组(CPI)
    cpi_dict = defaultdict(list)
    for p in all_plots:
        cpi_dict[p['t']].append(p)
    cpi_times = sorted(cpi_dict.keys())
    print(f"\n 加载: {len(all_plots)}点, {len(cpi_times)}个CPI")

    # 空域过滤
    def air_filter(plots):
        res = []
        for p in plots:
            if p['r'] < range_down or p['r'] > range_up: continue
            if p['el'] < ele_down_deg or p['el'] > ele_up_deg: continue
            if p['tazi'] < azi_down_deg or p['tazi'] > azi_up_deg: continue
            res.append(p)
        return res

    # 统计过滤
    n_raw = len(all_plots)
    n_az_filt = sum(1 for p in all_plots if p['tazi']<azi_down_deg or p['tazi']>azi_up_deg)
    n_el_filt = sum(1 for p in all_plots if p['el']<ele_down_deg or p['el']>ele_up_deg)
    n_r_filt = sum(1 for p in all_plots if p['r']<range_down or p['r']>range_up)
    print(f"\n 空域过滤统计:")
    print(f"   方位过滤({azi_down_deg}°~{azi_up_deg}°): {n_az_filt}点")
    print(f"   俯仰过滤({ele_down_deg}°~{ele_up_deg}°): {n_el_filt}点")
    print(f"   距离过滤({range_down}~{range_up}m): {n_r_filt}点")

    # 运行跟踪
    temp_tracks = []
    rel_tracks = []
    next_tid = 0; next_rid = 0
    total_asso = 0; total_miss = 0; total_died = 0
    starts_log = []
    track_life = defaultdict(list)

    for cpi_idx, t in enumerate(cpi_times):
        raw = cpi_dict[t]
        plots = air_filter(raw)
        for p in plots: p['used'] = False

        # 可靠航迹关联(GNN: 逐个找最优,点不重复分配)
        for rt in rel_tracks:
            rt.update_flag = False
            lag = t - rt.last_t
            if lag < time_down or lag > time_up: continue
            dt = lag/1000.0
            pred = rt.kf.predict(dt)
            if pred is None: continue
            best_p = None; best_md = ASSO_MAH_TH
            gate = PRE_GATE + dt*120.0
            for p in plots:
                if p['used']: continue
                d = euc(p['xyz'], pred)
                if d > gate: continue
                pd = [max(rt.kf.P[0][0],25), max(rt.kf.P[2][2],25), max(rt.kf.P[4][4],25)]
                md = sum((p['xyz'][i]-pred[i])**2/pd[i] for i in range(3))
                if md < best_md:
                    best_md = md; best_p = p
            if best_p is not None:
                best_p['used'] = True
                rt.kf.update(best_p['xyz'])
                rt.last_t = t; rt.miss_cnt = 0; rt.update_flag = True
                rt.plot_count += 1; rt.last_range = best_p['r']
                total_asso += 1
                v3d = rt.kf.velocity()
                track_life[rt.id].append((t, best_p['r'], best_p['alt'], best_p['v'], v3d))
            else:
                rt.miss_cnt += 1; rt.last_t = t; total_miss += 1
                v3d = rt.kf.velocity()
                track_life[rt.id].append((t, 0,0,0,v3d,1))

        # 临时航迹更新
        new_temps = []
        for tt in temp_tracks:
            lag = t - tt.last_t
            if lag > time_up or lag < time_down: continue
            dt = lag/1000.0
            best_p = None; best_d = 1e18

            if tt.state == 1:
                gate = (Vmax*dt + 150.0)
                gate_sq = gate*gate
                for p in plots:
                    if p['used']: continue
                    d_sq = sum((p['xyz'][i]-tt.p0[i])**2 for i in range(3))
                    if d_sq < gate_sq and d_sq < best_d:
                        best_d = d_sq; best_p = p
                if best_p is not None:
                    if tt.try_add_2(best_p['xyz'], dt):
                        best_p['used'] = True; new_temps.append(tt)
                    else:
                        tt.miss_cnt += 1
                        if tt.miss_cnt < 2: new_temps.append(tt)
                else:
                    tt.miss_cnt += 1
                    if tt.miss_cnt < 2 and lag <= time_up: new_temps.append(tt)
            elif tt.state == 2:
                for p in plots:
                    if p['used']: continue
                    if tt.try_add_3(p['xyz'], dt):
                        best_p = p; break
                if best_p is not None:
                    best_p['used'] = True
                    rt = RelTrack(tt, next_rid)
                    rel_tracks.append(rt)
                    starts_log.append({'t':t, 'id':next_rid, 'v':rt.kf.velocity(),
                                      'r':best_p['r'], 'alt':best_p['alt']})
                    track_life[next_rid].append((t, best_p['r'], best_p['alt'], best_p['v'], rt.kf.velocity()))
                    next_rid += 1
                else:
                    tt.miss_cnt += 1
                    if tt.miss_cnt < 2 and lag <= time_up: new_temps.append(tt)
        temp_tracks = new_temps

        # 新起始(每帧最多MAX_NEW_PER_FRAME个)
        new_count = 0
        for p in plots:
            if new_count >= MAX_NEW_PER_FRAME: break
            if not p['used']:
                tt = TempTrack(p, p['xyz'], next_tid)
                temp_tracks.append(tt)
                next_tid += 1; new_count += 1

        # 消亡
        alive = []
        for rt in rel_tracks:
            if rt.miss_cnt <= MAX_MISS:
                alive.append(rt)
            else:
                total_died += 1
        rel_tracks = alive

    # 结果统计
    print(f"\n{'='*70}")
    print(" 跟踪结果统计")
    print(f"{'='*70}")
    print(f"  临时航迹总数: {next_tid}")
    print(f"  可靠航迹起始总数: {next_rid}")
    print(f"  成功关联次数: {total_asso}")
    print(f"  漏帧预测次数: {total_miss}")
    print(f"  消亡航迹数: {total_died}")
    print(f"  当前存活可靠航迹: {len(rel_tracks)}")

    if starts_log:
        vs = [s['v'] for s in starts_log]
        rs = [s['r'] for s in starts_log]
        print(f"\n 起始航迹速度: {min(vs):.1f}~{max(vs):.1f} m/s, 均值{sum(vs)/len(vs):.1f}")
        print(f" 起始航迹距离: {min(rs):.0f}~{max(rs):.0f} m")

    # 长航迹
    long_trks = []
    for tid in track_life:
        hist = track_life[tid]
        n_real = sum(1 for h in hist if not (len(h)>5 and isinstance(h[-1],int) and h[-1]==1))
        if n_real >= 10:
            long_trks.append((tid, hist, n_real))
    long_trks.sort(key=lambda x: -x[2])

    print(f"\n 长航迹统计(有效点数>=10): {len(long_trks)}条")
    print(f" {'ID':>4} {'有效点':>5} {'总点':>5} {'起始T':>10} {'终止T':>10} {'时长(s)':>7} "
          f"{'距min':>7} {'距max':>7} {'高min':>6} {'高max':>6} {'vmin':>6} {'vmax':>6} {'平均v':>6}")
    print(" "+"-"*105)
    for tid, hist, n in long_trks[:20]:
        real_h = [h for h in hist if not (len(h)>5 and isinstance(h[-1],int) and h[-1]==1)]
        if not real_h: continue
        t0=real_h[0][0]; t1=real_h[-1][0]
        rs=[h[1] for h in real_h]; als=[h[2] for h in real_h]
        vs_real=[h[4] for h in real_h]
        print(f" {tid:>4} {n:>5} {len(hist):>5} {t0:>10} {t1:>10} {(t1-t0)/1000:>7.0f} "
              f"{min(rs):>7.0f} {max(rs):>7.0f} {min(als):>6.0f} {max(als):>6.0f} "
              f"{min(vs_real):>6.1f} {max(vs_real):>6.1f} {sum(vs_real)/len(vs_real):>6.1f}")

    # 结论
    print(f"\n{'='*70}")
    print(" 验证结论")
    print(f"{'='*70}")
    issues = []
    if next_rid == 0:
        issues.append("❌ 无法起始任何可靠航迹!")
    elif len(long_trks) == 0:
        issues.append("❌ 没有长航迹,关联失败!")
    elif len(long_trks) < 3:
        issues.append(f"⚠️ 长航迹数偏少({len(long_trks)}条),可能漏跟")
    if not issues:
        print(" ✅ 算法流程验证通过! 修正后的参数能有效跟踪无人机目标。")
        print(f"   最长航迹跟踪时长: {(long_trks[0][1][-1][0]-long_trks[0][1][0][0])/1000:.0f}秒")
    else:
        for i in issues: print(" ",i)

if __name__=='__main__':
    main()
