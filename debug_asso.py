# -*- coding: utf-8 -*-
"""分析Frame55/56/72为什么正确点没关联上"""

import re
import math

pi = 3.141592653589793

# =============== 参数 ========================================
ASSO_VELOCITY_CHANGE_THRESHOLD = 8.0
PRE_GATE_DISTANCE  = 60.0
HARD_RESIDUAL_GATE = 100.0
VELOCITY_COS_THRESHOLD = 0.55
DRONE_MAX_SPEED = 20.0
COAST_TIME_THRESH = 3.0
TRACK_ASSO_TH = 11.3

sigma_r = 20.0    # matrix.c定义
sigma_a = 0.15 * pi/180.0
sigma_b = 0.15 * pi/180.0

# =============== 读取无人机实测数据 ==========================
drone_points = []
with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', 'r', encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        # 提取字段
        def get_field(pat, s):
            m = re.search(pat, s)
            return float(m.group(1)) if m else None
        bw = get_field(r'波位号:(\d+)', line)
        t  = get_field(r'时间戳:(\d+)', line)
        az = get_field(r'方位:([-\d.]+)', line)
        r  = get_field(r'距离:([\d.]+)', line)
        el = get_field(r'俯仰:([-\d.]+)', line)
        v  = get_field(r'速度:([-\d.]+)', line)
        if bw is not None and t is not None and az is not None:
            drone_points.append({
                'bw': int(bw), 't': t, 'az': az, 'el': el, 'r': r, 'v': v
            })

# =============== Frame38建立时的航迹状态 ======================
# Frame38 t=904372ms: Az=17.39, El=4.77, R=1153.1, V=10.9m/s
# 笛卡尔: 初始化后状态
init_az_deg = 17.39
init_el_deg = 4.77
init_r_m    = 1153.1
init_v      = 10.9  # m/s，方向向外（远离雷达）

init_az_rad = init_az_deg * pi/180.0
init_el_rad = init_el_deg * pi/180.0

# 初始X1[6]（假设速度沿视线方向向外）
# 坐标系与track.c一致：方位角在 track.c 中使用 (pi/2 - azi) 计算
# 这里简化：X = R*cos(azi)*cos(eli), Y=R*sin(azi)*cos(eli), Z=R*sin(eli)
# 速度沿视线方向向外 => Vx=V*cos(az)*cos(el), Vy=V*sin(az)*cos(el), Vz=V*sin(el)
cur_x0 = init_r_m * math.cos(init_az_rad) * math.cos(init_el_rad)
cur_y0 = init_r_m * math.sin(init_az_rad) * math.cos(init_el_rad)
cur_z0 = init_r_m * math.sin(init_el_rad)
cur_vx0 = init_v * math.cos(init_az_rad) * math.cos(init_el_rad)
cur_vy0 = init_v * math.sin(init_az_rad) * math.cos(init_el_rad)
cur_vz0 = init_v * math.sin(init_el_rad)

init_xyz = (cur_x0, cur_y0, cur_z0)
init_vxyz = (cur_vx0, cur_vy0, cur_vz0)
print(f"初始化航迹: az={init_az_deg:.2f}°, el={init_el_deg:.2f}°, r={init_r_m:.1f}m, v={init_v:.1f}m/s")
print(f"  X=({cur_x0:.1f}, {cur_y0:.1f}, {cur_z0:.1f})m, V=({cur_vx0:.2f}, {cur_vy0:.2f}, {cur_vz0:.2f})m/s")

# =============== 预测到Frame55/56/72时的位置 ======================
def predict_to(cur_x, cur_y, cur_z, cvx, cvy, cvz, T_sec):
    """纯预测位置（CV模型，无噪声）"""
    return (cur_x + cvx*T_sec,
            cur_y + cvy*T_sec,
            cur_z + cvz*T_sec)

def xyz2az_el_r(x, y, z):
    rho = math.sqrt(x*x + y*y)
    if rho < 1.0: rho = 1.0
    az = math.atan2(y, x) * 180.0/pi
    el = math.atan2(z, rho) * 180.0/pi
    r  = math.sqrt(x*x + y*y + z*z)
    return az, el, r

def calc_gate(T_asso):
    if T_asso < COAST_TIME_THRESH:
        dt_ref = T_asso
        gate = PRE_GATE_DISTANCE + T_asso * 12.0
        if gate > 150.0: gate = 150.0
        az_gate = 3.0 + T_asso * 1.2
        if az_gate > 6.0: az_gate = 6.0
        el_gate = 3.0 + T_asso * 1.2
        if el_gate > 6.0: el_gate = 6.0
    else:
        dt_ref = COAST_TIME_THRESH * 0.5
        gate = 120.0
        az_gate = 5.0
        el_gate = 5.0
    return dt_ref, gate, az_gate, el_gate

def analyze_frame(frame_name, t_frame, init_t=904372.0):
    """分析某一帧能否关联到无人机点"""
    print(f"\n{'='*70}")
    print(f"分析 {frame_name} (t={t_frame:.0f}ms)")
    T_asso = (t_frame - init_t) / 1000.0
    print(f"  距上更新 T_asso = {T_asso:.2f}s ({T_asso/2.788:.1f}个扫描周期)")
    
    dt_ref, gate, az_gate, el_gate = calc_gate(T_asso)
    print(f"  门限: gate={gate:.1f}m, az_gate={az_gate:.1f}°, el_gate={el_gate:.1f}°, dt_ref={dt_ref:.2f}s")
    
    # 预测到当前（纯CV，无更新）
    # 注意：上更新时间戳是 init_t；但中间predict_flag时，track_asso用的是上次reliable更新的mSecond
    # 实际上Frame38建立后mSecond=904372，到Frame55时 T_asso = (907160-904372)/1000 = 2.788s
    # 预测位置：由于没有更新，一直用纯CV
    px, py, pz = predict_to(cur_x0, cur_y0, cur_z0, cur_vx0, cur_vy0, cur_vz0, T_asso)
    p_az, p_el, p_r = xyz2az_el_r(px, py, pz)
    prev_vel = math.sqrt(cur_vx0**2 + cur_vy0**2 + cur_vz0**2)
    print(f"  预测位置: az={p_az:.2f}°, el={p_el:.2f}°, r={p_r:.1f}m, v={prev_vel:.1f}m/s")
    
    # 找所有波位2/3且时间戳在t_frame±500ms的无人机点
    candidates = [d for d in drone_points 
                  if (d['bw'] in (2,3)) and abs(d['t'] - t_frame) < 500]
    if not candidates:
        print(f"  ❌ 波位2/3无无人机点！时间窗口内无匹配")
        return
    
    for d in candidates:
        print(f"\n  --- 候选点: bw={d['bw']}, t={d['t']:.0f}ms ---")
        print(f"      实测: az={d['az']:.4f}°, el={d['el']:.4f}°, r={d['r']:.1f}m, v={d['v']:.2f}m/s (径向)")
        
        # 转为笛卡尔坐标
        maz_rad = d['az']*pi/180.0
        mel_rad = d['el']*pi/180.0
        mx = d['r'] * math.cos(maz_rad)*math.cos(mel_rad)
        my = d['r'] * math.sin(maz_rad)*math.cos(mel_rad)
        mz = d['r'] * math.sin(mel_rad)
        T_dot = (d['t'] - init_t) / 1000.0
        if T_dot < 0.001: T_dot = T_asso
        print(f"      时间差: {T_dot:.3f}s (T_asso用于关联)")
        
        # (1) 角度门检查
        daz = d['az'] - p_az
        if daz > 180: daz -= 360
        if daz < -180: daz += 360
        del_ = d['el'] - p_el
        az_pass = abs(daz) <= az_gate
        el_pass = abs(del_) <= el_gate
        print(f"      (1)角度门: Δaz={daz:+.2f}° (门={az_gate:.1f}°) → {'✓' if az_pass else '❌ FAIL'}")
        print(f"         Δel={del_:+.2f}° (门={el_gate:.1f}°) → {'✓' if el_pass else '❌ FAIL'}")
        
        # 初始角度硬约束（init_az=17.39, init_el=4.77, 门±12°/±8°）
        daz0 = d['az'] - init_az_deg
        if daz0 > 180: daz0 -= 360
        if daz0 < -180: daz0 += 360
        del0 = d['el'] - init_el_deg
        abs_az_ok = abs(daz0) <= 12.0
        abs_el_ok = abs(del0) <= 8.0
        dr_ok = abs(d['r'] - init_r_m) <= 1500.0
        print(f"      (1b)初始硬约束: Δaz0={daz0:+.2f}° (≤12°)→{'✓' if abs_az_ok else '❌ FAIL'}, Δel0={del0:+.2f}° (≤8°)→{'✓' if abs_el_ok else '❌ FAIL'}, Δr={abs(d['r']-init_r_m):.1f}m (≤1500m)→{'✓' if dr_ok else '❌ FAIL'}")
        
        # (2) 速度门：径向速度绝对值
        v_radial_abs = abs(d['v'])
        v_drone_ok = v_radial_abs <= (DRONE_MAX_SPEED + 5.0)
        print(f"      (2)径向速度门: |v|={v_radial_abs:.1f}m/s (≤{DRONE_MAX_SPEED+5:.0f}) → {'✓' if v_drone_ok else '❌ FAIL'}")
        
        # (3) 预距离门 (用dt_ref预测的位置，与代码track_asso.c:144-166一致)
        # 但注意代码用的是 mSecond 差 (d['t'] - init_t) 计算T_asso传入
        _, gate_here, _, _ = calc_gate(T_dot)
        # 实际用 dt_ref 预测
        px_ref, py_ref, pz_ref = predict_to(cur_x0, cur_y0, cur_z0, cur_vx0, cur_vy0, cur_vz0, dt_ref if T_dot < COAST_TIME_THRESH else COAST_TIME_THRESH*0.5)
        dx = mx - px_ref
        dy = my - py_ref
        dz = mz - pz_ref
        dist_sq = dx*dx + dy*dy + dz*dz
        gate_here_sq = gate_here * gate_here
        dist_gate_ok = dist_sq <= gate_here_sq
        dist_m = math.sqrt(dist_sq)
        print(f"      (3)预距离门: pred({px_ref:.0f},{py_ref:.0f},{pz_ref:.0f}) vs meas({mx:.0f},{my:.0f},{mz:.0f})")
        print(f"         距离={dist_m:.1f}m (门={gate_here:.1f}m) → {'✓' if dist_gate_ok else '❌ FAIL'}")
        
        # (4) 提前物理速度检查：用位置差/时间差 估算速度
        if T_dot > 0.01:
            est_vel = math.sqrt(dx*dx + dy*dy + dz*dz) / T_dot
        else:
            est_vel = prev_vel
        est_vel_ok = est_vel <= (DRONE_MAX_SPEED + ASSO_VELOCITY_CHANGE_THRESHOLD)
        print(f"      (4)位置差估算速度: {est_vel:.1f}m/s (门≤{DRONE_MAX_SPEED+ASSO_VELOCITY_CHANGE_THRESHOLD:.0f}) → {'✓' if est_vel_ok else '❌ FAIL'}")
        
        # (5) 最终更新检查：norm, vel_change, residual_dist, vcos
        # 简化：假设关联后状态 X_fused 若用观测则速度 ~ 位置差速度
        # 速度变化 norm - prev_vel
        norm = est_vel
        vel_change = abs(norm - prev_vel)
        vel_norm_ok = 2.0 <= norm <= DRONE_MAX_SPEED
        vel_change_ok = vel_change <= ASSO_VELOCITY_CHANGE_THRESHOLD
        print(f"      (5)最终速度约束: norm={norm:.1f} (2..{DRONE_MAX_SPEED})→{'✓' if vel_norm_ok else '❌ FAIL'}")
        print(f"         Δvel={vel_change:.1f}m/s (≤{ASSO_VELOCITY_CHANGE_THRESHOLD})→{'✓' if vel_change_ok else '❌ FAIL'}")
        print(f"         残差={dist_m:.1f}m (≤{HARD_RESIDUAL_GATE})→{'✓' if dist_m <= HARD_RESIDUAL_GATE else '❌ FAIL'}")
        # vcos: 预测速度方向 vs 观测位置差方向
        if prev_vel > 0.5 and est_vel > 0.5:
            vcos = (cur_vx0*dx + cur_vy0*dy + cur_vz0*dz) / (prev_vel * est_vel * T_dot if T_dot>0.01 else 1)
            # 注意：这里用dx/dy/dz直接除以T_dot得到观测速度向量
            ovx = dx/T_dot if T_dot>0.01 else 0
            ovy = dy/T_dot if T_dot>0.01 else 0
            ovz = dz/T_dot if T_dot>0.01 else 0
            vcos_correct = (cur_vx0*ovx + cur_vy0*ovy + cur_vz0*ovz)/(prev_vel*math.sqrt(ovx*ovx+ovy*ovy+ovz*ovz)+1e-6)
            print(f"         vcos={vcos_correct:.2f} (≥{VELOCITY_COS_THRESHOLD})→{'✓' if vcos_correct>=VELOCITY_COS_THRESHOLD else '❌ FAIL'}")

# Frame55 (t=907160): 波位2
analyze_frame("Frame055 (波位2)", 907160.0)
# Frame56 (t=907324): 波位3
analyze_frame("Frame056 (波位3)", 907324.0)
# Frame72 (t=909948): 波位2/3？
analyze_frame("Frame072 (下一圈波位2/3)", 909948.0)
