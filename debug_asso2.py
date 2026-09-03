# -*- coding: utf-8 -*-
"""精确模拟 track_asso.c 关联流程，找出无人机点具体被拒原因"""

import re
import math

pi = 3.141592653589793

# ==== 参数（和代码一致）
ASSO_VELOCITY_CHANGE_THRESHOLD = 8.0
PRE_GATE_DISTANCE  = 60.0
HARD_RESIDUAL_GATE = 100.0
VELOCITY_COS_THRESHOLD = 0.55
DRONE_MAX_SPEED = 20.0
COAST_TIME_THRESH = 3.0
TRACK_ASSO_TH = 11.3

sigma_r = 20.0    # 距离误差标准差（m），matrix.c 定义 sigma_r=20m
sigma_a = 0.15 * pi/180.0   # 方位误差（0.15°→rad）
sigma_b = 0.15 * pi/180.0   # 俯仰误差

# ============== 读取无人机点 ==========================
drone_points = []
with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', 'r', encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line: continue
        def get(pat, s):
            m = re.search(pat, s)
            return float(m.group(1)) if m else None
        bw = get(r'波位号:(\d+)', line)
        t  = get(r'时间戳:(\d+)', line)
        az = get(r'方位:([-\d.]+)', line)
        r  = get(r'距离:([\d.]+)', line)
        el = get(r'俯仰:([-\d.]+)', line)
        v  = get(r'速度:([-\d.]+)', line)
        if bw and t and az is not None:
            drone_points.append({'bw': int(bw), 't': t, 'az': az, 'el': el, 'r': r, 'v': v})

# ============== 精确模拟可靠航迹状态演化 ==================
# Frame38建立：Az=17.39°, El=4.77°, R=1153.1m, V=10.9m/s（CV, 沿视线方向远离）
init_az, init_el, init_r, init_v = 17.39*pi/180, 4.77*pi/180, 1153.1, 10.9
# 笛卡尔（雷达坐标系）
x0 = init_r * math.cos(init_az) * math.cos(init_el)
y0 = init_r * math.sin(init_az) * math.cos(init_el)
z0 = init_r * math.sin(init_el)
vx0 = init_v * math.cos(init_az) * math.cos(init_el)
vy0 = init_v * math.sin(init_az) * math.cos(init_el)
vz0 = init_v * math.sin(init_el)
print(f"初始化 X({x0:.1f},{y0:.1f},{z0:.1f})m, V({vx0:.2f},{vy0:.2f},{vz0:.2f})m/s")

# 用CV模型按164ms/帧，从Frame38(904372)逐帧推到Frame54(906996)共16帧
x, y, z, vx, vy, vz = x0, y0, z0, vx0, vy0, vz0
cur_mSecond = 904372.0
for f in range(38, 54):  # 推到Frame54开始时
    dt = 0.164  # 164ms/帧
    x += vx * dt
    y += vy * dt
    z += vz * dt
    cur_mSecond += 164.0
print(f"Frame54进入track_asso前: X({x:.1f},{y:.1f},{z:.1f})m, t={cur_mSecond:.0f}ms")
print(f"  Az={math.atan2(y,x)*180/pi:.2f}°, El={math.atan2(z,math.sqrt(x*x+y*y))*180/pi:.2f}°, R={math.sqrt(x*x+y*y+z*z):.1f}m")

# ============== 现在模拟Frame55 (t=907160ms) track_asso关联流程
frame_msec = 907160.0
last_update_t = cur_mSecond  # 906996ms（上一个predict_flag更新的mSecond）

# 1. 找第一个有效点的T_asso
T_asso = (frame_msec - last_update_t) / 1000.0
print(f"\n进入Frame55关联: T_asso=(907160-906996)/1000={T_asso:.3f}s")

# 2. gate计算（和track_asso.c line 144-166一致）
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
gate_sq = gate * gate
print(f"  dt_ref={dt_ref:.3f}s, gate={gate:.1f}m, az_gate={az_gate:.2f}°, el_gate={el_gate:.2f}°")

# 3. 计算参考预测 pred_x/y/z 和 pred_az/el
pred_x = x + dt_ref * vx
pred_y = y + dt_ref * vy
pred_z = z + dt_ref * vz
pred_rho = math.sqrt(pred_x*pred_x + pred_y*pred_y)
if pred_rho < 1: pred_rho = 1
pred_az = math.atan2(pred_y, pred_x) * 180/pi
pred_el = math.atan2(pred_z, pred_rho) * 180/pi
print(f"  参考预测: ({pred_x:.0f},{pred_y:.0f},{pred_z:.0f})m -> az={pred_az:.2f}°, el={pred_el:.2f}°")
prev_vel = math.sqrt(vx*vx+vy*vy+vz*vz)
print(f"  prev_vel={prev_vel:.1f}m/s")

# 4. 筛选波位2/3候选点（bw=2/3，且 t=907160/907324）
candidates = [d for d in drone_points 
              if d['bw'] in (2,3) and abs(d['t']-frame_msec) < 300]
# 将候选点按时间戳，模拟在target_data数组中的顺序（假设波位2的点先）
# 模拟杂波点先填充（例如Frame55 n=1，只有无人机点1个）
# 假设Frame55只有1个无人机点（n=1），就是bw=2的点
candidates.sort(key=lambda d: d['t'])  # 先Frame55(bw=2,t=907160)，再Frame56(bw=3,t=907324)
# 只有 t==907160 的点才在本帧
current_frame_dots = [d for d in candidates if abs(d['t']-frame_msec) < 1]

if not current_frame_dots:
    print("本帧无波位2/3无人机点！")
    exit()

print(f"\n本帧候选波位2/3无人机点: {len(current_frame_dots)}个")
for pt in current_frame_dots:
    print(f"  bw={pt['bw']}, t={pt['t']:.0f}ms, az={pt['az']:.4f}°, el={pt['el']:.4f}°, r={pt['r']:.1f}m, v_radial={pt['v']:.2f}m/s")

# ============== 对每个点模拟 track_asso 所有门限 =================================
for idx, pt in enumerate(current_frame_dots):
    print(f"\n--- 检查候选点[{idx}] bw={pt['bw']} ---")
    maz_deg = pt['az']; mel_deg = pt['el']; mr_m = pt['r']; mv = pt['v']
    # 转笛卡尔
    maz = maz_deg * pi/180.0
    mel = mel_deg * pi/180.0
    mx = mr_m * math.cos(maz) * math.cos(mel)
    my = mr_m * math.sin(maz) * math.cos(mel)
    mz = mr_m * math.sin(mel)
    mrho = math.sqrt(mx*mx + my*my)
    if mrho < 1: mrho = 1
    maz2 = math.atan2(my, mx) * 180/pi
    mel2 = math.atan2(mz, mrho) * 180/pi

    # 时间窗口
    lag_time = frame_msec - last_update_t  # 所有点都是本帧，统一
    if not (0 <= lag_time <= 25000):
        print(f"  ❌ 时间窗口不通过: lag_time={lag_time:.0f}ms")
        continue
    o_sec = lag_time / 1000.0

    # (1) 角度门
    daz = maz_deg - pred_az
    if daz > 180: daz -= 360
    if daz < -180: daz += 360
    del_ = mel_deg - pred_el
    ok1 = abs(daz) <= az_gate and abs(del_) <= el_gate
    print(f"  (1)角度门: Δaz={daz:+.3f}°(门{az_gate:.2f}°), Δel={del_:+.3f}°(门{el_gate:.2f}°) → {'✓' if ok1 else '❌'}")
    if not ok1: continue

    # (1b) 初始硬约束
    daz0 = maz_deg - (init_az*180/pi)
    if daz0 > 180: daz0 -= 360
    if daz0 < -180: daz0 += 360
    del0 = mel_deg - (init_el*180/pi)
    dr0 = abs(mr_m - init_r)
    ok1b = abs(daz0) <= 12 and abs(del0) <= 8 and dr0 <= 1500
    print(f"  (1b)初始约束: Δaz0={daz0:+.2f}°(≤12°), Δel0={del0:+.2f}°(≤8°), Δr0={dr0:.1f}m(≤1500m) → {'✓' if ok1b else '❌'}")
    if not ok1b: continue

    # (2) 目标类型速度硬检查（无人机target_type=2）
    ok2 = abs(mv) <= (DRONE_MAX_SPEED + 5.0)
    print(f"  (2)径向速度门: |v|={abs(mv):.1f}(≤25) → {'✓' if ok2 else '❌'}")
    if not ok2: continue

    # (3) 预距离门（用参考预测和观测）
    dx = mx - pred_x
    dy = my - pred_y
    dz = mz - pred_z
    dist_sq = dx*dx + dy*dy + dz*dz
    dist = math.sqrt(dist_sq)
    ok3 = dist_sq <= gate_sq
    print(f"  (3)预距离门: dist={dist:.1f}m(≤{gate:.1f}m) → {'✓' if ok3 else '❌'}")
    if not ok3: continue

    # (4) 位置差估算速度
    if o_sec > 0.01:
        est_vel = dist / o_sec
    else:
        est_vel = prev_vel
    ok4 = est_vel <= (DRONE_MAX_SPEED + ASSO_VELOCITY_CHANGE_THRESHOLD)
    print(f"  (4)估算速度门: est_vel={est_vel:.1f}m/s(≤28) → {'✓' if ok4 else '❌'}")
    if not ok4: continue

    print(f"  ✓ 所有预门通过！继续计算马氏距离 d...")

    # (5) 计算马氏距离 d = imm_d_cal() — 关键！
    # 先用 ballistic_predict 推 X, P 到 best_T=o_sec
    # CV模型简化：X_pred = [x+vx*T, vx, y+vy*T, vy, z+vz*T, vz]
    T_val = o_sec
    Xp = [0]*6
    Xp[0] = x + vx*T_val
    Xp[1] = vx
    Xp[2] = y + vy*T_val
    Xp[3] = vy
    Xp[4] = z + vz*T_val
    Xp[5] = vz

    # 观测 Z = [mx, my, mz]
    Z_obs = [mx, my, mz]
    Z_pred = [Xp[0], Xp[2], Xp[4]]  # H=[[1,0,0,0,0,0],[0,0,1,0,0,0],[0,0,0,0,1,0]]

    # R 矩阵（基于观测的球坐标转笛卡尔误差协方差）
    rho = math.sqrt(Z_obs[0]**2 + Z_obs[1]**2 + Z_obs[2]**2)
    theta = math.atan2(Z_obs[1], Z_obs[0])   # azimuth (rad)
    eps = math.atan2(Z_obs[2], math.sqrt(Z_obs[0]**2 + Z_obs[1]**2))  # elevation (rad)
    ct, st = math.cos(theta), math.sin(theta)
    ce, se = math.cos(eps), math.sin(eps)
    sr2, sa2, sb2 = sigma_r**2, sigma_a**2, sigma_b**2
    # 偏导 d(X,Y,Z)/d(r,theta,eps)
    dxdr, dxdt, dxde = ct*ce, -rho*st*ce, -rho*ct*se
    dydr, dydt, dyde = st*ce,  rho*ct*ce, -rho*st*se
    dzdr, dzde        = se,     rho*ce
    R = [[0.0]*3 for _ in range(3)]
    R[0][0] = dxdr*dxdr*sr2 + dxdt*dxdt*sa2 + dxde*dxde*sb2
    R[0][1] = dxdr*dydr*sr2 + dxdt*dydt*sa2 + dxde*dyde*sb2
    R[0][2] = dxdr*dzdr*sr2 + dxde*dzde*sb2
    R[1][0] = R[0][1]
    R[1][1] = dydr*dydr*sr2 + dydt*dydt*sa2 + dyde*dyde*sb2
    R[1][2] = dydr*dzdr*sr2 + dyde*dzde*sb2
    R[2][0] = R[0][2]; R[2][1] = R[1][2]
    R[2][2] = dzdr*dzdr*sr2 + dzde*dzde*sb2

    # P 矩阵（CV模型过程噪声的协方差传播简化，从初始化P开始）
    # 初始化P：kalman_filter_init 产生的。这里我们假设一个典型初始P：~(sigma_r)^2位置
    # 真实代码中：从new_reliable带过来的6×6 P矩阵
    # 用初始化典型值：位置方差 400 (20m)^2，速度方差 ~ (20m/s / T_init)^2 ~ ~250
    # 为真实模拟，假设：P 从 Frame38 开始按 Q 累积 T=2.788s×(55-38)步
    # 简化：直接取 sigma_cv=1.0f 的过程噪声累积（imm.c CV sigma_cv=2.0？）
    sigma_cv_sq = 2.0**2  # CV 模型 sigma_cv=2.0
    Ts2 = T_val**2
    Ts3 = Ts2 * T_val / 2.0
    Ts4 = Ts2 * Ts2 / 4.0
    # 用一个近似初始P（6x6）
    P = [[0.0]*6 for _ in range(6)]
    init_pos_var = sigma_r**2
    init_vel_var = 100.0  # (10m/s)^2
    for i in range(6):
        P[i][i] = init_pos_var if i%2==0 else init_vel_var
    # 加累积过程噪声Q
    Q_acc = sigma_cv_sq * (1.0 if T_val < 3 else 3.0)  # 简化
    for i in range(3):
        P[2*i][2*i]     += Q_acc * Ts4  # pos
        P[2*i][2*i + 1] += Q_acc * Ts3  # cross
        P[2*i + 1][2*i] += Q_acc * Ts3  # symmetric
        P[2*i + 1][2*i + 1] += Q_acc * Ts2  # vel

    # S = H*P*H' + R (H 选择 [0, 2, 4]位置分量)
    def get_H_subset_P(P):
        S = [[0.0]*3 for _ in range(3)]
        idx = [0, 2, 4]  # x, y, z 分量在 X[6] 中的索引
        for i in range(3):
            for j in range(3):
                S[i][j] = P[idx[i]][idx[j]] + R[i][j]
        return S
    S = get_H_subset_P(P)

    # 计算 S^-1 （3x3）
    def mat3_inv(A):
        a, b, c = A[0]
        d, e, f = A[1]
        g, h, i = A[2]
        det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g)
        if abs(det) < 1e-20: return None
        inv_det = 1.0/det
        return [
            [(e*i-f*h)*inv_det, (c*h-b*i)*inv_det, (b*f-c*e)*inv_det],
            [(f*g-d*i)*inv_det, (a*i-c*g)*inv_det, (c*d-a*f)*inv_det],
            [(d*h-e*g)*inv_det, (b*g-a*h)*inv_det, (a*e-b*d)*inv_det]
        ]
    S_inv = mat3_inv(S)
    if S_inv is None:
        print(f"  ❌ 马氏距离奇异！跳过")
        continue

    nu = [Z_obs[k] - Z_pred[k] for k in range(3)]
    # d = nu' * S_inv * nu
    d_val = 0.0
    for i in range(3):
        row_sum = 0.0
        for j in range(3):
            row_sum += nu[j] * S_inv[j][i]
        d_val += row_sum * nu[i]
    ok5 = d_val < TRACK_ASSO_TH
    print(f"  (5)马氏距离: d={d_val:.2f} (阈值 TRACK_ASSO_TH={TRACK_ASSO_TH:.1f}, 残差=({nu[0]:.1f},{nu[1]:.1f},{nu[2]:.1f})m) → {'✓' if ok5 else '❌ FAIL (核心关联被拒!)'}")

    # (6)最终速度检查 & vcos
    norm = est_vel
    vel_change = abs(norm - prev_vel)
    residual_dist = dist
    if prev_vel > 0.5 and norm > 0.5:
        ovx, ovy, ovz = dx/o_sec, dy/o_sec, dz/o_sec
        vcos = (vx*ovx + vy*ovy + vz*ovz) / (prev_vel * norm + 1e-6)
    else:
        vcos = 1.0
    ok_norm = 2.0 <= norm <= DRONE_MAX_SPEED
    ok_vchange = vel_change <= ASSO_VELOCITY_CHANGE_THRESHOLD
    ok_res = residual_dist <= HARD_RESIDUAL_GATE
    ok_vcos = vcos >= VELOCITY_COS_THRESHOLD
    print(f"  (6)最终: norm={norm:.1f}(≤20)→{'✓' if ok_norm else '❌'}, Δv={vel_change:.1f}(≤8)→{'✓' if ok_vchange else '❌'}")
    print(f"     残差={residual_dist:.1f}m(≤100)→{'✓' if ok_res else '❌'}, vcos={vcos:.2f}(≥0.55)→{'✓' if ok_vcos else '❌'}")
    all_ok = ok5 and ok_norm and ok_vchange and ok_res and ok_vcos
    print(f"  ⇒ 综合: {'✅ 关联成功!' if all_ok else '❌ 关联失败'}")
