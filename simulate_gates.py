import re, math

# 1. 解析data_entry.c的输入点数据（gen_data_entry.py写的格式），先看无人机实测数据
# 直接从无人机实测数据.txt获取时间戳匹配点
meas_pts = {}  # t -> list of dicts
with open('无人机实测数据.txt', 'r', encoding='utf-8', errors='replace') as f:
    for line in f:
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
            meas_pts[t].append({'bw':bw,'az':az,'el':el,'r':r,'v':v})

# 2. 航迹初始化状态(来自Frame38输出)
init_t = 904372  # Frame 38 t=ms
init_az = 17.39   # deg
init_el = 4.77    # deg
init_r = 1153.1   # m
init_v = 10.9     # m/s (3D合速度)
# 无人机远离方向，用球面坐标近似：用径向速度≈合速度近似
# 3D速度方向: init_az/init_el方向

# 假设速度方向和位置方向一致(标准CV假设)
def pred_az_el_r_v(t_ms):
    """预测t时刻的方位/俯仰/距离/3D速度"""
    dt_s = (t_ms - init_t) / 1000.0
    # 简单恒速外推(无人机init_r方向)
    dr = init_v * dt_s  # 远离方向距离增量
    pred_r = init_r + dr
    # 方位俯仰近似不变(无人机无明显方向变化，只有远离/靠近)
    pred_az = init_az
    pred_el = init_el
    pred_v = init_v
    return pred_az, pred_el, pred_r, pred_v, dt_s

# 3. 精确检查关键帧门限：Frame 55/56等时间戳
check_ts = [907160, 907324, 909948]  # Frame55/56/72 对应 t
print("=== 模拟track_asso门限检查（关键帧）===")
print(f"航迹初始化: t={init_t}ms, Az={init_az}°, El={init_el}°, R={init_r}m, V={init_v}m/s")
print()

# 我们的门限常量(与代码一致)
PRE_GATE_DISTANCE = 60.0
HARD_RESIDUAL_GATE = 100.0
AZ_GATE_BASE = 3.0
AZ_GATE_GROW = 1.2  # deg/s
AZ_GATE_MAX = 6.0
DRONE_MAX_SPEED = 20.0
ASSO_VEL_CHANGE = 8.0
MAX_AZ_DEV_FROM_INIT = 12.0
MAX_EL_DEV_FROM_INIT = 8.0
MAX_R_DEV_FROM_INIT = 1500.0
COAST_THRESH = 3.0  # seconds

for t_ms in sorted(meas_pts.keys()):
    if t_ms < 904372 or t_ms > 915000: continue  # 只看Frame38附近
    pts = meas_pts[t_ms]
    pred_az, pred_el, pred_r, pred_v, dt_s = pred_az_el_r_v(t_ms)
    # 找波位2/3
    bw23_pts = [p for p in pts if p['bw'] in (2,3)]
    if not bw23_pts: continue
    if dt_s < 0.5 or dt_s > 15: continue  # 只看有意义的间隔

    print(f"--- t={t_ms}ms, ΔT={dt_s:.2f}s ---")
    print(f"  预测: Az={pred_az:.2f}°, El={pred_el:.2f}°, R={pred_r:.1f}m, V={pred_v:.1f}m/s")

    # 计算门限
    if dt_s < COAST_THRESH:
        gate = PRE_GATE_DISTANCE + dt_s * 12.0
        if gate > 150: gate = 150
        az_gate = AZ_GATE_BASE + dt_s * AZ_GATE_GROW
        if az_gate > AZ_GATE_MAX: az_gate = AZ_GATE_MAX
        el_gate = az_gate
    else:
        gate = 120
        az_gate = 5.0
        el_gate = 5.0
    print(f"  门限: gate={gate:.1f}m, az_gate={az_gate:.2f}°, el_gate={el_gate:.2f}°")
    print(f"  绝对约束: |ΔAz|<{MAX_AZ_DEV_FROM_INIT}°, |ΔEl|<{MAX_EL_DEV_FROM_INIT}°, |ΔR|<{MAX_R_DEV_FROM_INIT}m")

    # 检查每个波位2/3点
    for p in bw23_pts:
        daz = p['az'] - pred_az
        if daz > 180: daz -= 360
        if daz < -180: daz += 360
        del_ = p['el'] - pred_el
        # 3D距离差(近似用dr + 角度差贡献)
        dr_3d = math.sqrt((p['r']-pred_r)**2 + (pred_r*math.pi/180*daz)**2 + (pred_r*math.pi/180*del_)**2)
        # 估算速度
        est_v = dr_3d / dt_s if dt_s > 0.01 else 0
        # 各层门限检查
        check_ang = abs(daz) < az_gate and abs(del_) < el_gate
        check_gate = dr_3d < gate
        check_abs_az = abs(p['az'] - init_az) < MAX_AZ_DEV_FROM_INIT
        check_abs_el = abs(p['el'] - init_el) < MAX_EL_DEV_FROM_INIT
        check_abs_r = abs(p['r'] - init_r) < MAX_R_DEV_FROM_INIT
        check_vel_pt = p['v'] < (DRONE_MAX_SPEED + 5)
        check_vel_est = est_v < (DRONE_MAX_SPEED + ASSO_VEL_CHANGE)
        # 波位速度变化量
        vel_change = abs(est_v - pred_v)
        check_vchange = vel_change < ASSO_VEL_CHANGE

        overall = check_ang and check_gate and check_abs_az and check_abs_el and check_abs_r and check_vel_pt and check_vel_est and check_vchange

        mark_p = "" if overall else "  **被拒**"
        reason = []
        if not check_ang: reason.append(f"角度门ΔAz={daz:.2f}/ΔEl={del_:.2f}超{az_gate:.1f}°")
        if not check_gate: reason.append(f"距门{dr_3d:.1f}>{gate:.1f}m")
        if not check_abs_az: reason.append("绝对Az超")
        if not check_abs_el: reason.append(f"绝对El(Δ={p['el']-init_el:+.2f})超")
        if not check_abs_r: reason.append(f"绝对R(Δ={p['r']-init_r:+.0f}m)超")
        if not check_vel_pt: reason.append(f"点V={p['v']:.1f}>25")
        if not check_vel_est: reason.append(f"估V={est_v:.1f}>{DRONE_MAX_SPEED+ASSO_VEL_CHANGE:.1f}")
        if not check_vchange: reason.append(f"V突变={vel_change:.1f}>{ASSO_VEL_CHANGE:.1f}")
        reason_str = "; ".join(reason) if reason else "全部通过 ✓"

        print(f"  点bw{p['bw']}: Az={p['az']:.2f} El={p['el']:.2f} R={p['r']:.0f}m V={p['v']:.1f}m/s")
        print(f"    ΔAz={daz:+.2f}° ΔEl={del_:+.2f}° ΔR={p['r']-pred_r:+.1f}m 3D={dr_3d:.1f}m 估V={est_v:.1f}m/s V突变={vel_change:.1f}")
        print(f"    检查结果: {reason_str}{mark_p}")

# 4. 检查Frame 226异常点是否会被拦截
print()
print("=== Frame 226 (t=935204ms, ΔT=30.83s) 异常点拦截验证 ===")
hijack_pts = meas_pts.get(935204, [])
pred_az, pred_el, pred_r, pred_v, dt_s = pred_az_el_r_v(935204)
print(f"  预测(即使漂移也按init): Az={pred_az:.2f}, El={pred_el:.2f}, R={init_r+10.9*30.83:.1f}m, V={pred_v}")
print(f"  ΔT={dt_s:.2f}s, coast模式")
print(f"  绝对约束: |ΔAz|<{MAX_AZ_DEV_FROM_INIT}°, |ΔEl|<{MAX_EL_DEV_FROM_INIT}°, |ΔR|<{MAX_R_DEV_FROM_INIT}m, V<25m/s")
# 用输出中实际的劫持点值(AZ=24.42, El=27.91, R=3729.7, V=56.0)
hz = {'az':24.42, 'el':27.91, 'r':3729.7, 'v':63.3}  # 63.3匹配的实测V
daz0 = hz['az']-init_az; del0=hz['el']-init_el; dr0=hz['r']-init_r
print(f"  劫持点: Az={hz['az']:.2f}(Δ={daz0:+.2f}°), El={hz['el']:.2f}(Δ={del0:+.2f}°), R={hz['r']:.0f}(Δ={dr0:+.0f}m), V={hz['v']:.1f}")
checks = []
checks.append( (abs(daz0) < MAX_AZ_DEV_FROM_INIT, f"ΔAz <12°", abs(daz0) < MAX_AZ_DEV_FROM_INIT) )
checks.append( (abs(del0) < MAX_EL_DEV_FROM_INIT, f"ΔEl <8°", abs(del0) < MAX_EL_DEV_FROM_INIT) )
checks.append( (abs(dr0) < MAX_R_DEV_FROM_INIT, f"|ΔR| <1500m", abs(dr0) < MAX_R_DEV_FROM_INIT) )
checks.append( (hz['v'] < 25, f"点V <25m/s", hz['v']<25) )
print(f"  拦截检查:")
for cond, desc, val in checks:
    print(f"    {desc}: {'通过 ✓' if val else f'拦截 ✗ (值不满足)'}")
