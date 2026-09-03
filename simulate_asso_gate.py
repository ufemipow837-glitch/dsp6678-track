# -*- coding: utf-8 -*-
"""
模拟track_asso.c的所有门检查，找Frame55点被拒的具体层
输入：F38初始化状态 + Frame 55点迹数据
输出：每层门检查通过/拒绝详情
"""
import math

pi = 3.141592653589793

# === 门限常量（与track_asso.c L21-35严格一致） ===
ASSO_VELOCITY_CHANGE_THRESHOLD = 12.0
PRE_GATE_DISTANCE  = 100.0
PRE_GATE_COEFF     = 25.0
PRE_GATE_MAX      = 1000.0
HARD_RESIDUAL_GATE = 150.0
VELOCITY_COS_THRESHOLD = 0.40
DRONE_MAX_SPEED = 20.0
COAST_TIME_THRESH = 3.0
BEAM_PENALTY_COEFF = 1.5
DRONE_RANGE_MIN     = 0.0
DRONE_RANGE_MAX  = 6000.0
VEL_GATE_BASE      = 12.0
VEL_GATE_COEFF      = 2.0
VEL_GATE_MAX       = 30.0

TRACK_ASSO_TH = 11.3

# === Frame 38初始化的航迹状态（输出数据） ===
# Track[0]: Az=17.39deg El=4.77deg R=1153.1m V=10.9m/s
# 注意：输出数据中的V=10.9是合速度（绝对值）
# 现在要反推笛卡尔坐标X1[0..5]
init_az_deg = 17.39
init_el_deg = 4.77
init_r = 1153.1
init_spd = 10.9  # 合速度（远离方向）
init_t_ms = 904372  # Frame38时间戳

# 反推X,Y,Z：Az是方位角（X轴0°，逆时针？）
# track.c L293-295 坐标转换公式：
# X = r * cos(π/2 - azi) * cos(ele) = r * sin(azi) * cos(ele)
# Y = r * sin(π/2 - azi) * cos(ele) = r * cos(azi) * cos(ele)
# Z = r * sin(ele)
# 注意：azi是弧度
def radec2xyz(azi_deg, ele_deg, r):
    azi = azi_deg * pi / 180.0
    ele = ele_deg * pi / 180.0
    x = r * math.sin(azi) * math.cos(ele)
    y = r * math.cos(azi) * math.cos(ele)
    z = r * math.sin(ele)
    return x, y, z

# 初始化位置
X0, Y0, Z0 = radec2xyz(init_az_deg, init_el_deg, init_r)
print(f"初始化位置 (Frame38): X={X0:.2f}, Y={Y0:.2f}, Z={Z0:.2f}, R={init_r:.1f}m")
print(f"初始化方位: Az={init_az_deg:.2f}°, El={init_el_deg:.2f}°")
print(f"初始化速度: 合速度={init_spd:.2f}m/s，方向: 远离雷达（径向向外）")

# 初始化速度：假设纯CV模型，速度方向=径向向外（R递增，远离）
# 即 Vx, Vy, Vz 指向 (X0,Y0,Z0) 方向，合速度=10.9
# 径向单位向量
pr = math.sqrt(X0**2 + Y0**2 + Z0**2)
if pr < 1.0: pr = 1.0
er_x = X0 / pr
er_y = Y0 / pr
er_z = Z0 / pr
Vx = er_x * init_spd
Vy = er_y * init_spd
Vz = er_z * init_spd
print(f"初始化速度向量: Vx={Vx:.3f}, Vy={Vy:.3f}, Vz={Vz:.3f}")
vr_init = Vx*er_x + Vy*er_y + Vz*er_z
print(f"初始化径向速度投影 vr_pred = {vr_init:.2f}m/s (正=远离)")

# === 假设Frame55点迹（实测data_entry.c中的值）===
# Frame 55: t=907160ms, n=1
# 从输出数据看：Frame 55 Track[0]预测值：Az=17.60°, El=5.07°, R=1213.3m
# 先找data_entry.c Frame55（frameSn=55）的点
# 更直接地：Frame4 (t=898796ms) 已经是beam2无人机点 azi=16.82°, R=1094.84, V=+11.74
# 我们推算Frame55应该有无人机点，用线性插值推算
F55_t_ms = 907160
T_since_init = (F55_t_ms - init_t_ms) / 1000.0
print(f"\nFrame55时间: {F55_t_ms}ms, 距初始化 T_asso = {T_since_init:.3f}s")

# 先算Frame55的航迹预测位置（纯CV外推）
pred_x = X0 + T_since_init * Vx
pred_y = Y0 + T_since_init * Vy
pred_z = Z0 + T_since_init * Vz
pred_r = math.sqrt(pred_x**2 + pred_y**2 + pred_z**2)
pred_rho = math.sqrt(pred_x**2 + pred_y**2)
pred_az = math.atan2(pred_y, pred_x) * 180.0 / pi
pred_el = math.atan2(pred_z, pred_rho) * 180.0 / pi
print(f"\n航迹预测位置（Frame55）: Az={pred_az:.2f}°, El={pred_el:.2f}°, R={pred_r:.1f}m")
print(f"  笛卡尔: X={pred_x:.1f}, Y={pred_y:.1f}, Z={pred_z:.1f}")

# 预测径向速度（不变，CV模型）
vr_pred = Vx * (pred_x/pred_r) + Vy * (pred_y/pred_r) + Vz * (pred_z/pred_r)
print(f"  预测径向速度 vr_pred = {vr_pred:.2f}m/s")

# ====== 算所有门 ======
gate = PRE_GATE_DISTANCE + T_since_init * PRE_GATE_COEFF
if gate > PRE_GATE_MAX: gate = PRE_GATE_MAX
az_gate = 5.0 + T_since_init * 2.0
if az_gate > 20.0: az_gate = 20.0
el_gate = 5.0 + T_since_init * 2.0
if el_gate > 20.0: el_gate = 20.0
vel_gate = VEL_GATE_BASE + VEL_GATE_COEFF * T_since_init
if vel_gate > VEL_GATE_MAX: vel_gate = VEL_GATE_MAX

print(f"\n===== 各门限值（T_asso={T_since_init:.2f}s） =====")
print(f"  预筛距离门: {PRE_GATE_DISTANCE} + {PRE_GATE_COEFF}×{T_since_init:.1f} = {gate:.1f}m (上限{PRE_GATE_MAX}m)")
print(f"  方位门: {az_gate:.1f}°, 俯仰门: {el_gate:.1f}°")
print(f"  速度门: {VEL_GATE_BASE}+{VEL_GATE_COEFF}×{T_since_init:.1f} = {vel_gate:.1f}m/s (上限{VEL_GATE_MAX}m/s)")

# ====== 假设Frame55的真实无人机点（用实测数据特征）======
# 输出数据Frame55: Track预测 R=1213.3m, Az=17.60°, El=5.07°
# 真实无人机点（推测）: 应在预测位置附近，R=1210~1220m, Az≈17.6°, El≈5.1°, Vr≈+11.7m/s
# 先测试一个与预测几乎重合的点，看是否能过
print(f"\n===== 测试1：理想无人机点（几乎在预测位置） =====")
dot_az = 17.60  # 与预测一致
dot_el = 5.07
dot_r = 1215.0
dot_vr = 11.74  # 正，远离（与data_entry.c F4一致）
dx, dy, dz = radec2xyz(dot_az, dot_el, dot_r)
print(f"点迹: Az={dot_az}°, El={dot_el}°, R={dot_r:.1f}m, Vr={dot_vr:.2f}m/s, xyz=({dx:.1f},{dy:.1f},{dz:.1f})")

failed_at = None
# 第1层：粗筛硬约束（ttype==2）
init_az = 17.39
init_el = 4.77
daz0 = dot_az - init_az
del0 = dot_el - init_el
if daz0 > 180: daz0 -= 360
if daz0 < -180: daz0 += 360
print(f"\n[L1-粗筛硬约束] ttype=2 无人机:")
print(f"  方位差ΔAz0(相对init)={daz0:.2f}° 限±25° → {'✓' if abs(daz0)<=25 else '✗ 拒绝'}")
print(f"  俯仰差ΔEl0(相对init)={del0:.2f}° 限±20° → {'✓' if abs(del0)<=20 else '✗ 拒绝'}")
print(f"  绝对距离R={dot_r:.1f}m 限0~6000 → {'✓' if 0<=dot_r<=6000 else '✗ 拒绝'}")
print(f"  点迹径向速度|Vr|={abs(dot_vr):.2f}m/s 限≤30 → {'✓' if abs(dot_vr)<=30 else '✗ 拒绝'}")
if not (abs(daz0)<=25 and abs(del0)<=20 and 0<=dot_r<=6000 and abs(dot_vr)<=30):
    failed_at = "L1"

# 第2层：方位/俯仰门（相对预测值）
if not failed_at:
    mrho = math.sqrt(dx*dx + dy*dy)
    maz = math.atan2(dy, dx) * 180.0 / pi
    mel = math.atan2(dz, mrho) * 180.0 / pi
    daz = maz - pred_az
    if daz > 180: daz -= 360
    if daz < -180: daz += 360
    del2 = mel - pred_el
    print(f"\n[L2-方位/俯仰门] 相对预测:")
    print(f"  点迹 Az={maz:.3f}°, El={mel:.3f}° (直接算xyz反推)")
    print(f"  ΔAz(相对预测)={daz:.3f}° 限±{az_gate:.1f}° → {'✓' if abs(daz)<=az_gate else '✗ 拒绝'}")
    print(f"  ΔEl(相对预测)={del2:.3f}° 限±{el_gate:.1f}° → {'✓' if abs(del2)<=el_gate else '✗ 拒绝'}")
    if not (abs(daz) <= az_gate and abs(del2) <= el_gate):
        failed_at = "L2"

# 第3层：距离门（笛卡尔距离）
if not failed_at:
    distx = dx - pred_x
    disty = dy - pred_y
    distz = dz - pred_z
    dist = math.sqrt(distx**2 + disty**2 + distz**2)
    print(f"\n[L3-距离门] 预测距离:")
    print(f"  Δxyz=({distx:.1f},{disty:.1f},{distz:.1f}), |Δ|={dist:.1f}m 限≤{gate:.1f}m → {'✓' if dist<=gate else '✗ 拒绝'}")
    if dist > gate:
        failed_at = "L3"

# 第4层：速度门（预测径向速度 vs 实测径向速度）
if not failed_at:
    pr_dot = math.sqrt(pred_x**2 + pred_y**2 + pred_z**2)
    if pr_dot < 1.0: pr_dot = 1.0
    er_dx = pred_x / pr_dot
    er_dy = pred_y / pr_dot
    er_dz = pred_z / pr_dot
    # 用初始化速度投影到当前预测方向
    vr_pred2 = Vx * er_dx + Vy * er_dy + Vz * er_dz
    # 注意：vr_pred 应该是速度外推后的
    # CV模型速度不变，直接用Vx,Vy,Vz
    dv_r = dot_vr - vr_pred2
    print(f"\n[L4-速度门] 径向速度一致性:")
    print(f"  预测径向速度 vr_pred={vr_pred2:.2f}m/s")
    print(f"  实测径向速度 vr_dot={dot_vr:.2f}m/s")
    print(f"  差值 ΔVr={dv_r:.2f}m/s 限≤{vel_gate:.1f}m/s → {'✓' if abs(dv_r)<=vel_gate else '✗ 拒绝'}")
    # 方向相反检查
    sign_ok = True
    if (vr_pred2 * dot_vr) < 0:
        if abs(vr_pred2) > 5 and abs(dot_vr) > 5:
            sign_ok = False
    print(f"  方向一致性(符号): vr_pred*vr_dot={vr_pred2*dot_vr:.2f} {'✗ 符号相反且都>5，拒绝!' if not sign_ok else '✓'}")
    if not (abs(dv_r) <= vel_gate and sign_ok):
        failed_at = "L4"

# 第5层：辅助速度检查（位置差/时间差=合速度）
if not failed_at:
    est_vel = math.sqrt(distx**2 + disty**2 + distz**2) / T_since_init
    print(f"\n[L5-辅助速度检查] 位置差估算合速度:")
    print(f"  est_vel={est_vel:.2f}m/s 限≤40 → {'✓' if est_vel<=40 else '✗ 拒绝'}")
    if est_vel > 40:
        failed_at = "L5"

if failed_at:
    print(f"\n>>> 结论：点在 [{failed_at}] 层被拒！")
else:
    print(f"\n>>> 结论：所有门通过！可进入马氏距离筛选。")

# ====== 现在测试最坏情况：初始化速度方向与真实Vr不一致 ======
print("\n" + "="*80)
print("【关键测试】如果初始化估算的合速度方向（位置差）与真实径向Vr的方向不一致，会怎样？")
print("="*80)
print("场景：初始化3点的位置因为横向移动，合速度方向有较大横向分量。")
print("      → 径向投影vr_pred可能较小（比如+3m/s），但雷达实测Vr=+11.74m/s。")
print("      → ΔVr=8.74m/s，在门限内容易过。但如果投影符号反了呢？")
print("")
print("极端场景：无人机斜飞，位置差估算的径向投影为负，但雷达Vr为正 → 符号相反！")
bad_vr_pred = -6.0  # 初始化时估算错误，径向投影为负（靠近），但实际Vr=+11.74（远离）
dv_r_bad = 11.74 - (-6.0)
print(f"  错误vr_pred={bad_vr_pred}m/s (负=靠近)")
print(f"  实测vr_dot=+11.74m/s (正=远离)")
print(f"  ΔVr={dv_r_bad:.2f}m/s ≤ vel_gate={vel_gate:.1f}? → {'过' if abs(dv_r_bad)<=vel_gate else '被拒'}")
print(f"  vr_pred*vr_dot={bad_vr_pred*11.74:.2f}<0 且 |vr_pred|>5 且 |vr_dot|>5 → {'✗ L4第6条直接拒绝!' if (bad_vr_pred*11.74)<0 and abs(bad_vr_pred)>5 and abs(11.74)>5 else '放过'}")
print("")
print("→ 如果初始化时vr_pred方向搞反（负），而实测Vr=+11.74（正），且都>5 → L4-6直接拒绝！")
print("  → 正确无人机点从第一关开始全被拒 → predict_flag持续增长")
print("  → 连续160帧后航迹被track_die删除")
print("  → Frame233附近3个高速杂波点初始化新航迹")
print("  → 新航迹spd≈150∈[60,200] → target_type=0（未知），约束宽松")
print("  → target_type=0且prev_vel>60 → 走炮弹分支，Vmax=1500，门限完全放开")
print("  → 用户看到速度从10.9突然跳到153！")

# 再验证Frame233新假航迹的target_type
print("\n" + "="*80)
print("【验证假航迹】Frame 233 新假航迹 V=153.2m/s → target_type是什么？")
print("="*80)
spd_fake = 153.2
if spd_fake > 200:
    ttype = 1
    print(f"  spd={spd_fake:.1f} >200 → init_hint=1 → target_type=1 (炮弹)")
elif spd_fake < 60:
    ttype = 2
    print(f"  spd={spd_fake:.1f} <60 → init_hint=2 → target_type=2 (无人机)")
else:
    ttype = 0
    print(f"  60≤spd={spd_fake:.1f}≤200 → init_hint=0 → target_type=0 (未知)")

print(f"\n  target_type={ttype}时，track_asso.c的最终7层检查走哪条分支？:")
if ttype == 0:
    # track_asso.c L509: else分支（未知）
    print("  → 进入 ttype==0 (else)分支:")
    if spd_fake < 60:
        print(f"    prev_vel={spd_fake:.1f}<60 → 无人机约束(vmax=30m/s)")
    else:
        print(f"    prev_vel={spd_fake:.1f}≥60 → 炮弹宽松约束!")
        print(f"      v_max_allowed = Vmax=1500m/s (完全放开!)")
        print(f"      vel_change_thresh = 50m/s (超宽!)")
        print(f"      resid_gate = HARD_RESIDUAL_GATE={HARD_RESIDUAL_GATE}m")
        print(f"      vcos ≥ VELOCITY_COS_THRESHOLD={VELOCITY_COS_THRESHOLD}")
    print("  → 结论：假航迹target_type=0且spd>60时，约束和炮弹一样宽，完全没速度门!")
elif ttype == 1:
    print("  → 进入 ttype==1 (炮弹)分支: 高速约束, 完全放开速度")
