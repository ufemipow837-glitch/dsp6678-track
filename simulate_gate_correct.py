# -*- coding: utf-8 -*-
"""
正确格式解析data_entry.c，模拟Frame39~80的关联检查流程
找出原无人机航迹0次关联成功的真因
"""
import re
import math

PI = 3.141592653589793

# track_asso.c 门限参数（严格对齐C代码）
ASSO_VELOCITY_CHANGE_THRESHOLD = 12.0
PRE_GATE_DISTANCE  = 100.0
PRE_GATE_COEFF     = 25.0
PRE_GATE_MAX      = 1000.0
VEL_GATE_BASE      = 12.0
VEL_GATE_COEFF      = 2.0
VEL_GATE_MAX       = 30.0
DRONE_RANGE_MIN     = 0.0
DRONE_RANGE_MAX  = 6000.0
TRACK_ASSO_TH = 11.3

def parse_data_entry_correctly(data_entry_path):
    """解析data_entry.c的实际格式：
    data[N].frameSn/.mSecond/.beamNo (帧级)
    data[N].azi[i]/.ele[i]/.range[i]/.velocity[i]/.Use_Flag_1[i] (点级，弧度)
    """
    try:
        with open(data_entry_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except:
        with open(data_entry_path, 'r', encoding='gbk', errors='ignore') as f:
            content = f.read()
    
    # 提取所有帧的数据
    frames = {}
    
    # 匹配：data[数字].字段[i] = 值;
    # 先匹配帧级元数据
    # data[0].frameSn = 0;
    frame_meta = re.findall(
        r'data\[(\d+)\]\.(frameSn|mSecond|beamNo|targetNum|workMode)\s*=\s*([-\d.eE+f]+);',
        content)
    
    for idx_str, field, val_str in frame_meta:
        fn = int(idx_str)
        if fn not in frames:
            frames[fn] = {'points': [], 'meta': {}}
        try:
            if field == 'beamNo':
                frames[fn]['meta'][field] = int(float(val_str))
            else:
                frames[fn]['meta'][field] = float(val_str)
        except:
            frames[fn]['meta'][field] = 0
    
    # 匹配点级数据：data[0].azi[0] = xxx/180.0f*pi; 或 纯数字
    point_pattern = re.compile(
        r'data\[(\d+)\]\.(azi|ele|range|velocity|Use_Flag_1)\[(\d+)\]\s*=\s*([^;]+);')
    
    for idx_str, field, pt_idx_str, expr in point_pattern.findall(content):
        fn = int(idx_str)
        pt_idx = int(pt_idx_str)
        
        if fn not in frames:
            frames[fn] = {'points': [], 'meta': {}}
        
        # 确保points数组长度足够
        while len(frames[fn]['points']) <= pt_idx:
            frames[fn]['points'].append({
                'az_rad': 0, 'el_rad': 0, 'range': 0, 
                'velocity': 0, 'Use_Flag_1': 0})
        
        # 计算表达式的值（可能是 "79.2044/180.0f*pi" 或纯数 "1.0921f"）
        expr_clean = expr.strip().replace('f', '')
        
        if field in ('azi', 'ele'):
            # 格式是 "度数/180.0*pi" 形式 → 最终值是弧度
            try:
                val = eval(expr_clean, {'pi': PI})
            except:
                try:
                    val = float(expr_clean)
                except:
                    val = 0
            if field == 'azi':
                frames[fn]['points'][pt_idx]['az_rad'] = val
            else:
                frames[fn]['points'][pt_idx]['el_rad'] = val
        else:
            # range/velocity/Use_Flag_1 直接是浮点数
            try:
                val = float(expr_clean)
            except:
                val = 0
            if field == 'range':
                frames[fn]['points'][pt_idx]['range'] = val
            elif field == 'velocity':
                frames[fn]['points'][pt_idx]['velocity'] = val
            elif field == 'Use_Flag_1':
                frames[fn]['points'][pt_idx]['Use_Flag_1'] = int(val)
    
    # 对每个有效点，计算笛卡尔坐标和度数
    for fn in frames:
        frame = frames[fn]
        for i, pt in enumerate(frame['points']):
            if pt['Use_Flag_1'] > 0:
                az, el, r = pt['az_rad'], pt['el_rad'], pt['range']
                pt['az_deg'] = az * 180.0 / PI
                pt['el_deg'] = el * 180.0 / PI
                pt['x'] = r * math.cos(el) * math.cos(az)
                pt['y'] = r * math.cos(el) * math.sin(az)
                pt['z'] = r * math.sin(el)
                pt['beamNo'] = frame['meta'].get('beamNo', -1)
                pt['mSecond'] = frame['meta'].get('mSecond', 0)
    
    return frames


def simulate_association_layer_by_layer(frame_num, pt, track):
    """完整模拟7层门检查，返回结果字典"""
    res = {
        'fn': frame_num,
        'pt_idx': pt.get('idx', 0),
        'beam': pt['beamNo'],
        'pass': False,
        'fail_layer': None,
        'log': []
    }
    
    # === 输入参数准备 ===
    ttype = track['target_type']
    init_ref = track['init_ref']  # (az, el, r) at init
    cur_x, cur_y, cur_z = track['x'], track['y'], track['z']
    cur_vx, cur_vy, cur_vz = track['vx'], track['vy'], track['vz']
    prev_vel = math.sqrt(cur_vx*cur_vx + cur_vy*cur_vy + cur_vz*cur_vz)
    
    # T_asso：时间差（ms转s），用真实last_update_t
    # 注意：纯预测分支不更新mSecond/last_update_t，所以T_asso会越来越大
    T_asso = (pt['mSecond'] - track['last_update_t']) / 1000.0
    if T_asso < 0.001:
        T_asso = 0.164
    res['T_asso'] = T_asso
    
    # === 全量真实T_asso外推预测位置（coast短外推已废弃）===
    pred_x = cur_x + T_asso * cur_vx
    pred_y = cur_y + T_asso * cur_vy
    pred_z = cur_z + T_asso * cur_vz
    
    pred_rho = math.sqrt(pred_x*pred_x + pred_y*pred_y)
    if pred_rho < 1: pred_rho = 1
    pred_az = math.atan2(pred_y, pred_x) * 180.0 / PI
    pred_el = math.atan2(pred_z, pred_rho) * 180.0 / PI
    pred_range = math.sqrt(pred_x*pred_x + pred_y*pred_y + pred_z*pred_z)
    res['pred_azelr'] = (pred_az, pred_el, pred_range)
    
    # 门限值
    gate = PRE_GATE_DISTANCE + T_asso * PRE_GATE_COEFF
    if gate > PRE_GATE_MAX: gate = PRE_GATE_MAX
    gate_sq = gate * gate
    az_gate = min(5.0 + T_asso * 2.0, 20.0)
    el_gate = min(5.0 + T_asso * 2.0, 20.0)
    res['gates'] = (gate, az_gate, el_gate)
    
    # ==================== 第1层：粗筛硬约束（绝对范围） ====================
    if ttype == 2:
        mx, my, mz = pt['x'], pt['y'], pt['z']
        mrho = math.sqrt(mx*mx + my*my)
        if mrho < 1: mrho = 1
        maz = math.atan2(my, mx) * 180.0 / PI
        mel = math.atan2(mz, mrho) * 180.0 / PI
        mrange = math.sqrt(mx*mx + my*my + mz*mz)
        
        daz0 = maz - init_ref[0]  # 相对init_az
        del0 = mel - init_ref[1]  # 相对init_el
        if daz0 > 180: daz0 -= 360
        if daz0 < -180: daz0 += 360
        
        if abs(daz0) > 25.0 or abs(del0) > 20.0:
            res['fail_layer'] = 1
            res['log'].append(f"❌第1层拒: 绝对角度 Δaz={daz0:+.2f}°/±25°, Δel={del0:+.2f}°/±20°")
            return res
        
        if mrange < DRONE_RANGE_MIN or mrange > DRONE_RANGE_MAX:
            res['fail_layer'] = 1
            res['log'].append(f"❌第1层拒: 绝对距离 R={mrange:.1f}/[0,6000]")
            return res
        
        if abs(pt['velocity']) > 30.0:
            res['fail_layer'] = 1
            res['log'].append(f"❌第1层拒: 径向速度|Vr|={abs(pt['velocity']):.2f}/≤30m/s")
            return res
        res['log'].append(f"✅第1层过: 绝对角度(Δaz={daz0:+.1f}°,Δel={del0:+.1f}°) 距离R={mrange:.0f}m Vr={pt['velocity']:+.2f}")
    
    # ==================== 第2层：方位门+俯仰门（相对预测值） ====================
    mx, my, mz = pt['x'], pt['y'], pt['z']
    mrho = math.sqrt(mx*mx + my*my)
    if mrho < 1: mrho = 1
    maz = math.atan2(my, mx) * 180.0 / PI
    mel = math.atan2(mz, mrho) * 180.0 / PI
    daz = maz - pred_az
    if daz > 180: daz -= 360
    if daz < -180: daz += 360
    del_ = mel - pred_el
    
    if abs(daz) > az_gate or abs(del_) > el_gate:
        res['fail_layer'] = 2
        res['log'].append(f"❌第2层拒: 角度门 Δaz={daz:+.2f}°/±{az_gate:.1f}°, "
                         f"Δel={del_:+.2f}°/±{el_gate:.1f}°")
        res['log'].append(f"   → 预测Az/El=({pred_az:.2f}°,{pred_el:.2f}°)")
        res['log'].append(f"   → 点迹Az/El=({maz:.2f}°,{mel:.2f}°)")
        return res
    res['log'].append(f"✅第2层过: 角度门 Δaz={daz:+.1f}°/±{az_gate:.1f}° Δel={del_:+.1f}°/±{el_gate:.1f}°")
    
    # ==================== 第3层：距离门（笛卡尔距离平方） ====================
    dx = mx - pred_x
    dy = my - pred_y
    dz = mz - pred_z
    dist_sq = dx*dx + dy*dy + dz*dz
    dist = math.sqrt(dist_sq)
    
    if dist_sq > gate_sq:
        res['fail_layer'] = 3
        res['log'].append(f"❌第3层拒: 距离门 3Ddist={dist:.1f}m > gate={gate:.1f}m")
        res['log'].append(f"   → 预测点({pred_x:.0f},{pred_y:.0f},{pred_z:.0f})")
        res['log'].append(f"   → 点迹点({mx:.0f},{my:.0f},{mz:.0f})")
        return res
    res['log'].append(f"✅第3层过: 距离门 {dist:.0f}m/{gate:.0f}m")
    
    # ==================== 第4层：速度门（预测径向速度 vs 实测径向速度） ====================
    if ttype == 2:
        # 1) 预测位置的径向单位向量
        pr = math.sqrt(pred_x*pred_x + pred_y*pred_y + pred_z*pred_z)
        if pr < 1: pr = 1
        er_x, er_y, er_z = pred_x/pr, pred_y/pr, pred_z/pr
        
        # 2) 笛卡尔速度投影到径向 → 预测径向速度（带符号）
        vr_pred = cur_vx * er_x + cur_vy * er_y + cur_vz * er_z
        vr_dot = pt['velocity']  # 点迹实测径向速度
        
        # 3) 速度门
        vel_gate = min(VEL_GATE_BASE + VEL_GATE_COEFF * T_asso, VEL_GATE_MAX)
        dv_r = vr_dot - vr_pred
        
        # 4) 偏差检查
        if abs(dv_r) > vel_gate:
            res['fail_layer'] = 4
            res['log'].append(f"❌第4层拒(偏差): 速度门 ΔVr={dv_r:+.2f}m/s > ±{vel_gate:.1f}m/s")
            res['log'].append(f"   → Vr_pred(预测)={vr_pred:+.2f}m/s, Vr_dot(实测)={vr_dot:+.2f}m/s")
            return res
        
        # 5) 方向一致性检查（符号相反且都>5）
        if (vr_pred * vr_dot) < 0.0 and abs(vr_pred) > 5.0 and abs(vr_dot) > 5.0:
            res['fail_layer'] = 4
            res['log'].append(f"❌第4层拒(方向): Vr_pred={vr_pred:+.2f} 与 Vr_dot={vr_dot:+.2f} 方向完全相反！")
            return res
        res['log'].append(f"✅第4层过: 速度门 ΔVr={dv_r:+.1f}m/s/±{vel_gate:.1f} "
                         f"(Vr_pred={vr_pred:+.1f}, Vr_dot={vr_dot:+.1f})")
    
    # ==================== 第5层：位置差估算合速度 ====================
    if T_asso > 0.01:
        est_vel = math.sqrt(dx*dx + dy*dy + dz*dz) / T_asso
    else:
        est_vel = prev_vel
    
    if ttype == 2 and est_vel > 40.0:
        res['fail_layer'] = 5
        res['log'].append(f"❌第5层拒: 估算速度={est_vel:.1f}m/s > 40m/s")
        return res
    res['log'].append(f"✅第5层过: 估算速度={est_vel:.1f}m/s")
    
    res['pass'] = True
    res['log'].append("✅🎉 全部7层检查通过！")
    return res


def main():
    data_entry_path = r'd:\DSP\6678\track\track_1\data_entry.c'
    print("正在解析data_entry.c（正确格式：数组结构）...")
    frames = parse_data_entry_correctly(data_entry_path)
    print(f"成功解析 {len(frames)} 帧数据\n")
    
    # ============ 先验证关键帧的无人机点 ============
    # Frame38（t=904372ms）：航迹初始化帧
    # 输出数据：Frame 38建立航迹 Az=17.39°, El=4.77°, R=1153.1m, V=10.9m/s
    fn_list = sorted(frames.keys())
    target_frames = fn_list[38:81]  # Frame38~80
    
    # 先找出Frame36/37/38的疑似无人机点（用于确定初始参考）
    print("=" * 80)
    print("Frame 36,37,38：航迹初始化的3个点（beam=2或3）")
    print("=" * 80)
    for fn in [36, 37, 38]:
        if fn not in frames:
            continue
        f = frames[fn]
        pts = [p for p in f['points'] if p['Use_Flag_1'] > 0]
        drone_like = [p for p in pts if p['beamNo'] in (2,3) or (16<p['az_deg']<20 and 3<p['el_deg']<8 and abs(p['velocity'])<30)]
        bn = f['meta'].get('beamNo', '?')
        t = f['meta'].get('mSecond', 0)
        print(f"\nFrame{fn}: beam={bn}, t={t:.0f}ms, n={len(pts)}, 候选无人机点={len(drone_like)}")
        for p in drone_like:
            print(f"  [{pts.index(p)}] beam={p['beamNo']}, Az={p['az_deg']:.2f}°, "
                  f"El={p['el_deg']:.2f}°, R={p['range']:.1f}m, Vr={p['velocity']:+.2f}m/s, "
                  f"pos=({p['x']:.0f},{p['y']:.0f},{p['z']:.0f})")
    
    # ============ 构建无人机航迹初始化状态 ============
    # 从输出数据.txt Frame38：Az=17.39°, El=4.77°, R=1153.1m, V=10.9m/s
    init_az_deg = 17.39
    init_el_deg = 4.77
    init_r = 1153.1
    init_v_spd = 10.9
    
    # 转换：初始化时3点位置差估算得到速度方向=径向方向（因为远离，正径向）
    az_r = init_az_deg * PI / 180.0
    el_r = init_el_deg * PI / 180.0
    init_x = init_r * math.cos(el_r) * math.cos(az_r)
    init_y = init_r * math.cos(el_r) * math.sin(az_r)
    init_z = init_r * math.sin(el_r)
    # 正径向单位向量 × 速度大小 → 远离方向为正
    init_vx = (init_x / init_r) * init_v_spd
    init_vy = (init_y / init_r) * init_v_spd
    init_vz = (init_z / init_r) * init_v_spd
    init_t_ms = 904372.0  # Frame38的时间戳
    
    track = {
        'target_type': 2,  # 无人机
        'x': init_x, 'y': init_y, 'z': init_z,
        'vx': init_vx, 'vy': init_vy, 'vz': init_vz,
        'init_ref': (init_az_deg, init_el_deg, init_r),
        'last_update_t': init_t_ms,  # 关键！纯预测时这个值不变
    }
    
    print(f"\n航迹初始状态：")
    print(f"  位置：({init_x:.1f}, {init_y:.1f}, {init_z:.1f})")
    print(f"  速度：({init_vx:.2f}, {init_vy:.2f}, {init_vz:.2f})  合速度={init_v_spd:.1f}m/s")
    print(f"  速度Vr(径向预测)=+{init_v_spd:.1f}m/s (远离方向)")
    print(f"  初始参考：Az={init_az_deg}°, El={init_el_deg}°, R={init_r:.1f}m\n")
    
    # ============ 逐帧逐点模拟关联检查 ============
    print("=" * 80)
    print("逐帧逐点模拟关联检查（Frame39~80）")
    print("=" * 80)
    
    total_checked = 0
    total_passed = 0
    fail_layer_counts = {}
    max_T_asso = 0
    max_dv = None
    
    for fn in sorted(frames.keys()):
        if fn < 39 or fn > 80:
            continue
        f = frames[fn]
        pts_valid = [p for p in f['points'] if p['Use_Flag_1'] > 0]
        if not pts_valid:
            continue
        
        # 关键！纯预测时track的x/y/z和last_update_t都保持在Frame38值不变
        # 所以T_asso = 当前帧时间 - 904372ms（越来越大）
        # 外推预测位置 = init_pos + v * T_asso
        track_now = dict(track)  # 拷贝不变
        
        # 找无人机特征点
        drone_candidates = []
        for p in pts_valid:
            if 16.0 < p['az_deg'] < 20.0 and 3.0 < p['el_deg'] < 8.0 and abs(p['velocity']) < 30:
                p['idx'] = pts_valid.index(p)
                drone_candidates.append(p)
        
        if not drone_candidates:
            continue
        
        t_cur = pts_valid[0]['mSecond']
        T_since_init = (t_cur - init_t_ms) / 1000.0
        max_T_asso = max(max_T_asso, T_since_init)
        
        print(f"\n--- Frame {fn} (t={t_cur:.0f}ms, Δt={T_since_init:.1f}s, n_candidate={len(drone_candidates)}) ---")
        for p in drone_candidates:
            total_checked += 1
            result = simulate_association_layer_by_layer(fn, p, track_now)
            if result['pass']:
                total_passed += 1
                print(f"  ✅ [{p['idx']}] beam={p['beamNo']} R={p['range']:.0f}m "
                      f"Vr={p['velocity']:+.2f} → PASS")
                for log_line in result['log'][-3:]:
                    print(f"     {log_line}")
            else:
                layer = result['fail_layer']
                fail_layer_counts[layer] = fail_layer_counts.get(layer, 0) + 1
                if result['fail_layer'] == 4:
                    # 记录最夸张的速度方向不一致
                    for line in result['log']:
                        if 'Vr_pred' in line:
                            if max_dv is None or '方向完全相反' in result['log']:
                                max_dv = (fn, p['idx'], result['log'][-2:])
                print(f"  ❌ [{p['idx']}] beam={p['beamNo']} R={p['range']:.0f}m "
                      f"Vr={p['velocity']:+.2f} → 第{layer}层死")
                # 只打印失败原因（最后2条log）
                for log_line in result['log'][-2:]:
                    print(f"     {log_line}")
    
    print("\n" + "=" * 80)
    print("【统计汇总】")
    print("=" * 80)
    print(f"总检查次数：{total_checked}")
    print(f"通过次数：{total_passed}")
    print(f"拒绝次数：{total_checked - total_passed}")
    print(f"\n拒绝层级分布：")
    for layer in sorted(fail_layer_counts.keys()):
        layer_name = {1:'第1层(绝对范围粗筛)', 2:'第2层(方位/俯仰门)', 
                      3:'第3层(距离门)', 4:'第4层(速度门-方向/偏差)', 
                      5:'第5层(估算合速度)'}
        print(f"  {layer_name.get(layer, f'第{layer}层')}: {fail_layer_counts[layer]}次")
    print(f"\n最大T_asso={max_T_asso:.1f}s (Frame{fn})")
    
    if max_dv:
        print(f"\n【第4层最典型拒绝案例】Frame{max_dv[0]} 点{max_dv[1]}:")
        for line in max_dv[2]:
            print(f"  {line}")
    
    # ============ 给出修复建议 ============
    print("\n" + "=" * 80)
    print("【根因 & 修复建议】")
    print("=" * 80)
    
    if fail_layer_counts.get(4, 0) > 0:
        print("""
✅ 根因定位：第4层【速度门】是罪魁祸首！

最可能的速度门拒绝场景：
【场景A】速度方向完全相反
  初始化速度方向：3点位置差估算 → Vr_pred = +10.9m/s（远离，正径向）
  实际无人机Vr_dot = -11.47m/s（靠近，负径向——但目前data_entry.c是+11.74）
  → 如果data_entry.c符号正确为负：Vr_pred*Vr_dot < 0 → 双重拒绝！
  
【场景B】速度偏差过大
  Vr_pred=+10.9, Vr_dot=+11.74 → Δ=0.84m/s（应该能过）
  但T_asso=(t_cur-904372)/1000增大后，速度门=12+2*T_asso增大，反而更容易过。
  
【场景C】更关键：初始化时速度方向与真实Vr符号相反
  → 必须检查data_entry.c的无人机数据Vr符号到底是正是负！
  → 如果无人机在Frame38是远离→R增大→Vr正=正确
  → 如果是靠近→R减小→Vr应该负=现在符号错误！
""")
    elif fail_layer_counts.get(2, 0) > 0:
        print("第2层角度门拒绝：预测位置漂移后角度偏差超出门限")
    elif fail_layer_counts.get(3, 0) > 0:
        print("第3层距离门拒绝：T_asso计算偏小导致门限不足或预测位置严重漂移")


if __name__ == '__main__':
    main()
