# -*- coding: utf-8 -*-
"""
逐门检查模拟：为什么Frame38-199无人机航迹0次关联成功？
模拟track_asso.c的完整门限流程，看正确无人机点死在哪一层
"""
import re
import math

PI = 3.141592653589793

# 从track_asso.c复制的门限参数
ASSO_VELOCITY_CHANGE_THRESHOLD = 12.0
PRE_GATE_DISTANCE  = 100.0
PRE_GATE_COEFF     = 25.0
PRE_GATE_MAX      = 1000.0
VEL_GATE_BASE      = 12.0
VEL_GATE_COEFF      = 2.0
VEL_GATE_MAX       = 30.0
DRONE_RANGE_MIN     = 0.0
DRONE_RANGE_MAX  = 6000.0
TRACK_ASSO_TH = 11.3  # matrix.c中的值

def get_frame_points_from_dataentry(data_entry_path, target_frames):
    """从data_entry.c提取指定帧的点迹数据"""
    try:
        with open(data_entry_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except:
        with open(data_entry_path, 'r', encoding='gbk', errors='ignore') as f:
            content = f.read()
    
    result = {}
    
    for frame_num in target_frames:
        chunk_start = (frame_num // 50) * 50
        func_pattern = rf'void fill_frames_{chunk_start}_{chunk_start+49}\(.*?\)\s*\{{(.*?)\n\}}'
        func_match = re.search(func_pattern, content, re.DOTALL)
        
        if not func_match:
            continue
        
        func_body = func_match.group(1)
        
        # 提取k==N的case块
        case_pattern = rf'if\(k\s*==\s*{frame_num}\)\s*\{{(.*?)(?=\n\s*if\(k\s*==\s*\d+\)|\n\s*\}})'
        case_match = re.search(case_pattern, func_body, re.DOTALL)
        
        if not case_match:
            # 试试另一种格式
            case_pattern2 = rf'k\s*==\s*{frame_num}\s*\)?\s*:\s*\{{(.*?)(?=\n\s*k\s*==|\n\s*\}})'
            case_match = re.search(case_pattern2, func_body, re.DOTALL)
        
        if not case_match:
            print(f"  [WARN] Frame {frame_num}: 未找到case块")
            continue
        
        block = case_match.group(1)
        
        idx_pattern = r'target_data\[(\d+)\]\.(\w+)\s*=\s*([-\d.eE+]+)'
        data_map = {}
        
        for m in re.finditer(idx_pattern, block):
            idx = int(m.group(1))
            field = m.group(2)
            try:
                val = float(m.group(3))
            except:
                val = 0
            if idx not in data_map:
                data_map[idx] = {}
            data_map[idx][field] = val
        
        points = []
        for idx, data in sorted(data_map.items()):
            if 'Use_Flag_1' in data and data['Use_Flag_1'] > 0.5:
                p = {
                    'idx': idx,
                    'az': data.get('Azimuth', 0),
                    'el': data.get('Pitch', 0),
                    'r': data.get('Range', 0),
                    'vr': data.get('velocity', 0),
                    'beamNo': int(data.get('bpn', -1)),
                    'x': data.get('x', 0),
                    'y': data.get('y', 0),
                    'z': data.get('z', 0),
                    'mSecond': data.get('mSecond', 0)
                }
                # 球坐标转笛卡尔（验证x/y/z是否正确）
                az_r = p['az'] * PI / 180.0
                el_r = p['el'] * PI / 180.0
                px = p['r'] * math.cos(el_r) * math.cos(az_r)
                py = p['r'] * math.cos(el_r) * math.sin(az_r)
                pz = p['r'] * math.sin(el_r)
                p['calc_x'] = px
                p['calc_y'] = py
                p['calc_z'] = pz
                points.append(p)
        
        result[frame_num] = points
    
    return result

def find_drone_points(points):
    """从所有点中找出符合无人机特征的点：beam=2或3，且|Vr|≤20m/s，R在1000-2500m"""
    drone_pts = []
    for p in points:
        is_drone_like = False
        if p['beamNo'] in [2, 3] and abs(p['vr']) < 25 and 800 < p['r'] < 5000:
            is_drone_like = True
        if 16 < p['az'] < 20 and 3 < p['el'] < 8 and 800 < p['r'] < 5000 and abs(p['vr']) < 25:
            is_drone_like = True
        if is_drone_like:
            drone_pts.append(p)
    return drone_pts

def simulate_association_checks(frame_num, point, track_state, track_init):
    """完整模拟track_asso.c的7层门检查
    返回：(是否通过, 被拒层级, 详情)
    """
    result = {
        'frame': frame_num,
        'point_idx': point['idx'],
        'pass': False,
        'fail_layer': None,
        'details': []
    }
    
    ttype = track_state['target_type']
    
    # 计算T_asso
    T_asso = (point['mSecond'] - track_state['last_update_t']) / 1000.0
    if T_asso < 0.001:
        T_asso = 0.164  # 默认一个CPI
    result['T_asso'] = T_asso
    
    # 预测位置（CV模型匀速外推）
    pred_x = track_state['x'] + T_asso * track_state['vx']
    pred_y = track_state['y'] + T_asso * track_state['vy']
    pred_z = track_state['z'] + T_asso * track_state['vz']
    result['pred_xyz'] = (pred_x, pred_y, pred_z)
    
    # 预测方位俯仰
    pred_rho = math.sqrt(pred_x*pred_x + pred_y*pred_y)
    if pred_rho < 1: pred_rho = 1
    pred_az = math.atan2(pred_y, pred_x) * 180.0 / PI
    pred_el = math.atan2(pred_z, pred_rho) * 180.0 / PI
    result['pred_azel'] = (pred_az, pred_el)
    
    # 门限值
    gate = PRE_GATE_DISTANCE + T_asso * PRE_GATE_COEFF
    if gate > PRE_GATE_MAX: gate = PRE_GATE_MAX
    gate_sq = gate * gate
    az_gate = 5.0 + T_asso * 2.0
    if az_gate > 20.0: az_gate = 20.0
    el_gate = 5.0 + T_asso * 2.0
    if el_gate > 20.0: el_gate = 20.0
    result['gates'] = {'gate': gate, 'az_gate': az_gate, 'el_gate': el_gate}
    
    # ==================== 第1层：粗筛硬约束（绝对范围） ====================
    if ttype == 2:
        mx, my, mz = point['x'], point['y'], point['z']
        mrho = math.sqrt(mx*mx + my*my)
        if mrho < 1: mrho = 1
        maz = math.atan2(my, mx) * 180.0 / PI
        mel = math.atan2(mz, mrho) * 180.0 / PI
        mrange = math.sqrt(mx*mx + my*my + mz*mz)
        
        # 角度范围检查（相对init_az/el）
        daz0 = maz - track_init['init_az']
        del0 = mel - track_init['init_el']
        if daz0 > 180: daz0 -= 360
        if daz0 < -180: daz0 += 360
        if abs(daz0) > 25.0 or abs(del0) > 20.0:
            result['fail_layer'] = 1
            result['details'].append(f"第1层拒: 角度超范围 daz={daz0:+.2f}°(限±25°), del={del0:+.2f}°(限±20°)")
            return result
        result['details'].append(f"第1层过: 角度daz={daz0:+.2f}°, del={del0:+.2f}°")
        
        # 距离范围检查（绝对）
        if mrange < DRONE_RANGE_MIN or mrange > DRONE_RANGE_MAX:
            result['fail_layer'] = 1
            result['details'].append(f"第1层拒: 距离超范围 R={mrange:.1f}m(限0-6000m)")
            return result
        result['details'].append(f"第1层过: 距离R={mrange:.1f}m")
        
        # 速度绝对值检查
        if abs(point['vr']) > 30.0:
            result['fail_layer'] = 1
            result['details'].append(f"第1层拒: 径向速度|Vr|={abs(point['vr']):.1f}>30m/s")
            return result
        result['details'].append(f"第1层过: Vr={point['vr']:+.2f}m/s")
    
    # ==================== 第2层：方位门+俯仰门（相对预测值） ====================
    mx, my, mz = point['x'], point['y'], point['z']
    mrho = math.sqrt(mx*mx + my*my)
    if mrho < 1: mrho = 1
    maz = math.atan2(my, mx) * 180.0 / PI
    mel = math.atan2(mz, mrho) * 180.0 / PI
    daz = maz - pred_az
    if daz > 180: daz -= 360
    if daz < -180: daz += 360
    del_ = mel - pred_el
    
    if abs(daz) > az_gate or abs(del_) > el_gate:
        result['fail_layer'] = 2
        result['details'].append(f"第2层拒: 角度门 daz={daz:+.2f}°(限±{az_gate:.1f}°), "
                                f"del={del_:+.2f}°(限±{el_gate:.1f}°)")
        result['details'].append(f"  预测: Az={pred_az:.2f}°, El={pred_el:.2f}°")
        result['details'].append(f"  点迹: Az={maz:.2f}°, El={mel:.2f}°")
        return result
    result['details'].append(f"第2层过: 角度门 daz={daz:+.2f}°/±{az_gate:.1f}°, "
                            f"del={del_:+.2f}°/±{el_gate:.1f}°")
    
    # ==================== 第3层：距离门（笛卡尔距离） ====================
    dx = point['x'] - pred_x
    dy = point['y'] - pred_y
    dz = point['z'] - pred_z
    dist_sq = dx*dx + dy*dy + dz*dz
    dist = math.sqrt(dist_sq)
    
    if dist_sq > gate_sq:
        result['fail_layer'] = 3
        result['details'].append(f"第3层拒: 距离门 dist={dist:.1f}m > gate={gate:.1f}m")
        result['details'].append(f"  预测位置: ({pred_x:.1f},{pred_y:.1f},{pred_z:.1f})")
        result['details'].append(f"  点迹位置: ({point['x']:.1f},{point['y']:.1f},{point['z']:.1f})")
        return result
    result['details'].append(f"第3层过: 距离门 dist={dist:.1f}m / gate={gate:.1f}m")
    
    # ==================== 第4层：速度门（预测径向速度 vs 实测径向速度） ====================
    if ttype == 2:
        pr = math.sqrt(pred_x*pred_x + pred_y*pred_y + pred_z*pred_z)
        if pr < 1: pr = 1
        er_x = pred_x / pr
        er_y = pred_y / pr
        er_z = pred_z / pr
        
        # 预测径向速度（笛卡尔速度投影到径向）
        vr_pred = track_state['vx'] * er_x + track_state['vy'] * er_y + track_state['vz'] * er_z
        vr_dot = point['vr']
        
        # 速度门自适应
        vel_gate = VEL_GATE_BASE + VEL_GATE_COEFF * T_asso
        if vel_gate > VEL_GATE_MAX: vel_gate = VEL_GATE_MAX
        
        dv_r = vr_dot - vr_pred
        
        # 先检查偏差大小
        if abs(dv_r) > vel_gate:
            result['fail_layer'] = 4
            result['details'].append(f"第4层拒: 速度门 ΔVr={dv_r:+.2f}m/s > 门限{vel_gate:.1f}m/s")
            result['details'].append(f"  预测Vr={vr_pred:+.2f}m/s, 实测Vr={vr_dot:+.2f}m/s")
            return result
        result['details'].append(f"第4层过: 速度门 ΔVr={dv_r:+.2f}m/s / ±{vel_gate:.1f}m/s")
        
        # 再检查方向
        if (vr_pred * vr_dot) < 0.0:
            if abs(vr_pred) > 5.0 and abs(vr_dot) > 5.0:
                result['fail_layer'] = 4
                result['details'].append(f"第4层拒: 速度方向相反！Vr_pred={vr_pred:+.2f}, Vr_dot={vr_dot:+.2f}")
                return result
            result['details'].append(f"  方向不同但幅度小：Vr_pred={vr_pred:+.2f}, Vr_dot={vr_dot:+.2f}")
    
    # ==================== 第5层：辅助速度检查（位置差估算合速度） ====================
    o_sec = T_asso
    if o_sec > 0.01:
        est_vel = math.sqrt(dx*dx + dy*dy + dz*dz) / o_sec
    else:
        est_vel = math.sqrt(track_state['vx']**2 + track_state['vy']**2 + track_state['vz']**2)
    
    if ttype == 2:
        if est_vel > 40.0:
            result['fail_layer'] = 5
            result['details'].append(f"第5层拒: 估算速度={est_vel:.1f}m/s > 40m/s")
            return result
        result['details'].append(f"第5层过: 估算速度={est_vel:.1f}m/s")
    
    # ==================== 第6层：马氏距离门（TRACK_ASSO_TH=11.3） ====================
    # 简化：如果前面的门都过了，说明位置/速度都在合理范围内，
    # 马氏距离是否通过取决于协方差矩阵。
    # 粗略估算：如果dist<=gate*0.8且角度<=门限*0.8，则大概率马氏距离通过
    mahal_est = (dist / max(gate, 1)) * 6.0  # 粗略估算
    result['mahal_est'] = mahal_est
    if mahal_est > TRACK_ASSO_TH * 1.5:
        result['details'].append(f"第6层存疑: 马氏距离估算={mahal_est:.1f}（阈值{TRACK_ASSO_TH}）可能不通过")
        # 不直接return，因为马氏距离计算更复杂
    else:
        result['details'].append(f"第6层过: 马氏距离估算={mahal_est:.1f} / {TRACK_ASSO_TH}")
    
    result['pass'] = True
    return result


def main():
    data_entry_path = r'd:\DSP\6678\track\track_1\data_entry.c'
    
    # 提取Frame38(初始化)、Frame39、50、55、56、72、80、100、150、180的点
    test_frames = [38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
                   51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
                   65, 70, 72, 75, 80, 90, 100, 120, 150, 180]
    
    print("正在从data_entry.c提取点迹数据...")
    frame_data = get_frame_points_from_dataentry(data_entry_path, test_frames)
    
    print(f"\n成功提取 {len(frame_data)} 帧数据\n")
    
    # 先看Frame38的初始化数据
    if 38 in frame_data:
        print("=" * 80)
        print("Frame38: 航迹初始化帧的点迹")
        print("=" * 80)
        pts = frame_data[38]
        drone_pts = find_drone_points(pts)
        print(f"  总有效点数: {len(pts)}")
        print(f"  疑似无人机点: {len(drone_pts)}")
        for p in drone_pts:
            print(f"    [{p['idx']}] beam={p['beamNo']}, Az={p['az']:.2f}°, El={p['el']:.2f}°, "
                  f"R={p['r']:.1f}m, Vr={p['vr']:+.2f}m/s "
                  f"pos=({p['x']:.1f},{p['y']:.1f},{p['z']:.1f}) "
                  f"calc_pos=({p['calc_x']:.1f},{p['calc_y']:.1f},{p['calc_z']:.1f})")
        # 其他点（高速杂波）
        other_highspeed = [p for p in pts if abs(p['vr']) > 50]
        if other_highspeed:
            print(f"\n  高速杂波(|Vr|>50): {len(other_highspeed)}")
            for p in other_highspeed:
                print(f"    [{p['idx']}] beam={p['beamNo']}, Az={p['az']:.2f}°, R={p['r']:.1f}m, Vr={p['vr']:+.2f}")
    
    # 模拟航迹状态（基于Frame38初始化值）
    # 从输出数据.txt: Frame38 Track[0]: Az=17.39deg El=4.77deg R=1153.1m V=10.9m/s
    init_az = 17.39
    init_el = 4.77
    init_r = 1153.1
    init_v = 10.9
    az_r = init_az * PI / 180.0
    el_r = init_el * PI / 180.0
    
    # 初始化位置（球→笛），注意：无人机V=10.9m/s远离雷达→x/y/z方向
    # 位置：
    init_x = init_r * math.cos(el_r) * math.cos(az_r)
    init_y = init_r * math.cos(el_r) * math.sin(az_r)
    init_z = init_r * math.sin(el_r)
    
    # 初始化速度：方向=位置径向单位向量 × 速度大小（假设远离，即正径向）
    # 但实测：data_entry.c中Vr=-11.47（靠近雷达），所以速度符号相反！
    # 这里有个关键矛盾：初始化时3点位置差估算的速度方向 vs 实测Vr符号
    # Frame38输出V=10.9m/s（正=远离），但data_entry.c中正确无人机点的Vr=-11.47（负=靠近）
    # 这意味着初始化速度方向就错了！
    init_vx = (init_x / init_r) * init_v  # 正方向=远离
    init_vy = (init_y / init_r) * init_v
    init_vz = (init_z / init_r) * init_v
    
    track_init = {
        'init_az': init_az,
        'init_el': init_el,
        'init_r': init_r
    }
    
    track_state = {
        'x': init_x, 'y': init_y, 'z': init_z,
        'vx': init_vx, 'vy': init_vy, 'vz': init_vz,
        'target_type': 2,  # 无人机
        'last_update_t': 904372,  # Frame38的t=904372ms
        'mSecond': 904372
    }
    
    print("\n" + "=" * 80)
    print("模拟Frame40-80：逐门检查正确无人机点是否能通过")
    print("=" * 80)
    print(f"航迹初始化状态：pos=({init_x:.1f},{init_y:.1f},{init_z:.1f})")
    print(f"  vel=({init_vx:.2f},{init_vy:.2f},{init_vz:.2f}) 合速度={init_v:.1f}m/s(远离)")
    print(f"  注意：如果初始化速度方向与实测Vr相反(靠近=-11.47m/s)，第4层速度门会直接拒绝！\n")
    
    total_checks = 0
    total_pass = 0
    fail_counts = {1:0, 2:0, 3:0, 4:0, 5:0, 6:0}
    
    # 逐帧检查
    for fn in sorted(frame_data.keys()):
        if fn <= 38:
            continue
        if fn > 100:  # 先看前100帧
            break
        
        pts = frame_data[fn]
        drone_pts = find_drone_points(pts)
        
        if not drone_pts:
            continue
        
        # 更新track_state为纯预测（因为之前关联都失败了）
        # 用Frame38的时间戳作为基准
        t_now = drone_pts[0]['mSecond']
        dt = (t_now - 904372) / 1000.0  # 总外推时间
        cur_x = init_x + init_vx * dt
        cur_y = init_y + init_vy * dt
        cur_z = init_z + init_vz * dt
        
        track_state_curr = dict(track_state)
        track_state_curr['x'] = cur_x
        track_state_curr['y'] = cur_y
        track_state_curr['z'] = cur_z
        
        print(f"\n--- Frame {fn} (dt={dt:.1f}s since init, n_drone_pts={len(drone_pts)}) ---")
        
        for dp in drone_pts:
            # 重新计算基于上次update的T_asso
            # 因为纯预测，last_update_t一直是Frame38的时间
            T_since = (dp['mSecond'] - 904372) / 1000.0
            track_state_curr['last_update_t'] = 904372
            
            res = simulate_association_checks(fn, dp, track_state_curr, track_init)
            total_checks += 1
            
            status = "✅通过" if res['pass'] else "❌拒绝"
            layer_info = f"(死在第{res['fail_layer']}层)" if res['fail_layer'] else ""
            
            print(f"  [{dp['idx']}] beam={dp['beamNo']} Vr={dp['vr']:+.2f} R={dp['r']:.0f} → {status} {layer_info}")
            if not res['pass']:
                fail_counts[res['fail_layer']] = fail_counts.get(res['fail_layer'], 0) + 1
                # 打印最后2条detail（即拒绝原因）
                for d in res['details'][-2:]:
                    print(f"      → {d}")
            else:
                total_pass += 1
                # 只打印关键
                print(f"      T_asso={res['T_asso']:.2f}s, 距门{res['gates']['gate']:.0f}m, "
                      f"估算速度≈通过")
    
    print("\n" + "=" * 80)
    print("统计汇总")
    print("=" * 80)
    print(f"总检查次数: {total_checks}")
    print(f"通过次数:   {total_pass}")
    print(f"拒绝次数:   {total_checks - total_pass}")
    print(f"各层拒绝分布:")
    for layer in sorted(fail_counts.keys()):
        cnt = fail_counts[layer]
        if cnt > 0:
            names = {1:'第1层(绝对范围粗筛)', 2:'第2层(角度门)', 
                     3:'第3层(距离门)', 4:'第4层(速度门)', 
                     5:'第5层(位置差估算速度)', 6:'第6层(马氏距离)'}
            print(f"  {names.get(layer, f'第{layer}层')}: {cnt}次")
    
    # 关键诊断
    print("\n" + "=" * 80)
    print("【核心根因诊断】")
    print("=" * 80)
    if fail_counts.get(4, 0) > 0:
        print("""
❌❌❌ 最可能的根因：第4层速度门！
初始化速度方向估算错误！
  Frame38输出：V=10.9m/s（正=远离雷达）
  data_entry.c中正确无人机点实测：Vr≈-11.47m/s（负=靠近雷达）
  → 初始化速度方向就与实测Vr完全相反！
  → 第4层速度门先检查 ΔVr = Vr_dot - Vr_pred
       Vr_pred = +10.9m/s (远离)
       Vr_dot  = -11.47m/s (靠近)
       ΔVr = -22.4m/s > 速度门(12+2*T≈15~20m/s) → 直接拒绝！
  → 即使通过了偏差检查，后面还有：
       Vr_pred * Vr_dot < 0（方向相反）且 |Vr|都>5 → 双重拒绝！
  
这就是160帧0次关联成功的根因——初始化速度方向就错了！
修正方法：
  A. 初始化时用点迹自带的Vr（实测径向速度）修正速度方向，而不是纯位置差
  B. 或者第4层速度门前几帧先用宽松阈值（允许初始化校正）
""")

if __name__ == '__main__':
    main()
