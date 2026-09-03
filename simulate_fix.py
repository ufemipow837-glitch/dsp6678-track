"""
模拟修改后的初始化检查逻辑，验证Frame38无人机能否通过
"""
import math

# Frame38-40 数据（从data_entry.c解析）
frames_data = {
    38: {'frame': 38, 'mSecond': 904372.0, 'beamNo': 2, 'az': 17.3900, 'el': 4.7655, 'r': 1153.14, 'vr': -11.1945},
    39: {'frame': 39, 'mSecond': 904536.0, 'beamNo': 3, 'az': 18.3887, 'el': 3.7986, 'r': 1154.56, 'vr': -11.1945},
    40: {'frame': 40, 'mSecond': 904700.0, 'beamNo': 4, 'az': 28.0507, 'el': -1.2231, 'r': 1314.73, 'vr': -4.6416},
}

# 提取3点
p0 = frames_data[38]
p1 = frames_data[39]
p2 = frames_data[40]

print("=" * 70)
print("模拟 new_reliable.c 修改后的初始化检查")
print("=" * 70)
print(f"\n3点数据:")
for name, p in [("Frame38", p0), ("Frame39", p1), ("Frame40", p2)]:
    print(f"  {name}: Az={p['az']:.2f}°, El={p['el']:.2f}°, R={p['r']:.1f}m, Vr={p['vr']:.2f}m/s, beam={p['beamNo']}")

# 模拟检查流程
final_ok = 1
vr0, vr1, vr2 = p0['vr'], p1['vr'], p2['vr']
r0, r1, r2 = p0['r'], p1['r'], p2['r']
az0, az1, az2 = p0['az'], p1['az'], p2['az']
el0, el1, el2 = p0['el'], p1['el'], p2['el']
beam0, beam1, beam2 = float(p0['beamNo']), float(p1['beamNo']), float(p2['beamNo'])

# ① Vr符号一致性检查
s0 = 1 if vr0 > 0 else (-1 if vr0 < 0 else 0)
s1 = 1 if vr1 > 0 else (-1 if vr1 < 0 else 0)
s2 = 1 if vr2 > 0 else (-1 if vr2 < 0 else 0)
print(f"\n--- ① Vr符号一致性检查 ---")
print(f"  Vr0={vr0:.2f}(sign={s0}), Vr1={vr1:.2f}(sign={s1}), Vr2={vr2:.2f}(sign={s2})")
if s0 != 0 and s1 != 0 and s2 != 0:
    if (s0 != s1) or (s1 != s2) or (s0 != s2):
        print(f"  ❌ Vr方向不一致！")
        final_ok = 0
    else:
        print(f"  ✅ Vr方向一致（都为负）")

# Vr量级检查
vr_min = min(vr0, vr1, vr2)
vr_max = max(vr0, vr1, vr2)
vr_avg = (abs(vr0) + abs(vr1) + abs(vr2)) / 3.0
print(f"\n--- Vr统计 ---")
print(f"  Vr_min={vr_min:.2f}, Vr_max={vr_max:.2f}, Vr_avg={vr_avg:.2f}")
if abs(vr_max) > 0.01 and abs(vr_min) > 0.01:
    vr_ratio = abs(vr_max) / (abs(vr_min) + 1e-6)
    print(f"  Vr_ratio={vr_ratio:.2f}")
    if vr_ratio > 5.0:
        print(f"  ❌ Vr量级差异太大")
        final_ok = 0
    else:
        print(f"  ✅ Vr量级差异合理")

# ② 目标类型判断
print(f"\n--- ② 目标类型判断（基于Vr_avg={vr_avg:.2f}）---")
if vr_avg > 80.0:
    drone_like = 0
    print(f"  → 炮弹")
else:
    drone_like = 1
    print(f"  → 无人机 ✅")

# ③ 距离单调性检查
print(f"\n--- ③ 距离单调性检查 ---")
inc = (r1 > r0) and (r2 > r1)
dec = (r1 < r0) and (r2 < r1)
stable = abs(r1 - r0) < 5 and abs(r2 - r1) < 5
print(f"  R0={r0:.1f}, R1={r1:.1f}, R2={r2:.1f}")
print(f"  递增={inc}, 递减={dec}, 稳定={stable}")
if not inc and not dec and not stable:
    if drone_like and vr_avg < 15.0:
        print(f"  R非单调但低速无人机，放行")
    else:
        print(f"  ❌ R非单调且不满足放行条件")
        final_ok = 0
else:
    print(f"  ✅ R单调")

delta_r = r2 - r0

# ④ 方位跨度检查
az_span = abs(az2 - az0)
if az_span > 180:
    az_span = 360 - az_span
print(f"\n--- ④ 方位跨度检查 ---")
print(f"  Az0={az0:.2f}°, Az2={az2:.2f}°, Az_span={az_span:.2f}°")
if az_span > 30:
    print(f"  ❌ 方位跨度太大")
    final_ok = 0
else:
    print(f"  ✅ 方位跨度合理")

# ⑤ 俯仰跨度检查
el_span = abs(el2 - el0)
print(f"\n--- ⑤ 俯仰跨度检查 ---")
print(f"  El0={el0:.2f}°, El2={el2:.2f}°, El_span={el_span:.2f}°")
if el_span > 25:
    print(f"  ❌ 俯仰跨度太大")
    final_ok = 0
else:
    print(f"  ✅ 俯仰跨度合理")

# ⑥ 绝对扇区约束
print(f"\n--- ⑥ 绝对扇区约束 ---")
az_norm = az2
while az_norm > 180:
    az_norm -= 360
while az_norm < -180:
    az_norm += 360
if vr_avg < 150:
    print(f"  Az_norm={az_norm:.2f}°, El2={el2:.2f}°, R2={r2:.1f}m")
    if az_norm > 60 or az_norm < -20:
        print(f"  ❌ 方位超出[-20°,+60°]")
        final_ok = 0
    elif el2 < -15 or el2 > 75:
        print(f"  ❌ 俯仰超出[-15°,+75°]")
        final_ok = 0
    elif r2 > 8000 or r0 > 8000:
        print(f"  ❌ 距离超过8000m")
        final_ok = 0
    else:
        print(f"  ✅ 扇区约束通过")

# ⑦ 波位合理性检查
beam_span = abs(beam2 - beam0)
print(f"\n--- ⑦ 波位合理性检查 ---")
print(f"  Beam0={beam0}, Beam2={beam2}, Beam_span={beam_span:.0f}")
if beam_span > 5:
    if az_span > 15:
        print(f"  ❌ 跨太多波位且方位跨度大")
        final_ok = 0
    else:
        print(f"  跨波位但方位跨度可接受，放行")
else:
    print(f"  ✅ 波位合理")

# ⑧ 交叉验证
print(f"\n--- ⑧ Vr与R变化率交叉验证 ---")
t0, t2 = p0['mSecond'], p2['mSecond']
dt_s = (t2 - t0) / 1000.0
if dt_s > 0.01:
    Vr_from_R = delta_r / dt_s
    print(f"  delta_r={delta_r:.1f}m, dt={dt_s:.3f}s, Vr_from_R={Vr_from_R:.2f}m/s")
    print(f"  vr2={vr2:.2f}")
    if (vr2 > 1.0 and Vr_from_R < -1.0) or (vr2 < -1.0 and Vr_from_R > 1.0):
        print(f"  ❌ Vr与R变化率方向矛盾！")
        final_ok = 0
    else:
        print(f"  ✅ Vr与R变化率方向一致")

print(f"\n{'='*70}")
if final_ok:
    print(f"✅✅✅ 所有检查通过！可以初始化无人机航迹！")
else:
    print(f"❌❌❌ 检查未通过，航迹被拒绝")
print(f"{'='*70}")
