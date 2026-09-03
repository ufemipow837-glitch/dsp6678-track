import re, sys
NR = r"d:\DSP\6678\track\track_1\new_reliable.c"
with open(NR, "r", encoding="utf-8", errors="ignore") as f:
    nr = f.read()

A6 = nr.find("// ===== ⑥ 绝对扇区约束 =====")
A7 = nr.find("// ===== ⑦ 波位合理性检查 =====", A6)
assert A6>=0 and A7>=0, "A6/A7 titles missing"

NEW_6TO7 = '''                // ===== ⑥ 绝对扇区约束 =====
                // 用雷达实测的az2/el2检查目标是否在合理扇区内
                if(!drone_like || vr_avg < 600.0f){
                    float az_norm = az2;
                    while(az_norm > 180.0f)  az_norm -= 360.0f;
                    while(az_norm < -180.0f) az_norm += 360.0f;
                    if(vr_avg < 150.0f){
                        if(az_norm > 90.0f || az_norm < -90.0f){
                            final_ok = 0;
                        }
                        if(el2 < -15.0f || el2 > 75.0f){
                            final_ok = 0;
                        }
                        if(r2 > 9000.0f || r0 > 9000.0f){
                            final_ok = 0;
                        }
                    }
                }

                /* [FIX phy-gate v2] 物理硬门(交叉H/R/Vr/beam)：
                 * drone_like=1 必须全满足:
                 *   H_avg in [40,160]m, H_min>=20m, 所有R<=6000m,
                 *   单点|Vr|<=28, 至少2点beamNo in [1,3], beam跨度<=4
                 * 非无人机:  H_max<20 或 H_max>25km 或 R>15km -> 拒
                 */
                {
                    const float PHY_PI_D = 0.0174532925f;
                    float h0_ = r0 * (float)sin(el0 * PHY_PI_D);
                    float h1_ = r1 * (float)sin(el1 * PHY_PI_D);
                    float h2_ = r2 * (float)sin(el2 * PHY_PI_D);
                    float hmin_ = h0_, hmax_ = h0_, havg_;
                    if(h1_ < hmin_) hmin_ = h1_;
                    if(h2_ < hmin_) hmin_ = h2_;
                    if(h1_ > hmax_) hmax_ = h1_;
                    if(h2_ > hmax_) hmax_ = h2_;
                    havg_ = (h0_ + h1_ + h2_) / 3.0f;
                    if(drone_like){
                        if(havg_ < 40.0f || havg_ > 160.0f) final_ok = 0;
                        if(hmin_ < 20.0f)                      final_ok = 0;
                        if(r0 > 6000.0f || r1 > 6000.0f || r2 > 6000.0f) final_ok = 0;
                        {
                            float a0 = (vr0 < 0.0f) ? -vr0 : vr0;
                            float a1 = (vr1 < 0.0f) ? -vr1 : vr1;
                            float a2 = (vr2 < 0.0f) ? -vr2 : vr2;
                            if(a0 > 28.0f || a1 > 28.0f || a2 > 28.0f) final_ok = 0;
                        }
                        {
                            int core = 0;
                            if(beam0 >= 1.0f && beam0 <= 3.0f) core++;
                            if(beam1 >= 1.0f && beam1 <= 3.0f) core++;
                            if(beam2 >= 1.0f && beam2 <= 3.0f) core++;
                            if(core < 2) final_ok = 0;
                            {
                                float bs = (beam2 >= beam0) ? (beam2 - beam0) : (beam0 - beam2);
                                if(bs > 4.0f) final_ok = 0;
                            }
                        }
                    } else {
                        if(hmax_ < 20.0f)                          final_ok = 0;
                        if(hmax_ > 25000.0f || hmin_ < -50.0f)    final_ok = 0;
                        if(r0 > 15000.0f || r2 > 15000.0f)        final_ok = 0;
                    }
                }

                // ===== ⑧ 波位合理性检查(原⑦) ====='''
nr = nr[:A6] + NEW_6TO7 + nr[A7:]
# 删除重复的 ⑧ 交叉验证 已变成⑨
nr = nr.replace("// ===== ⑧ 交叉验证：Vr符号与R变化趋势一致性 =====",
                "// ===== ⑨ 交叉验证：Vr符号与R变化趋势一致性 =====")

with open(NR, "w", encoding="utf-8") as f:
    f.write(nr)
print("OK phy-gate inserted + cross-validate renumbered.")