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

                /* [FIX phy-gate v2] 物理硬门交叉：
                 * drone_like=1必须全满足: H_avg[40,160], H_min>=20, R<=6000, |Vr|<=28,
                 *                         至少2点beamNo in [1,3], beam跨度<=4
                 * 非无人机: H<20 / H>25km / R>15km -> 拒
                 */
                {
                    const float PHY_PI_D = 0.0174532925f;
                    float h0_ = r0 * sinf(el0 * PHY_PI_D);
                    float h1_ = r1 * sinf(el1 * PHY_PI_D);
                    float h2_ = r2 * sinf(el2 * PHY_PI_D);
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
                            float a0 = vr0<0.0f?-vr0:vr0;
                            float a1 = vr1<0.0f?-vr1:vr1;
                            float a2 = vr2<0.0f?-vr2:vr2;
                            if(a0 > 28.0f || a1 > 28.0f || a2 > 28.0f) final_ok = 0;
                        }
                        {
                            int core = 0;
                            if(beam0 >= 1.0f && beam0 <= 3.0f) core++;
                            if(beam1 >= 1.0f && beam1 <= 3.0f) core++;
                            if(beam2 >= 1.0f && beam2 <= 3.0f) core++;
                            if(core < 2) final_ok = 0;
                            {
                                float bs = (beam2>=beam0)?(beam2-beam0):(beam0-beam2);
                                if(bs > 4.0f) final_ok = 0;
                            }
                        }
                    } else {
                        if(hmax_ < 20.0f)                        final_ok = 0;
                        if(hmax_ > 25000.0f || hmin_ < -50.0f)  final_ok = 0;
                        if(r0 > 15000.0f || r2 > 15000.0f)      final_ok = 0;
                    }
                }

                // ===== ⑧ 波位合理性检查(原⑦) ====='''

nr = nr[:A6] + NEW_6TO7 + nr[A7:]
nr = nr.replace("// ===== ⑧ 交叉验证：Vr符号与R变化趋势一致性 =====",
                "// ===== ⑨ 交叉验证：Vr符号与R变化趋势一致性 =====")
print("[fix1a] phy-gate inserted + cross-validate renumbered")

# 1b 严格类型分类 + rollback goto
CLS_OLD = '''            // ===== 基于3点平均Vr的目标分类 =====
            // 使用3点径向速度平均值（而非仅第3点），分类更鲁棒
            {
                float vr0_abs, vr1_abs, vr2_abs;
                float vr_avg_3pt;
                vr0_abs = fabsf(track_asso_data[best_idx*3+0].radar_vr);
                vr1_abs = fabsf(track_asso_data[best_idx*3+1].radar_vr);
                vr2_abs = fabsf(track_asso_data[best_idx*3+2].radar_vr);
                vr_avg_3pt = (vr0_abs + vr1_abs + vr2_abs) / 3.0f;

                if(vr_avg_3pt < 30.0f){
                    reliable_track[idx].target_type = 2;   // 无人机 (低速, <30m/s)
                    init_hint = 2;
                } else if(vr_avg_3pt > 80.0f){
                    reliable_track[idx].target_type = 1;   // 炮弹 (高速, >80m/s)
                    init_hint = 1;
                } else {
                    reliable_track[idx].target_type = 0;   // 其他 (30-80m/s, 直升机/飞机等)
                    init_hint = 0;
                }
            }'''
assert CLS_OLD in nr, "classification old anchor missing"
CLS_NEW = '''            // ===== 基于3点平均Vr的目标分类(v2 严格一致) =====
            // avg + 单点max + drone_like标记 三者必须一致；否则final_ok=0回滚
            {
                float vr0_abs, vr1_abs, vr2_abs;
                float vr_avg_3pt, vr_max_abs;
                vr0_abs = fabsf(track_asso_data[best_idx*3+0].radar_vr);
                vr1_abs = fabsf(track_asso_data[best_idx*3+1].radar_vr);
                vr2_abs = fabsf(track_asso_data[best_idx*3+2].radar_vr);
                vr_avg_3pt = (vr0_abs + vr1_abs + vr2_abs) / 3.0f;
                vr_max_abs = vr0_abs;
                if(vr1_abs > vr_max_abs) vr_max_abs = vr1_abs;
                if(vr2_abs > vr_max_abs) vr_max_abs = vr2_abs;

                if(vr_avg_3pt < 30.0f && vr_max_abs < 30.0f && drone_like){
                    reliable_track[idx].target_type = 2;   /* 无人机 */
                    init_hint = 2;
                } else if(vr_avg_3pt > 80.0f && vr_max_abs > 80.0f && !drone_like){
                    reliable_track[idx].target_type = 1;   /* 炮弹 */
                    init_hint = 1;
                } else if(vr_avg_3pt >= 30.0f && vr_avg_3pt <= 80.0f){
                    reliable_track[idx].target_type = 0;   /* 其他 */
                    init_hint = 0;
                } else {
                    /* 分类矛盾 -> 撤销该次创建 */
                    final_ok = 0;
                    reliable_track[idx].target_type = 0;
                    init_hint = 0;
                }
            }
            /* [FIX rollback v2] final_ok=0时：回滚计数+释放索引+恢复asso_flag=1让后续3点再尝试 */
            if(!final_ok){
                (*reliable_track_num)--;
                (*reliable_num_all)--;
                {
                    int ri2_ = (*reliable_track_num);
                    if(ri2_ < 0) ri2_ = 0;
                    memset(&reliable_track[ri2_], 0, sizeof(reliable_track[ri2_]));
                }
                track_asso_data[best_idx*3+0].asso_flag = 1;
                track_asso_data[best_idx*3+1].asso_flag = 1;
                track_asso_data[best_idx*3+2].asso_flag = 1;
                goto reliable_reject_rollback;
            }'''
nr = nr.replace(CLS_OLD, CLS_NEW)
print("[fix1b] strict class + goto rollback applied")

# 1c 写回容量激进裁剪 (1-point候选)
WBM = "already-processed skip */"
wi = nr.find(WBM)
assert wi > 0, "wb marker not found"
ci = nr.find("continue", wi - 300)
ci = nr.find(";", ci) + 1
while ci < len(nr) and nr[ci] in "\r\n":
    ci += 1
PATCH = '''
        /* [FIX wb-cap 1-point clutter trim] 防temp_before=99永久满载卡死新3点初始化
         * 仅对 cnt_asso<=1 (1/0-point候选项)裁剪；2/3-point组合一律保留 */
        {
            int cnt_asso = 0;
            if(track_asso_data[i*3+0].asso_flag == 1) cnt_asso++;
            if(track_asso_data[i*3+1].asso_flag == 1) cnt_asso++;
            if(track_asso_data[i*3+2].asso_flag == 1) cnt_asso++;
            if(cnt_asso <= 1){
                float r_  = track_asso_data[i*3+0].radar_r;
                float v_  = track_asso_data[i*3+0].radar_vr;
                float e_  = track_asso_data[i*3+0].radar_el;
                unsigned int bn_ = track_asso_data[i*3+0].radar_beamNo;
                float av_ = v_ < 0.0f ? -v_ : v_;
                int bad = 0;
                if(r_  > 8000.0f)                          bad = 1;
                if(av_ < 0.3f)                              bad = 1;
                if(e_  < -10.0f || e_ > 70.0f)              bad = 1;
                if(bn_ > 7u && r_ > 2500.0f)                bad = 1;
                if(bad){
                    track_asso_data[i*3+0].asso_flag = 0;
                    track_asso_data[i*3+1].asso_flag = 0;
                    track_asso_data[i*3+2].asso_flag = 0;
                    continue;
                }
            }
        }
'''
nr = nr[:ci] + PATCH + nr[ci:]
print("[fix1c] wb-cap trim inserted at pos", ci)

with open(NR, "w", encoding="utf-8") as f:
    f.write(nr)
print("[OK] new_reliable.c written.")