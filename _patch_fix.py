# -*- coding: utf-8 -*-
import io, sys

def patch(path, old, new, tag):
    with io.open(path, 'r', encoding='utf-8', errors='replace') as f:
        s = f.read()
    cnt = s.count(old)
    if cnt != 1:
        print('FAIL [%s] expect 1 match got %d' % (tag, cnt))
        return False
    s = s.replace(old, new, 1)
    with io.open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(s)
    print('OK   [%s] patched' % tag)
    return True

ok = True
p = r'd:\DSP\6678\track\track_1\track_asso.c'

old1 = '''        if(best_dot_idx < 0 || best_dot_idx >= cpi_num){
            reliable_track[loop_of_track].predict_flag++;
            reliable_track[loop_of_track].rapid_fail_count++;
            if(T_asso > 0.001f){
                memcpy(&imm_s, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
                imm_miss(&imm_s, T_asso);
                imm_get_fused_state(&imm_s, Xm, Pm);
                memcpy(&reliable_track[loop_of_track].imm, &imm_s, sizeof(IMM_STATE));
                memcpy(reliable_track[loop_of_track].X0, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P0, Pm, sizeof(Pm));
                memcpy(reliable_track[loop_of_track].X1, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P1, Pm, sizeof(Pm));
                // ===== 关键修复：不更新mSecond =====
                // predict_flag(纯预测)分支保持mSecond为上次关联成功时刻！
                // 这样下帧T_asso会用真实的时间差（可能5.5s、8.3s）计算真实门限
                // 如果强制更新为本帧时间，T_asso会缩成0.164s，门限严重偏小→正确点全部被拒
            }
            continue;
        }
        if(target_data[best_dot_idx].Use_Flag_1 == 0){
            reliable_track[loop_of_track].predict_flag++;
            reliable_track[loop_of_track].rapid_fail_count++;
            if(T_asso > 0.001f){
                memcpy(&imm_s, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
                imm_miss(&imm_s, T_asso);
                imm_get_fused_state(&imm_s, Xm, Pm);
                memcpy(&reliable_track[loop_of_track].imm, &imm_s, sizeof(IMM_STATE));
                memcpy(reliable_track[loop_of_track].X0, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P0, Pm, sizeof(Pm));
                memcpy(reliable_track[loop_of_track].X1, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P1, Pm, sizeof(Pm));
                // 关键修复：不更新mSecond，保持真实时间差
            }
            continue;
        }
        if(best_dist_val >= TRACK_ASSO_TH){
            reliable_track[loop_of_track].predict_flag++;
            reliable_track[loop_of_track].rapid_fail_count++;
            if(T_asso > 0.001f){
                memcpy(&imm_s, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
                imm_miss(&imm_s, T_asso);
                imm_get_fused_state(&imm_s, Xm, Pm);
                memcpy(&reliable_track[loop_of_track].imm, &imm_s, sizeof(IMM_STATE));
                memcpy(reliable_track[loop_of_track].X0, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P0, Pm, sizeof(Pm));
                memcpy(reliable_track[loop_of_track].X1, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P1, Pm, sizeof(Pm));
                // 关键修复：不更新mSecond，保持真实时间差
            }
            continue;
        }'''
new1 = '''        if(best_dot_idx < 0 || best_dot_idx >= cpi_num){
            /* [FIX coast-once 2026-08-31] miss帧不在此推进IMM状态：coast外推统一由
               track_predict每帧imm_miss(lag)做一次。此前本分支imm_miss(T_asso)与
               track_predict imm_miss(lag)双重外推，实测coast速率=2x真值
               (11.74m/s无人机漂3.85m/帧=11.74*0.328，且方向符号错误)。
               mSecond由track_predict每帧刷新，T_asso恒约0.164s(单帧)，门限按单帧计算。 */
            reliable_track[loop_of_track].predict_flag++;
            reliable_track[loop_of_track].rapid_fail_count++;
            continue;
        }
        if(target_data[best_dot_idx].Use_Flag_1 == 0){
            /* [FIX coast-once] 同上：外推只由track_predict做，此处仅计数 */
            reliable_track[loop_of_track].predict_flag++;
            reliable_track[loop_of_track].rapid_fail_count++;
            continue;
        }
        if(best_dist_val >= TRACK_ASSO_TH){
            /* [FIX coast-once] 同上：马氏距离超阈拒绝，外推只由track_predict做 */
            reliable_track[loop_of_track].predict_flag++;
            reliable_track[loop_of_track].rapid_fail_count++;
            continue;
        }'''
ok &= patch(p, old1, new1, 'track_asso miss-blocks x3')

old2 = '''        } else {
            reliable_track[loop_of_track].predict_flag++;
            reliable_track[loop_of_track].rapid_fail_count++;
            if(T_asso > 0.001f){
                memcpy(&imm_s, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
                imm_miss(&imm_s, T_asso);
                imm_get_fused_state(&imm_s, Xm, Pm);
                memcpy(&reliable_track[loop_of_track].imm, &imm_s, sizeof(IMM_STATE));
                memcpy(reliable_track[loop_of_track].X0, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P0, Pm, sizeof(Pm));
                memcpy(reliable_track[loop_of_track].X1, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P1, Pm, sizeof(Pm));
                // 关键修复：不更新mSecond，保持真实时间差
            }
        }'''
new2 = '''        } else {
            /* [FIX coast-once] 7层检查拒绝：仅累计失败计数，外推由track_predict统一做 */
            reliable_track[loop_of_track].predict_flag++;
            reliable_track[loop_of_track].rapid_fail_count++;
        }'''
ok &= patch(p, old2, new2, 'track_asso reject else-block')

old3 = '''    IMM_STATE imm_s;
    float Xm[6], Pm[6][6];
    float frame_msec;'''
new3 = '''    float frame_msec;'''
ok &= patch(p, old3, new3, 'track_asso unused vars')

p2 = r'd:\DSP\6678\track\track_1\new_reliable.c'

old4 = '''                        // 用相对比例检查：R变化量不超过R本身的20%（远距离绝对误差大）
                        float rel_delta = abs(delta_r) / (R_ref + 1.0f);'''
new4 = '''                        // 用相对比例检查：R变化量不超过R本身的20%（远距离绝对误差大）
                        /* [FIX abs-float] C6x abs()为整型重载，浮点必须用三目取绝对值 */
                        float rel_delta = ((delta_r < 0.0f) ? -delta_r : delta_r) / (R_ref + 1.0f);'''
ok &= patch(p2, old4, new4, 'new_reliable abs() float')

old5 = '''        if(track_asso_data[i*3+0].radar_beamNo == 0xFFFFFFFFu) continue;

        /* [FIX wb-cap 1-point clutter trim] 防temp_before=99永久满载卡死新3点初始化
         * 仅对 cnt_asso<=1 (1/0-point候选项)裁剪；2/3-point组合一律保留 */'''
new5 = '''        if(track_asso_data[i*3+0].radar_beamNo == 0xFFFFFFFFu) continue;

        /* [FIX wb-dedup 2026-08-31] 回写仅处理全新1点候选：
         *  - 2点候选已由track_initial的get_track_asso1就地memcpy回temp_track slot1，
         *    再追加只会产生重复条目(同一点链被两个temp槽位匹配->分叉航迹)；
         *  - 3点候选：建航成功者flags已清零(下面skip)，被拒者其temp链已在
         *    track_initial清flag作废，且count==3条目在track_initial永不参与匹配，
         *    追加即死条目。故cnt>=2一律不回写。 */
        {
            int cnt_wb = 0;
            if(track_asso_data[i*3+0].asso_flag == 1) cnt_wb++;
            if(track_asso_data[i*3+1].asso_flag == 1) cnt_wb++;
            if(track_asso_data[i*3+2].asso_flag == 1) cnt_wb++;
            if(cnt_wb >= 2) continue;
        }

        /* [FIX wb-cap 1-point clutter trim] 防temp_before=99永久满载卡死新3点初始化
         * 仅对 cnt_asso<=1 (1/0-point候选项)裁剪；2/3-point组合一律保留 */'''
ok &= patch(p2, old5, new5, 'new_reliable writeback cnt>=2 skip')

old6 = '''    /* [BUG17 SAFETY FALLBACK] if wrote_back=0 even when candidates exist, force-copy
     * all i*3+0 slots where radar_beamNo != 0 (valid candidate). This bypasses any
     * asso_flag corruption caused by [num_temp][3] vs [i*3+j] pointer aliasing. */
    if(k == 0){
        int fb;
        printf("    [DBG new_rel] FALLBACK k==0 num_cand=%d temp_before=%d\\n", num_candidates, (*temp_track_num));
        fflush(stdout);
        for(fb = 0; fb < num_candidates; fb++){
            if(track_asso_data[fb*3+0].radar_beamNo != 0xFFFFFFFFu){
                if((*temp_track_num + k) < num_temp-1){
                    int dst_base = ((*temp_track_num) + k) * 3;
                    int src_base = fb * 3;
                    /* use memcpy-safe struct copy (not aliased assignment) */
                    memcpy(&temp_track[dst_base+0], &track_asso_data[src_base+0], sizeof(temp_track[0]));
                    memcpy(&temp_track[dst_base+1], &track_asso_data[src_base+1], sizeof(temp_track[0]));
                    memcpy(&temp_track[dst_base+2], &track_asso_data[src_base+2], sizeof(temp_track[0]));
                    temp_track[dst_base+0].asso_flag = 1;
                    temp_track[dst_base+1].asso_flag = 0;
                    temp_track[dst_base+2].asso_flag = 0;
                    printf("    [DBG new_rel] fb copy cand[%d] az=%.2f r=%.1f bn=%u -> slot[%d]\\n",
                           fb, track_asso_data[src_base+0].radar_az,
                           track_asso_data[src_base+0].radar_r,
                           (unsigned)track_asso_data[src_base+0].radar_beamNo,
                           (*temp_track_num)+k);
                    fflush(stdout);
                    k++;
                }
            }
        }
    }

    (*temp_track_num) += k;'''
new6 = '''    /* [FIX wb-fallback-removed 2026-08-31] 原FALLBACK在k==0时无视flags/裁剪门
     * 强制回写全部候选(含已建航/已拒绝/3点组合)，是temp_track Frame31即99满载的
     * 直接原因；正常回写路径已覆盖所有有效1点候选，不再需要兜底。 */

    (*temp_track_num) += k;'''
ok &= patch(p2, old6, new6, 'new_reliable FALLBACK removed')

p3 = r'd:\DSP\6678\track\track_1\track_initial.c'
old7 = '''            {
                /* [BUG1 fix] use local ms var (global lag_time stores seconds after matrix) */
                float lag_ms = frame_mSecond - temp_track[loop_of_track*3+count-1].mSecond;
                if(lag_ms > time_up || lag_ms < time_down){
                    continue;
                }
            }'''
new7 = '''            {
                /* [BUG1 fix] use local ms var (global lag_time stores seconds after matrix) */
                float lag_ms = frame_mSecond - temp_track[loop_of_track*3+count-1].mSecond;
                /* [FIX temp-ttl 2026-08-31] 1点候选无速度确认：无人机所在波位每个
                 * 扫描周期(17帧约2.78s)才照射一次，给2.1周期余量=6s仍未凑成2点即按
                 * 杂波过期；2/3点组合保持time_up(25s)。否则杂波1点候选按25s堆积，
                 * num_temp=100槽位Frame31即满载，新3点初始化无法写回。 */
                float ttl_ms = (count == 1) ? 6000.0f : time_up;
                if(lag_ms > ttl_ms || lag_ms < time_down){
                    continue;
                }
            }'''
ok &= patch(p3, old7, new7, 'track_initial 1pt TTL')

print('ALL OK' if ok else 'SOME FAILED')
sys.exit(0 if ok else 1)
