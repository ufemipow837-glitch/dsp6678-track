import re, sys
NR = r"d:\DSP\6678\track\track_1\new_reliable.c"
with open(NR, "r", encoding="utf-8", errors="ignore") as f:
    nr = f.read()
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
with open(NR, "w", encoding="utf-8") as f:
    f.write(nr)
print("[fix1c OK] wb-cap trim inserted at pos %d. File written." % ci)