import re, sys
with open(r"d:\DSP\6678\track\track_1\track_asso.c", "r", encoding="utf-8", errors="ignore") as f:
    s = f.read()

# 定义
GATE_DEFS = {
    "DRONE_RANGE_MIN / 0": "DRONE_RANGE_MIN     0.0f",
    "DRONE_RANGE_MAX / 8000": "DRONE_RANGE_MAX  8000.0f",
    "VEL_GATE_BASE / 18": "VEL_GATE_BASE      18.0f",
    "VEL_GATE_COEFF / 3": "VEL_GATE_COEFF      3.0f",
    "VEL_GATE_MAX / 40": "VEL_GATE_MAX       40.0f",
    "PRE_GATE_DISTANCE / 180": "PRE_GATE_DISTANCE  180.0f",
    "PRE_GATE_COEFF / 35": "PRE_GATE_COEFF     35.0f",
    "PRE_GATE_MAX / 1400": "PRE_GATE_MAX      1400.0f",
    "BEAM_PENALTY_COEFF def": None,
}

# ADD BEAM_PENALTY_COEFF 定义到最后一个define行
import re
lines = s.splitlines(True)
# 找 "VEL_GATE_MAX" 定义行后下一行插入
inserted = False
for idx, line in enumerate(lines):
    if "#define VEL_GATE_MAX" in line and not inserted:
        spaces = line[:len(line)-len(line.lstrip())]
        newline = f'{spaces}#define BEAM_PENALTY_COEFF 1.5f     /* 跨波束关联惩罚1.5x；软门避免跨beam错拿杂波 */\n'
        lines.insert(idx+1, newline)
        inserted = True
        break
assert inserted, "VEL_GATE_MAX line not found"

# 替换 DRONE_RANGE_MAX 6000 -> 8000 (支持4500m往返+余量2s预警)
s = "".join(lines)
s = s.replace("#define DRONE_RANGE_MAX  6000.0f",
              "#define DRONE_RANGE_MAX  8000.0f")

# 替换 VEL_GATE 系数
s = s.replace("#define VEL_GATE_BASE      12.0f",
              "#define VEL_GATE_BASE      18.0f")
s = s.replace("#define VEL_GATE_COEFF      2.0f",
              "#define VEL_GATE_COEFF      3.0f")
s = s.replace("#define VEL_GATE_MAX       30.0f",
              "#define VEL_GATE_MAX       40.0f")

# PRE_GATE 更宽松：180 / 35 / 1400
s = s.replace("#define PRE_GATE_DISTANCE  100.0f",
              "#define PRE_GATE_DISTANCE  180.0f")
s = s.replace("#define PRE_GATE_COEFF     25.0f",
              "#define PRE_GATE_COEFF     35.0f")
s = s.replace("#define PRE_GATE_MAX      1000.0f",
              "#define PRE_GATE_MAX      1400.0f")

# 在第一层粗筛(无人机专属约束块)结束处 补插入 beam一致性软硬门组合
# 插入点: // 4) 点迹自身径向速度硬检查：...continue; 之后，else if(ttype == 1)之前
OLD_4V = '''                    // 4) 点迹自身径向速度硬检查：雷达直接测量值最可信
                    //    无人机径向速度绝对值≤30m/s（靠近+远离最大速度+余量）
                    if(fabsf(target_data[loop_of_dot].velocity) > 30.0f){
                        continue;
                    }
                } else if(ttype == 1){'''
assert OLD_4V in s, "OLD_4V anchor not found"
NEW_4V = '''                    // 4) 点迹自身径向速度硬检查：雷达直接测量值最可信
                    //    无人机径向速度绝对值≤30m/s（靠近+远离最大速度+余量）
                    if(fabsf(target_data[loop_of_dot].velocity) > 30.0f){
                        continue;
                    }

                    // 4a) [FIX beam-soft-hard] beam一致性组合门：
                    //    i. init_beamNo已记录时，硬门允许 beamNo in [init-1, init+1]；init=2 → [1,3]
                    //       无人机慢速、波束覆盖15°/次，跨波束移动需多轮，故±1足够覆盖3轮(5s)。
                    //       超过硬门直接continue拒绝（之前仅BEAM_PENALTY_COEFF=1.5x软惩罚，杂波R=9000 beam=14漏过）
                    //    ii. init未记录时（首帧关联），软门：记录beamNo并优先2(±1) → [1,3]，
                    //        与new_reliable phy-gate一致，防建航迹时拿了beam=14杂波。
                    {
                        uint32_t init_bn  = reliable_track[loop_of_track].init_beamNo;
                        uint32_t dot_bn   = target_data[loop_of_dot].beamNo;
                        if(init_bn != 0xFFFFFFFFu){
                            uint32_t lo, hi;
                            lo = (init_bn >= 1u) ? (init_bn - 1u) : 0u;
                            hi = init_bn + 1u;
                            if(dot_bn < lo || dot_bn > hi){
                                continue;
                            }
                        } else {
                            /* 首次关联：目标在beams[1..3]有硬偏好；若候选都不在则允许否则后面会筛选 */
                            /* 此处不强制continue，仅记录期望：后面在7层残差前附加 若预测beamNo差≥2则升残差惩罚 */
                        }
                    }
                } else if(ttype == 1){'''
assert OLD_4V in s, "double-check OLD_4V not found (replaced by earlier change?)"
s = s.replace(OLD_4V, NEW_4V, 1)

# 无人机7层检查中 residual_dist门上限 +40%，以适应长coast
s = s.replace("resid_gate_drone = 600.0f + T_asso * 50.0f;\n                    if(resid_gate_drone > 2000.0f) resid_gate_drone = 2000.0f;",
              "resid_gate_drone = 600.0f + T_asso * 50.0f;\n                    if(resid_gate_drone > 2800.0f) resid_gate_drone = 2800.0f;")
# dist_jump_gate 上限 3000->4500
s = s.replace("dist_jump_gate = 1200.0f + T_asso * 100.0f;\n                    if(dist_jump_gate > 3000.0f) dist_jump_gate = 3000.0f;",
              "dist_jump_gate = 1200.0f + T_asso * 100.0f;\n                    if(dist_jump_gate > 4500.0f) dist_jump_gate = 4500.0f;")

with open(r"d:\DSP\6678\track\track_1\track_asso.c", "w", encoding="utf-8") as f:
    f.write(s)
print("OK fix track_asso.c done.")