#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include <string.h>
#include "struct.h"
#include "matrix.h"
#include "global_variable.h"
#include "kalman.h"
#include "matrix_mul.h"
#include "matrix_process.h"
#include "imm.h"
#include <DSPF_sp_lud/c66/DSPF_sp_lud.h>
#include <DSPF_sp_lud_inv/c66/DSPF_sp_lud_inv.h>
#include "track_initial.h"
#include <stdint.h>
#include <float.h>

#pragma pack(4)
extern struct debugger_track track_debugger;

#define ASSO_VELOCITY_CHANGE_THRESHOLD 80.0f
#define PRE_GATE_DISTANCE  250.0f

static void compute_R_from_Z(const float *Z, float R_out[3][3])
{
    float rho, theta, eps;
    float ct, st, ce, se;
    float sr2, sa2, sb2;
    float dxdr, dxdt, dxde, dydr, dydt, dyde, dzdr, dzde;

    rho = sqrtf(Z[0]*Z[0] + Z[1]*Z[1] + Z[2]*Z[2]);
    theta = atan2f(Z[1], Z[0]);
    eps = atan2f(Z[2], sqrtf(Z[0]*Z[0] + Z[1]*Z[1]));
    ct = cosf(theta); st = sinf(theta);
    ce = cosf(eps);   se = sinf(eps);
    sr2 = sigma_r * sigma_r;
    sa2 = sigma_a * sigma_a;
    sb2 = sigma_b * sigma_b;

    dxdr = ct*ce;
    dxdt = -rho*st*ce;
    dxde = -rho*ct*se;
    dydr = st*ce;
    dydt =  rho*ct*ce;
    dyde = -rho*st*se;
    dzdr = se;
    dzde =  rho*ce;

    R_out[0][0] = dxdr*dxdr*sr2 + dxdt*dxdt*sa2 + dxde*dxde*sb2;
    R_out[0][1] = dxdr*dydr*sr2 + dxdt*dydt*sa2 + dxde*dyde*sb2;
    R_out[0][2] = dxdr*dzdr*sr2 + dxde*dzde*sb2;
    R_out[1][0] = R_out[0][1];
    R_out[1][1] = dydr*dydr*sr2 + dydt*dydt*sa2 + dyde*dyde*sb2;
    R_out[1][2] = dydr*dzdr*sr2 + dyde*dzde*sb2;
    R_out[2][0] = R_out[0][2];
    R_out[2][1] = R_out[1][2];
    R_out[2][2] = dzdr*dzdr*sr2 + dzde*dzde*sb2;
}

void track_asso(struct target_track (*target_data),
                struct reliable (*reliable_track),
                int (*reliable_track_num))
{
    int i;
    static int dot_used[60];
    int init_count;
    int ref_dot;
    float best_dist_val;
    int best_dot_idx;
    float best_T;
    float cur_x, cur_y, cur_z, cur_vx, cur_vy, cur_vz;
    float T_asso;
    float lt;
    float pred_x, pred_y, pred_z;
    float gate, gate_sq;
    float dx, dy, dz, dist_sq;
    float R_mat[3][3];
    IMM_STATE imm_tmp;
    float Xp[6], Pp[6][6];
    float prev_vel;
    IMM_STATE imm_work;
    float X_pred_fused[6], P_pred_fused[6][6];
    float X_fused[6], P_fused[6][6];
    float vel_change;
    int rd;
    IMM_STATE imm_s;
    float Xm[6], Pm[6][6];
    float frame_msec;
    float lt_frame;
    int found_ts;

    if((*reliable_track_num) <= 0 || (*reliable_track_num) > num_reliable) return;

    init_count = (cpi_num < 60) ? cpi_num : 60;
    for(i = 0; i < init_count; i++){
        dot_used[i] = 0;
    }

    for(i = 0; i < (*reliable_track_num); i++){
        reliable_track[i].track_update_flag = 0;
    }

    frame_msec = target_data[0].mSecond;

    for(loop_of_track = 0; loop_of_track < (*reliable_track_num); loop_of_track++){

        best_dist_val = FLT_MAX;
        best_dot_idx = -1;
        best_T = 0.0f;
        T_asso = 0.0f;

        cur_x = reliable_track[loop_of_track].X1[0];
        cur_y = reliable_track[loop_of_track].X1[2];
        cur_z = reliable_track[loop_of_track].X1[4];
        cur_vx = reliable_track[loop_of_track].X1[1];
        cur_vy = reliable_track[loop_of_track].X1[3];
        cur_vz = reliable_track[loop_of_track].X1[5];

        for(ref_dot = 0; ref_dot < cpi_num; ref_dot++){
            if(target_data[ref_dot].Use_Flag_1 == 1){
                lt = target_data[ref_dot].mSecond - reliable_track[loop_of_track].mSecond;
                if(lt >= time_down && lt <= time_up){
                    T_asso = lt / 1000.0f;
                    break;
                }
            }
        }

        if(T_asso < 0.001f){
            lt_frame = frame_msec - reliable_track[loop_of_track].mSecond;
            if(lt_frame >= time_down && lt_frame <= time_up){
                T_asso = lt_frame / 1000.0f;
            }
        }

        pred_x = cur_x + T_asso * cur_vx;
        pred_y = cur_y + T_asso * cur_vy;
        pred_z = cur_z + T_asso * cur_vz;
        gate = PRE_GATE_DISTANCE + T_asso * 150.0f;
        gate_sq = gate * gate;

        for(loop_of_dot = 0; loop_of_dot < cpi_num; loop_of_dot++){
            if(target_data[loop_of_dot].Use_Flag_1 == 0) continue;
            if(dot_used[loop_of_dot] == 1) continue;

            lag_time = target_data[loop_of_dot].mSecond - reliable_track[loop_of_track].mSecond;

            if(lag_time < time_down || lag_time > time_up){
                continue;
            }

            dx = target_data[loop_of_dot].x - pred_x;
            dy = target_data[loop_of_dot].y - pred_y;
            dz = target_data[loop_of_dot].z - pred_z;
            dist_sq = dx*dx + dy*dy + dz*dz;
            if(dist_sq > gate_sq){
                continue;
            }

            o = lag_time / 1000.0f;
            T = o;

            Z_obse[0] = target_data[loop_of_dot].x;
            Z_obse[1] = target_data[loop_of_dot].y;
            Z_obse[2] = target_data[loop_of_dot].z;

            compute_R_from_Z(Z_obse, R_mat);

            memcpy(&imm_tmp, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
            imm_predict(&imm_tmp, o, Xp, Pp);
            d = imm_d_cal(&imm_tmp, o, Z_obse, (const float*)R_mat);

            if(d < best_dist_val){
                best_dist_val = d;
                best_dot_idx = loop_of_dot;
                best_T = o;
            }
        }

        if(best_dot_idx < 0 || best_dot_idx >= cpi_num){
            reliable_track[loop_of_track].predict_flag++;
            if(T_asso > 0.001f){
                found_ts = 0;
                memcpy(&imm_s, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
                imm_miss(&imm_s, T_asso);
                imm_get_fused_state(&imm_s, Xm, Pm);
                memcpy(&reliable_track[loop_of_track].imm, &imm_s, sizeof(IMM_STATE));
                memcpy(reliable_track[loop_of_track].X0, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P0, Pm, sizeof(Pm));
                memcpy(reliable_track[loop_of_track].X1, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P1, Pm, sizeof(Pm));
                for(rd = 0; rd < cpi_num; rd++){
                    if(target_data[rd].Use_Flag_1 == 1){
                        reliable_track[loop_of_track].mSecond = target_data[rd].mSecond;
                        found_ts = 1;
                        break;
                    }
                }
                if(!found_ts){
                    reliable_track[loop_of_track].mSecond = frame_msec;
                }
            }
            continue;
        }
        if(target_data[best_dot_idx].Use_Flag_1 == 0){
            reliable_track[loop_of_track].predict_flag++;
            if(T_asso > 0.001f){
                memcpy(&imm_s, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
                imm_miss(&imm_s, T_asso);
                imm_get_fused_state(&imm_s, Xm, Pm);
                memcpy(&reliable_track[loop_of_track].imm, &imm_s, sizeof(IMM_STATE));
                memcpy(reliable_track[loop_of_track].X0, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P0, Pm, sizeof(Pm));
                memcpy(reliable_track[loop_of_track].X1, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P1, Pm, sizeof(Pm));
                reliable_track[loop_of_track].mSecond = frame_msec;
            }
            continue;
        }
        if(best_dist_val >= TRACK_ASSO_TH){
            reliable_track[loop_of_track].predict_flag++;
            if(T_asso > 0.001f){
                memcpy(&imm_s, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
                imm_miss(&imm_s, T_asso);
                imm_get_fused_state(&imm_s, Xm, Pm);
                memcpy(&reliable_track[loop_of_track].imm, &imm_s, sizeof(IMM_STATE));
                memcpy(reliable_track[loop_of_track].X0, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P0, Pm, sizeof(Pm));
                memcpy(reliable_track[loop_of_track].X1, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P1, Pm, sizeof(Pm));
                reliable_track[loop_of_track].mSecond = target_data[best_dot_idx].mSecond;
            }
            continue;
        }

        Z_obse[0] = target_data[best_dot_idx].x;
        Z_obse[1] = target_data[best_dot_idx].y;
        Z_obse[2] = target_data[best_dot_idx].z;

        prev_vel = sqrtf(reliable_track[loop_of_track].X1[1] * reliable_track[loop_of_track].X1[1] +
                         reliable_track[loop_of_track].X1[3] * reliable_track[loop_of_track].X1[3] +
                         reliable_track[loop_of_track].X1[5] * reliable_track[loop_of_track].X1[5]);

        compute_R_from_Z(Z_obse, R_mat);

        memcpy(&imm_work, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
        imm_predict(&imm_work, best_T, X_pred_fused, P_pred_fused);
        imm_update(&imm_work, best_T, Z_obse, (const float*)R_mat);

        imm_get_fused_state(&imm_work, X_fused, P_fused);

        norm = sqrtf(X_fused[1]*X_fused[1] + X_fused[3]*X_fused[3] + X_fused[5]*X_fused[5]);
        vel_change = fabsf(norm - prev_vel);

        if(norm >= track_debugger.Vmin && norm <= track_debugger.Vmax && vel_change <= ASSO_VELOCITY_CHANGE_THRESHOLD){
            dot_used[best_dot_idx] = 1;

            memcpy(&reliable_track[loop_of_track].imm, &imm_work, sizeof(IMM_STATE));

            memcpy(reliable_track[loop_of_track].X0, X_pred_fused, sizeof(X_pred_fused));
            memcpy(reliable_track[loop_of_track].P0, P_pred_fused, sizeof(P_pred_fused));
            memcpy(reliable_track[loop_of_track].X1, X_fused, sizeof(X_fused));
            memcpy(reliable_track[loop_of_track].P1, P_fused, sizeof(P_fused));

            reliable_track[loop_of_track].track_update_flag = 1;
            reliable_track[loop_of_track].predict_flag = 0;
            reliable_track[loop_of_track].tgtnum  = target_data[best_dot_idx].tgtnum;
            reliable_track[loop_of_track].Year    = target_data[best_dot_idx].Year;
            reliable_track[loop_of_track].Month   = target_data[best_dot_idx].Month;
            reliable_track[loop_of_track].Day     = target_data[best_dot_idx].Day;
            reliable_track[loop_of_track].Hour    = target_data[best_dot_idx].Hour;
            reliable_track[loop_of_track].Minute  = target_data[best_dot_idx].Minute;
            reliable_track[loop_of_track].Second  = target_data[best_dot_idx].Second;
            reliable_track[loop_of_track].mSecond = target_data[best_dot_idx].mSecond;
            target_data[best_dot_idx].Use_Flag_1 = 0;
        } else {
            reliable_track[loop_of_track].predict_flag++;
            if(best_T > 0.001f){
                memcpy(&imm_s, &reliable_track[loop_of_track].imm, sizeof(IMM_STATE));
                imm_miss(&imm_s, best_T);
                imm_get_fused_state(&imm_s, Xm, Pm);
                memcpy(&reliable_track[loop_of_track].imm, &imm_s, sizeof(IMM_STATE));
                memcpy(reliable_track[loop_of_track].X0, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P0, Pm, sizeof(Pm));
                memcpy(reliable_track[loop_of_track].X1, Xm, sizeof(Xm));
                memcpy(reliable_track[loop_of_track].P1, Pm, sizeof(Pm));
                reliable_track[loop_of_track].mSecond = target_data[best_dot_idx].mSecond;
            }
        }
    }
}
