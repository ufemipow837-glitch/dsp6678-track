#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include <float.h>
#include "struct.h"
#include "track.h"
#include "matrix.h"
#include "global_variable.h"
#include "kalman.h"
#include "matrix_mul.h"
#include "matrix_process.h"
#include <DSPF_sp_lud/c66/DSPF_sp_lud.h>
#include <DSPF_sp_lud_inv/c66/DSPF_sp_lud_inv.h>

#pragma pack(4)
extern struct debugger_track track_debugger;

#define TRACK_QUALITY_THRESHOLD 0.7f
#define VELOCITY_CONSISTENCY_THRESHOLD 0.3f
#define ANGLE_CHANGE_THRESHOLD 90.0f
#define INIT_SIGMA_A 0.3f
#define INIT_SIGMA_B 0.3f
#define INIT_SIGMA_R 5.0f
#define MAX_NEW_TRACKS_PER_FRAME 20

static float calculate_track_quality(float *pos1, float *pos2, float *pos3, float dt) {
    float quality;
    float v12_x, v12_y, v12_z;
    float v23_x, v23_y, v23_z;
    float v12_norm, v23_norm;
    float dot_product, cos_angle;
    float velocity_consistency;
    float min_norm, max_norm, speed_ratio;
    float v_x, v_y, v_z, speed;
    float speed_norm;

    quality = 0.0f;
    if (pos3 != NULL) {
        v12_x = (pos2[0] - pos1[0]) / dt;
        v12_y = (pos2[1] - pos1[1]) / dt;
        v12_z = (pos2[2] - pos1[2]) / dt;
        v23_x = (pos3[0] - pos2[0]) / dt;
        v23_y = (pos3[1] - pos2[1]) / dt;
        v23_z = (pos3[2] - pos2[2]) / dt;
        
        v12_norm = sqrtf(v12_x*v12_x + v12_y*v12_y + v12_z*v12_z);
        v23_norm = sqrtf(v23_x*v23_x + v23_y*v23_y + v23_z*v23_z);
        
        dot_product = v12_x*v23_x + v12_y*v23_y + v12_z*v23_z;
        cos_angle = dot_product / (v12_norm * v23_norm + FLT_EPSILON);
        if(cos_angle > 1.0f) cos_angle = 1.0f;
        if(cos_angle < -1.0f) cos_angle = -1.0f;
        
        velocity_consistency = (1.0f + cos_angle) / 2.0f;
        min_norm = (v12_norm < v23_norm) ? v12_norm : v23_norm;
        max_norm = (v12_norm > v23_norm) ? v12_norm : v23_norm;
        speed_ratio = min_norm / (max_norm + FLT_EPSILON);
        
        quality = (velocity_consistency + speed_ratio) / 2.0f;
    } else {
        v_x = (pos2[0] - pos1[0]) / dt;
        v_y = (pos2[1] - pos1[1]) / dt;
        v_z = (pos2[2] - pos1[2]) / dt;
        speed = sqrtf(v_x*v_x + v_y*v_y + v_z*v_z);
        
        if (speed >= track_debugger.Vmin && speed <= track_debugger.Vmax) {
            quality = 0.8f;
        } else if (speed > 0) {
            speed_norm = fabsf(speed - track_debugger.Vmin) / (track_debugger.Vmax - track_debugger.Vmin + FLT_EPSILON);
            quality = 0.5f * (1.0f - speed_norm);
        }
    }
    return quality;
}

static void get_asso_info1_dist(struct target_track *dot, struct temp_track *tt, float *dist_out) {
    float d1, d2, d3;
    d1 = dot->x - tt[0].X[0];
    d2 = dot->y - tt[0].X[2];
    d3 = dot->z - tt[0].X[4];
    *dist_out = d1*d1 + d2*d2 + d3*d3;
}

static void get_asso_info2_dist(struct target_track *dot, struct temp_track *tt, int loop_of_track_val, float *dist_out) {
    float rho, theta, eps;
    float ct, st, ce, se;
    float sr, sa, sb;
    float sr2, sa2, sb2;
    float dxdr, dxdt, dxde, dydr, dydt, dyde, dzdr, dzdt, dzde;
    static unsigned short p[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

    Z_obse[0] = dot->x;
    Z_obse[1] = dot->y;
    Z_obse[2] = dot->z;

    cart2sph(&Z_obse[0], &Z_obse[1], &Z_obse[2], &rho, &theta, &eps);

    ct = cosf(theta); st = sinf(theta);
    ce = cosf(eps);   se = sinf(eps);
    sr = INIT_SIGMA_R; sa = INIT_SIGMA_A / 180.0f * pi; sb = INIT_SIGMA_B / 180.0f * pi;
    sr2 = sr*sr; sa2 = sa*sa; sb2 = sb*sb;
    dxdr = ct*ce; dxdt = -rho*st*ce; dxde = -rho*ct*se;
    dydr = st*ce; dydt =  rho*ct*ce; dyde = -rho*st*se;
    dzdr = se;     dzdt = 0.0f;     dzde =  rho*ce;
    R[0][0] = dxdr*dxdr*sr2 + dxdt*dxdt*sa2 + dxde*dxde*sb2;
    R[0][1] = dxdr*dydr*sr2 + dxdt*dydt*sa2 + dxde*dyde*sb2;
    R[0][2] = dxdr*dzdr*sr2 + dxdt*dzdt*sa2 + dxde*dzde*sb2;
    R[1][0] = R[0][1];
    R[1][1] = dydr*dydr*sr2 + dydt*dydt*sa2 + dyde*dyde*sb2;
    R[1][2] = dydr*dzdr*sr2 + dydt*dzdt*sa2 + dyde*dzde*sb2;
    R[2][0] = R[0][2];
    R[2][1] = R[1][2];
    R[2][2] = dzdr*dzdr*sr2 + dzdt*dzdt*sa2 + dzde*dzde*sb2;

    memcpy(X_last, tt[loop_of_track_val * 3 + 1].X, sizeof(tt[loop_of_track_val * 3 + 1].X));
    memcpy(P_last, tt[loop_of_track_val * 3 + 1].P, sizeof(tt[loop_of_track_val * 3 + 1].P));
    DSP_float_mul_general((float *)&F[0], 6, 6, (float *)&X_last[0], 1, (float *)&X_predict[0]);
    DSP_float_mul_general((float *)F, 6, 6, (float *)P_last, 6, (float *)FP);
    DSP_float_mul_general((float *)FP, 6, 6, (float *)F_trans, 6, (float *)FPF);
    DSP_float_add((float *)FPF, (float *)GQG, 6, 6, (float *)P_predict);
    DSP_float_mul_general((float *)H, 3, 6, (float *)X_predict, 1, (float *)&Z_predict[0]);
    DSP_float_mul_general((float *)H, 3, 6, (float *)P_predict, 6, (float *)HP);
    DSP_float_mul_general((float *)HP, 3, 6, (float *)H_trans, 3, (float *)HPH);
    DSP_float_add((float *)HPH, (float *)R, 3, 3, (float *)S);
    DSPF_sp_lud(3, (float *)S, (float *)S_L, (float *)S_U, (unsigned short *)p);
    DSPF_sp_lud_inverse(3, (unsigned short *)p, (float *)S_L, (float *)S_U, (float *)S_inv);
    Z_D[0][0] = Z_obse[0] - Z_predict[0][0];
    Z_D[1][0] = Z_obse[1] - Z_predict[1][0];
    Z_D[2][0] = Z_obse[2] - Z_predict[2][0];
    Z_D_trans[0][0] = Z_D[0][0];
    Z_D_trans[0][1] = Z_D[1][0];
    Z_D_trans[0][2] = Z_D[2][0];
    DSP_float_mul03((float *)Z_D_trans, 1, 3, (float *)S_inv, 3, (float *)Z_D_trans_S_inv);
    *dist_out = Z_D_trans_S_inv[0][0]*Z_D[0][0] + Z_D_trans_S_inv[0][1]*Z_D[1][0] + Z_D_trans_S_inv[0][2]*Z_D[2][0];
}

static void get_track_asso1(struct target_track *target_data_ptr,
                            struct temp_track *temp_track_ptr,
                            int loop_of_track_val,
                            int *track_asso_num,
                            struct temp_track *track_asso_data){
    if((*track_asso_num) < num_temp){
        Z0[0] = temp_track_ptr[loop_of_track_val*3+0].X[0];
        Z0[1] = temp_track_ptr[loop_of_track_val*3+0].X[2];
        Z0[2] = temp_track_ptr[loop_of_track_val*3+0].X[4];
        Z1[0] = target_data_ptr->x;
        Z1[1] = target_data_ptr->y;
        Z1[2] = target_data_ptr->z;
        kalman_filter_init(&Z0[0], &Z1[0], &X0[0], &P0[0][0]);
        *track_asso_num = *track_asso_num + 1;
        track_asso_data[(*track_asso_num-1)*3+0] = temp_track_ptr[loop_of_track_val*3+0];
        memcpy(track_asso_data[(*track_asso_num-1)*3+1].X, X0, sizeof(X0));
        memcpy(track_asso_data[(*track_asso_num-1)*3+1].P, P0, sizeof(P0));
    
        track_asso_data[(*track_asso_num-1)*3+1].tgtnum    = target_data_ptr->tgtnum;
        track_asso_data[(*track_asso_num-1)*3+1].beamNo    = target_data_ptr->beamNo;
        track_asso_data[(*track_asso_num-1)*3+1].Year     = target_data_ptr->Year;
        track_asso_data[(*track_asso_num-1)*3+1].Month    = target_data_ptr->Month;
        track_asso_data[(*track_asso_num-1)*3+1].Day      = target_data_ptr->Day;
        track_asso_data[(*track_asso_num-1)*3+1].Hour     = target_data_ptr->Hour;
        track_asso_data[(*track_asso_num-1)*3+1].Minute   = target_data_ptr->Minute;
        track_asso_data[(*track_asso_num-1)*3+1].Second   = target_data_ptr->Second;
        track_asso_data[(*track_asso_num-1)*3+1].mSecond  = target_data_ptr->mSecond;
        track_asso_data[(*track_asso_num-1)*3+1].asso_flag = 1;
        track_asso_data[(*track_asso_num-1)*3+2].asso_flag = 0;
    }
}

static void get_track_asso2(struct target_track *target_data_ptr,
                            struct temp_track *temp_track_ptr,
                            int loop_of_track_val,
                            int *track_asso_num,
                            struct temp_track *track_asso_data){
    if((*track_asso_num) < num_temp){
        Z1[0] = temp_track_ptr[loop_of_track_val*3+1].X[0];
        Z1[1] = temp_track_ptr[loop_of_track_val*3+1].X[2];
        Z1[2] = temp_track_ptr[loop_of_track_val*3+1].X[4];
        Z2[0] = target_data_ptr->x;
        Z2[1] = target_data_ptr->y;
        Z2[2] = target_data_ptr->z;
        kalman_filter_init(&Z1[0], &Z2[0], &X0[0], &P0[0][0]);
        *track_asso_num = *track_asso_num + 1;
        track_asso_data[(*track_asso_num-1)*3+0] = temp_track_ptr[loop_of_track_val*3+0];
        track_asso_data[(*track_asso_num-1)*3+1] = temp_track_ptr[loop_of_track_val*3+1];
    
        memcpy(track_asso_data[(*track_asso_num-1)*3+2].X, X0, sizeof(X0));
        memcpy(track_asso_data[(*track_asso_num-1)*3+2].P, P0, sizeof(P0));
    
        track_asso_data[(*track_asso_num-1)*3+2].tgtnum    = target_data_ptr->tgtnum;
        track_asso_data[(*track_asso_num-1)*3+2].beamNo    = target_data_ptr->beamNo;
        track_asso_data[(*track_asso_num-1)*3+2].Year     = target_data_ptr->Year;
        track_asso_data[(*track_asso_num-1)*3+2].Month    = target_data_ptr->Month;
        track_asso_data[(*track_asso_num-1)*3+2].Day      = target_data_ptr->Day;
        track_asso_data[(*track_asso_num-1)*3+2].Hour     = target_data_ptr->Hour;
        track_asso_data[(*track_asso_num-1)*3+2].Minute   = target_data_ptr->Minute;
        track_asso_data[(*track_asso_num-1)*3+2].Second   = target_data_ptr->Second;
        track_asso_data[(*track_asso_num-1)*3+2].mSecond  = target_data_ptr->mSecond;
        track_asso_data[(*track_asso_num-1)*3+2].asso_flag = 1;
    }
}

static void get_track_begin(struct target_track *current_dot,
                            int *track_asso_num,
                            struct temp_track *track_asso_data){
    int idx;
    if((*track_asso_num) < MAX_NEW_TRACKS_PER_FRAME){
        if(current_dot->range < track_debugger.range_down || current_dot->range > track_debugger.range_up){
            return;
        }
        if(current_dot->ele < track_debugger.ele_down || current_dot->ele > track_debugger.ele_up){
            return;
        }
        if(current_dot->azi < track_debugger.azi_down || current_dot->azi > track_debugger.azi_up){
            return;
        }
        (*track_asso_num) = (*track_asso_num) + 1;
        idx = (*track_asso_num) - 1;
        track_asso_data[idx*3+0].X[0] = current_dot->x;
        track_asso_data[idx*3+0].X[1] = 0;
        track_asso_data[idx*3+0].X[2] = current_dot->y;
        track_asso_data[idx*3+0].X[3] = 0;
        track_asso_data[idx*3+0].X[4] = current_dot->z;
        track_asso_data[idx*3+0].X[5] = 0;
        memcpy(track_asso_data[idx*3+0].P, zeros, sizeof(zeros));

        track_asso_data[idx*3+0].tgtnum    = current_dot->tgtnum;
        track_asso_data[idx*3+0].beamNo    = current_dot->beamNo;
        track_asso_data[idx*3+0].Year      = current_dot->Year;
        track_asso_data[idx*3+0].Month     = current_dot->Month;
        track_asso_data[idx*3+0].Day       = current_dot->Day;
        track_asso_data[idx*3+0].Hour      = current_dot->Hour;
        track_asso_data[idx*3+0].Minute    = current_dot->Minute;
        track_asso_data[idx*3+0].Second    = current_dot->Second;
        track_asso_data[idx*3+0].mSecond   = current_dot->mSecond;
        track_asso_data[idx*3+0].asso_flag = 1;
        track_asso_data[idx*3+1].asso_flag = 0;
        track_asso_data[idx*3+2].asso_flag = 0;
    }
}

void track_initial(
                struct target_track (*target_data),
                struct temp_track (*temp_track),
                int (*temp_track_num),
                struct temp_track (*track_asso_data),
                int (*track_asso_num)){

    int i,k;
    float pos1[3], pos2[3], pos3[3];
    float track_quality;
    float frame_mSecond;
    static int tt_used[num_temp];
    int count;
    int best_tt_idx;
    float best_tt_dist;
    int best_tt_count;
    float gate, gate_sq, dist_sq;
    float dist_maha;
    float dx, dy, dz, dx0, dy0, dz0;
    float dot_p, n1, n0, angle, ca;

    lag_time = 0;
    loop_of_dot = 0;
    frame_mSecond = 0.0f;
    
    *track_asso_num = 0;
    for(i = 0; i < num_temp; i++){
        tt_used[i] = 0;
        track_asso_data[i*3+0].asso_flag = 0;
        track_asso_data[i*3+1].asso_flag = 0;
        track_asso_data[i*3+2].asso_flag = 0;
    }
    
    for(loop_of_dot = 0; loop_of_dot < cpi_num; loop_of_dot++){
        if(target_data[loop_of_dot].Use_Flag_1 == 1){
            frame_mSecond = target_data[loop_of_dot].mSecond;
            break;
        }
    }
    if(frame_mSecond < 1.0f){
        frame_mSecond = target_data[0].mSecond;
    }
    
    k = 0;
    if((*temp_track_num) > 0 && (*temp_track_num) <= num_temp){
        for(loop_of_track = 0; loop_of_track < (*temp_track_num); loop_of_track++){
            count = 0;
            for(i = 0; i < 3; i++){
                if(temp_track[loop_of_track*3+i].asso_flag == 1){
                    count++;
                }
            }
            if(count == 0) continue;
            
            lag_time = frame_mSecond - temp_track[loop_of_track*3+count-1].mSecond;
            if(lag_time > time_up || lag_time < time_down){
                continue;
            }
            
            if(k != loop_of_track){
                temp_track[k*3+0] = temp_track[loop_of_track*3+0];
                temp_track[k*3+1] = temp_track[loop_of_track*3+1];
                temp_track[k*3+2] = temp_track[loop_of_track*3+2];
            }
            k++;
        }
        (*temp_track_num) = k;
    }
    
    for(loop_of_dot = 0; loop_of_dot < cpi_num; loop_of_dot++){
        if(target_data[loop_of_dot].Use_Flag_1 == 0) continue;
        
        best_tt_idx = -1;
        best_tt_dist = FLT_MAX;
        best_tt_count = 0;
        
        if((*temp_track_num) > 0){
            for(loop_of_track = 0; loop_of_track < (*temp_track_num); loop_of_track++){
                if(tt_used[loop_of_track]) continue;
                
                count = 0;
                for(i = 0; i < 3; i++){
                    if(temp_track[loop_of_track*3+i].asso_flag == 1){
                        count++;
                    }
                }
                
                lag_time = target_data[loop_of_dot].mSecond - temp_track[loop_of_track*3+count-1].mSecond;
                if(lag_time < time_down || lag_time > time_up) continue;
                
                T = lag_time / 1000.0f;
                matrix(&T);
                
                if(count == 1){
                    gate = 1500.0f * T + 150.0f;
                    gate_sq = gate * gate;
                    get_asso_info1_dist(&target_data[loop_of_dot], &temp_track[loop_of_track*3], &dist_sq);
                    if(dist_sq < gate_sq && dist_sq < best_tt_dist){
                        best_tt_dist = dist_sq;
                        best_tt_idx = loop_of_track;
                        best_tt_count = 1;
                    }
                }
                else if(count == 2){
                    get_asso_info2_dist(&target_data[loop_of_dot], &temp_track[0], loop_of_track, &dist_maha);
                    
                    dx = target_data[loop_of_dot].x - temp_track[loop_of_track*3+1].X[0];
                    dy = target_data[loop_of_dot].y - temp_track[loop_of_track*3+1].X[2];
                    dz = target_data[loop_of_dot].z - temp_track[loop_of_track*3+1].X[4];
                    dx0 = temp_track[loop_of_track*3+1].X[0] - temp_track[loop_of_track*3+0].X[0];
                    dy0 = temp_track[loop_of_track*3+1].X[2] - temp_track[loop_of_track*3+0].X[2];
                    dz0 = temp_track[loop_of_track*3+1].X[4] - temp_track[loop_of_track*3+0].X[4];
                    dot_p = dx*dx0 + dy*dy0 + dz*dz0;
                    n1 = sqrtf(dx*dx+dy*dy+dz*dz);
                    n0 = sqrtf(dx0*dx0+dy0*dy0+dz0*dz0);
                    angle = 180.0f;
                    if(n1 > 0.1f && n0 > 0.1f){
                        ca = dot_p / (n1*n0);
                        if(ca > 1.0f) ca = 1.0f;
                        if(ca < -1.0f) ca = -1.0f;
                        angle = acosf(ca) * 180.0f / pi;
                    }
                    
                    if(dist_maha < TRACK_INIT_TH && angle <= ANGLE_CHANGE_THRESHOLD && dist_maha < best_tt_dist){
                        best_tt_dist = dist_maha;
                        best_tt_idx = loop_of_track;
                        best_tt_count = 2;
                    }
                }
            }
        }
        
        if(best_tt_idx >= 0){
            tt_used[best_tt_idx] = 1;
            lag_time = target_data[loop_of_dot].mSecond - temp_track[best_tt_idx*3+best_tt_count-1].mSecond;
            T = lag_time / 1000.0f;
            matrix(&T);
            
            if(best_tt_count == 1){
                pos1[0] = temp_track[best_tt_idx*3+0].X[0];
                pos1[1] = temp_track[best_tt_idx*3+0].X[2];
                pos1[2] = temp_track[best_tt_idx*3+0].X[4];
                pos2[0] = target_data[loop_of_dot].x;
                pos2[1] = target_data[loop_of_dot].y;
                pos2[2] = target_data[loop_of_dot].z;
                track_quality = calculate_track_quality(pos1, pos2, NULL, T);
                if(track_quality >= VELOCITY_CONSISTENCY_THRESHOLD){
                    get_track_asso1(&target_data[loop_of_dot], &temp_track[0], best_tt_idx, &(*track_asso_num), &track_asso_data[0]);
                } else {
                    get_track_begin(&target_data[loop_of_dot], &(*track_asso_num), &track_asso_data[0]);
                }
            }
            else if(best_tt_count == 2){
                pos1[0] = temp_track[best_tt_idx*3+0].X[0];
                pos1[1] = temp_track[best_tt_idx*3+0].X[2];
                pos1[2] = temp_track[best_tt_idx*3+0].X[4];
                pos2[0] = temp_track[best_tt_idx*3+1].X[0];
                pos2[1] = temp_track[best_tt_idx*3+1].X[2];
                pos2[2] = temp_track[best_tt_idx*3+1].X[4];
                pos3[0] = target_data[loop_of_dot].x;
                pos3[1] = target_data[loop_of_dot].y;
                pos3[2] = target_data[loop_of_dot].z;
                track_quality = calculate_track_quality(pos1, pos2, pos3, T);
                if(track_quality >= TRACK_QUALITY_THRESHOLD){
                    get_track_asso2(&target_data[loop_of_dot], &temp_track[0], best_tt_idx, &(*track_asso_num), &track_asso_data[0]);
                } else {
                    get_track_begin(&target_data[loop_of_dot], &(*track_asso_num), &track_asso_data[0]);
                }
            }
        }
        else{
            get_track_begin(&target_data[loop_of_dot], &(*track_asso_num), &track_asso_data[0]);
        }
    }
}
