#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "struct.h"
#include "track.h"
#include "matrix.h"
#include "global_variable.h"
#include "imm.h"

#pragma pack(4)

extern struct debugger_track track_debugger;

#define MIN_POINTS_FOR_RELIABLE 3
#define CONFIDENCE_THRESHOLD 0.9f
#define MAX_RELIABLE_TRACKS 1

static float calculate_confidence_for_track(struct temp_track *track0, struct temp_track *track1, struct temp_track *track2) {
    float T1, T2;
    float v1x, v1y, v1z, v2x, v2y, v2z;
    float v1_norm, v2_norm;
    float dot_prod, cos_theta;
    float angle_consistency, min_norm, max_norm, speed_ratio;
    float v1_valid, v2_valid;
    float confidence;
    float vx, vy, vz, spd;

    T1 = (track1->mSecond - track0->mSecond) / 1000.0f;
    T2 = (track2->mSecond - track1->mSecond) / 1000.0f;

    if(T1 < 0.001f || T2 < 0.001f) return 0.0f;

    v1x = (track1->X[0] - track0->X[0]) / T1;
    v1y = (track1->X[2] - track0->X[2]) / T1;
    v1z = (track1->X[4] - track0->X[4]) / T1;
    v2x = (track2->X[0] - track1->X[0]) / T2;
    v2y = (track2->X[2] - track1->X[2]) / T2;
    v2z = (track2->X[4] - track1->X[4]) / T2;

    v1_norm = sqrtf(v1x*v1x + v1y*v1y + v1z*v1z);
    v2_norm = sqrtf(v2x*v2x + v2y*v2y + v2z*v2z);

    if(v1_norm < 1.0f || v2_norm < 1.0f) return 0.0f;

    dot_prod = v1x*v2x + v1y*v2y + v1z*v2z;
    cos_theta = dot_prod / (v1_norm * v2_norm + FLT_EPSILON);
    if(cos_theta > 1.0f) cos_theta = 1.0f;
    if(cos_theta < -1.0f) cos_theta = -1.0f;

    angle_consistency = (1.0f + cos_theta) / 2.0f;
    min_norm = (v1_norm < v2_norm) ? v1_norm : v2_norm;
    max_norm = (v1_norm > v2_norm) ? v1_norm : v2_norm;
    speed_ratio = min_norm / (max_norm + FLT_EPSILON);

    v1_valid = (v1_norm >= track_debugger.Vmin && v1_norm <= track_debugger.Vmax) ? 1.0f : 0.3f;
    v2_valid = (v2_norm >= track_debugger.Vmin && v2_norm <= track_debugger.Vmax) ? 1.0f : 0.3f;

    confidence = (angle_consistency + speed_ratio + v1_valid + v2_valid) / 4.0f;

    vx = track2->X[1];
    vy = track2->X[3];
    vz = track2->X[5];
    spd = sqrtf(vx*vx + vy*vy + vz*vz);
    if(spd < track_debugger.Vmin || spd > track_debugger.Vmax) {
        confidence *= 0.5f;
    }

    return confidence;
}

void new_reliable(struct reliable (*reliable_track),
                  struct temp_track (*temp_track),
                  struct temp_track (*track_asso_data),
                  int (*reliable_track_num),
                  int (*reliable_num_all),
                  int (*temp_track_num),
                  int (*track_asso_num)){

    int i,j,count,k;
    int num_candidates;
    int best_idx;
    float best_confidence;
    float confidence;
    float vx, vy, vz, spd;
    int init_hint;
    int idx;

    k = 0;
    beng_check3_00 = 0;

    if((*track_asso_num) > 0 && (*track_asso_num) < num_temp){
        num_candidates = (*track_asso_num);

        best_idx = -1;
        best_confidence = CONFIDENCE_THRESHOLD - 0.01f;

        for(i = 0; i < num_candidates; i++){
            count = 0;
            for(j = 0; j < 3; j++){
                if(track_asso_data[i*3+j].asso_flag == 1){
                    count++;
                }
            }

            if(count >= MIN_POINTS_FOR_RELIABLE){
                confidence = calculate_confidence_for_track(
                    &track_asso_data[i*3+0],
                    &track_asso_data[i*3+1],
                    &track_asso_data[i*3+2]);

                if(confidence > best_confidence){
                    vx = track_asso_data[i*3+2].X[1];
                    vy = track_asso_data[i*3+2].X[3];
                    vz = track_asso_data[i*3+2].X[5];
                    spd = sqrtf(vx*vx + vy*vy + vz*vz);

                    if(spd >= track_debugger.Vmin && spd <= track_debugger.Vmax){
                        best_confidence = confidence;
                        best_idx = i;
                    }
                }
            }
        }

        if(best_idx >= 0 && (*reliable_track_num) < MAX_RELIABLE_TRACKS){
            (*reliable_track_num)++;
            (*reliable_num_all)++;
            idx = (*reliable_track_num) - 1;

            memcpy(reliable_track[idx].X0, track_asso_data[best_idx*3+2].X, sizeof(track_asso_data[best_idx*3+2].X));
            memcpy(reliable_track[idx].P0, track_asso_data[best_idx*3+2].P, sizeof(track_asso_data[best_idx*3+2].P));
            memcpy(reliable_track[idx].X1, track_asso_data[best_idx*3+2].X, sizeof(track_asso_data[best_idx*3+2].X));
            memcpy(reliable_track[idx].P1, track_asso_data[best_idx*3+2].P, sizeof(track_asso_data[best_idx*3+2].P));

            vx = track_asso_data[best_idx*3+2].X[1];
            vy = track_asso_data[best_idx*3+2].X[3];
            vz = track_asso_data[best_idx*3+2].X[5];
            spd = sqrtf(vx*vx + vy*vy + vz*vz);
            init_hint = 0;
            if(spd > 200.0f) init_hint = 1;
            else if(spd < 60.0f) init_hint = 2;
            imm_init(&reliable_track[idx].imm, track_asso_data[best_idx*3+2].X,
                     (const float (*)[6])track_asso_data[best_idx*3+2].P, init_hint);

            reliable_track[idx].Year      = track_asso_data[best_idx*3+2].Year;
            reliable_track[idx].Month     = track_asso_data[best_idx*3+2].Month;
            reliable_track[idx].Day       = track_asso_data[best_idx*3+2].Day;
            reliable_track[idx].Hour      = track_asso_data[best_idx*3+2].Hour;
            reliable_track[idx].Minute    = track_asso_data[best_idx*3+2].Minute;
            reliable_track[idx].Second    = track_asso_data[best_idx*3+2].Second;
            reliable_track[idx].mSecond   = track_asso_data[best_idx*3+2].mSecond;
            reliable_track[idx].tgtnum    = track_asso_data[best_idx*3+2].tgtnum;
            reliable_track[idx].num_P     = (*reliable_num_all);

            reliable_track[idx].track_update_flag = 1;
            reliable_track[idx].predict_flag = 0;

            track_asso_data[best_idx*3+0].asso_flag = 0;
            track_asso_data[best_idx*3+1].asso_flag = 0;
            track_asso_data[best_idx*3+2].asso_flag = 0;
        }

        for(i = 0; i < num_candidates; i++){
            if(track_asso_data[i*3+0].asso_flag == 1){
                if((*temp_track_num+k) < num_temp-1){
                    temp_track[(*temp_track_num+k)*3+0] = track_asso_data[i*3+0];
                    temp_track[(*temp_track_num+k)*3+1] = track_asso_data[i*3+1];
                    temp_track[(*temp_track_num+k)*3+2] = track_asso_data[i*3+2];
                    k++;
                }
                track_asso_data[i*3+0].asso_flag = 0;
                track_asso_data[i*3+1].asso_flag = 0;
                track_asso_data[i*3+2].asso_flag = 0;
            }
        }

        (*temp_track_num) += k;
        (*track_asso_num) = 0;
    }
}
