#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include "struct.h"
#include "matrix.h"
#include "global_variable.h"
#include "kalman.h"
#include "imm.h"
#include "matrix_mul.h"
#include "matrix_process.h"

#pragma pack(4)

void track_predict(struct target_track (*target_data),
                   struct reliable *reliable_point,
                   int (*reliable_track_num))
{
    int n, i, j;
    float lag_time_val;
    float T_total;
    float T_mat;
    float X_fused[6];
    float P_fused[6][6];
    
    if ((*reliable_track_num) <= 0) return;

    for (n = 0; n < (*reliable_track_num); n++) {
        lag_time_val = target_data[0].mSecond - reliable_point[n].mSecond;
        
        if (lag_time_val < 0.5f) continue;
        
        T_total = lag_time_val / 1000.0f;
        
        T_mat = T_total;
        matrix(&T_mat);

        imm_miss(&reliable_point[n].imm, T_total);
        imm_get_fused_state(&reliable_point[n].imm, X_fused, P_fused);

        memcpy(reliable_point[n].X1, X_fused, sizeof(X_fused));
        memcpy(reliable_point[n].P1, P_fused, sizeof(P_fused));
        memcpy(reliable_point[n].X0, X_fused, sizeof(X_fused));
        memcpy(reliable_point[n].P0, P_fused, sizeof(P_fused));
        
        reliable_point[n].Year      = target_data[0].Year;
        reliable_point[n].Month     = target_data[0].Month;
        reliable_point[n].Day       = target_data[0].Day;
        reliable_point[n].Hour      = target_data[0].Hour;
        reliable_point[n].Minute    = target_data[0].Minute;
        reliable_point[n].Second    = target_data[0].Second;
        reliable_point[n].mSecond   = target_data[0].mSecond;
        
        for(i = 0; i < 6; i++){
            for(j = 0; j < 6; j++){
                P_text[i][j] = reliable_point[n].P1[i][j];
            }
        }
    }
}
