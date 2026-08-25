#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include <string.h>
#include "struct.h"
#include "track.h"
#include "matrix.h"
#include "global_variable.h"
#include "imm.h"
#include "matrix_mul.h"

#pragma pack(4)

void tas_predict(struct MISSIONPARA *radar,
				struct reliable *reliable_point,
				int (*reliable_track_num),
				struct MANAGEPARA *par)
{
	if ((*reliable_track_num)>0)
	{
		int i = 0;
		for(i = 0; i < *reliable_track_num; i++){
			float T_total;
			IMM_STATE imm_tas;
			float X_fused[6];
			float P_fused[6][6];
			int j;

			(void)radar;

			T_total = beam_time;

			memcpy(&imm_tas, &reliable_point[i].imm, sizeof(IMM_STATE));
			imm_miss(&imm_tas, T_total);
			imm_get_fused_state(&imm_tas, X_fused, P_fused);

			for(j = 0; j < 6; j++){
				X_predict[j][0] = X_fused[j];
			}

			X_temp1[0][0] = X_predict[0][0];
			X_temp1[1][0] = X_predict[2][0];
			X_temp1[2][0] = X_predict[4][0];

			DSP_float_mul33((float *)Tz_rot2, 3, 3, (float *)X_temp1, 1, (float *)X_temp2);
			DSP_float_mul33((float *)Tx_rot2, 3, 3, (float *)X_temp2, 1, (float *)X_temp1);
			DSP_float_mul33((float *)Ty_rot2, 3, 3, (float *)X_temp1, 1, (float *)X_temp2);

			par[i].velocity = sqrtf(X_predict[1][0]*X_predict[1][0] + X_predict[3][0]*X_predict[3][0] + X_predict[5][0]*X_predict[5][0]);
			par[i].range = sqrtf(X_predict[0][0] * X_predict[0][0] + X_predict[2][0] * X_predict[2][0] + X_predict[4][0] * X_predict[4][0]);
			par[i].scanAz = (pi/2 - atan2f(X_temp2[1][0],X_temp2[0][0])) * 180.0f/pi;
			par[i].scanEl = atan2f(X_temp2[2][0] , sqrtf(X_temp2[0][0] * X_temp2[0][0] + X_temp2[1][0] * X_temp2[1][0])) * 180.0f/pi;

			par[i].num_P	= reliable_point[i].num_P;
		}
	}

}
