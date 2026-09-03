#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include <string.h>
#include "struct.h"
#include "matrix.h"
#include "matrix_mul.h"
#include "global_variable.h"
#include "kalman.h"
#include <DSPF_sp_lud/c66/DSPF_sp_lud.h>
#include <DSPF_sp_lud_inv/c66/DSPF_sp_lud_inv.h>

#pragma pack(4)
#define MAX_MISSING_FRAMES_SEARCH 160  // 增加容错帧数(覆盖多圈扫描周期)

static void convert_to_sph_and_check_boundary(struct reliable *rp, float *azi_out, float *ele_out, float *range_out, int *die)
{
    float x, y, z;
    float range_val;
    float azi, ele;
    float horiz;

    x = rp->X1[0];
    y = rp->X1[2];
    z = rp->X1[4];
    range_val = sqrtf(x*x + y*y + z*z);

    X_temp1[0][0] = x;
    X_temp1[1][0] = y;
    X_temp1[2][0] = z;

    DSP_float_mul33((float *)Tz_rot2, 3, 3, (float *)X_temp1, 1, (float *)X_temp2);
    DSP_float_mul33((float *)Tx_rot2, 3, 3, (float *)X_temp2, 1, (float *)X_temp1);
    DSP_float_mul33((float *)Ty_rot2, 3, 3, (float *)X_temp1, 1, (float *)X_temp2);

    azi = pi/2 - atan2f(X_temp2[1][0], X_temp2[0][0]);
    horiz = sqrtf(X_temp2[0][0]*X_temp2[0][0] + X_temp2[1][0]*X_temp2[1][0]);
    ele = atan2f(X_temp2[2][0], horiz);

    *azi_out = azi;
    *ele_out = ele;
    *range_out = range_val;

    *die = (azi > azi_up) || (azi < azi_down) ||
           (ele > ele_up) || (ele < ele_down) ||
           (range_val > range_up) || (range_val < range_down) ||
           (z < H_diff);
}

static void record_track_end(struct reliable *rp, struct TARGETTRACE_END *te, float azi, float ele, float range)
{
    te->tgtnum    = rp->tgtnum;
    te->traceType = rp->predict_flag;
    te->batchNum  = rp->num_P;
    te->Year      = rp->Year;
    te->Month     = rp->Month;
    te->Day       = rp->Day;
    te->Hour      = rp->Hour;
    te->Minute    = rp->Minute;
    te->Second    = rp->Second;
    te->mSecond   = rp->mSecond;
    te->X0   = rp->X0[0]; te->Vx0 = rp->X0[1];
    te->Y0   = rp->X0[2]; te->Vy0 = rp->X0[3];
    te->Z0   = rp->X0[4]; te->Vz0 = rp->X0[5];
    te->X1   = rp->X1[0]; te->Vx1 = rp->X1[1];
    te->Y1   = rp->X1[2]; te->Vy1 = rp->X1[3];
    te->Z1   = rp->X1[4]; te->Vz1 = rp->X1[5];
    te->range = range;
    te->azi   = azi * 180.0f/pi;
    te->ele   = ele * 180.0f/pi;
    te->Vel   = sqrtf(rp->X1[1]*rp->X1[1] + rp->X1[3]*rp->X1[3] + rp->X1[5]*rp->X1[5]);
    te->num_P = rp->num_P;
}

void track_die(struct reliable (*reliable_point), 
				struct TARGETTRACE_END (*track_end), 
				int (*reliable_track_num),
				struct SHOTPOINT (*shotpoint))
{
	int i, k1 = 0, write_idx = 0;
	float azi, ele, range_val;
	int die, die_by_boundary, die_by_predict;

	if (work_model == 1) {
		if ((*reliable_track_num) <= 0) return;

		for(i = 0; i < (*reliable_track_num); i++)
		{
                        convert_to_sph_and_check_boundary(&reliable_point[i], &azi, &ele, &range_val, &die);
                        die_by_predict = (reliable_point[i].predict_flag > MAX_MISSING_FRAMES_SEARCH);

                        if(die || die_by_predict){
                                record_track_end(&reliable_point[i], &track_end[k1], azi, ele, range_val);
                                k1++;
                        } else {
				if(write_idx != i){
					reliable_point[write_idx] = reliable_point[i];
				}
				write_idx++;
			}
		}

		(*reliable_track_num) = write_idx;

		for(i = 0; i < k1; i++){
			track_end[i].num = k1;
		}
	}
	else {
		for(i = 0; i < (*reliable_track_num); i++){
			convert_to_sph_and_check_boundary(&reliable_point[i], &azi, &ele, &range_val, &die_by_boundary);

			die_by_predict = (reliable_point[i].predict_flag > MAX_MISSING_FRAMES_SEARCH);

			if (die_by_predict || die_by_boundary) {
				record_track_end(&reliable_point[i], &track_end[k1], azi, ele, range_val);
				k1++;
			} else {
				if(write_idx != i){
					reliable_point[write_idx] = reliable_point[i];
				}
				write_idx++;
			}
		}

		(*reliable_track_num) = write_idx;

		for(i = 0; i < k1; i++){
			track_end[i].num = k1;
		}
	}
}
