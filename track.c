#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include "dot_coh.h"
#include "track_asso.h"
#include "track_initial.h"
#include "track_predict.h"
#include "track_die.h"
#include "new_reliable.h"
#include "struct.h"
#include "matrix.h"
#include "matrix_mul.h"
#include "global_variable.h"
#include "tas_predict.h"
#include "stdlib.h"
#include <stdint.h>
#include <float.h>

#pragma pack(4)

#define TAS_WINDOW_FRAMES      17   /* TWS 一圈扫描约 17 帧 */
#define TAS_WINDOW_THRESHOLD   3    /* 窗口内累计 3 次 update 即认为航迹稳定 */
#define MAX_PREDICT_WITHOUT_UPDATE 5
#define TAS_CONFIDENCE_THRESHOLD 0.85f
#define MAX_SIMULTANEOUS_TRACKS 5
#define DEFAULT_TAS_SWITCH_RANGE 2500.0f   // TAS默认切换距离2500m

typedef enum {
    TRACK_STATE_SEARCHING = 0,
    TRACK_STATE_INITIALIZING = 1,
    TRACK_STATE_RELIABLE = 2,
    TRACK_STATE_TAS = 3
} TrackState;

float evaluate_track_confidence(struct reliable *track) {
    extern struct debugger_track track_debugger;
    float confidence = 0.5f;
    float speed;

    if(track->predict_flag == 0){
        confidence = 0.9f;
    } else if(track->predict_flag <= MAX_PREDICT_WITHOUT_UPDATE){
        confidence = 0.9f - 0.1f * track->predict_flag;
    } else {
        confidence = 0.3f;
    }

    speed = sqrtf(track->X1[1]*track->X1[1] + track->X1[3]*track->X1[3] + track->X1[5]*track->X1[5]);
    if(speed >= track_debugger.Vmin && speed <= track_debugger.Vmax){
        confidence *= 1.1f;
    } else {
        confidence *= 0.8f;
    }

    if(confidence > 1.0f) confidence = 1.0f;
    if(confidence < 0.0f) confidence = 0.0f;
    return confidence;
}

static struct target_track target_data_3[cpi_num];  /* 暂定定义, 完整初始化在 L246 */

int should_switch_to_tas(struct reliable *track, float tas_switch_range) {
    static int upd_in_window[num_reliable] = {0};
    static int last_update_framesn[num_reliable] = {-1};
    float confidence;
    float X, Vx, Y, Vy, Z, Vz;
    float range, Vr;
    float effective_range;
    int idx = track->num_P - 1;
    int cur_framesn = target_data_3[0].frameSn;

    /* ========== 1. 滞回锁已置位 → 直接保持TAS ========== */
    if(track->tas_latched) {
        /* 回退条件：目标重新远离(Vr>0) 且 距离 > 2倍门槛才解锁 */
        X  = track->X1[0];  Vx = track->X1[1];
        Y  = track->X1[2];  Vy = track->X1[3];
        Z  = track->X1[4];  Vz = track->X1[5];
        range = sqrtf(X*X + Y*Y + Z*Z);
        if(range > 1.0f) {
            Vr = (X*Vx + Y*Vy + Z*Vz) / range;
            if(range > tas_switch_range * 2.0f && Vr > 0.0f) {
                track->tas_latched = 0;
                upd_in_window[idx] = 0;
                last_update_framesn[idx] = -1;
                return 0;
            }
        }
        return 1;
    }

    /* ========== 2. 滑窗累计 update 次数 ========== */
    if(track->track_update_flag == 1) {
        if(cur_framesn - last_update_framesn[idx] > TAS_WINDOW_FRAMES) {
            upd_in_window[idx] = 1;
        } else {
            upd_in_window[idx]++;
        }
        last_update_framesn[idx] = cur_framesn;
    }

    /* ========== 3. 窗口门限 ========== */
    if(upd_in_window[idx] < TAS_WINDOW_THRESHOLD){
        printf("[TAS-DENY] upd_in_window=%d < %d\n", upd_in_window[idx], TAS_WINDOW_THRESHOLD);
        return 0;
    }

    /* ========== 4. 置信度门限 ========== */
    confidence = evaluate_track_confidence(track);
    if(confidence < TAS_CONFIDENCE_THRESHOLD){
        printf("[TAS-DENY] confidence=%.3f < 0.85\n", confidence);
        return 0;
    }

    /* ========== 5. 计算径向速度 ========== */
    X  = track->X1[0];  Vx = track->X1[1];
    Y  = track->X1[2];  Vy = track->X1[3];
    Z  = track->X1[4];  Vz = track->X1[5];
    range = sqrtf(X*X + Y*Y + Z*Z);
    if(range < 1.0f) return 0;

    /* 注意: 此 Vr 与 track_renew.Vel 符号相反
       Vr = + 表示目标远离 (range 增大)
       Vr = - 表示目标靠近 (range 减小) */
    Vr = (X*Vx + Y*Vy + Z*Vz) / range;
    effective_range = (tas_switch_range > 0.0f) ? tas_switch_range : DEFAULT_TAS_SWITCH_RANGE;

    /* ========== 6. 方向+距离联合决策 ========== */
    if(Vr > 0.0f) {
        /* Vr>0 = 目标远离：无距离门槛，窗口内 update 足够就切 TAS */
        track->tas_latched = 1;
        return 1;
    } else {
        /* Vr<0 = 目标靠近：需进入切换距离门槛内才转 TAS */
        if(range <= effective_range) {
            track->tas_latched = 1;
            return 1;
        }
        /* 靠近但还在门槛外 → 继续 TWS 跟踪 */
        return 0;
    }
}


//extern void  CSL_tscEnable(void);
//unsigned int StartTime = 0;
//unsigned int EndTime = 0;
//unsigned int Total_cycle_ticks = 0;
//extern cregister volatile unsigned int TSCL;
//
//uint32_t utilReadTime32()
//{
//    uint32_t timeVal;
//    timeVal = TSCL;
//    return timeVal;
//}
//
//#define a 6378137
//#define e2 0.006694

void track(
		struct debugger_track (*track_debugger),
        struct MISSIONPARA (*radar),
		struct TARGETPIONT_1 (*TARGETPIONT_track),
		struct temp_track (*temp_track),
		struct temp_track (*track_asso_data),
        struct reliable (*reliable_track),
		struct TARGETTRACE_RENEW (*track_renew),
		struct TARGETTRACE_END (*track_end),
		struct MANAGEPARA (*par),
		int (*reliable_track_num),
		struct SHOTPOINT (*shotpoint),
		int (*open_USEFLAG))
{
	int i,j = 0;

	if(*reliable_track_num > 0){
		for(i = 0; i < 6; i++){
			for(j = 0; j < 6; j++){
				P_text[i][j] = reliable_track[0].P1[i][j];
			}
		}
	}

	out_num = 0;

	s_dan		= (*track_debugger).d_dan * (*track_debugger).d_dan * pi / 4;
	m_dan		= (*track_debugger).m_dan;
	p_air		= (*track_debugger).p_air;

	cx_resis			= (*track_debugger).cx;
	cy_resis			= (*track_debugger).cy;
	cz_resis			= (*track_debugger).cz;
	wx_vel			= (*track_debugger).wx;
	wy_vel			= (*track_debugger).wy;
	wz_vel			= (*track_debugger).wz;
//	beam1 		= track_debugger.beam1;
//	beam2 		= track_debugger.beam2;
	beam_time 	= (*track_debugger).beam_time;
	time_down 	= (*track_debugger).time_down;
	time_up 	= (*track_debugger).time_up;
	azi_down 	= (*track_debugger).azi_down;
	azi_up 		= (*track_debugger).azi_up;
	ele_down 	= (*track_debugger).ele_down;
	ele_up 		= (*track_debugger).ele_up;
	range_down 	= (*track_debugger).range_down;
	range_up 	= (*track_debugger).range_up;

//	N = 0;
//	Xr = 0;
//	Yr = 0;
//	Zr = 0;
//	s1 = 0;
//	s2 = 0;
//	c1 = 0;
//	c2 = 0;
//	P = 0;
	H_diff = (*radar).altTarget - (*radar).altRadar;
	alt_radar_msl = (*radar).altRadar;
//	N = a/sqrt(1-e2*sin(radar.latRadar/180*pi)*sin(radar.latRadar/180*pi));
//	Xr = (N + radar.altRadar) * cos(radar.latRadar/180*pi) * cos(radar.lonRadar/180*pi);
//	Yr = (N + radar.altRadar) * cos(radar.latRadar/180*pi) * sin(radar.lonRadar/180*pi);
//	Zr = (N*(1-e2) + radar.altRadar) * sin(radar.latRadar/180*pi);
//	s1 = sin(radar.lonRadar/180*pi);
//	s2 = sin(radar.latRadar/180*pi);
//	c1 = cos(radar.lonRadar/180*pi);
//	c2 = cos(radar.latRadar/180*pi);
	inAz1 = -1*(*radar).initAz/180*pi;
	inEL1 =  (*radar).initEL/180*pi;
	inRo1 = -1*(*radar).initRo/180*pi;

//	inAz2 = radar.initAz/180*pi;
//	inEL2 = -radar.initEL/180*pi;
//	inRo2 = radar.initRo/180*pi;

	lambda_theta = expf(-sigma_a * sigma_a / 2);
	lambda_theta1 = expf(-2 * sigma_a * sigma_a);
	lambda_eps = expf(-sigma_b * sigma_b / 2);
	lambda_eps1 = expf(-2 * sigma_b * sigma_b);


	static int temp_track_num = 0;
	static int track_asso_num = 0;
    static int reliable_num_all = 0;
	static int tas_switch_flag[num_reliable] = {0};

	static struct target_track dot_data[2][cpi_num] = {0};
	static struct target_track target_data_3[cpi_num] = {0};
	static struct reliable tas_data_pre[num_reliable] = {0};
	static int k = 0;

	crood_rot(&inEL1,&inRo1,&inAz1);

	if(*open_USEFLAG == 0){
		temp_track_num = 0;
		track_asso_num = 0;
		reliable_num_all = 0;
		k = 0;
		{
			int ii, jj;
			for(ii = 0; ii < 2; ii++){
				for(jj = 0; jj < cpi_num; jj++){
					dot_data[ii][jj].Use_Flag_1 = 0;
				}
			}
		}
		for(i = 0; i < num_reliable; i++){
			tas_switch_flag[i] = 0;
		}
	}

//	double azii = 0;
//	double elee = 0;

	work_model = TARGETPIONT_track->workMode;

	beng_check3_00 = -1;
	beng_check3_0 = -1;
	beng_check4_0 = -1;
	beng_check1_1 = -1;
	beng_check2_1 = -1;
	beng_check3_1 = -1;
	beng_check4_1 = -1;
	beng_check1_2 = -1;
	beng_check2_2 = -1;
	beng_check3_2 = -1;
	beng_check4_2 = -1;
	beng_check1_3 = -1;
	beng_check2_3 = -1;
	beng_check3_3 = -1;
	beng_check4_3 = -1;
	beng_check1_4 = -1;
	beng_check2_4 = -1;
	beng_check3_4 = -1;
	beng_check4_4 = -1;
	beng_check1_5 = -1;
	beng_check2_5 = -1;
	beng_check3_5 = -1;
	beng_check4_5 = -1;
	beng_check1_6 = -1;
	beng_check2_6 = -1;
	beng_check3_6 = -1;
	beng_check4_6 = -1;
	beng_check1_7 = -1;
	beng_check2_7 = -1;
	beng_check3_7 = -1;
	beng_check4_7 = -1;
	beng_check1_8 = -1;
	beng_check2_8 = -1;
	beng_check3_8 = -1;
	beng_check4_8 = -1;
	beng_check1_9 = -1;
	beng_check2_9 = -1;
	beng_check3_9 = -1;
	beng_check4_9 = -1;

	for(i = 0; i < num_reliable; i++){
		track_end[i].num = 0;
	}

	if(TARGETPIONT_track->targetNum > cpi_num){
		out_num = 1;
	}

	if(work_model == 1){
		k = 1;
	}

	if(TARGETPIONT_track->targetNum == 0){
		dot_data[k][0].range	= 0;
		dot_data[k][0].azi		= 0;
		dot_data[k][0].ele		= 0;
		dot_data[k][0].velocity = 0;
		dot_data[k][0].x		= 0;
		dot_data[k][0].y		= 0;
		dot_data[k][0].z		= 0;
		for(i = 0; i < cpi_num; i++){
			dot_data[k][i].Use_Flag_1 = 0;
		}
		dot_data[k][0].beamNo	= TARGETPIONT_track->beamNo;
		dot_data[k][0].frameSn	= TARGETPIONT_track->frameSn;
		dot_data[k][0].Year		= TARGETPIONT_track->Year;
		dot_data[k][0].Month	= TARGETPIONT_track->Month;
		dot_data[k][0].Day		= TARGETPIONT_track->Day;
		dot_data[k][0].Hour		= TARGETPIONT_track->Hour;
		dot_data[k][0].Minute 	= TARGETPIONT_track->Minute;
		dot_data[k][0].Second 	= TARGETPIONT_track->Second;
		dot_data[k][0].mSecond	= TARGETPIONT_track->mSecond;
	}
	else{
		for(i = 0; i < cpi_num; i++){
			dot_data[k][i].Use_Flag_1 = 0;
		}
		for(i = 0; i < TARGETPIONT_track->targetNum; i++){
			dot_data[k][i].range	= TARGETPIONT_track->range[i];
			X_temp1[0][0] = TARGETPIONT_track->range[i]*cosf(1.5707963f - TARGETPIONT_track->azi[i])*cosf(TARGETPIONT_track->ele[i]);
			X_temp1[1][0] = TARGETPIONT_track->range[i]*sinf(1.5707963f - TARGETPIONT_track->azi[i])*cosf(TARGETPIONT_track->ele[i]);
			X_temp1[2][0] = TARGETPIONT_track->range[i]*sinf(TARGETPIONT_track->ele[i]);

			DSP_float_mul33((float *)Ty_rot1, 3, 3, (float *)X_temp1, 1, (float *)X_temp2);
			DSP_float_mul33((float *)Tx_rot1, 3, 3, (float *)X_temp2, 1, (float *)X_temp1);
			DSP_float_mul33((float *)Tz_rot1, 3, 3, (float *)X_temp1, 1, (float *)X_temp2);
			dot_data[k][i].x = X_temp2[0][0];
			dot_data[k][i].y = X_temp2[1][0];
			dot_data[k][i].z = X_temp2[2][0];
			dot_data[k][i].azi	= atan2f(dot_data[k][i].y,dot_data[k][i].x);
			dot_data[k][i].ele	= atan2f(dot_data[k][i].z , sqrtf(dot_data[k][i].x * dot_data[k][i].x + dot_data[k][i].y * dot_data[k][i].y));

			dot_data[k][i].velocity = TARGETPIONT_track->velocity[i];
			dot_data[k][i].Use_Flag_1 = 1;
			dot_data[k][i].tgtnum	= TARGETPIONT_track->tgtnum;
			dot_data[k][i].beamNo	= TARGETPIONT_track->beamNo;
			dot_data[k][i].frameSn	= TARGETPIONT_track->frameSn;
			dot_data[k][i].Year		= TARGETPIONT_track->Year;
			dot_data[k][i].Month	= TARGETPIONT_track->Month;
			dot_data[k][i].Day		= TARGETPIONT_track->Day;
			dot_data[k][i].Hour		= TARGETPIONT_track->Hour;
			dot_data[k][i].Minute 	= TARGETPIONT_track->Minute;
			dot_data[k][i].Second 	= TARGETPIONT_track->Second;
			dot_data[k][i].mSecond	= TARGETPIONT_track->mSecond;
		}
	}

	if((*reliable_track_num) > num_reliable){
		out_num = 2;
	}

	if(k == 1){
		for(i = 0; i < cpi_num; i++){
			target_data_3[i].Use_Flag_1 = 0;
		}

		dot_coh(&dot_data[0][0],
				&target_data_3[0]);

		target_data_3[0].frameSn = TARGETPIONT_track->frameSn;
		target_data_3[0].mSecond = TARGETPIONT_track->mSecond;

		//if(work_model == 1){
		if(1){

			track_asso(&target_data_3[0],
						&reliable_track[0],
						&(*reliable_track_num));
		}
//		beng_check1_1 = (*reliable_track_num);
//		beng_check2_1 = temp_track_num;
//		beng_check3_1 = target_data_3[0].frameSn;
//		beng_check4_1 = track_end[0].num;


		if(work_model == 0){
		//if(1){
			if(*reliable_track_num < MAX_SIMULTANEOUS_TRACKS){
				track_initial(	&target_data_3[0],
		  							&temp_track[0],
									&temp_track_num,
									&track_asso_data[0],
									&track_asso_num);

	//			beng_check1_2 = (*reliable_track_num);
	//			beng_check2_2 = temp_track_num;
				beng_check3_00 = target_data_3[0].frameSn;
	//			beng_check4_2 = track_end[0].num;

				new_reliable(&reliable_track[0],
							 &temp_track[0],
							 &track_asso_data[0],
							 &(*reliable_track_num),
							 &reliable_num_all,
							 &temp_track_num,
							 &track_asso_num);

	//			beng_check1_3 = (*reliable_track_num);
	//			beng_check2_3 = temp_track_num;
				beng_check2_4 = target_data_3[0].frameSn;
	//			beng_check4_3 = track_end[0].num;
			}
		}
		if(1){
			target_data_3[0].frameSn = TARGETPIONT_track->frameSn;
			target_data_3[0].mSecond = TARGETPIONT_track->mSecond;

			track_predict(&target_data_3[0],&reliable_track[0], &(*reliable_track_num));
		}

//		beng_check1_4 = (*reliable_track_num);
//		beng_check2_4 = temp_track_num;
//		beng_check3_4 = target_data_3[0].frameSn;
//		beng_check4_4 = track_end[0].num;

  		track_die(&reliable_track[0],
 		  		&track_end[0],
 		  		&(*reliable_track_num),
 				&shotpoint[0]);

		if((*reliable_track_num) > MAX_SIMULTANEOUS_TRACKS){
			int best_idx = 0;
			float best_quality = -1.0f;
			float quality;
			struct reliable temp_swap;
			for(i = 0; i < (*reliable_track_num); i++){
				quality = evaluate_track_confidence(&reliable_track[i]);
				if(reliable_track[i].track_update_flag == 1){
					quality += 0.5f;
				}
				if(quality > best_quality){
					best_quality = quality;
					best_idx = i;
				}
			}
			if(best_idx != 0){
				temp_swap = reliable_track[0];
				reliable_track[0] = reliable_track[best_idx];
				reliable_track[best_idx] = temp_swap;
			}
			(*reliable_track_num) = MAX_SIMULTANEOUS_TRACKS;
		}

		for (i = 0; i < (*reliable_track_num); i++) {
			tas_data_pre[i] = reliable_track[i];

			if(should_switch_to_tas(&reliable_track[i], (*track_debugger).tas_switch_range)){
				tas_switch_flag[i] = 1;
			}
			
			if(tas_switch_flag[i] && reliable_track[i].track_update_flag == 1){
				reliable_track[i].predict_flag = -1;
				tas_data_pre[i].predict_flag = -1;
			}
		}

		//  		beng_check1_6 = (*reliable_track_num);
		//  		beng_check2_6 = temp_track_num;
		//  		beng_check3_6 = target_data_3[0].frameSn;
		//  		beng_check4_6 = track_end[0].num;

		tas_predict(&(*radar),
					&tas_data_pre[0],
					&(*reliable_track_num),
					&par[0]);

		//		beng_check1_7 = (*reliable_track_num);
		//		beng_check2_7 = temp_track_num;
		//		beng_check3_7 = target_data_3[0].frameSn;
		//		beng_check4_7 = track_end[0].num;
		if ((*reliable_track_num) > 0) {
			for (j = 0; j < (*reliable_track_num); j++) {
				track_renew[j].tgtnum = reliable_track[j].tgtnum;

				track_renew[j].traceType = tas_switch_flag[j] ? 1 : 0;  /* 0=TWS 1=TAS */
				
				track_renew[j].batchNum = reliable_track[j].num_P;
				track_renew[j].Year = reliable_track[j].Year;
				track_renew[j].Month = reliable_track[j].Month;
				track_renew[j].Day = reliable_track[j].Day;
				track_renew[j].Hour = reliable_track[j].Hour;
				track_renew[j].Minute = reliable_track[j].Minute;
				track_renew[j].Second = reliable_track[j].Second;
				track_renew[j].mSecond = reliable_track[j].mSecond;

				track_renew[j].X = reliable_track[j].X1[0];
				track_renew[j].Vx = reliable_track[j].X1[1];
				track_renew[j].Y = reliable_track[j].X1[2];
				track_renew[j].Vy = reliable_track[j].X1[3];
				track_renew[j].Z = reliable_track[j].X1[4];
				track_renew[j].Vz = reliable_track[j].X1[5];

				X_temp1[0][0] = reliable_track[j].X1[0];
				X_temp1[1][0] = reliable_track[j].X1[2];
				X_temp1[2][0] = reliable_track[j].X1[4];

				DSP_float_mul33((float *)Tz_rot2, 3, 3, (float *)X_temp1, 1, (float *)X_temp2);
				DSP_float_mul33((float *)Tx_rot2, 3, 3, (float *)X_temp2, 1, (float *)X_temp1);
				DSP_float_mul33((float *)Ty_rot2, 3, 3, (float *)X_temp1, 1, (float *)X_temp2);

				track_renew[j].range = sqrtf(reliable_track[j].X1[0] * reliable_track[j].X1[0] + reliable_track[j].X1[2] * reliable_track[j].X1[2] + reliable_track[j].X1[4] * reliable_track[j].X1[4]);
				track_renew[j].azi = (3.14159265f/2.0f - atan2f(X_temp2[1][0],X_temp2[0][0])) * 180.0f/3.14159265f;
				track_renew[j].ele = atan2f(X_temp2[2][0] , sqrtf(X_temp2[0][0] * X_temp2[0][0] + X_temp2[1][0] * X_temp2[1][0])) * 180.0f/3.14159265f;
				{
				/* Vr = radial velocity WITH SIGN
				   drone AWAY    -> Vr negative (matches Vr=-11.7 m/s convention)
				   drone APPROACH -> Vr positive
				   Vr = -(vx*x + vy*y + vz*z) / R */
				float ix2 = reliable_track[j].X1[0];
				float iy2 = reliable_track[j].X1[2];
				float iz2 = reliable_track[j].X1[4];
				float iR2 = sqrtf(ix2*ix2 + iy2*iy2 + iz2*iz2);
				float ivx2 = reliable_track[j].X1[1];
				float ivy2 = reliable_track[j].X1[3];
				float ivz2 = reliable_track[j].X1[5];
				track_renew[j].Vel = -(ivx2*ix2 + ivy2*iy2 + ivz2*iz2) / (iR2 + 1e-6f);
			}
				
				track_renew[j].num_P = reliable_track[j].num_P;
				track_renew[j].num = (*reliable_track_num);
			}
		}
//		beng_check1_8 = (*reliable_track_num);
//		beng_check2_8 = temp_track_num;
//		beng_check3_8 = target_data_3[0].frameSn;
//		beng_check4_8 = track_end[0].num;
	}

	for(i = 0; i < cpi_num; i++){
		dot_data[0][i] = dot_data[k][i];

	}

//	beng_check1_9 = (*reliable_track_num);
//	beng_check2_9 = temp_track_num;
//	beng_check3_9 = target_data_3[0].frameSn;
//	beng_check4_9 = track_end[0].num;

	k = 1;
}
