#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include "matrix.h"
#include "matrix_process.h"
#include "matrix_mul.h"
#include "struct.h"
#include "global_variable.h"

#pragma pack(4)
/*
 * global_variable.c
 *
 *  Created on: 2023��12��21��
 *      Author: 22431
 */
float X_text[6] = {0};
float P_text[6][6] = {0};
float Z_text[3] = {0};


int work_model;

int loop_of_track;
int loop_of_dot;

float o;
float lag_time;

float d;
float alpha1;
int asso_info1[100] = {0};
int asso_info2[100] = {0};
int info_count1 = 0;
int info_count2 = 0;

float Z_obse[3] = {0};
float X_filter[6] = {0};
float P_filter[6][6] = {0};
float R[3][3] = {0};
float range;
float azi;
float ele;
float X_last[6][1] = {0};
float P_last[6][6] = {0};
float X_predict[6][1] = {0};
float P_predict[6][6] = {0};
float Z_predict[3][1] = {0};

float FP[6][6] = {0};
float FPF[6][6] = {0};
float S[3][3] = {0};
float HP[3][6] = {0};
float HPH[3][3] = {0};
float S_inv[3][3] = {0};
float S_L[3][3] = {0};
float S_U[3][3] = {0};
unsigned short Permu[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
float Z_D[3][1] = {0};
float Z_D_trans[1][3] = {0};
float Z_D_trans_S_inv[1][3] = {0};
float d1;
float D1[1][3] = {0};
float D2[1][3] = {0};
float sum_D1_2;
float norm_D1;
float norm_D2;

float Z0[3] = {0};
float Z1[3] = {0};
float Z2[3] = {0};
float X0[6] = {0};
float P0[6][6] = {0};

float norm;


//float v_k;
//float azi_v;
//float ele_v;
//float wx2;
//float wy2;
//float wz2;
float vr_xyz;
float angle_hang;
float a1;
float a2;
float a3;
float F1[6][6] = {0};
float F2[6][1] = {0};
float phiX[6][6] = {0};
float FX[6][1] = { 0 };
float p_trans[6][6] = { 0 };
float pP[6][6] = { 0 };
float pPp[6][6] = { 0 };

float K[6][3] = { 0 };
float K_trans[3][6] = { 0 };
float PH[6][3] = { 0 };
float Z_obser_unbias_D[3][1] = { 0 };
float KZ_D[6][1] = { 0 };
float KH[6][6] = { 0 };
float I[6][6] = {0};
float I_KH1[6][6] = { 0 };
float I_KH2[6][6] = { 0 };
float KR[6][3] = { 0 };
float KRK[6][6] = { 0 };
float IP[6][6] = { 0 };
float IPI[6][6] = { 0 };

//float v_k1 = 0;
//float vr01 = 0;
//float k01 = 0;
//float l01 = 0;
//float j01 = 0;
//float m01 = 0;
//float p01 = 0;
//float n01 = 0;
//
//float v_k2 = 0;
//float vr02 = 0;
//float k02 = 0;
//float l02 = 0;
//float j02 = 0;
//float m02 = 0;
//float p02 = 0;
//float n02 = 0;
//
//float v_k3 = 0;
//float vr03 = 0;
//float k03 = 0;
//float l03 = 0;
//float j03 = 0;
//float m03 = 0;
//float p03 = 0;
//float n03 = 0;
//
//float v_k4 = 0;
//float vr04 = 0;
//float k04 = 0;
//float l04 = 0;
//float j04 = 0;
//float m04 = 0;
//float p04 = 0;
//float n04 = 0;

/* ========================================================================
 * 核心航迹/点迹池全局变量定义（统一放在此处，方便集成到实物工程）
 * 编译此文件后，外部无需再重复定义这些变量，直接包含头文件即可使用
 * ======================================================================== */

#pragma DATA_SECTION(reliable_track, ".far:DDR");
struct reliable reliable_track[num_reliable] = {0};

#pragma DATA_SECTION(target_data, ".far:DDR");
struct TARGETPIONT_1 target_data[100] = {0};

#pragma DATA_SECTION(temp_track, ".far:DDR");
struct temp_track temp_track[num_temp][3] = {0};

#pragma DATA_SECTION(track_asso_data, ".far:DDR");
struct temp_track track_asso_data[num_temp][3] = {0};

struct MISSIONPARA radar = {0};
struct TARGETPIONT_1 TARGETPIONT = {0};

#pragma DATA_SECTION(track_renew, ".far:DDR");
struct TARGETTRACE_RENEW track_renew[num_reliable] = {0};

#pragma DATA_SECTION(track_end, ".far:DDR");
struct TARGETTRACE_END track_end[num_reliable] = {0};

#pragma DATA_SECTION(par, ".far:DDR");
struct MANAGEPARA par[num_reliable] = {0};

struct debugger_track track_debugger = {0};
int reliable_track_num = 0;

#pragma DATA_SECTION(shotpoint, ".far:DDR");
struct SHOTPOINT shotpoint[num_reliable] = {0};

int open_USEFLAG = 1;
