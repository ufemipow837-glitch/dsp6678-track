/*
 * global_variable.h
 *
 *  Created on: 2023年12月21日
 *      Author: 22431
 */

#ifndef INCLUDE_GLOBAL_VARIABLE_H_
#define INCLUDE_GLOBAL_VARIABLE_H_

#pragma once

#pragma pack(4)

#include "struct.h"
#include "matrix.h"

#define a 6378137
#define e2 0.006694

#define START_THOLD 3
// 注意：TRACK_ASSO_TH 已移至 matrix.h/c 中作为 float 变量定义，不要重复宏定义
// 注意：num_temp, num_reliable, cpi_num, g 已在 matrix.h 中定义，不要重复定义

extern float X_text[6];
extern float P_text[6][6];
extern float Z_text[3];

//initial
extern int work_model;			//
extern int loop_of_track;
extern int loop_of_dot;

extern float o;
extern float lag_time;

extern float d;
extern float alpha1;
extern int asso_info1[100];
extern int asso_info2[100];
extern int info_count1;
extern int info_count2;

extern float Z_obse[3];												    ///////////////////��ӡ
extern float X_filter[6];												///////////////////��ӡ
extern float P_filter[6][6];
extern float R[3][3];												    ///////////////////��ӡ
extern float range;
extern float azi;
extern float ele;
extern float X_last[6][1];
extern float P_last[6][6];
extern float X_predict[6][1];											///////////////////��ӡ
extern float P_predict[6][6];
extern float Z_predict[3][1];

extern float FP[6][6];
extern float FPF[6][6];
extern float S[3][3];												    ///////////////////��ӡ
extern float HP[3][6];
extern float HPH[3][3];
extern float S_inv[3][3];
extern float S_L[3][3];
extern float S_U[3][3];
//extern int P[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
extern float Z_D[3][1];
extern float Z_D_trans[1][3];
extern float Z_D_trans_S_inv[1][3];
extern float d1;
extern float D1[1][3];
extern float D2[1][3];
extern float sum_D1_2;
extern float norm_D1;
extern float norm_D2;

extern float Z0[3];
extern float Z1[3];
extern float Z2[3];
extern float X0[6];
extern float P0[6][6];

extern float norm;

//extern float v_k;
//extern float azi_v;
//extern float ele_v;
//extern float wx2;
//extern float wy2;
//extern float wz2;
extern float vr_xyz;
extern float angle_hang;
extern float a1;
extern float a2;
extern float a3;
extern float F1[6][6];													///////////////////////��ӡ
extern float F2[6][1];													///////////////////////��ӡ
extern float phiX[6][6];
extern float FX[6][1];													///////////////////////��ӡ
extern float p_trans[6][6];
extern float pP[6][6];
extern float pPp[6][6];


extern float K[6][3];												    ///////////////////��ӡ
extern float K_trans[3][6];
extern float PH[6][3];
extern float Z_obser_unbias_D[3][1];									///////////////////��ӡ
extern float KZ_D[6][1];
extern float KH[6][6];
extern float I[6][6];
extern float I_KH1[6][6];
extern float I_KH2[6][6];
extern float KR[6][3];
extern float KRK[6][6];
extern float IP[6][6];
extern float IPI[6][6];
//
//extern float v_k1;
//extern float vr01;
//extern float k01;
//extern float l01;
//extern float j01;
//extern float m01;
//extern float p01;
//extern float n01;
//extern float v_k2;
//extern float vr02;
//extern float k02;
//extern float l02;
//extern float j02;
//extern float m02;
//extern float p02;
//extern float n02;
//extern float v_k3;
//extern float vr03;
//extern float k03;
//extern float l03;
//extern float j03;
//extern float m03;
//extern float p03;
//extern float n03;
//extern float v_k4;
//extern float vr04;
//extern float k04;
//extern float l04;
//extern float j04;
//extern float m04;
//extern float p04;
//extern float n04;

/* ========================================================================
 * 核心航迹/点迹池全局变量（定义在 global_variable.c）
 * 外部只需包含此头文件即可使用，无需再重复定义
 * ======================================================================== */
extern struct reliable reliable_track[];
extern struct TARGETPIONT_1 target_data[];
extern struct temp_track temp_track[][3];
extern struct temp_track track_asso_data[][3];
extern struct MISSIONPARA radar;
extern struct TARGETPIONT_1 TARGETPIONT;
extern struct TARGETTRACE_RENEW track_renew[];
extern struct TARGETTRACE_END track_end[];
extern struct MANAGEPARA par[];
extern struct debugger_track track_debugger;
extern int reliable_track_num;
extern struct SHOTPOINT shotpoint[];
extern int open_USEFLAG;


#endif /* INCLUDE_GLOBAL_VARIABLE_H_ */
