#ifndef MATRIX_H_
#define MATRIX_H_

#pragma pack(4)

#pragma once

#define pi 3.141592653589793
#define g 9.8f
//#define TRACK_INIT_TH 3
//#define TRACK_ASSO_TH 10
//#define START_THOLD 3
#define num_temp 100
#define num_reliable 50
#define cpi_num 60
#define cpi_num2 120

extern int TRACK_INIT_TH;
extern float TRACK_ASSO_TH;
extern int START_THOLD;

extern float DELAT_V_PREDICT[4];
extern float ASSO_USE[3];

//extern int beam1;
//extern int beam2;
extern float beam_time;

extern float time_down;
extern float time_up;

extern float azi_down;
extern float azi_up;
extern float ele_down;
extern float ele_up;
extern float range_down;
extern float range_up;

extern float s_dan;
extern float m_dan;
extern float p_air;

extern float cx_resis;
extern float cy_resis;
extern float cz_resis;
extern float wx_vel;
extern float wy_vel;
extern float wz_vel;

extern float N;
extern float Xr;
extern float Yr;
extern float Zr;
extern float s1;
extern float s2;
extern float c1;
extern float c2;
extern float P;
extern float inAz1;
extern float inEL1;
extern float inRo1;
//extern float inAz2;
//extern float inEL2;
//extern float inRo2;

//extern float x_data_1;
//extern float y_data_1;
//extern float z_data_1;
//extern float x_data_2;
//extern float y_data_2;
//extern float z_data_2;

extern float X_temp1[3][1];
extern float X_temp2[3][1];
extern float Tx_rot1[3][3];
extern float Ty_rot1[3][3];
extern float Tz_rot1[3][3];
extern float Tx_rot2[3][3];
extern float Ty_rot2[3][3];
extern float Tz_rot2[3][3];

extern float H_diff;
extern float alt_radar_msl;

extern float T_ASSO;
extern float D_ASSO;
extern float S_ASSO[3][3];
extern float P_ASSO[6][6];
extern float Z_D_ASSO[3][1];
extern float Z_D_S_ASSO[1][3];
extern int beng_check3_0;
extern int beng_check4_0;
extern int beng_check3_00;

extern int beng_check1_1;
extern int beng_check2_1;
extern int beng_check3_1;
extern int beng_check4_1;
extern int beng_check1_2;
extern int beng_check2_2;
extern int beng_check3_2;
extern int beng_check4_2;
extern int beng_check1_3;
extern int beng_check2_3;
extern int beng_check3_3;
extern int beng_check4_3;
extern int beng_check1_4;
extern int beng_check2_4;
extern int beng_check3_4;
extern int beng_check4_4;
extern int beng_check1_5;
extern int beng_check2_5;
extern int beng_check3_5;
extern int beng_check4_5;
extern int beng_check1_6;
extern int beng_check2_6;
extern int beng_check3_6;
extern int beng_check4_6;
extern int beng_check1_7;
extern int beng_check2_7;
extern int beng_check3_7;
extern int beng_check4_7;
extern int beng_check1_8;
extern int beng_check2_8;
extern int beng_check3_8;
extern int beng_check4_8;
extern int beng_check1_9;
extern int beng_check2_9;
extern int beng_check3_9;
extern int beng_check4_9;

extern int beng_check;
extern float ttt;
extern int out_num;

extern float T;
//extern float Vmax;
extern float sigma_a;
extern float sigma_b;
extern float sigma_r;
extern float alpha;

extern float lambda_theta;
extern float lambda_theta1;
extern float lambda_eps;
extern float lambda_eps1;

extern float F[6][6];
extern float Q[3][3];
extern float G[6][3];
extern float H[3][6];
extern float GQG[6][6];

//extern float F1[6][6];
//extern float F2[6][1];
//extern float phiX[6][6];
//extern float p_trans[6][6];

//extern float F_singer[9][9];
//extern float Q_singer[9][9];
//extern float F_singer_1[9][9];
//extern float H_singer[3][9];
extern float zeros[6][6];

//extern float F_singer_trans[9][9];
//extern float Q_singer_trans[9][9];
//extern float H_singer_trans[9][3];

extern float F_trans[6][6];
extern float G_trans[3][6];
extern float H_trans[6][3];

void crood_rot(float *ax,float *ay,float *az);
void matrix(float *time);

#endif /* MATRIX_H_ */

