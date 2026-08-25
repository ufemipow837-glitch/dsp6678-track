#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include "matrix.h"
#include "matrix_process.h"
#include "matrix_mul.h"

#pragma pack(4)

int TRACK_INIT_TH = 3;
float TRACK_ASSO_TH = 15.0f;
int START_THOLD = 3;


float DELAT_V_PREDICT[4] = {0};
float ASSO_USE[3] = {0};

//int beam1;
//int beam2;
float beam_time;

float time_down;
float time_up;

float azi_down;
float azi_up;
float ele_down;
float ele_up;
float range_down;
float range_up;

float s_dan;
float m_dan;
float p_air;

float cx_resis;
float cy_resis;
float cz_resis;
float wx_vel;
float wy_vel;
float wz_vel;

float N;
float Xr;
float Yr;
float Zr;
float s1;
float s2;
float c1;
float c2;
float P;
float inAz1;
float inEL1;
float inRo1;
//float inAz2;
//float inEL2;
//float inRo2;

//float x_data_1;
//float y_data_1;
//float z_data_1;
//float x_data_2;
//float y_data_2;
//float z_data_2;
float X_temp1[3][1];
float X_temp2[3][1];
float Tx_rot1[3][3];
float Ty_rot1[3][3];
float Tz_rot1[3][3];
float Tx_rot2[3][3];
float Ty_rot2[3][3];
float Tz_rot2[3][3];

float H_diff;
float alt_radar_msl = 0.0f;
float T_ASSO;
float D_ASSO;
float S_ASSO[3][3];
float P_ASSO[6][6];
float Z_D_ASSO[3][1];
float Z_D_S_ASSO[1][3];
int beng_check3_00;
int beng_check3_0;
int beng_check4_0;
int beng_check1_1;
int beng_check2_1;
int beng_check3_1;
int beng_check4_1;
int beng_check1_2;
int beng_check2_2;
int beng_check3_2;
int beng_check4_2;
int beng_check1_3;
int beng_check2_3;
int beng_check3_3;
int beng_check4_3;
int beng_check1_4;
int beng_check2_4;
int beng_check3_4;
int beng_check4_4;
int beng_check1_5;
int beng_check2_5;
int beng_check3_5;
int beng_check4_5;
int beng_check1_6;
int beng_check2_6;
int beng_check3_6;
int beng_check4_6;
int beng_check1_7;
int beng_check2_7;
int beng_check3_7;
int beng_check4_7;
int beng_check1_8;
int beng_check2_8;
int beng_check3_8;
int beng_check4_8;
int beng_check1_9;
int beng_check2_9;
int beng_check3_9;
int beng_check4_9;

float ttt;
int out_num;

float T ;
//float Vmax = 680.0;
float sigma_a = 0.3f / 180 * pi;
float sigma_b = 0.3f / 180 * pi;
float sigma_r = 5.0f;
//float alpha = 0.05;

float lambda_theta;
float lambda_theta1;
float lambda_eps;
float lambda_eps1;

float F[6][6];
float Q[3][3];
float G[6][3];
float H[3][6];
float GQG[6][6];

//float F1[6][6];
//float F2[6][1];
//float phiX[6][6];
//float p_trans[6][6];

//float F0[3][3];
//float F_singer[9][9];
//float F_singer_1[9][9];
//float Q0[3][3];
//float Q_singer[9][9];
//float H_singer[3][9];
float zeros[6][6] = {0};

//float F_singer_trans[9][9];
//float Q_singer_trans[9][9];
//float H_singer_trans[9][3];

float F_trans[6][6];
float G_trans[3][6];
float H_trans[6][3];

void crood_rot(float *ax,float *ay,float *az) {
	Tx_rot1[0][0] = 1;				Tx_rot1[0][1] = 0;				Tx_rot1[0][2] = 0;
	Tx_rot1[1][0] = 0;				Tx_rot1[1][1] = cosf(*ax);		Tx_rot1[1][2] = -sinf(*ax);
	Tx_rot1[2][0] = 0;				Tx_rot1[2][1] = sinf(*ax);		Tx_rot1[2][2] = cosf(*ax);

	Ty_rot1[0][0] = cosf(*ay);		Ty_rot1[0][1] = 0;				Ty_rot1[0][2] = sinf(*ay);
	Ty_rot1[1][0] = 0;				Ty_rot1[1][1] = 1;				Ty_rot1[1][2] = 0;
	Ty_rot1[2][0] = -sinf(*ay);		Ty_rot1[2][1] = 0;				Ty_rot1[2][2] = cosf(*ay);

	Tz_rot1[0][0] = cosf(*az);		Tz_rot1[0][1] = -sinf(*az);		Tz_rot1[0][2] = 0;
	Tz_rot1[1][0] = sinf(*az);		Tz_rot1[1][1] = cosf(*az);		Tz_rot1[1][2] = 0;
	Tz_rot1[2][0] = 0;				Tz_rot1[2][1] = 0;				Tz_rot1[2][2] = 1;

	Tx_rot2[0][0] = 1;				Tx_rot2[0][1] = 0;				Tx_rot2[0][2] = 0;
	Tx_rot2[1][0] = 0;				Tx_rot2[1][1] = cosf(-(*ax));	Tx_rot2[1][2] = -sinf(-(*ax));
	Tx_rot2[2][0] = 0;				Tx_rot2[2][1] = sinf(-(*ax));	Tx_rot2[2][2] = cosf(-(*ax));

	Ty_rot2[0][0] = cosf(-(*ay));	Ty_rot2[0][1] = 0;				Ty_rot2[0][2] = sinf(-(*ay));
	Ty_rot2[1][0] = 0;				Ty_rot2[1][1] = 1;				Ty_rot2[1][2] = 0;
	Ty_rot2[2][0] = -sinf(-(*ay));	Ty_rot2[2][1] = 0;				Ty_rot2[2][2] = cosf(-(*ay));

	Tz_rot2[0][0] = cosf(-(*az));	Tz_rot2[0][1] = -sinf(-(*az));	Tz_rot2[0][2] = 0;
	Tz_rot2[1][0] = sinf(-(*az));	Tz_rot2[1][1] = cosf(-(*az));	Tz_rot2[1][2] = 0;
	Tz_rot2[2][0] = 0;				Tz_rot2[2][1] = 0;				Tz_rot2[2][2] = 1;
}

void matrix(float *time) {
	int ii,jj;
	float sigma_a_local;
	float T_nominal;
	float T_ratio;
	float sa2;
	float T2;
	float T3;
	float T4;

	T = *time;

    F[0][0] = 1;F[0][1] = T;F[0][2] = 0;F[0][3] = 0;F[0][4] = 0;F[0][5] = 0;
    F[1][0] = 0;F[1][1] = 1;F[1][2] = 0;F[1][3] = 0;F[1][4] = 0;F[1][5] = 0;
	F[2][0] = 0;F[2][1] = 0;F[2][2] = 1;F[2][3] = T;F[2][4] = 0;F[2][5] = 0;
	F[3][0] = 0;F[3][1] = 0;F[3][2] = 0;F[3][3] = 1;F[3][4] = 0;F[3][5] = 0;
	F[4][0] = 0;F[4][1] = 0;F[4][2] = 0;F[4][3] = 0;F[4][4] = 1;F[4][5] = T;
	F[5][0] = 0;F[5][1] = 0;F[5][2] = 0;F[5][3] = 0;F[5][4] = 0;F[5][5] = 1;
	DSP_float_trans((float *)F, 6, 6, (float *)F_trans);

    H[0][0] = 1; H[0][1] = 0; H[0][2] = 0; H[0][3] = 0; H[0][4] = 0; H[0][5] = 0;
	H[1][0] = 0; H[1][1] = 0; H[1][2] = 1; H[1][3] = 0; H[1][4] = 0; H[1][5] = 0;
	H[2][0] = 0; H[2][1] = 0; H[2][2] = 0; H[2][3] = 0; H[2][4] = 1; H[2][5] = 0;
	DSP_float_trans((float *)H, 3, 6, (float *)H_trans);

    sigma_a_local = 10.0f;
    T_nominal = 0.04f;
    T_ratio = T / T_nominal;
    if(T_ratio < 0.5f) T_ratio = 0.5f;
    if(T_ratio > 5.0f) T_ratio = 5.0f;
    sa2 = sigma_a_local * sigma_a_local * T_ratio;

    T2 = T * T;
    T3 = T2 * T / 2.0f;
    T4 = T2 * T2 / 4.0f;

    for(ii = 0; ii < 6; ii++){
        for(jj = 0; jj < 6; jj++){
            GQG[ii][jj] = 0.0f;
        }
    }
    GQG[0][0] = sa2 * T4;  GQG[0][1] = sa2 * T3;
    GQG[1][0] = sa2 * T3;  GQG[1][1] = sa2 * T2;
    GQG[2][2] = sa2 * T4;  GQG[2][3] = sa2 * T3;
    GQG[3][2] = sa2 * T3;  GQG[3][3] = sa2 * T2;
    GQG[4][4] = sa2 * T4;  GQG[4][5] = sa2 * T3;
    GQG[5][4] = sa2 * T3;  GQG[5][5] = sa2 * T2;

}
