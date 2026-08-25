#include <stdio.h>
#include <math.h>
#include <string.h>
#include "kalman.h"
#include "matrix.h"
#include "global_variable.h"
#include "matrix_mul.h"
#include "matrix_process.h"
#include <DSPF_sp_lud/c66/DSPF_sp_lud.h>
#include <DSPF_sp_lud_inv/c66/DSPF_sp_lud_inv.h>

#pragma pack(4)

#define PI  3.141592653589793f
#define SIZE 9
#define MATRIX_SIZE 6
#define SOUND_SPEED_KALMAN 343.0f
#define AIR_DENSITY_SEA_KALMAN 1.225f
#define KALMAN_STEP_TIME 0.004f

static float kalman_calc_air_density(float altitude) {
    return AIR_DENSITY_SEA_KALMAN * expf(-altitude / 8500.0f);
}

static float kalman_calc_drag_coeff(float speed, float base_cd) {
    float mach;
    float cd;
    float t;
    mach = speed / SOUND_SPEED_KALMAN;
    if(mach < 0.8f){
        cd = base_cd * (0.95f + 0.2f * (mach - 0.3f));
    } else if(mach < 1.0f){
        t = (mach - 0.8f) / 0.2f;
        cd = base_cd * (1.05f + t * 0.75f);
    } else if(mach < 1.2f){
        t = (mach - 1.0f) / 0.2f;
        cd = base_cd * (1.8f - t * 0.4f);
    } else {
        cd = base_cd * (1.4f - 0.05f * (mach - 1.2f));
        if(cd < base_cd * 0.8f) cd = base_cd * 0.8f;
    }
    return cd;
}

static void compute_R_stable(float rho, float theta, float eps)
{
    float ct, st, ce, se;
    float sr2, sa2, sb2;
    float dxdr, dxdt, dxde, dydr, dydt, dyde, dzdr, dzde;
    ct = cosf(theta); st = sinf(theta);
    ce = cosf(eps);   se = sinf(eps);
    sr2 = sigma_r * sigma_r;
    sa2 = sigma_a * sigma_a;
    sb2 = sigma_b * sigma_b;

    dxdr = ct*ce;
    dxdt = -rho*st*ce;
    dxde = -rho*ct*se;
    dydr = st*ce;
    dydt =  rho*ct*ce;
    dyde = -rho*st*se;
    dzdr = se;
    dzde =  rho*ce;

    R[0][0] = dxdr*dxdr*sr2 + dxdt*dxdt*sa2 + dxde*dxde*sb2;
    R[0][1] = dxdr*dydr*sr2 + dxdt*dydt*sa2 + dxde*dyde*sb2;
    R[0][2] = dxdr*dzdr*sr2 + dxde*dzde*sb2;
    R[1][0] = R[0][1];
    R[1][1] = dydr*dydr*sr2 + dydt*dydt*sa2 + dyde*dyde*sb2;
    R[1][2] = dydr*dzdr*sr2 + dyde*dzde*sb2;
    R[2][0] = R[0][2];
    R[2][1] = R[1][2];
    R[2][2] = dzdr*dzdr*sr2 + dzde*dzde*sb2;
}

void ballistic_predict_state(float *X_in, float *P_in, float T_total, float *X_out, float *P_out) {
    int num_steps;
    float T_step;
    float X_cur[6];
    float P_cur[6][6];
    float sigma_a;
    float sa2;
    int i,j,n;
    float vx, vy, vz, alt_msl, vr, rho_air, cd_eff;
    float a_drag;
    float Ts2, Ts3, Ts4;
    float phi_step[6][6];
    float Q_step[6][6];
    float X_next[6];
    float phi_temp[6][6];
    float P_temp[6][6];
    float phi_trans[6][6];

    num_steps = (int)(T_total / KALMAN_STEP_TIME) + 1;
    if(num_steps > 50) num_steps = 50;
    if(num_steps < 1) num_steps = 1;
    T_step = T_total / num_steps;

    for(i = 0; i < 6; i++) X_cur[i] = X_in[i];
    for(i = 0; i < 6; i++){
        for(j = 0; j < 6; j++){
            P_cur[i][j] = P_in[i*6+j];
        }
    }

    sigma_a = 10.0f;
    sa2 = sigma_a * sigma_a;

    for(n = 0; n < num_steps; n++){
        vx = X_cur[1];
        vy = X_cur[3];
        vz = X_cur[5];
        alt_msl = X_cur[4] + alt_radar_msl;

        vr = sqrtf((vx - wx_vel)*(vx - wx_vel) +
                   (vy - wy_vel)*(vy - wy_vel) +
                   (vz - wz_vel)*(vz - wz_vel));

        rho_air = kalman_calc_air_density(alt_msl);
        cd_eff = kalman_calc_drag_coeff(vr, cx_resis);

        a_drag = rho_air * s_dan / 2.0f / m_dan * cd_eff * vr;

        for(i = 0; i < 6; i++){
            for(j = 0; j < 6; j++){
                phi_step[i][j] = 0.0f;
                Q_step[i][j] = 0.0f;
            }
        }
        phi_step[0][0] = 1.0f; phi_step[0][1] = T_step;
        phi_step[1][1] = 1.0f - a_drag * T_step;
        phi_step[2][2] = 1.0f; phi_step[2][3] = T_step;
        phi_step[3][3] = 1.0f - a_drag * T_step;
        phi_step[4][4] = 1.0f; phi_step[4][5] = T_step;
        phi_step[5][5] = 1.0f - a_drag * T_step;

        Ts2 = T_step * T_step;
        Ts3 = Ts2 * T_step / 2.0f;
        Ts4 = Ts2 * Ts2 / 4.0f;
        Q_step[0][0] = sa2 * Ts4;  Q_step[0][1] = sa2 * Ts3;
        Q_step[1][0] = sa2 * Ts3;  Q_step[1][1] = sa2 * Ts2;
        Q_step[2][2] = sa2 * Ts4;  Q_step[2][3] = sa2 * Ts3;
        Q_step[3][2] = sa2 * Ts3;  Q_step[3][3] = sa2 * Ts2;
        Q_step[4][4] = sa2 * Ts4;  Q_step[4][5] = sa2 * Ts3;
        Q_step[5][4] = sa2 * Ts3;  Q_step[5][5] = sa2 * Ts2;

        X_next[0] = X_cur[0] + T_step * vx;
        X_next[1] = (1.0f - a_drag * T_step) * vx + a_drag * T_step * wx_vel;
        X_next[2] = X_cur[2] + T_step * vy;
        X_next[3] = (1.0f - a_drag * T_step) * vy + a_drag * T_step * wy_vel;
        X_next[4] = X_cur[4] + T_step * vz;
        X_next[5] = (1.0f - a_drag * T_step) * vz - g * T_step + a_drag * T_step * wz_vel;

        DSP_float_mul_general((float*)phi_step, 6, 6, (float*)P_cur, 6, (float*)phi_temp);
        DSP_float_trans((float*)phi_step, 6, 6, (float*)phi_trans);
        DSP_float_mul_general((float*)phi_temp, 6, 6, (float*)phi_trans, 6, (float*)P_temp);
        DSP_float_add((float*)P_temp, (float*)Q_step, 6, 6, (float*)P_cur);

        memcpy(X_cur, X_next, sizeof(X_next));
    }

    for(i = 0; i < 6; i++) X_out[i] = X_cur[i];
    if(P_out != NULL){
        for(i = 0; i < 6; i++){
            for(j = 0; j < 6; j++){
                P_out[i*6+j] = P_cur[i][j];
            }
        }
    }
}


void cart2sph(float *x, float *y, float *z, float* rho, float* theta, float* eps)
{
    *rho = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    *theta = atan2f((*y), (*x));
    *eps = atan2f((*z) , sqrtf((*x) * (*x) + (*y) * (*y)));
}


float d_cal(float *Z, float *X, float *P)
{
	int i,j;
	float X_pred_state[6];
	float P_pred_state[6][6];
	float rho, theta, eps;
	static unsigned short Permu[3][3] = { {1,0,0},{0,1,0},{0,0,1} };
	float d_val;

	i = 0; j = 0;

	ballistic_predict_state(X, P, T, X_pred_state, (float*)P_pred_state);

	for(i = 0; i < 6; i++){
		X_predict[i][0] = X_pred_state[i];
	}
	for(i = 0; i < 6; i++){
		for(j = 0; j < 6; j++){
			P_predict[i][j] = P_pred_state[i][j];
		}
	}

	DSP_float_mul_general((float*)H, 3, 6, (float*)X_predict, 1, (float*)Z_predict);
	cart2sph(&Z[0], &Z[1], &Z[2], &rho, &theta, &eps);
	compute_R_stable(rho, theta, eps);

	DSP_float_mul_general((float*)H, 3, 6, (float*)P_predict, 6, (float*)HP);
	DSP_float_mul_general((float*)HP, 3, 6, (float*)H_trans, 3, (float*)HPH);
	DSP_float_add((float*)HPH, (float*)R, 3, 3, (float*)S);

	Z_D[0][0] = Z[0] - Z_predict[0][0];
	Z_D[1][0] = Z[1] - Z_predict[1][0];
	Z_D[2][0] = Z[2] - Z_predict[2][0];
	Z_D_trans[0][0] = Z_D[0][0];
	Z_D_trans[0][1] = Z_D[1][0];
	Z_D_trans[0][2] = Z_D[2][0];

	DSPF_sp_lud(3, (float*)S, (float*)S_L, (float*)S_U, (unsigned short*)Permu);
	DSPF_sp_lud_inverse(3, (unsigned short*)Permu, (float*)S_L, (float*)S_U, (float*)S_inv);

	DSP_float_mul03((float*)Z_D_trans, 1, 3, (float*)S_inv, 3, (float*)Z_D_trans_S_inv);
	d_val = Z_D_trans_S_inv[0][0] * Z_D[0][0] + Z_D_trans_S_inv[0][1] * Z_D[1][0] + Z_D_trans_S_inv[0][2] * Z_D[2][0];
	return d_val;
}

void kalman_filter_init(float *Z0, float *Z1, float *X, float *P)
{
	float rho, theta, eps;
	X[0] = Z1[0];
	X[1] = (Z1[0] - Z0[0]) / T;
	X[2] = Z1[1];
	X[3] = (Z1[1] - Z0[1]) / T;
	X[4] = Z1[2];
	X[5] = (Z1[2] - Z0[2]) / T;

	cart2sph(&Z1[0], &Z1[1], &Z1[2], &rho, &theta, &eps);
	compute_R_stable(rho, theta, eps);

	P[0*6+0] = R[0][0];
	P[0*6+1] = R[0][0] / T;
	P[0*6+2] = R[0][1];
	P[0*6+3] = R[0][1] / T;
	P[0*6+4] = R[0][2];
	P[0*6+5] = R[0][2] / T;

	P[1*6+0] = R[0][0] / T;
	P[1*6+1] = 2 * R[0][0] / powf(T, 2.0f);
	P[1*6+2] = R[0][1] / T;
	P[1*6+3] = 2 * R[0][1] / powf(T, 2.0f);
	P[1*6+4] = R[0][2] / T;
	P[1*6+5] = 2 * R[0][2] / powf(T, 2.0f);

	P[2*6+0] = R[0][1];
	P[2*6+1] = R[0][1] / T;
	P[2*6+2] = R[1][1];
	P[2*6+3] = R[1][1] / T;
	P[2*6+4] = R[1][2];
	P[2*6+5] = R[1][2] / T;

	P[3*6+0] = R[0][1] / T;
	P[3*6+1] = 2 * R[0][1] / powf(T, 2.0f);
	P[3*6+2] = R[1][1] / T;
	P[3*6+3] = 2 * R[1][1] / powf(T, 2.0f);
	P[3*6+4] = R[1][2] / T;
	P[3*6+5] = 2 * R[1][2] / powf(T, 2.0f);

	P[4*6+0] = R[0][2];
	P[4*6+1] = R[0][2] / T;
	P[4*6+2] = R[1][2];
	P[4*6+3] = R[1][2] / T;
	P[4*6+4] = R[2][2];
	P[4*6+5] = R[2][2] / T;

	P[5*6+0] = R[0][2] / T;
	P[5*6+1] = 2 * R[0][2] / powf(T, 2.0f);
	P[5*6+2] = R[1][2] / T;
	P[5*6+3] = 2 * R[1][2] / powf(T, 2.0f);
	P[5*6+4] = R[2][2] / T;
	P[5*6+5] = 2 * R[2][2] / powf(T, 2.0f);
}


void Extended_Kalman_Filter(float *X_track, float *P_track, float *Z_dot, float *X_filter, float *P_filter)
{
	int i,j;
	float X_pred_state[6];
	float P_pred_state[6][6];
	float rho, theta, eps;
	static unsigned short Permu[3][3] = { {1,0,0},{0,1,0},{0,0,1} };

	i = 0; j = 0;

	for(i = 0; i < 6; i++){
		X_text[i]=X_track[i];
	}

	for(i = 0; i < 3; i++){
		Z_text[i]=Z_dot[i];
	}

	ballistic_predict_state(X_track, P_track, T, X_pred_state, (float*)P_pred_state);

	for(i = 0; i < 6; i++){
		X_predict[i][0] = X_pred_state[i];
	}
	for(i = 0; i < 6; i++){
		for(j = 0; j < 6; j++){
			P_predict[i][j] = P_pred_state[i][j];
		}
	}

	DSP_float_mul_general((float*)H, 3, 6, (float*)X_predict, 1, (float*)Z_predict);
	cart2sph(&Z_dot[0], &Z_dot[1], &Z_dot[2], &rho, &theta, &eps);
	compute_R_stable(rho, theta, eps);

	DSP_float_mul_general((float*)H, 3, 6, (float*)P_predict, 6, (float*)HP);
	DSP_float_mul_general((float*)HP, 3, 6, (float*)H_trans, 3, (float*)HPH);
	DSP_float_add((float*)HPH, (float*)R, 3, 3, (float*)S);

	DSPF_sp_lud(3, (float*)S, (float*)S_L, (float*)S_U, (unsigned short*)Permu);
	DSPF_sp_lud_inverse(3, (unsigned short*)Permu, (float*)S_L, (float*)S_U, (float*)S_inv);

	DSP_float_mul_general((float*)P_predict, 6, 6, (float*)H_trans, 3, (float*)PH);
	DSP_float_mul_general((float*)PH, 6, 3, (float*)S_inv, 3, (float*)K);
	DSP_float_trans((float*)K, 6, 3, (float*)K_trans);

	Z_obser_unbias_D[0][0] = Z_dot[0] - Z_predict[0][0];
	Z_obser_unbias_D[1][0] = Z_dot[1] - Z_predict[1][0];
	Z_obser_unbias_D[2][0] = Z_dot[2] - Z_predict[2][0];

	DSP_float_mul_general((float*)K, 6, 3, (float*)Z_obser_unbias_D, 1, (float*)KZ_D);
	X_filter[0] = KZ_D[0][0] + X_predict[0][0];
	X_filter[1] = KZ_D[1][0] + X_predict[1][0];
	X_filter[2] = KZ_D[2][0] + X_predict[2][0];
	X_filter[3] = KZ_D[3][0] + X_predict[3][0];
	X_filter[4] = KZ_D[4][0] + X_predict[4][0];
	X_filter[5] = KZ_D[5][0] + X_predict[5][0];

	DSP_float_mul_general((float*)K, 6, 3, (float*)H, 6, (float*)KH);
	for(i = 0; i < 6; i++){
		I[i][i] = 1;
	}
	DSP_float_min((float*)I, (float*)KH, 6, 6, (float*)I_KH1);
	DSP_float_trans((float*)I_KH1, 6, 6, (float*)I_KH2);
	DSP_float_mul_general((float*)K, 6, 3, (float*)R, 3, (float*)KR);
	DSP_float_mul_general((float*)KR, 6, 3, (float*)K_trans, 6, (float*)KRK);
	DSP_float_mul_general((float*)I_KH1, 6, 6, (float*)P_predict, 6, (float*)IP);
	DSP_float_mul_general((float*)IP, 6, 6, (float*)I_KH2, 6, (float*)IPI);
	DSP_float_add((float*)IPI, (float*)KRK, 6, 6, (float*)P_filter);
}

void ballistic_ekf_update(float *X_in, float *P_in, float *Z_dot, float T_val,
                          float *X_out, float *P_out, const float *R_mat, float *Lambda_out)
{
	int i,j;
	float X_pred_state[6];
	float P_pred_state[6][6];
	float H_lcl[3][6];
	float HT_lcl[6][3];
	float Xp[6][1];
	float Pp[6][6];
	float Zp[3][1];
	float Hp[3][6];
	float HpH[3][3];
	float S_lcl[3][3];
	float S_L_lcl[3][3], S_U_lcl[3][3];
	float S_inv_lcl[3][3];
	unsigned short Permu[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	float PH_lcl[6][3];
	float K_lcl[6][3];
	float KT_lcl[3][6];
	float Z_nu[3][1];
	float KZ_lcl[6][1];
	float KH_lcl[6][6];
	float I_lcl[6][6];
	float I_KH1_lcl[6][6];
	float I_KH2_lcl[6][6];
	float KR_lcl[6][3];
	float KRK_lcl[6][6];
	float IP_lcl[6][6];
	float IPI_lcl[6][6];
	float Z_nu_t[1][3];
	float Z_nu_t_Sinv[1][3];
	float det_S;
	float nis;
	float denom;

	for(i = 0; i < 3; i++){
		for(j = 0; j < 6; j++){
			H_lcl[i][j] = 0.0f;
		}
	}
	H_lcl[0][0] = 1.0f;
	H_lcl[1][2] = 1.0f;
	H_lcl[2][4] = 1.0f;
	DSP_float_trans((float*)H_lcl, 3, 6, (float*)HT_lcl);

	ballistic_predict_state(X_in, P_in, T_val, X_pred_state, (float*)P_pred_state);

	for(i = 0; i < 6; i++){
		Xp[i][0] = X_pred_state[i];
	}
	for(i = 0; i < 6; i++){
		for(j = 0; j < 6; j++){
			Pp[i][j] = P_pred_state[i][j];
		}
	}

	DSP_float_mul_general((float*)H_lcl, 3, 6, (float*)Xp, 1, (float*)Zp);

	DSP_float_mul_general((float*)H_lcl, 3, 6, (float*)Pp, 6, (float*)Hp);
	DSP_float_mul_general((float*)Hp, 3, 6, (float*)HT_lcl, 3, (float*)HpH);
	for(i = 0; i < 3; i++){
		for(j = 0; j < 3; j++){
			S_lcl[i][j] = HpH[i][j] + R_mat[i*3+j];
		}
	}

	Z_nu[0][0] = Z_dot[0] - Zp[0][0];
	Z_nu[1][0] = Z_dot[1] - Zp[1][0];
	Z_nu[2][0] = Z_dot[2] - Zp[2][0];

	memcpy(S_L_lcl, S_lcl, sizeof(S_L_lcl));
	DSPF_sp_lud(3, (float*)S_L_lcl, (float*)S_L_lcl, (float*)S_U_lcl, (unsigned short*)Permu);
	DSPF_sp_lud_inverse(3, (unsigned short*)Permu, (float*)S_L_lcl, (float*)S_U_lcl, (float*)S_inv_lcl);

	Z_nu_t[0][0] = Z_nu[0][0]; Z_nu_t[0][1] = Z_nu[1][0]; Z_nu_t[0][2] = Z_nu[2][0];
	DSP_float_mul_general((float*)Z_nu_t, 1, 3, (float*)S_inv_lcl, 3, (float*)Z_nu_t_Sinv);
	nis = Z_nu_t_Sinv[0][0]*Z_nu[0][0] + Z_nu_t_Sinv[0][1]*Z_nu[1][0] + Z_nu_t_Sinv[0][2]*Z_nu[2][0];

	DSP_float_mul_general((float*)Pp, 6, 6, (float*)HT_lcl, 3, (float*)PH_lcl);
	DSP_float_mul_general((float*)PH_lcl, 6, 3, (float*)S_inv_lcl, 3, (float*)K_lcl);
	DSP_float_trans((float*)K_lcl, 6, 3, (float*)KT_lcl);

	DSP_float_mul_general((float*)K_lcl, 6, 3, (float*)Z_nu, 1, (float*)KZ_lcl);
	X_out[0] = KZ_lcl[0][0] + Xp[0][0];
	X_out[1] = KZ_lcl[1][0] + Xp[1][0];
	X_out[2] = KZ_lcl[2][0] + Xp[2][0];
	X_out[3] = KZ_lcl[3][0] + Xp[3][0];
	X_out[4] = KZ_lcl[4][0] + Xp[4][0];
	X_out[5] = KZ_lcl[5][0] + Xp[5][0];

	DSP_float_mul_general((float*)K_lcl, 6, 3, (float*)H_lcl, 6, (float*)KH_lcl);
	for(i = 0; i < 6; i++){
		for(j = 0; j < 6; j++){
			I_lcl[i][j] = (i == j) ? 1.0f : 0.0f;
		}
	}
	DSP_float_min((float*)I_lcl, (float*)KH_lcl, 6, 6, (float*)I_KH1_lcl);
	DSP_float_trans((float*)I_KH1_lcl, 6, 6, (float*)I_KH2_lcl);
	DSP_float_mul_general((float*)K_lcl, 6, 3, (float*)R_mat, 3, (float*)KR_lcl);
	DSP_float_mul_general((float*)KR_lcl, 6, 3, (float*)KT_lcl, 6, (float*)KRK_lcl);
	DSP_float_mul_general((float*)I_KH1_lcl, 6, 6, (float*)Pp, 6, (float*)IP_lcl);
	DSP_float_mul_general((float*)IP_lcl, 6, 6, (float*)I_KH2_lcl, 6, (float*)IPI_lcl);
	for(i = 0; i < 6; i++){
		for(j = 0; j < 6; j++){
			P_out[i*6+j] = IPI_lcl[i][j] + KRK_lcl[i][j];
		}
	}

	det_S = S_lcl[0][0]*(S_lcl[1][1]*S_lcl[2][2] - S_lcl[1][2]*S_lcl[2][1])
	      - S_lcl[0][1]*(S_lcl[1][0]*S_lcl[2][2] - S_lcl[1][2]*S_lcl[2][0])
	      + S_lcl[0][2]*(S_lcl[1][0]*S_lcl[2][1] - S_lcl[1][1]*S_lcl[2][0]);
	if(det_S < 0.0f) det_S = -det_S;
	if(det_S < 1e-20f) det_S = 1e-20f;
	denom = sqrtf(powf(2.0f*PI, 3.0f) * det_S);
	if(denom < 1e-20f) denom = 1e-20f;
	*Lambda_out = expf(-0.5f * nis) / denom;
	if(*Lambda_out < 1e-30f) *Lambda_out = 1e-30f;
}

void ballistic_ekf_update_only(float *X_pred_in, float *P_pred_in, float *Z_dot,
                               float *X_out, float *P_out, const float *R_mat, float *Lambda_out)
{
	int i,j;
	float H_lcl[3][6];
	float HT_lcl[6][3];
	float Xp[6][1];
	float Pp[6][6];
	float Zp[3][1];
	float Hp[3][6];
	float HpH[3][3];
	float S_lcl[3][3];
	float S_L_lcl[3][3], S_U_lcl[3][3];
	float S_inv_lcl[3][3];
	unsigned short Permu[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	float PH_lcl[6][3];
	float K_lcl[6][3];
	float KT_lcl[3][6];
	float Z_nu[3][1];
	float KZ_lcl[6][1];
	float KH_lcl[6][6];
	float I_lcl[6][6];
	float I_KH1_lcl[6][6];
	float I_KH2_lcl[6][6];
	float KR_lcl[6][3];
	float KRK_lcl[6][6];
	float IP_lcl[6][6];
	float IPI_lcl[6][6];
	float Z_nu_t[1][3];
	float Z_nu_t_Sinv[1][3];
	float det_S;
	float nis;
	float denom;

	for(i = 0; i < 3; i++){
		for(j = 0; j < 6; j++){
			H_lcl[i][j] = 0.0f;
		}
	}
	H_lcl[0][0] = 1.0f;
	H_lcl[1][2] = 1.0f;
	H_lcl[2][4] = 1.0f;
	DSP_float_trans((float*)H_lcl, 3, 6, (float*)HT_lcl);

	for(i = 0; i < 6; i++){
		Xp[i][0] = X_pred_in[i];
	}
	for(i = 0; i < 6; i++){
		for(j = 0; j < 6; j++){
			Pp[i][j] = P_pred_in[i*6+j];
		}
	}

	DSP_float_mul_general((float*)H_lcl, 3, 6, (float*)Xp, 1, (float*)Zp);

	DSP_float_mul_general((float*)H_lcl, 3, 6, (float*)Pp, 6, (float*)Hp);
	DSP_float_mul_general((float*)Hp, 3, 6, (float*)HT_lcl, 3, (float*)HpH);
	for(i = 0; i < 3; i++){
		for(j = 0; j < 3; j++){
			S_lcl[i][j] = HpH[i][j] + R_mat[i*3+j];
		}
	}

	Z_nu[0][0] = Z_dot[0] - Zp[0][0];
	Z_nu[1][0] = Z_dot[1] - Zp[1][0];
	Z_nu[2][0] = Z_dot[2] - Zp[2][0];

	memcpy(S_L_lcl, S_lcl, sizeof(S_L_lcl));
	DSPF_sp_lud(3, (float*)S_L_lcl, (float*)S_L_lcl, (float*)S_U_lcl, (unsigned short*)Permu);
	DSPF_sp_lud_inverse(3, (unsigned short*)Permu, (float*)S_L_lcl, (float*)S_U_lcl, (float*)S_inv_lcl);

	Z_nu_t[0][0] = Z_nu[0][0]; Z_nu_t[0][1] = Z_nu[1][0]; Z_nu_t[0][2] = Z_nu[2][0];
	DSP_float_mul_general((float*)Z_nu_t, 1, 3, (float*)S_inv_lcl, 3, (float*)Z_nu_t_Sinv);
	nis = Z_nu_t_Sinv[0][0]*Z_nu[0][0] + Z_nu_t_Sinv[0][1]*Z_nu[1][0] + Z_nu_t_Sinv[0][2]*Z_nu[2][0];

	DSP_float_mul_general((float*)Pp, 6, 6, (float*)HT_lcl, 3, (float*)PH_lcl);
	DSP_float_mul_general((float*)PH_lcl, 6, 3, (float*)S_inv_lcl, 3, (float*)K_lcl);
	DSP_float_trans((float*)K_lcl, 6, 3, (float*)KT_lcl);

	DSP_float_mul_general((float*)K_lcl, 6, 3, (float*)Z_nu, 1, (float*)KZ_lcl);
	X_out[0] = KZ_lcl[0][0] + Xp[0][0];
	X_out[1] = KZ_lcl[1][0] + Xp[1][0];
	X_out[2] = KZ_lcl[2][0] + Xp[2][0];
	X_out[3] = KZ_lcl[3][0] + Xp[3][0];
	X_out[4] = KZ_lcl[4][0] + Xp[4][0];
	X_out[5] = KZ_lcl[5][0] + Xp[5][0];

	DSP_float_mul_general((float*)K_lcl, 6, 3, (float*)H_lcl, 6, (float*)KH_lcl);
	for(i = 0; i < 6; i++){
		for(j = 0; j < 6; j++){
			I_lcl[i][j] = (i == j) ? 1.0f : 0.0f;
		}
	}
	DSP_float_min((float*)I_lcl, (float*)KH_lcl, 6, 6, (float*)I_KH1_lcl);
	DSP_float_trans((float*)I_KH1_lcl, 6, 6, (float*)I_KH2_lcl);

	DSP_float_mul_general((float*)K_lcl, 6, 3, (float*)R_mat, 3, (float*)KR_lcl);
	DSP_float_mul_general((float*)KR_lcl, 6, 3, (float*)KT_lcl, 6, (float*)KRK_lcl);
	DSP_float_mul_general((float*)I_KH1_lcl, 6, 6, (float*)Pp, 6, (float*)IP_lcl);
	DSP_float_mul_general((float*)IP_lcl, 6, 6, (float*)I_KH2_lcl, 6, (float*)IPI_lcl);
	for(i = 0; i < 6; i++){
		for(j = 0; j < 6; j++){
			P_out[i*6+j] = IPI_lcl[i][j] + KRK_lcl[i][j];
		}
	}

	det_S = S_lcl[0][0]*(S_lcl[1][1]*S_lcl[2][2] - S_lcl[1][2]*S_lcl[2][1])
	      - S_lcl[0][1]*(S_lcl[1][0]*S_lcl[2][2] - S_lcl[1][2]*S_lcl[2][0])
	      + S_lcl[0][2]*(S_lcl[1][0]*S_lcl[2][1] - S_lcl[1][1]*S_lcl[2][0]);
	if(det_S < 0.0f) det_S = -det_S;
	if(det_S < 1e-20f) det_S = 1e-20f;
	denom = sqrtf(powf(2.0f*PI, 3.0f) * det_S);
	if(denom < 1e-20f) denom = 1e-20f;
	*Lambda_out = expf(-0.5f * nis) / denom;
	if(*Lambda_out < 1e-30f) *Lambda_out = 1e-30f;
}
