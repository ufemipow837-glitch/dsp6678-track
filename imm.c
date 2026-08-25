#include <math.h>
#include <string.h>
#include <float.h>
#include "imm.h"
#include "matrix.h"
#include "kalman.h"
#include "global_variable.h"
#include "matrix_mul.h"
#include "matrix_process.h"
#include <DSPF_sp_lud/c66/DSPF_sp_lud.h>
#include <DSPF_sp_lud_inv/c66/DSPF_sp_lud_inv.h>

#pragma pack(4)

#define IMM_PI 3.141592653589793f
#define IMM_DIM 6
#define IMM_OBS_DIM 3

static const float imm_pi[IMM_N][IMM_N] = {
    {0.90f, 0.08f, 0.02f},
    {0.05f, 0.85f, 0.10f},
    {0.05f, 0.25f, 0.70f}
};

static float imm_sigma_cv       = 5.0f;
static float imm_sigma_maneuver = 30.0f;

static float imm_mu_init_default[IMM_N] = {0.30f, 0.50f, 0.20f};
static float imm_mu_init_shell[IMM_N]   = {0.70f, 0.20f, 0.10f};
static float imm_mu_init_drone[IMM_N]   = {0.10f, 0.70f, 0.20f};

static float H_cv_local[IMM_OBS_DIM][IMM_DIM];
static float HT_cv_local[IMM_DIM][IMM_OBS_DIM];
static int imm_H_built = 0;

static void imm_build_H(void)
{
    int i, j;
    if(imm_H_built) return;
    for(i = 0; i < IMM_OBS_DIM; i++){
        for(j = 0; j < IMM_DIM; j++){
            H_cv_local[i][j] = 0.0f;
        }
    }
    H_cv_local[0][0] = 1.0f;
    H_cv_local[1][2] = 1.0f;
    H_cv_local[2][4] = 1.0f;

    for(i = 0; i < IMM_DIM; i++){
        for(j = 0; j < IMM_OBS_DIM; j++){
            HT_cv_local[i][j] = H_cv_local[j][i];
        }
    }
    imm_H_built = 1;
}

static void imm_build_cv_matrices(float T, float sigma_a, float Phi[IMM_DIM][IMM_DIM], float Q[IMM_DIM][IMM_DIM])
{
    int i, j;
    float T2 = T * T;
    float T3 = T2 * T / 2.0f;
    float T4 = T2 * T2 / 4.0f;
    float sa2 = sigma_a * sigma_a;

    for(i = 0; i < IMM_DIM; i++){
        for(j = 0; j < IMM_DIM; j++){
            Phi[i][j] = 0.0f;
            Q[i][j] = 0.0f;
        }
    }

    Phi[0][0] = 1.0f; Phi[0][1] = T;
    Phi[1][1] = 1.0f;
    Phi[2][2] = 1.0f; Phi[2][3] = T;
    Phi[3][3] = 1.0f;
    Phi[4][4] = 1.0f; Phi[4][5] = T;
    Phi[5][5] = 1.0f;

    Q[0][0] = sa2 * T4;  Q[0][1] = sa2 * T3;
    Q[1][0] = sa2 * T3;  Q[1][1] = sa2 * T2;
    Q[2][2] = sa2 * T4;  Q[2][3] = sa2 * T3;
    Q[3][2] = sa2 * T3;  Q[3][3] = sa2 * T2;
    Q[4][4] = sa2 * T4;  Q[4][5] = sa2 * T3;
    Q[5][4] = sa2 * T3;  Q[5][5] = sa2 * T2;
}

static void imm_cv_predict(const float *X_in, const float (*P_in)[IMM_DIM], float T,
                           float sigma_a, float *X_out, float (*P_out)[IMM_DIM])
{
    float Phi[IMM_DIM][IMM_DIM];
    float Qcv[IMM_DIM][IMM_DIM];
    float tmp1[IMM_DIM][IMM_DIM];
    float tmp2[IMM_DIM][IMM_DIM];
    float PhiT[IMM_DIM][IMM_DIM];
    int i, j;

    imm_build_cv_matrices(T, sigma_a, Phi, Qcv);

    X_out[0] = X_in[0] + T * X_in[1];
    X_out[1] = X_in[1];
    X_out[2] = X_in[2] + T * X_in[3];
    X_out[3] = X_in[3];
    X_out[4] = X_in[4] + T * X_in[5];
    X_out[5] = X_in[5];

    DSP_float_mul_general((float*)Phi, IMM_DIM, IMM_DIM, (float*)P_in, IMM_DIM, (float*)tmp1);
    DSP_float_trans((float*)Phi, IMM_DIM, IMM_DIM, (float*)PhiT);
    DSP_float_mul_general((float*)tmp1, IMM_DIM, IMM_DIM, (float*)PhiT, IMM_DIM, (float*)tmp2);

    for(i = 0; i < IMM_DIM; i++){
        for(j = 0; j < IMM_DIM; j++){
            P_out[i][j] = tmp2[i][j] + Qcv[i][j];
        }
    }
}

static float imm_kf_update(float *X_pred, float (*P_pred)[IMM_DIM],
                           const float *Z, const float *R_mat,
                           float *X_upd, float (*P_upd)[IMM_DIM])
{
    float HP[IMM_OBS_DIM][IMM_DIM];
    float HPH[IMM_OBS_DIM][IMM_OBS_DIM];
    float S_loc[IMM_OBS_DIM][IMM_OBS_DIM];
    float S_inv_loc[IMM_OBS_DIM][IMM_OBS_DIM];
    float S_L_loc[IMM_OBS_DIM][IMM_OBS_DIM];
    float S_U_loc[IMM_OBS_DIM][IMM_OBS_DIM];
    unsigned short Permu_loc[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    float PH[IMM_DIM][IMM_OBS_DIM];
    float K_loc[IMM_DIM][IMM_OBS_DIM];
    float KH_loc[IMM_DIM][IMM_DIM];
    float Imm_loc[IMM_DIM][IMM_DIM];
    float I_KH_loc[IMM_DIM][IMM_DIM];
    float IKHT_loc[IMM_DIM][IMM_DIM];
    float KR_loc[IMM_DIM][IMM_OBS_DIM];
    float KRK_loc[IMM_DIM][IMM_DIM];
    float tmpIP_loc[IMM_DIM][IMM_DIM];
    float tmpIPI_loc[IMM_DIM][IMM_DIM];
    float KT_loc[IMM_OBS_DIM][IMM_DIM];
    float z_pred[IMM_OBS_DIM];
    float nu[IMM_OBS_DIM];
    float det_S;
    float nis;
    float Lambda;
    float denom;
    int i, j, k;

    imm_build_H();

    z_pred[0] = X_pred[0];
    z_pred[1] = X_pred[2];
    z_pred[2] = X_pred[4];

    nu[0] = Z[0] - z_pred[0];
    nu[1] = Z[1] - z_pred[1];
    nu[2] = Z[2] - z_pred[2];

    DSP_float_mul_general((float*)H_cv_local, IMM_OBS_DIM, IMM_DIM, (float*)P_pred, IMM_DIM, (float*)HP);
    DSP_float_mul_general((float*)HP, IMM_OBS_DIM, IMM_DIM, (float*)HT_cv_local, IMM_OBS_DIM, (float*)HPH);

    for(i = 0; i < IMM_OBS_DIM; i++){
        for(j = 0; j < IMM_OBS_DIM; j++){
            S_loc[i][j] = HPH[i][j] + R_mat[i*3+j];
        }
    }

    for(i = 0; i < IMM_OBS_DIM; i++){
        for(j = 0; j < IMM_OBS_DIM; j++){
            S_L_loc[i][j] = S_loc[i][j];
        }
    }
    DSPF_sp_lud(3, (float*)S_L_loc, (float*)S_L_loc, (float*)S_U_loc, (unsigned short*)Permu_loc);
    DSPF_sp_lud_inverse(3, (unsigned short*)Permu_loc, (float*)S_L_loc, (float*)S_U_loc, (float*)S_inv_loc);

    det_S = S_loc[0][0]*(S_loc[1][1]*S_loc[2][2] - S_loc[1][2]*S_loc[2][1])
          - S_loc[0][1]*(S_loc[1][0]*S_loc[2][2] - S_loc[1][2]*S_loc[2][0])
          + S_loc[0][2]*(S_loc[1][0]*S_loc[2][1] - S_loc[1][1]*S_loc[2][0]);
    if(det_S < 0.0f) det_S = -det_S;

    DSP_float_mul_general((float*)P_pred, IMM_DIM, IMM_DIM, (float*)HT_cv_local, IMM_OBS_DIM, (float*)PH);
    DSP_float_mul_general((float*)PH, IMM_DIM, IMM_OBS_DIM, (float*)S_inv_loc, IMM_OBS_DIM, (float*)K_loc);

    X_upd[0] = X_pred[0] + K_loc[0][0]*nu[0] + K_loc[0][1]*nu[1] + K_loc[0][2]*nu[2];
    X_upd[1] = X_pred[1] + K_loc[1][0]*nu[0] + K_loc[1][1]*nu[1] + K_loc[1][2]*nu[2];
    X_upd[2] = X_pred[2] + K_loc[2][0]*nu[0] + K_loc[2][1]*nu[1] + K_loc[2][2]*nu[2];
    X_upd[3] = X_pred[3] + K_loc[3][0]*nu[0] + K_loc[3][1]*nu[1] + K_loc[3][2]*nu[2];
    X_upd[4] = X_pred[4] + K_loc[4][0]*nu[0] + K_loc[4][1]*nu[1] + K_loc[4][2]*nu[2];
    X_upd[5] = X_pred[5] + K_loc[5][0]*nu[0] + K_loc[5][1]*nu[1] + K_loc[5][2]*nu[2];

    DSP_float_mul_general((float*)K_loc, IMM_DIM, IMM_OBS_DIM, (float*)H_cv_local, IMM_DIM, (float*)KH_loc);
    for(i = 0; i < IMM_DIM; i++){
        for(j = 0; j < IMM_DIM; j++){
            Imm_loc[i][j] = (i == j) ? 1.0f : 0.0f;
            I_KH_loc[i][j] = Imm_loc[i][j] - KH_loc[i][j];
        }
    }

    DSP_float_trans((float*)I_KH_loc, IMM_DIM, IMM_DIM, (float*)IKHT_loc);

    DSP_float_mul_general((float*)I_KH_loc, IMM_DIM, IMM_DIM, (float*)P_pred, IMM_DIM, (float*)tmpIP_loc);
    DSP_float_mul_general((float*)tmpIP_loc, IMM_DIM, IMM_DIM, (float*)IKHT_loc, IMM_DIM, (float*)tmpIPI_loc);

    DSP_float_trans((float*)K_loc, IMM_DIM, IMM_OBS_DIM, (float*)KT_loc);
    for(i = 0; i < IMM_DIM; i++){
        for(j = 0; j < IMM_OBS_DIM; j++){
            KR_loc[i][j] = 0.0f;
            for(k = 0; k < IMM_OBS_DIM; k++){
                KR_loc[i][j] += K_loc[i][k] * R_mat[k*3+j];
            }
        }
    }
    DSP_float_mul_general((float*)KR_loc, IMM_DIM, IMM_OBS_DIM, (float*)KT_loc, IMM_DIM, (float*)KRK_loc);

    for(i = 0; i < IMM_DIM; i++){
        for(j = 0; j < IMM_DIM; j++){
            P_upd[i][j] = tmpIPI_loc[i][j] + KRK_loc[i][j];
        }
    }

    nis = nu[0]*(S_inv_loc[0][0]*nu[0] + S_inv_loc[0][1]*nu[1] + S_inv_loc[0][2]*nu[2])
        + nu[1]*(S_inv_loc[1][0]*nu[0] + S_inv_loc[1][1]*nu[1] + S_inv_loc[1][2]*nu[2])
        + nu[2]*(S_inv_loc[2][0]*nu[0] + S_inv_loc[2][1]*nu[1] + S_inv_loc[2][2]*nu[2]);

    if(det_S < 1e-20f) det_S = 1e-20f;
    denom = sqrtf(powf(2.0f*IMM_PI, 3.0f) * det_S);
    if(denom < 1e-20f) denom = 1e-20f;
    Lambda = expf(-0.5f * nis) / denom;
    if(Lambda < 1e-30f) Lambda = 1e-30f;

    return Lambda;
}

void imm_init(IMM_STATE *imm, const float *X0, const float (*P0)[IMM_STATE_DIM], int init_model_hint)
{
    int i, j, m;
    const float *mu_init;

    if(init_model_hint == 1) mu_init = imm_mu_init_shell;
    else if(init_model_hint == 2) mu_init = imm_mu_init_drone;
    else mu_init = imm_mu_init_default;

    for(m = 0; m < IMM_N; m++){
        for(i = 0; i < IMM_STATE_DIM; i++){
            imm->X[m][i] = X0[i];
        }
        for(i = 0; i < IMM_STATE_DIM; i++){
            for(j = 0; j < IMM_STATE_DIM; j++){
                imm->P[m][i][j] = P0[i][j];
            }
        }
        imm->mu[m] = mu_init[m];
        imm->mu_pred[m] = mu_init[m];
    }
    imm->prob_updated = 1.0f;
    imm->dominant_model = (init_model_hint == 1) ? MD_BALLISTIC : MD_CV;
}

void imm_predict(IMM_STATE *imm, float T, float *X_pred, float (*P_pred)[IMM_STATE_DIM])
{
    float c_j[IMM_N];
    float mu_ij[IMM_N][IMM_N];
    float X0j[IMM_N][IMM_STATE_DIM];
    float P0j[IMM_N][IMM_STATE_DIM][IMM_STATE_DIM];
    float dX[IMM_STATE_DIM];
    float outer[IMM_STATE_DIM][IMM_STATE_DIM];
    float Xm_pred[IMM_N][IMM_STATE_DIM];
    float Pm_pred[IMM_N][IMM_STATE_DIM][IMM_STATE_DIM];
    int i, j, k, m;

    for(j = 0; j < IMM_N; j++){
        c_j[j] = 0.0f;
        for(i = 0; i < IMM_N; i++){
            c_j[j] += imm_pi[i][j] * imm->mu[i];
        }
        if(c_j[j] < 1e-9f) c_j[j] = 1e-9f;
    }

    for(m = 0; m < IMM_N; m++){
        imm->mu_pred[m] = c_j[m];
    }

    for(j = 0; j < IMM_N; j++){
        for(i = 0; i < IMM_N; i++){
            mu_ij[i][j] = imm_pi[i][j] * imm->mu[i] / c_j[j];
        }
    }

    for(j = 0; j < IMM_N; j++){
        for(i = 0; i < IMM_STATE_DIM; i++){
            X0j[j][i] = 0.0f;
            for(k = 0; k < IMM_N; k++){
                X0j[j][i] += mu_ij[k][j] * imm->X[k][i];
            }
        }
        for(i = 0; i < IMM_STATE_DIM; i++){
            for(k = 0; k < IMM_STATE_DIM; k++){
                P0j[j][i][k] = 0.0f;
            }
        }
        for(m = 0; m < IMM_N; m++){
            for(i = 0; i < IMM_STATE_DIM; i++){
                dX[i] = imm->X[m][i] - X0j[j][i];
            }
            for(i = 0; i < IMM_STATE_DIM; i++){
                for(k = 0; k < IMM_STATE_DIM; k++){
                    outer[i][k] = imm->P[m][i][k] + dX[i]*dX[k];
                }
            }
            for(i = 0; i < IMM_STATE_DIM; i++){
                for(k = 0; k < IMM_STATE_DIM; k++){
                    P0j[j][i][k] += mu_ij[m][j] * outer[i][k];
                }
            }
        }
    }

    for(m = 0; m < IMM_N; m++){
        if(m == MD_BALLISTIC){
            ballistic_predict_state(X0j[m], (float*)P0j[m], T, Xm_pred[m], (float*)Pm_pred[m]);
        } else if(m == MD_CV){
            imm_cv_predict(X0j[m], (const float (*)[IMM_DIM])P0j[m], T, imm_sigma_cv, Xm_pred[m], Pm_pred[m]);
        } else {
            imm_cv_predict(X0j[m], (const float (*)[IMM_DIM])P0j[m], T, imm_sigma_maneuver, Xm_pred[m], Pm_pred[m]);
        }
    }

    for(m = 0; m < IMM_N; m++){
        for(i = 0; i < IMM_STATE_DIM; i++){
            imm->X[m][i] = Xm_pred[m][i];
        }
        for(i = 0; i < IMM_STATE_DIM; i++){
            for(j = 0; j < IMM_STATE_DIM; j++){
                imm->P[m][i][j] = Pm_pred[m][i][j];
            }
        }
    }

    if(X_pred != NULL && P_pred != NULL){
        for(i = 0; i < IMM_STATE_DIM; i++){
            X_pred[i] = 0.0f;
            for(m = 0; m < IMM_N; m++){
                X_pred[i] += c_j[m] * imm->X[m][i];
            }
        }
        for(i = 0; i < IMM_STATE_DIM; i++){
            for(j = 0; j < IMM_STATE_DIM; j++){
                P_pred[i][j] = 0.0f;
            }
        }
        for(m = 0; m < IMM_N; m++){
            for(i = 0; i < IMM_STATE_DIM; i++){
                dX[i] = imm->X[m][i] - X_pred[i];
            }
            for(i = 0; i < IMM_STATE_DIM; i++){
                for(j = 0; j < IMM_STATE_DIM; j++){
                    P_pred[i][j] += c_j[m] * (imm->P[m][i][j] + dX[i]*dX[j]);
                }
            }
        }
    }

    imm->prob_updated = 0.0f;
}

float imm_d_cal(IMM_STATE *imm, float T, const float *Z, const float *R_mat)
{
    float z_pred[3];
    float nu[3];
    float HP[3][6];
    float HPH[3][3];
    float S_loc[3][3];
    float S_inv_loc[3][3];
    float S_L_loc[3][3], S_U_loc[3][3];
    unsigned short Permu[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    float X_fused[6];
    float P_fused[6][6];
    float d;
    int i, j;

    (void)T;

    imm_get_fused_state(imm, X_fused, P_fused);

    z_pred[0] = X_fused[0];
    z_pred[1] = X_fused[2];
    z_pred[2] = X_fused[4];
    nu[0] = Z[0] - z_pred[0];
    nu[1] = Z[1] - z_pred[1];
    nu[2] = Z[2] - z_pred[2];

    imm_build_H();
    DSP_float_mul_general((float*)H_cv_local, 3, 6, (float*)P_fused, 6, (float*)HP);
    DSP_float_mul_general((float*)HP, 3, 6, (float*)HT_cv_local, 3, (float*)HPH);
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            S_loc[i][j] = HPH[i][j] + R_mat[i*3+j];
        }
    }

    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            S_L_loc[i][j] = S_loc[i][j];
        }
    }
    DSPF_sp_lud(3, (float*)S_L_loc, (float*)S_L_loc, (float*)S_U_loc, (unsigned short*)Permu);
    DSPF_sp_lud_inverse(3, (unsigned short*)Permu, (float*)S_L_loc, (float*)S_U_loc, (float*)S_inv_loc);

    d = nu[0]*(S_inv_loc[0][0]*nu[0] + S_inv_loc[0][1]*nu[1] + S_inv_loc[0][2]*nu[2])
      + nu[1]*(S_inv_loc[1][0]*nu[0] + S_inv_loc[1][1]*nu[1] + S_inv_loc[1][2]*nu[2])
      + nu[2]*(S_inv_loc[2][0]*nu[0] + S_inv_loc[2][1]*nu[1] + S_inv_loc[2][2]*nu[2]);
    return d;
}

void imm_update(IMM_STATE *imm, float T, const float *Z, const float *R_mat)
{
    float X_upd[IMM_N][IMM_STATE_DIM];
    float P_upd[IMM_N][IMM_STATE_DIM][IMM_STATE_DIM];
    float Lambda[IMM_N];
    float c_j_pred[IMM_N];
    float mu_new[IMM_N];
    float Lambda_sum;
    float mu_sum;
    int m, i, j, k2;
    int best;
    float best_mu;

    (void)T;

    Lambda_sum = 0.0f;
    for(m = 0; m < IMM_N; m++){
        c_j_pred[m] = 0.0f;
        for(k2 = 0; k2 < IMM_N; k2++){
            c_j_pred[m] += imm_pi[k2][m] * imm->mu[k2];
        }
        if(c_j_pred[m] < 1e-9f) c_j_pred[m] = 1e-9f;
    }

    for(m = 0; m < IMM_N; m++){
        if(m == MD_BALLISTIC){
            ballistic_ekf_update_only(imm->X[m], (float*)imm->P[m], (float*)Z, X_upd[m], (float*)P_upd[m], R_mat, &Lambda[m]);
        } else if(m == MD_CV){
            Lambda[m] = imm_kf_update(imm->X[m], imm->P[m], Z, R_mat, X_upd[m], P_upd[m]);
        } else {
            Lambda[m] = imm_kf_update(imm->X[m], imm->P[m], Z, R_mat, X_upd[m], P_upd[m]);
        }
    }

    for(m = 0; m < IMM_N; m++){
        Lambda_sum += Lambda[m] * c_j_pred[m];
    }
    if(Lambda_sum < 1e-30f) Lambda_sum = 1e-30f;

    for(m = 0; m < IMM_N; m++){
        mu_new[m] = Lambda[m] * c_j_pred[m] / Lambda_sum;
        if(mu_new[m] < 0.001f) mu_new[m] = 0.001f;
    }
    mu_sum = 0.0f;
    for(m = 0; m < IMM_N; m++) mu_sum += mu_new[m];
    for(m = 0; m < IMM_N; m++) mu_new[m] /= mu_sum;

    for(m = 0; m < IMM_N; m++){
        for(i = 0; i < IMM_STATE_DIM; i++){
            imm->X[m][i] = X_upd[m][i];
        }
        for(i = 0; i < IMM_STATE_DIM; i++){
            for(j = 0; j < IMM_STATE_DIM; j++){
                imm->P[m][i][j] = P_upd[m][i][j];
            }
        }
        imm->mu[m] = mu_new[m];
    }

    best = 0;
    best_mu = imm->mu[0];
    for(m = 1; m < IMM_N; m++){
        if(imm->mu[m] > best_mu){
            best_mu = imm->mu[m];
            best = m;
        }
    }
    imm->dominant_model = best;
    imm->prob_updated = 1.0f;
}

void imm_miss(IMM_STATE *imm, float T)
{
    float X_pred[IMM_STATE_DIM];
    float P_pred[IMM_STATE_DIM][IMM_STATE_DIM];
    float s;
    int m, best;
    float bm;

    imm_predict(imm, T, X_pred, P_pred);
    for(m = 0; m < IMM_N; m++){
        imm->mu[m] = imm->mu_pred[m];
    }
    s = 0.0f;
    for(m = 0; m < IMM_N; m++) s += imm->mu[m];
    for(m = 0; m < IMM_N; m++) imm->mu[m] /= s;

    best = 0;
    bm = imm->mu[0];
    for(m = 1; m < IMM_N; m++){
        if(imm->mu[m] > bm){
            bm = imm->mu[m];
            best = m;
        }
    }
    imm->dominant_model = best;
    imm->prob_updated = 0.0f;
}

void imm_get_fused_state(const IMM_STATE *imm, float *X_out, float (*P_out)[IMM_STATE_DIM])
{
    int i, j, m;
    float dX[IMM_STATE_DIM];
    const float *mu_w;

    if(imm->prob_updated < 0.5f){
        mu_w = imm->mu_pred;
    } else {
        mu_w = imm->mu;
    }

    for(i = 0; i < IMM_STATE_DIM; i++){
        X_out[i] = 0.0f;
        for(m = 0; m < IMM_N; m++){
            X_out[i] += mu_w[m] * imm->X[m][i];
        }
    }
    for(i = 0; i < IMM_STATE_DIM; i++){
        for(j = 0; j < IMM_STATE_DIM; j++){
            P_out[i][j] = 0.0f;
        }
    }
    for(m = 0; m < IMM_N; m++){
        for(i = 0; i < IMM_STATE_DIM; i++){
            dX[i] = imm->X[m][i] - X_out[i];
        }
        for(i = 0; i < IMM_STATE_DIM; i++){
            for(j = 0; j < IMM_STATE_DIM; j++){
                P_out[i][j] += mu_w[m] * (imm->P[m][i][j] + dX[i]*dX[j]);
            }
        }
    }
}

int imm_get_dominant_model(const IMM_STATE *imm)
{
    return imm->dominant_model;
}

void imm_get_mu(const IMM_STATE *imm, float mu[IMM_N])
{
    int m;
    for(m = 0; m < IMM_N; m++) mu[m] = imm->mu[m];
}
