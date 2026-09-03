#ifndef IMM_H_
#define IMM_H_

#pragma pack(4)

#define IMM_N 3

#define MD_BALLISTIC 0
#define MD_CV        1
#define MD_MANEUVER  2

#define IMM_STATE_DIM 6

typedef struct {
    float X[IMM_N][IMM_STATE_DIM];
    float P[IMM_N][IMM_STATE_DIM][IMM_STATE_DIM];
    float mu[IMM_N];
    float mu_pred[IMM_N];
    float prob_updated;
    int   dominant_model;
    int   is_drone;
} IMM_STATE;

void imm_init(IMM_STATE *imm, const float *X0, const float (*P0)[IMM_STATE_DIM], int init_model_hint);
void imm_predict(IMM_STATE *imm, float T, float *X_pred, float (*P_pred)[IMM_STATE_DIM]);
float imm_d_cal(IMM_STATE *imm, float T, const float *Z, const float *R_mat);
void imm_update(IMM_STATE *imm, float T, const float *Z, const float *R_mat);
void imm_miss(IMM_STATE *imm, float T);
void imm_get_fused_state(const IMM_STATE *imm, float *X_out, float (*P_out)[IMM_STATE_DIM]);
int  imm_get_dominant_model(const IMM_STATE *imm);
void imm_get_mu(const IMM_STATE *imm, float mu[IMM_N]);

#endif