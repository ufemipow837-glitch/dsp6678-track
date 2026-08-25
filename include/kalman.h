/*
 * kalman.h
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#pragma pack(4)
#define SIZE 9

void cart2sph(float *x, float *y, float *z, float* rho, float* theta, float* eps);

float d_cal(float *Z, float *X, float *P);

void kalman_filter_init(float *Z0, float *Z1, float *X, float *P);

void Extended_Kalman_Filter(float *X_track, float *P_track, float *Z_dot, float *X_filter, float *P_filter);

void ballistic_predict_state(float *X_in, float *P_in, float T_total, float *X_out, float *P_out);

void ballistic_ekf_update(float *X_in, float *P_in, float *Z_dot, float T,
                          float *X_out, float *P_out, const float *R_mat, float *Lambda_out);

void ballistic_ekf_update_only(float *X_pred_in, float *P_pred_in, float *Z_dot,
                               float *X_out, float *P_out, const float *R_mat, float *Lambda_out);

#endif /* KALMAN_H_ */
