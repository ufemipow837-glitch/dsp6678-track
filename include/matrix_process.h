#ifndef MATRIX_PROCESS_H_
#define MATRIX_PROCESS_H_

#pragma pack(4)

void DSP_float_trans(float *A, int rows, int cols, float *B);
void DSP_float_sum(float *A, int rows, int cols, float *sum);
void DSP_float_norm(float *A, int rows, int cols, float *norm);
void DSP_float_add(float *A, float *B, int rows, int cols, float *C);
void DSP_float_min(float* A, float* B, int rows, int cols, float* C);
void DSP_float_mul_general(float *A, int r1, int c1, float *B, int c2, float *C);

void DSP_float_add33(float *A, float *B, int rows, int cols, float *C);
void DSP_float_min33(float* A, float* B, int rows, int cols, float* C);

#endif /* MATRIX_PROCESS_H_ */
