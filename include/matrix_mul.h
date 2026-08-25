/*
 * matrix_mul.h
 *
 *  Created on: 2023年11月3日
 *      Author: 22431
 */

#ifndef MATRIX_MUL_H_
#define MATRIX_MUL_H_

#pragma pack(4)

void DSP_float_mul03(float *A, int r1, int c1, float *B,  int c2, float *C);
void DSP_float_mul33(float *A, int r1, int c1, float *B,  int c2, float *C);
void DSP_float_mul36(float *A, int r1, int c1, float *B,  int c2, float *C);

#endif /* MATRIX_MUL_H_ */
