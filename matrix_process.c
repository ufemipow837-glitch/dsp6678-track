#include <stdio.h>
#include <c6x.h>
#include <math.h>
#include "matrix_process.h"

#pragma pack(4)

void DSP_float_trans(float *A, int rows, int cols, float *B){
	int i,j;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			B[j*rows + i] = A[i*cols + j];
		}
	}
}

void DSP_float_sum(float *A, int rows, int cols, float *sum){
	int i,j;
	*sum = 0.0f;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			*sum = *sum + A[i*cols + j];
		}
	}
}

void DSP_float_norm(float *A, int rows, int cols, float *norm){
	int i,j;
	*norm = 0.0f;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			*norm = *norm + A[i*cols + j]*A[i*cols + j];
		}
	}
	*norm = sqrtf(*norm);
}

void DSP_float_add(float *A, float *B, int rows, int cols, float *C){
	int i,j;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			C[i*cols + j] = A[i*cols + j] + B[i*cols + j];
		}
	}
}

void DSP_float_min(float* A, float* B, int rows, int cols, float* C) {
	int i, j;
	for(i = 0; i < rows; i++){
		for(j = 0; j < cols; j++){
			C[i*cols + j] = A[i*cols + j] - B[i*cols + j];
		}
	}
}

void DSP_float_mul_general(float *A, int r1, int c1, float *B, int c2, float *C){
	int i, j, k;
	float sum;
	for(i = 0; i < r1; i++){
		for(j = 0; j < c2; j++){
			sum = 0.0f;
			for(k = 0; k < c1; k++){
				sum += A[i*c1 + k] * B[k*c2 + j];
			}
			C[i*c2 + j] = sum;
		}
	}
}

void DSP_float_add33(float *A, float *B, int rows, int cols, float *C){
	int i,j;
	for(i = 0; i < rows; i+=3){
		for(j = 0; j < cols; j+=3){
			C[i*cols + j] = A[i*cols + j] + B[i*cols + j];
			C[i*cols + j+1] = A[i*cols + j+1] + B[i*cols + j+1];
			C[i*cols + j+2] = A[i*cols + j+2] + B[i*cols + j+2];
			C[(i+1)*cols + j] = A[(i+1)*cols + j] + B[(i+1)*cols + j];
			C[(i+1)*cols + j+1] = A[(i+1)*cols + j+1] + B[(i+1)*cols + j+1];
			C[(i+1)*cols + j+2] = A[(i+1)*cols + j+2] + B[(i+1)*cols + j+2];
			C[(i+2)*cols + j] = A[(i+2)*cols + j] + B[(i+2)*cols + j];
			C[(i+2)*cols + j+1] = A[(i+2)*cols + j+1] + B[(i+2)*cols + j+1];
			C[(i+2)*cols + j+2] = A[(i+2)*cols + j+2] + B[(i+2)*cols + j+2];
		}
	}
}

void DSP_float_min33(float* A, float* B, int rows, int cols, float* C) {
	int i, j;
	for (i = 0; i < rows; i += 3) {
		for (j = 0; j < cols; j += 3) {
			C[i * cols + j] = A[i * cols + j] - B[i * cols + j];
			C[i * cols + j + 1] = A[i * cols + j + 1] - B[i * cols + j + 1];
			C[i * cols + j + 2] = A[i * cols + j + 2] - B[i * cols + j + 2];
			C[(i + 1) * cols + j] = A[(i + 1) * cols + j] - B[(i + 1) * cols + j];
			C[(i + 1) * cols + j + 1] = A[(i + 1) * cols + j + 1] - B[(i + 1) * cols + j + 1];
			C[(i + 1) * cols + j + 2] = A[(i + 1) * cols + j + 2] - B[(i + 1) * cols + j + 2];
			C[(i + 2) * cols + j] = A[(i + 2) * cols + j] - B[(i + 2) * cols + j];
			C[(i + 2) * cols + j + 1] = A[(i + 2) * cols + j + 1] - B[(i + 2) * cols + j + 1];
			C[(i + 2) * cols + j + 2] = A[(i + 2) * cols + j + 2] - B[(i + 2) * cols + j + 2];
		}
	}
}
