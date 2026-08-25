#include <stdio.h>
#include <c6x.h>
#include <math.h>

#pragma pack(4)
/*
 * matrix_mul.c
 *
 *  Created on: 2023??1????
 *      Author: 22431
 */
void DSP_float_mul03(float *A, int r1, int c1, float *B,  int c2, float *C){
	int i,j,k;
	float sum1;
	for(k = 0; k < c2; k++){
		for(i = 0; i < r1; i++){
			sum1 = 0;
			for(j = 0; j < c1; j+=3){
				sum1 += A[i*c1 + j]*B[j*c2 + k] + A[i*c1 + (j+1)]*B[(j+1)*c2 + k] + A[i*c1 + (j+2)]*B[(j+2)*c2 + k];
			}
			C[i*c2 + k] = sum1;
		}
	}
}

void DSP_float_mul33(float *A, int r1, int c1, float *B,  int c2, float *C){
	int i,j,k;
	float sum1,sum2,sum3;
	for(k = 0; k < c2; k++){
		for(i = 0; i < r1; i+=3){
			sum1 = 0;
			sum2 = 0;
			sum3 = 0;
			for(j = 0; j < c1; j+=3){
				sum1 += A[i*c1 + j]*B[j*c2 + k] + A[i*c1 + (j+1)]*B[(j+1)*c2 + k] + A[i*c1 + (j+2)]*B[(j+2)*c2 + k];

				sum2 += A[(i+1)*c1 + j]*B[j*c2 + k] + A[(i+1)*c1 + (j+1)]*B[(j+1)*c2 + k] + A[(i+1)*c1 + (j+2)]*B[(j+2)*c2 + k];

				sum3 += A[(i+2)*c1 + j]*B[j*c2 + k] + A[(i+2)*c1 + (j+1)]*B[(j+1)*c2 + k] + A[(i+2)*c1 + (j+2)]*B[(j+2)*c2 + k];
			}
			C[i*c2 + k] = sum1;
			C[(i+1)*c2 + k] = sum2;
			C[(i+2)*c2 + k] = sum3;
		}
	}
}

void DSP_float_mul36(float *A, int r1, int c1, float *B,  int c2, float *C){
	int i,j,k;
	float sum1,sum2,sum3;
	for(k = 0; k < c2; k++){
		for(i = 0; i < r1; i+=3){
			sum1 = 0;
			sum2 = 0;
			sum3 = 0;
			for(j = 0; j < c1; j+=6){
				sum1 += A[i*c1 + j]*B[j*c2 + k] + A[i*c1 + (j+1)]*B[(j+1)*c2 + k] + A[i*c1 + (j+2)]*B[(j+2)*c2 + k]+
						A[i*c1 + (j+3)]*B[(j+3)*c2 + k] + A[i*c1 + (j+4)]*B[(j+4)*c2 + k] + A[i*c1 + (j+5)]*B[(j+5)*c2 + k];

				sum2 += A[(i+1)*c1 + j]*B[j*c2 + k] + A[(i+1)*c1 + (j+1)]*B[(j+1)*c2 + k] + A[(i+1)*c1 + (j+2)]*B[(j+2)*c2 + k]+
						A[(i+1)*c1 + (j+3)]*B[(j+3)*c2 + k] + A[(i+1)*c1 + (j+4)]*B[(j+4)*c2 + k] + A[(i+1)*c1 + (j+5)]*B[(j+5)*c2 + k];

				sum3 += A[(i+2)*c1 + j]*B[j*c2 + k] + A[(i+2)*c1 + (j+1)]*B[(j+1)*c2 + k] + A[(i+2)*c1 + (j+2)]*B[(j+2)*c2 + k]+
						A[(i+2)*c1 + (j+3)]*B[(j+3)*c2 + k] + A[(i+2)*c1 + (j+4)]*B[(j+4)*c2 + k] + A[(i+2)*c1 + (j+5)]*B[(j+5)*c2 + k];
			}
			C[i*c2 + k] = sum1;
			C[(i+1)*c2 + k] = sum2;
			C[(i+2)*c2 + k] = sum3;
		}
	}
}

