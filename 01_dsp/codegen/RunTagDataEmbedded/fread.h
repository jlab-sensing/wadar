/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fread.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

#ifndef FREAD_H
#define FREAD_H

/* Include Files */
#include "RunTagDataEmbedded_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int b_fread(double fileID, double A_data[]);

int c_fread(double fileID, double A_data[]);

int d_fread(double fileID, double A_data[]);

int e_fread(double fileID, double A_data[]);

void f_fread(double fileID, double sizeA, emxArray_real_T *A);

void g_fread(double fileID, double sizeA, emxArray_real_T *A);

double h_fread(double fileID, emxArray_real_T *A);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for fread.h
 *
 * [EOF]
 */
