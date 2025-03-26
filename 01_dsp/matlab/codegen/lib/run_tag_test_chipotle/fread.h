/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: fread.h
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

#ifndef FREAD_H
#define FREAD_H

/* Include Files */
#include "rtwtypes.h"
#include "run_tag_test_chipotle_types.h"
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
