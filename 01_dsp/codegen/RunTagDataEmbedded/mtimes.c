/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mtimes.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "mtimes.h"
#include "RunTagDataEmbedded_emxutil.h"
#include "RunTagDataEmbedded_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *A
 *                const double B_data[]
 *                emxArray_real_T *C
 * Return Type  : void
 */
void mtimes(const emxArray_real_T *A, const double B_data[], emxArray_real_T *C)
{
  const double *A_data;
  double *C_data;
  int b_i;
  int i;
  int k;
  int mc_tmp;
  A_data = A->data;
  mc_tmp = A->size[0];
  i = A->size[1];
  b_i = C->size[0];
  C->size[0] = A->size[0];
  emxEnsureCapacity_real_T(C, b_i);
  C_data = C->data;
  for (b_i = 0; b_i < mc_tmp; b_i++) {
    C_data[b_i] = 0.0;
  }
  for (k = 0; k < i; k++) {
    for (b_i = 0; b_i < mc_tmp; b_i++) {
      C_data[b_i] += A_data[b_i] * B_data[0];
    }
  }
}

/*
 * File trailer for mtimes.c
 *
 * [EOF]
 */
