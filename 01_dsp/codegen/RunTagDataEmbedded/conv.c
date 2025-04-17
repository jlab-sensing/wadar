/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: conv.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "conv.h"
#include "RunTagDataEmbedded_emxutil.h"
#include "RunTagDataEmbedded_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_creal_T *A
 *                emxArray_creal_T *C
 * Return Type  : void
 */
void conv(const emxArray_creal_T *A, emxArray_creal_T *C)
{
  static const double B[21] = {
      0.013468013468013469, 0.017258249579836595, 0.028257943196552896,
      0.045390367667586258, 0.066978481917091617, 0.090909090909090912,
      0.11483969990109019,  0.13642781415059554,  0.1535602386216289,
      0.16455993223834522,  0.16835016835016833,  0.16455993223834522,
      0.1535602386216289,   0.13642781415059554,  0.11483969990109019,
      0.090909090909090912, 0.066978481917091617, 0.045390367667586258,
      0.028257943196552896, 0.017258249579836595, 0.013468013468013469};
  const creal_T *A_data;
  creal_T *C_data;
  int b_k;
  int i;
  int k;
  int loop_ub;
  int nA;
  A_data = A->data;
  nA = A->size[0] - 12;
  loop_ub = A->size[0];
  i = C->size[0];
  C->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(C, i);
  C_data = C->data;
  for (i = 0; i < loop_ub; i++) {
    C_data[i].re = 0.0;
    C_data[i].im = 0.0;
  }
  if (A->size[0] > 0) {
    double d;
    for (k = 0; k < 10; k++) {
      i = (nA + k) + 1;
      for (b_k = 0; b_k <= i; b_k++) {
        loop_ub = (b_k - k) + 10;
        d = B[k];
        C_data[b_k].re += d * A_data[loop_ub].re;
        C_data[b_k].im += d * A_data[loop_ub].im;
      }
    }
    for (k = 0; k < 11; k++) {
      i = (nA - k) + 11;
      for (b_k = 0; b_k <= i; b_k++) {
        loop_ub = k + b_k;
        d = B[k + 10];
        C_data[loop_ub].re += d * A_data[b_k].re;
        C_data[loop_ub].im += d * A_data[b_k].im;
      }
    }
  }
}

/*
 * File trailer for conv.c
 *
 * [EOF]
 */
