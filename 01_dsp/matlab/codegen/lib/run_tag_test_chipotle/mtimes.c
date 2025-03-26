/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mtimes.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <emmintrin.h>
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
  int scalarLB;
  A_data = A->data;
  mc_tmp = A->size[0];
  i = A->size[1];
  scalarLB = C->size[0];
  C->size[0] = A->size[0];
  emxEnsureCapacity_real_T(C, scalarLB);
  C_data = C->data;
  for (b_i = 0; b_i < mc_tmp; b_i++) {
    C_data[b_i] = 0.0;
  }
  for (k = 0; k < i; k++) {
    int vectorUB;
    scalarLB = (mc_tmp / 2) << 1;
    vectorUB = scalarLB - 2;
    for (b_i = 0; b_i <= vectorUB; b_i += 2) {
      __m128d r;
      r = _mm_loadu_pd(&C_data[b_i]);
      _mm_storeu_pd(&C_data[b_i],
                    _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&A_data[b_i]),
                                             _mm_set1_pd(B_data[0]))));
    }
    for (b_i = scalarLB; b_i < mc_tmp; b_i++) {
      C_data[b_i] += A_data[b_i] * B_data[0];
    }
  }
}

/*
 * File trailer for mtimes.c
 *
 * [EOF]
 */
