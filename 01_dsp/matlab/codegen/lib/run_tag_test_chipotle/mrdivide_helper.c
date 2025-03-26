/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mrdivide_helper.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *A
 *                const double B_data[]
 *                int B_size
 *                emxArray_real_T *Y
 * Return Type  : void
 */
void mrdiv(const emxArray_real_T *A, const double B_data[], int B_size,
           emxArray_real_T *Y)
{
  emxArray_real_T *b_Y;
  const double *A_data;
  double *Y_data;
  double *b_Y_data;
  int i;
  A_data = A->data;
  emxInit_real_T(&b_Y, 1);
  if ((A->size[0] == 0) || (B_size == 0)) {
    int loop_ub;
    i = Y->size[0] * Y->size[1];
    Y->size[0] = A->size[0];
    Y->size[1] = B_size;
    emxEnsureCapacity_real_T(Y, i);
    Y_data = Y->data;
    loop_ub = A->size[0] * B_size;
    for (i = 0; i < loop_ub; i++) {
      Y_data[i] = 0.0;
    }
  } else {
    double B;
    int loop_ub;
    int scalarLB;
    int vectorUB;
    B = B_data[0];
    loop_ub = A->size[0];
    i = b_Y->size[0];
    b_Y->size[0] = A->size[0];
    emxEnsureCapacity_real_T(b_Y, i);
    b_Y_data = b_Y->data;
    scalarLB = (A->size[0] / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      _mm_storeu_pd(&b_Y_data[i],
                    _mm_div_pd(_mm_loadu_pd(&A_data[i]), _mm_set1_pd(B)));
    }
    for (i = scalarLB; i < loop_ub; i++) {
      b_Y_data[i] = A_data[i] / B;
    }
    loop_ub = b_Y->size[0];
    i = Y->size[0] * Y->size[1];
    Y->size[0] = b_Y->size[0];
    Y->size[1] = 1;
    emxEnsureCapacity_real_T(Y, i);
    Y_data = Y->data;
    for (i = 0; i < loop_ub; i++) {
      Y_data[i] = b_Y_data[i];
    }
  }
  emxFree_real_T(&b_Y);
}

/*
 * File trailer for mrdivide_helper.c
 *
 * [EOF]
 */
