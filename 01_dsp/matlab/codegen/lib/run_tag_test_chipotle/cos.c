/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: cos.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "cos.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_types.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *x
 * Return Type  : void
 */
void b_cos(emxArray_real_T *x)
{
  double *x_data;
  int k;
  int nx;
  x_data = x->data;
  nx = x->size[1];
  for (k = 0; k < nx; k++) {
    x_data[k] = cos(x_data[k]);
  }
}

/*
 * File trailer for cos.c
 *
 * [EOF]
 */
