/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: round.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "round.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_types.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *x
 * Return Type  : void
 */
void b_round(emxArray_real_T *x)
{
  double *x_data;
  int k;
  int nx;
  x_data = x->data;
  nx = x->size[1];
  for (k = 0; k < nx; k++) {
    double b_x;
    b_x = x_data[k];
    if (fabs(b_x) < 4.503599627370496E+15) {
      if (b_x >= 0.5) {
        b_x = floor(b_x + 0.5);
      } else if (b_x > -0.5) {
        b_x *= 0.0;
      } else {
        b_x = ceil(b_x - 0.5);
      }
    }
    x_data[k] = b_x;
  }
}

/*
 * File trailer for round.c
 *
 * [EOF]
 */
