/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: abs.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 02:59:58
 */

/* Include Files */
#include "abs.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const creal_T x[1024000]
 *                double y[1024000]
 * Return Type  : void
 */
void b_abs(const creal_T x[1024000], double y[1024000])
{
  int k;
  for (k = 0; k < 1024000; k++) {
    double a;
    double b;
    a = fabs(x[k].re);
    b = fabs(x[k].im);
    if (a < b) {
      a /= b;
      y[k] = b * sqrt(a * a + 1.0);
    } else if (a > b) {
      b /= a;
      y[k] = a * sqrt(b * b + 1.0);
    } else if (rtIsNaN(b)) {
      y[k] = rtNaN;
    } else {
      y[k] = a * 1.4142135623730951;
    }
  }
}

/*
 * File trailer for abs.c
 *
 * [EOF]
 */
