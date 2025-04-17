/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 04:43:50
 */

/* Include Files */
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double x[1024000]
 *                double ex[2000]
 * Return Type  : void
 */
void maximum(const double x[1024000], double ex[2000])
{
  int i;
  int j;
  for (j = 0; j < 2000; j++) {
    int ex_tmp;
    ex_tmp = j << 9;
    ex[j] = x[ex_tmp];
    for (i = 0; i < 511; i++) {
      double d;
      bool p;
      d = x[(i + ex_tmp) + 1];
      if (rtIsNaN(d)) {
        p = false;
      } else {
        double d1;
        d1 = ex[j];
        if (rtIsNaN(d1)) {
          p = true;
        } else {
          p = (d1 < d);
        }
      }
      if (p) {
        ex[j] = d;
      }
    }
  }
}

/*
 * File trailer for minOrMax.c
 *
 * [EOF]
 */
