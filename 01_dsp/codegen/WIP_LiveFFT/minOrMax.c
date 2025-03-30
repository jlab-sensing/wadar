/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 02:59:58
 */

/* Include Files */
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double x[2000]
 *                int *idx
 * Return Type  : double
 */
double b_maximum(const double x[2000], int *idx)
{
  double ex;
  int k;
  if (!rtIsNaN(x[0])) {
    *idx = 1;
  } else {
    bool exitg1;
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 2001)) {
      if (!rtIsNaN(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (*idx == 0) {
    ex = x[0];
    *idx = 1;
  } else {
    int i;
    ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 2001; k++) {
      double d;
      d = x[k - 1];
      if (ex < d) {
        ex = d;
        *idx = k;
      }
    }
  }
  return ex;
}

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
