/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ifWhileCond.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "ifWhileCond.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const boolean_T x_data[]
 *                int x_size
 * Return Type  : boolean_T
 */
boolean_T ifWhileCond(const boolean_T x_data[], int x_size)
{
  boolean_T y;
  y = (x_size != 0);
  if (y) {
    int k;
    boolean_T exitg1;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= x_size - 1)) {
      if (!x_data[0]) {
        y = false;
        exitg1 = true;
      } else {
        k = 1;
      }
    }
  }
  return y;
}

/*
 * File trailer for ifWhileCond.c
 *
 * [EOF]
 */
