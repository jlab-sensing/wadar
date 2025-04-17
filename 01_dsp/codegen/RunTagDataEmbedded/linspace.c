/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: linspace.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "linspace.h"
#include "RunTagDataEmbedded_emxutil.h"
#include "RunTagDataEmbedded_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double n
 *                emxArray_real_T *y
 * Return Type  : void
 */
void linspace(double n, emxArray_real_T *y)
{
  double *y_data;
  int i;
  int k;
  k = y->size[0] * y->size[1];
  y->size[0] = 1;
  i = (int)floor(n);
  y->size[1] = i;
  emxEnsureCapacity_real_T(y, k);
  y_data = y->data;
  y_data[i - 1] = 1.0;
  if (y->size[1] >= 2) {
    y_data[0] = 0.0;
    if (y->size[1] >= 3) {
      double delta1;
      delta1 = 1.0 / ((double)y->size[1] - 1.0);
      for (k = 0; k <= i - 3; k++) {
        y_data[k + 1] = ((double)k + 1.0) * delta1;
      }
    }
  }
}

/*
 * File trailer for linspace.c
 *
 * [EOF]
 */
