/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: linspace.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "linspace.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double N
 *                emxArray_real_T *y
 * Return Type  : void
 */
void linspace(double N, emxArray_real_T *y)
{
  double *y_data;
  int i;
  int i1;
  int k;
  i = y->size[0] * y->size[1];
  y->size[0] = 1;
  i1 = (int)floor(N);
  y->size[1] = i1;
  emxEnsureCapacity_real_T(y, i);
  y_data = y->data;
  y_data[i1 - 1] = 1.0;
  if (y->size[1] >= 2) {
    y_data[0] = 0.0;
    if (y->size[1] >= 3) {
      double delta1;
      delta1 = 1.0 / ((double)y->size[1] - 1.0);
      for (k = 0; k <= i1 - 3; k++) {
        y_data[k + 1] = (double)(k + 1) * delta1;
      }
    }
  }
}

/*
 * File trailer for linspace.c
 *
 * [EOF]
 */
