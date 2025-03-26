/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x[512]
 *                int *idx
 * Return Type  : double
 */
double b_maximum(const double x[512], int *idx)
{
  double ex;
  int b_idx;
  int b_k;
  if (!rtIsNaN(x[0])) {
    b_idx = 1;
  } else {
    int k;
    bool exitg1;
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 513)) {
      if (!rtIsNaN(x[k - 1])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (b_idx == 0) {
    ex = x[0];
    *idx = 1;
  } else {
    ex = x[b_idx - 1];
    *idx = b_idx;
    b_idx++;
    for (b_k = b_idx; b_k < 513; b_k++) {
      double d;
      d = x[b_k - 1];
      if (ex < d) {
        ex = d;
        *idx = b_k;
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const emxArray_real_T *x
 * Return Type  : double
 */
double maximum(const emxArray_real_T *x)
{
  const double *x_data;
  double ex;
  int b_k;
  int last;
  x_data = x->data;
  last = x->size[0];
  if (x->size[0] <= 2) {
    if (x->size[0] == 1) {
      ex = x_data[0];
    } else {
      ex = x_data[x->size[0] - 1];
      if ((!(x_data[0] < ex)) && ((!rtIsNaN(x_data[0])) || rtIsNaN(ex))) {
        ex = x_data[0];
      }
    }
  } else {
    int idx;
    if (!rtIsNaN(x_data[0])) {
      idx = 1;
    } else {
      int k;
      bool exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!rtIsNaN(x_data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = x_data[0];
    } else {
      ex = x_data[idx - 1];
      idx++;
      for (b_k = idx; b_k <= last; b_k++) {
        double d;
        d = x_data[b_k - 1];
        if (ex < d) {
          ex = d;
        }
      }
    }
  }
  return ex;
}

/*
 * File trailer for minOrMax.c
 *
 * [EOF]
 */
