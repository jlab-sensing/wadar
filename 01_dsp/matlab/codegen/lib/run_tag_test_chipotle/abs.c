/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: abs.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 27-Mar-2025 00:17:05
 */

/* Include Files */
#include "abs.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_creal_T *x
 *                emxArray_real_T *y
 * Return Type  : void
 */
void b_abs(const emxArray_creal_T *x, emxArray_real_T *y)
{
  const creal_T *x_data;
  double *y_data;
  int k;
  int nx_tmp;
  x_data = x->data;
  nx_tmp = x->size[0];
  k = y->size[0];
  y->size[0] = x->size[0];
  emxEnsureCapacity_real_T(y, k);
  y_data = y->data;
  for (k = 0; k < nx_tmp; k++) {
    double a;
    double b;
    a = fabs(x_data[k].re);
    b = fabs(x_data[k].im);
    if (a < b) {
      a /= b;
      y_data[k] = b * sqrt(a * a + 1.0);
    } else if (a > b) {
      b /= a;
      y_data[k] = a * sqrt(b * b + 1.0);
    } else if (rtIsNaN(b)) {
      y_data[k] = rtNaN;
    } else {
      y_data[k] = a * 1.4142135623730951;
    }
  }
}

/*
 * Arguments    : const creal_T x
 * Return Type  : double
 */
double c_abs(const creal_T x)
{
  double b;
  double y;
  y = fabs(x.re);
  b = fabs(x.im);
  if (y < b) {
    y /= b;
    y = b * sqrt(y * y + 1.0);
  } else if (y > b) {
    b /= y;
    y *= sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = rtNaN;
  } else {
    y *= 1.4142135623730951;
  }
  return y;
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                emxArray_real_T *y
 * Return Type  : void
 */
void d_abs(const emxArray_creal_T *x, emxArray_real_T *y)
{
  const creal_T *x_data;
  double *y_data;
  int k;
  int nx_tmp;
  x_data = x->data;
  nx_tmp = x->size[1];
  k = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = x->size[1];
  emxEnsureCapacity_real_T(y, k);
  y_data = y->data;
  for (k = 0; k < nx_tmp; k++) {
    double a;
    double b;
    a = fabs(x_data[k].re);
    b = fabs(x_data[k].im);
    if (a < b) {
      a /= b;
      y_data[k] = b * sqrt(a * a + 1.0);
    } else if (a > b) {
      b /= a;
      y_data[k] = a * sqrt(b * b + 1.0);
    } else if (rtIsNaN(b)) {
      y_data[k] = rtNaN;
    } else {
      y_data[k] = a * 1.4142135623730951;
    }
  }
}

/*
 * File trailer for abs.c
 *
 * [EOF]
 */
