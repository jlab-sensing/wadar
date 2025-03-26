/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: fft.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "fft.h"
#include "FFTImplementationCallback.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_creal_T *x
 *                double varargin_1
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void fft(const emxArray_creal_T *x, double varargin_1, emxArray_creal_T *y)
{
  emxArray_creal_T *xPerm;
  emxArray_creal_T *yPerm;
  emxArray_real_T *costab;
  emxArray_real_T *costab1q;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  const creal_T *x_data;
  creal_T *yPerm_data;
  creal_T *y_data;
  double *costab1q_data;
  double *costab_data;
  double *sintab_data;
  double *sintabinv_data;
  int b_k;
  int k;
  x_data = x->data;
  emxInit_creal_T(&xPerm, 2);
  emxInit_creal_T(&yPerm, 2);
  emxInit_real_T(&costab1q, 2);
  emxInit_real_T(&costab, 2);
  emxInit_real_T(&sintab, 2);
  emxInit_real_T(&sintabinv, 2);
  if ((x->size[0] == 0) || (x->size[1] == 0) || ((int)varargin_1 == 0)) {
    int nd2;
    nd2 = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = (int)varargin_1;
    emxEnsureCapacity_creal_T(y, nd2);
    y_data = y->data;
    nd2 = x->size[0] * (int)varargin_1;
    for (k = 0; k < nd2; k++) {
      y_data[k].re = 0.0;
      y_data[k].im = 0.0;
    }
  } else {
    double e;
    int b_n;
    int b_n2;
    int n;
    int n2;
    int nd2;
    bool useRadix2;
    n = x->size[1];
    nd2 = xPerm->size[0] * xPerm->size[1];
    xPerm->size[0] = x->size[1];
    n2 = x->size[0];
    xPerm->size[1] = x->size[0];
    emxEnsureCapacity_creal_T(xPerm, nd2);
    y_data = xPerm->data;
    for (k = 0; k < n2; k++) {
      for (b_k = 0; b_k < n; b_k++) {
        y_data[b_k + xPerm->size[0] * k] = x_data[k + x->size[0] * b_k];
      }
    }
    useRadix2 = (((int)varargin_1 > 0) &&
                 (((int)varargin_1 & ((int)varargin_1 - 1)) == 0));
    n2 = 1;
    if (useRadix2) {
      nd2 = (int)varargin_1;
    } else {
      if ((int)varargin_1 > 0) {
        nd2 = ((int)varargin_1 + (int)varargin_1) - 1;
        n = 31;
        if (nd2 <= 1) {
          n = 0;
        } else {
          bool exitg1;
          n2 = 0;
          exitg1 = false;
          while ((!exitg1) && (n - n2 > 1)) {
            b_n = (n2 + n) >> 1;
            b_n2 = 1 << b_n;
            if (b_n2 == nd2) {
              n = b_n;
              exitg1 = true;
            } else if (b_n2 > nd2) {
              n = b_n;
            } else {
              n2 = b_n;
            }
          }
        }
        n2 = 1 << n;
      }
      nd2 = n2;
    }
    e = 6.2831853071795862 / (double)nd2;
    n = (int)(((unsigned int)nd2 >> 1) >> 1);
    nd2 = costab1q->size[0] * costab1q->size[1];
    costab1q->size[0] = 1;
    costab1q->size[1] = n + 1;
    emxEnsureCapacity_real_T(costab1q, nd2);
    costab1q_data = costab1q->data;
    costab1q_data[0] = 1.0;
    nd2 = (int)((unsigned int)n >> 1) - 1;
    for (k = 0; k <= nd2; k++) {
      costab1q_data[k + 1] = cos(e * ((double)k + 1.0));
    }
    nd2 += 2;
    for (b_k = nd2; b_k < n; b_k++) {
      costab1q_data[b_k] = sin(e * (double)(n - b_k));
    }
    costab1q_data[n] = 0.0;
    if (!useRadix2) {
      b_n = costab1q->size[1] - 1;
      b_n2 = (costab1q->size[1] - 1) << 1;
      nd2 = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = b_n2 + 1;
      emxEnsureCapacity_real_T(costab, nd2);
      costab_data = costab->data;
      nd2 = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = b_n2 + 1;
      emxEnsureCapacity_real_T(sintab, nd2);
      sintab_data = sintab->data;
      costab_data[0] = 1.0;
      sintab_data[0] = 0.0;
      nd2 = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = b_n2 + 1;
      emxEnsureCapacity_real_T(sintabinv, nd2);
      sintabinv_data = sintabinv->data;
      for (k = 0; k < b_n; k++) {
        sintabinv_data[k + 1] = costab1q_data[(b_n - k) - 1];
      }
      nd2 = costab1q->size[1];
      for (k = nd2; k <= b_n2; k++) {
        sintabinv_data[k] = costab1q_data[k - b_n];
      }
      for (k = 0; k < b_n; k++) {
        costab_data[k + 1] = costab1q_data[k + 1];
        sintab_data[k + 1] = -costab1q_data[(b_n - k) - 1];
      }
      for (k = nd2; k <= b_n2; k++) {
        costab_data[k] = -costab1q_data[b_n2 - k];
        sintab_data[k] = -costab1q_data[k - b_n];
      }
      c_FFTImplementationCallback_dob(xPerm, n2, (int)varargin_1, costab,
                                      sintab, sintabinv, yPerm);
      yPerm_data = yPerm->data;
    } else {
      n = costab1q->size[1] - 1;
      n2 = (costab1q->size[1] - 1) << 1;
      nd2 = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = n2 + 1;
      emxEnsureCapacity_real_T(costab, nd2);
      costab_data = costab->data;
      nd2 = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = n2 + 1;
      emxEnsureCapacity_real_T(sintab, nd2);
      sintab_data = sintab->data;
      costab_data[0] = 1.0;
      sintab_data[0] = 0.0;
      for (k = 0; k < n; k++) {
        costab_data[k + 1] = costab1q_data[k + 1];
        sintab_data[k + 1] = -costab1q_data[(n - k) - 1];
      }
      nd2 = costab1q->size[1];
      for (k = nd2; k <= n2; k++) {
        costab_data[k] = -costab1q_data[n2 - k];
        sintab_data[k] = -costab1q_data[k - n];
      }
      c_FFTImplementationCallback_r2b(xPerm, (int)varargin_1, costab, sintab,
                                      yPerm);
      yPerm_data = yPerm->data;
    }
    n = yPerm->size[1];
    nd2 = y->size[0] * y->size[1];
    y->size[0] = yPerm->size[1];
    n2 = yPerm->size[0];
    y->size[1] = yPerm->size[0];
    emxEnsureCapacity_creal_T(y, nd2);
    y_data = y->data;
    for (k = 0; k < n2; k++) {
      for (b_k = 0; b_k < n; b_k++) {
        y_data[b_k + y->size[0] * k] = yPerm_data[k + yPerm->size[0] * b_k];
      }
    }
  }
  emxFree_real_T(&sintabinv);
  emxFree_real_T(&sintab);
  emxFree_real_T(&costab);
  emxFree_real_T(&costab1q);
  emxFree_creal_T(&yPerm);
  emxFree_creal_T(&xPerm);
}

/*
 * File trailer for fft.c
 *
 * [EOF]
 */
