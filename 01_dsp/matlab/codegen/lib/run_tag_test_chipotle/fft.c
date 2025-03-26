/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
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
  creal_T *xPerm_data;
  creal_T *y_data;
  double *costab1q_data;
  double *costab_data;
  double *sintab_data;
  double *sintabinv_data;
  int i;
  int k;
  int pow2p;
  x_data = x->data;
  emxInit_creal_T(&xPerm, 2);
  emxInit_creal_T(&yPerm, 2);
  emxInit_real_T(&costab1q, 2);
  emxInit_real_T(&costab, 2);
  emxInit_real_T(&sintab, 2);
  emxInit_real_T(&sintabinv, 2);
  if ((x->size[0] == 0) || (x->size[1] == 0) || ((int)varargin_1 == 0)) {
    int pmin;
    i = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = (int)varargin_1;
    emxEnsureCapacity_creal_T(y, i);
    y_data = y->data;
    pmin = x->size[0] * (int)varargin_1;
    for (i = 0; i < pmin; i++) {
      y_data[i].re = 0.0;
      y_data[i].im = 0.0;
    }
  } else {
    double e;
    int n;
    int pmax;
    int pmin;
    boolean_T useRadix2;
    pmin = x->size[1];
    i = xPerm->size[0] * xPerm->size[1];
    xPerm->size[0] = x->size[1];
    pmax = x->size[0];
    xPerm->size[1] = x->size[0];
    emxEnsureCapacity_creal_T(xPerm, i);
    xPerm_data = xPerm->data;
    for (i = 0; i < pmax; i++) {
      for (pow2p = 0; pow2p < pmin; pow2p++) {
        xPerm_data[pow2p + xPerm->size[0] * i] = x_data[i + x->size[0] * pow2p];
      }
    }
    useRadix2 = (((int)varargin_1 > 0) &&
                 (((int)varargin_1 & ((int)varargin_1 - 1)) == 0));
    pow2p = 1;
    if (useRadix2) {
      pmin = (int)varargin_1;
    } else {
      if ((int)varargin_1 > 0) {
        n = ((int)varargin_1 + (int)varargin_1) - 1;
        pmax = 31;
        if (n <= 1) {
          pmax = 0;
        } else {
          boolean_T exitg1;
          pmin = 0;
          exitg1 = false;
          while ((!exitg1) && (pmax - pmin > 1)) {
            k = (pmin + pmax) >> 1;
            pow2p = 1 << k;
            if (pow2p == n) {
              pmax = k;
              exitg1 = true;
            } else if (pow2p > n) {
              pmax = k;
            } else {
              pmin = k;
            }
          }
        }
        pow2p = 1 << pmax;
      }
      pmin = pow2p;
    }
    e = 6.2831853071795862 / (double)pmin;
    n = (int)((unsigned int)pmin >> 1) >> 1;
    i = costab1q->size[0] * costab1q->size[1];
    costab1q->size[0] = 1;
    costab1q->size[1] = n + 1;
    emxEnsureCapacity_real_T(costab1q, i);
    costab1q_data = costab1q->data;
    costab1q_data[0] = 1.0;
    pmin = n / 2 - 1;
    for (k = 0; k <= pmin; k++) {
      costab1q_data[k + 1] = cos(e * ((double)k + 1.0));
    }
    i = pmin + 2;
    for (k = i; k < n; k++) {
      costab1q_data[k] = sin(e * (double)(n - k));
    }
    costab1q_data[n] = 0.0;
    if (!useRadix2) {
      n = costab1q->size[1] - 1;
      pmax = (costab1q->size[1] - 1) << 1;
      i = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(costab, i);
      costab_data = costab->data;
      i = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(sintab, i);
      sintab_data = sintab->data;
      costab_data[0] = 1.0;
      sintab_data[0] = 0.0;
      i = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(sintabinv, i);
      sintabinv_data = sintabinv->data;
      for (k = 0; k < n; k++) {
        sintabinv_data[k + 1] = costab1q_data[(n - k) - 1];
      }
      i = costab1q->size[1];
      for (k = i; k <= pmax; k++) {
        sintabinv_data[k] = costab1q_data[k - n];
      }
      for (k = 0; k < n; k++) {
        costab_data[k + 1] = costab1q_data[k + 1];
        sintab_data[k + 1] = -costab1q_data[(n - k) - 1];
      }
      for (k = i; k <= pmax; k++) {
        costab_data[k] = -costab1q_data[pmax - k];
        sintab_data[k] = -costab1q_data[k - n];
      }
    } else {
      n = costab1q->size[1] - 1;
      pmax = (costab1q->size[1] - 1) << 1;
      i = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(costab, i);
      costab_data = costab->data;
      i = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(sintab, i);
      sintab_data = sintab->data;
      costab_data[0] = 1.0;
      sintab_data[0] = 0.0;
      for (k = 0; k < n; k++) {
        costab_data[k + 1] = costab1q_data[k + 1];
        sintab_data[k + 1] = -costab1q_data[(n - k) - 1];
      }
      i = costab1q->size[1];
      for (k = i; k <= pmax; k++) {
        costab_data[k] = -costab1q_data[pmax - k];
        sintab_data[k] = -costab1q_data[k - n];
      }
      sintabinv->size[0] = 1;
      sintabinv->size[1] = 0;
    }
    if (useRadix2) {
      c_FFTImplementationCallback_r2b(xPerm, (int)varargin_1, costab, sintab,
                                      yPerm);
      xPerm_data = yPerm->data;
    } else {
      c_FFTImplementationCallback_dob(xPerm, pow2p, (int)varargin_1, costab,
                                      sintab, sintabinv, yPerm);
      xPerm_data = yPerm->data;
    }
    pmin = yPerm->size[1];
    i = y->size[0] * y->size[1];
    y->size[0] = yPerm->size[1];
    pmax = yPerm->size[0];
    y->size[1] = yPerm->size[0];
    emxEnsureCapacity_creal_T(y, i);
    y_data = y->data;
    for (i = 0; i < pmax; i++) {
      for (pow2p = 0; pow2p < pmin; pow2p++) {
        y_data[pow2p + y->size[0] * i] = xPerm_data[i + yPerm->size[0] * pow2p];
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
