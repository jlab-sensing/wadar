/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 27-Mar-2025 00:17:05
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
  emxArray_creal_T *r;
  emxArray_creal_T *xPerm;
  emxArray_creal_T *yPerm;
  emxArray_real_T *costab;
  emxArray_real_T *costab1q;
  emxArray_real_T *sintab;
  emxArray_real_T *sintabinv;
  const creal_T *x_data;
  creal_T *xPerm_data;
  creal_T *yPerm_data;
  creal_T *y_data;
  double *costab1q_data;
  double *costab_data;
  double *sintab_data;
  double *sintabinv_data;
  int chan;
  int i;
  int j;
  int k;
  int pmin;
  x_data = x->data;
  emxInit_creal_T(&xPerm, 2);
  emxInit_creal_T(&yPerm, 2);
  emxInit_real_T(&costab1q, 2);
  emxInit_real_T(&costab, 2);
  emxInit_real_T(&sintab, 2);
  emxInit_real_T(&sintabinv, 2);
  emxInit_creal_T(&r, 1);
  if ((x->size[0] == 0) || (x->size[1] == 0) || ((int)varargin_1 == 0)) {
    int pmax;
    j = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = (int)varargin_1;
    emxEnsureCapacity_creal_T(y, j);
    y_data = y->data;
    pmax = x->size[0] * (int)varargin_1;
    for (j = 0; j < pmax; j++) {
      y_data[j].re = 0.0;
      y_data[j].im = 0.0;
    }
  } else {
    double e;
    int istart;
    int loop_ub;
    int pmax;
    int pow2p;
    bool useRadix2;
    pmax = x->size[1];
    j = xPerm->size[0] * xPerm->size[1];
    xPerm->size[0] = x->size[1];
    loop_ub = x->size[0];
    xPerm->size[1] = x->size[0];
    emxEnsureCapacity_creal_T(xPerm, j);
    xPerm_data = xPerm->data;
    for (j = 0; j < loop_ub; j++) {
      for (pmin = 0; pmin < pmax; pmin++) {
        xPerm_data[pmin + xPerm->size[0] * j] = x_data[j + x->size[0] * pmin];
      }
    }
    useRadix2 = (((int)varargin_1 > 0) &&
                 (((int)varargin_1 & ((int)varargin_1 - 1)) == 0));
    pmin = 1;
    if (useRadix2) {
      pmax = (int)varargin_1;
    } else {
      if ((int)varargin_1 > 0) {
        istart = ((int)varargin_1 + (int)varargin_1) - 1;
        pmax = 31;
        if (istart <= 1) {
          pmax = 0;
        } else {
          bool exitg1;
          pmin = 0;
          exitg1 = false;
          while ((!exitg1) && (pmax - pmin > 1)) {
            k = (pmin + pmax) >> 1;
            pow2p = 1 << k;
            if (pow2p == istart) {
              pmax = k;
              exitg1 = true;
            } else if (pow2p > istart) {
              pmax = k;
            } else {
              pmin = k;
            }
          }
        }
        pmin = 1 << pmax;
      }
      pmax = pmin;
    }
    e = 6.2831853071795862 / (double)pmax;
    istart = (int)((unsigned int)pmax >> 1) >> 1;
    j = costab1q->size[0] * costab1q->size[1];
    costab1q->size[0] = 1;
    costab1q->size[1] = istart + 1;
    emxEnsureCapacity_real_T(costab1q, j);
    costab1q_data = costab1q->data;
    costab1q_data[0] = 1.0;
    pmax = istart / 2 - 1;
    for (k = 0; k <= pmax; k++) {
      costab1q_data[k + 1] = cos(e * ((double)k + 1.0));
    }
    j = pmax + 2;
    for (k = j; k < istart; k++) {
      costab1q_data[k] = sin(e * (double)(istart - k));
    }
    costab1q_data[istart] = 0.0;
    if (!useRadix2) {
      istart = costab1q->size[1] - 1;
      pmax = (costab1q->size[1] - 1) << 1;
      j = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(costab, j);
      costab_data = costab->data;
      j = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(sintab, j);
      sintab_data = sintab->data;
      costab_data[0] = 1.0;
      sintab_data[0] = 0.0;
      j = sintabinv->size[0] * sintabinv->size[1];
      sintabinv->size[0] = 1;
      sintabinv->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(sintabinv, j);
      sintabinv_data = sintabinv->data;
      for (k = 0; k < istart; k++) {
        sintabinv_data[k + 1] = costab1q_data[(istart - k) - 1];
      }
      j = costab1q->size[1];
      for (k = j; k <= pmax; k++) {
        sintabinv_data[k] = costab1q_data[k - istart];
      }
      for (k = 0; k < istart; k++) {
        costab_data[k + 1] = costab1q_data[k + 1];
        sintab_data[k + 1] = -costab1q_data[(istart - k) - 1];
      }
      for (k = j; k <= pmax; k++) {
        costab_data[k] = -costab1q_data[pmax - k];
        sintab_data[k] = -costab1q_data[k - istart];
      }
    } else {
      istart = costab1q->size[1] - 1;
      pmax = (costab1q->size[1] - 1) << 1;
      j = costab->size[0] * costab->size[1];
      costab->size[0] = 1;
      costab->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(costab, j);
      costab_data = costab->data;
      j = sintab->size[0] * sintab->size[1];
      sintab->size[0] = 1;
      sintab->size[1] = pmax + 1;
      emxEnsureCapacity_real_T(sintab, j);
      sintab_data = sintab->data;
      costab_data[0] = 1.0;
      sintab_data[0] = 0.0;
      for (k = 0; k < istart; k++) {
        costab_data[k + 1] = costab1q_data[k + 1];
        sintab_data[k + 1] = -costab1q_data[(istart - k) - 1];
      }
      j = costab1q->size[1];
      for (k = j; k <= pmax; k++) {
        costab_data[k] = -costab1q_data[pmax - k];
        sintab_data[k] = -costab1q_data[k - istart];
      }
      sintabinv->size[0] = 1;
      sintabinv->size[1] = 0;
    }
    if (useRadix2) {
      int nRowsD2;
      int nRowsD4;
      int nRowsM2;
      int u1;
      int yPerm_idx_0;
      yPerm_idx_0 = (int)varargin_1;
      j = yPerm->size[0] * yPerm->size[1];
      yPerm->size[0] = (int)varargin_1;
      yPerm->size[1] = x->size[0];
      emxEnsureCapacity_creal_T(yPerm, j);
      yPerm_data = yPerm->data;
      if ((int)varargin_1 > xPerm->size[0]) {
        j = yPerm->size[0] * yPerm->size[1];
        yPerm->size[0] = (int)varargin_1;
        yPerm->size[1] = x->size[0];
        emxEnsureCapacity_creal_T(yPerm, j);
        yPerm_data = yPerm->data;
        pmax = (int)varargin_1 * x->size[0];
        for (j = 0; j < pmax; j++) {
          yPerm_data[j].re = 0.0;
          yPerm_data[j].im = 0.0;
        }
      }
      pmax = xPerm->size[0];
      u1 = (int)varargin_1;
      if (pmax <= u1) {
        u1 = pmax;
      }
      nRowsM2 = (int)varargin_1 - 2;
      nRowsD2 = (int)((unsigned int)(int)varargin_1 >> 1);
      nRowsD4 = nRowsD2 / 2;
      for (chan = 0; chan < loop_ub; chan++) {
        double temp_im;
        double temp_re;
        double temp_re_tmp;
        pow2p = chan * xPerm->size[0];
        j = r->size[0];
        r->size[0] = (int)varargin_1;
        emxEnsureCapacity_creal_T(r, j);
        y_data = r->data;
        if ((int)varargin_1 > xPerm->size[0]) {
          j = r->size[0];
          r->size[0] = (int)varargin_1;
          emxEnsureCapacity_creal_T(r, j);
          y_data = r->data;
          for (j = 0; j < yPerm_idx_0; j++) {
            y_data[j].re = 0.0;
            y_data[j].im = 0.0;
          }
        }
        pmax = 0;
        pmin = 0;
        for (i = 0; i <= u1 - 2; i++) {
          y_data[pmax] = xPerm_data[pow2p + i];
          istart = (int)varargin_1;
          useRadix2 = true;
          while (useRadix2) {
            istart >>= 1;
            pmin ^= istart;
            useRadix2 = ((pmin & istart) == 0);
          }
          pmax = pmin;
        }
        if (u1 - 2 >= 0) {
          pow2p = (pow2p + u1) - 1;
        }
        y_data[pmax] = xPerm_data[pow2p];
        if ((int)varargin_1 > 1) {
          for (i = 0; i <= nRowsM2; i += 2) {
            temp_re_tmp = y_data[i + 1].re;
            temp_im = y_data[i + 1].im;
            temp_re = y_data[i].re;
            e = y_data[i].im;
            y_data[i + 1].re = temp_re - temp_re_tmp;
            y_data[i + 1].im = y_data[i].im - y_data[i + 1].im;
            e += temp_im;
            y_data[i].re = temp_re + temp_re_tmp;
            y_data[i].im = e;
          }
        }
        pmax = 2;
        pmin = 4;
        k = nRowsD4;
        pow2p = ((nRowsD4 - 1) << 2) + 1;
        while (k > 0) {
          int b_temp_re_tmp;
          for (i = 0; i < pow2p; i += pmin) {
            b_temp_re_tmp = i + pmax;
            temp_re = y_data[b_temp_re_tmp].re;
            temp_im = y_data[b_temp_re_tmp].im;
            y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
            y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
            y_data[i].re += temp_re;
            y_data[i].im += temp_im;
          }
          istart = 1;
          for (j = k; j < nRowsD2; j += k) {
            double twid_im;
            double twid_re;
            int ihi;
            twid_re = costab_data[j];
            twid_im = sintab_data[j];
            i = istart;
            ihi = istart + pow2p;
            while (i < ihi) {
              b_temp_re_tmp = i + pmax;
              temp_re_tmp = y_data[b_temp_re_tmp].im;
              e = y_data[b_temp_re_tmp].re;
              temp_re = twid_re * e - twid_im * temp_re_tmp;
              temp_im = twid_re * temp_re_tmp + twid_im * e;
              y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
              y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
              y_data[i].re += temp_re;
              y_data[i].im += temp_im;
              i += pmin;
            }
            istart++;
          }
          k /= 2;
          pmax = pmin;
          pmin += pmin;
          pow2p -= pmax;
        }
        for (j = 0; j < yPerm_idx_0; j++) {
          yPerm_data[j + yPerm->size[0] * chan] = y_data[j];
        }
      }
    } else {
      c_FFTImplementationCallback_dob(xPerm, pmin, (int)varargin_1, costab,
                                      sintab, sintabinv, yPerm);
      yPerm_data = yPerm->data;
    }
    pmax = yPerm->size[1];
    j = y->size[0] * y->size[1];
    y->size[0] = yPerm->size[1];
    loop_ub = yPerm->size[0];
    y->size[1] = yPerm->size[0];
    emxEnsureCapacity_creal_T(y, j);
    y_data = y->data;
    for (j = 0; j < loop_ub; j++) {
      for (pmin = 0; pmin < pmax; pmin++) {
        y_data[pmin + y->size[0] * j] = yPerm_data[j + yPerm->size[0] * pmin];
      }
    }
  }
  emxFree_creal_T(&r);
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
