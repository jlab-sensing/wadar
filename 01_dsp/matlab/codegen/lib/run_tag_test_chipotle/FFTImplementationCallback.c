/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: FFTImplementationCallback.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "FFTImplementationCallback.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include "omp.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void d_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
                                            int unsigned_nRows,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            emxArray_creal_T *y);

/* Function Definitions */
/*
 * Arguments    : const emxArray_creal_T *x
 *                int unsigned_nRows
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
static void d_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
                                            int unsigned_nRows,
                                            const emxArray_real_T *costab,
                                            const emxArray_real_T *sintab,
                                            emxArray_creal_T *y)
{
  const creal_T *x_data;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  double im;
  double temp_im;
  double temp_re;
  double temp_re_tmp;
  int i;
  int iDelta2;
  int iheight;
  int iy;
  int j;
  int ju;
  int k;
  int nRowsD2;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  iy = y->size[0];
  y->size[0] = unsigned_nRows;
  emxEnsureCapacity_creal_T(y, iy);
  y_data = y->data;
  if (unsigned_nRows > x->size[0]) {
    iy = y->size[0];
    y->size[0] = unsigned_nRows;
    emxEnsureCapacity_creal_T(y, iy);
    y_data = y->data;
    for (iy = 0; iy < unsigned_nRows; iy++) {
      y_data[iy].re = 0.0;
      y_data[iy].im = 0.0;
    }
  }
  iDelta2 = x->size[0];
  if (iDelta2 > unsigned_nRows) {
    iDelta2 = unsigned_nRows;
  }
  iheight = unsigned_nRows - 2;
  nRowsD2 = (int)((unsigned int)unsigned_nRows >> 1);
  k = nRowsD2 / 2;
  iy = 0;
  ju = 0;
  for (i = 0; i <= iDelta2 - 2; i++) {
    boolean_T tst;
    y_data[iy] = x_data[i];
    iy = unsigned_nRows;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }
    iy = ju;
  }
  if (iDelta2 - 2 < 0) {
    iDelta2 = 0;
  } else {
    iDelta2--;
  }
  y_data[iy] = x_data[iDelta2];
  if (unsigned_nRows > 1) {
    for (i = 0; i <= iheight; i += 2) {
      temp_re_tmp = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      temp_re = y_data[i].re;
      im = y_data[i].im;
      y_data[i + 1].re = temp_re - temp_re_tmp;
      y_data[i + 1].im = y_data[i].im - y_data[i + 1].im;
      im += temp_im;
      y_data[i].re = temp_re + temp_re_tmp;
      y_data[i].im = im;
    }
  }
  iy = 2;
  iDelta2 = 4;
  iheight = ((k - 1) << 2) + 1;
  while (k > 0) {
    int b_temp_re_tmp;
    for (i = 0; i < iheight; i += iDelta2) {
      b_temp_re_tmp = i + iy;
      temp_re = y_data[b_temp_re_tmp].re;
      temp_im = y_data[b_temp_re_tmp].im;
      y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
      y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }
    ju = 1;
    for (j = k; j < nRowsD2; j += k) {
      double twid_im;
      double twid_re;
      int ihi;
      twid_re = costab_data[j];
      twid_im = sintab_data[j];
      i = ju;
      ihi = ju + iheight;
      while (i < ihi) {
        b_temp_re_tmp = i + iy;
        temp_re_tmp = y_data[b_temp_re_tmp].im;
        im = y_data[b_temp_re_tmp].re;
        temp_re = twid_re * im - twid_im * temp_re_tmp;
        temp_im = twid_re * temp_re_tmp + twid_im * im;
        y_data[b_temp_re_tmp].re = y_data[i].re - temp_re;
        y_data[b_temp_re_tmp].im = y_data[i].im - temp_im;
        y_data[i].re += temp_re;
        y_data[i].im += temp_im;
        i += iDelta2;
      }
      ju++;
    }
    k /= 2;
    iy = iDelta2;
    iDelta2 += iDelta2;
    iheight -= iy;
  }
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int n2blue
 *                int nfft
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                const emxArray_real_T *sintabinv
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void c_FFTImplementationCallback_dob(const emxArray_creal_T *x, int n2blue,
                                     int nfft, const emxArray_real_T *costab,
                                     const emxArray_real_T *sintab,
                                     const emxArray_real_T *sintabinv,
                                     emxArray_creal_T *y)
{
  emxArray_creal_T *fv;
  emxArray_creal_T *fy;
  emxArray_creal_T *r1;
  emxArray_creal_T *wwc;
  const creal_T *x_data;
  creal_T *fv_data;
  creal_T *fy_data;
  creal_T *r;
  creal_T *wwc_data;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  double nt_im;
  double temp_im;
  double temp_re;
  double twid_im;
  double twid_re;
  int b_i;
  int b_k;
  int b_y;
  int chan;
  int i;
  int ihi;
  int iy;
  int j;
  int ju;
  int k;
  int minNrowsNx;
  int nInt2;
  int nInt2m1;
  int nRowsD2;
  int rt;
  int temp_re_tmp;
  int xoff;
  boolean_T tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  nInt2m1 = (nfft + nfft) - 1;
  emxInit_creal_T(&wwc, 1);
  i = wwc->size[0];
  wwc->size[0] = nInt2m1;
  emxEnsureCapacity_creal_T(wwc, i);
  wwc_data = wwc->data;
  rt = 0;
  wwc_data[nfft - 1].re = 1.0;
  wwc_data[nfft - 1].im = 0.0;
  nInt2 = nfft << 1;
  for (k = 0; k <= nfft - 2; k++) {
    b_y = ((k + 1) << 1) - 1;
    if (nInt2 - rt <= b_y) {
      rt += b_y - nInt2;
    } else {
      rt += b_y;
    }
    nt_im = -3.1415926535897931 * (double)rt / (double)nfft;
    i = (nfft - k) - 2;
    wwc_data[i].re = cos(nt_im);
    wwc_data[i].im = -sin(nt_im);
  }
  i = nInt2m1 - 1;
  for (k = i; k >= nfft; k--) {
    wwc_data[k] = wwc_data[(nInt2m1 - k) - 1];
  }
  nInt2m1 = x->size[0];
  i = y->size[0] * y->size[1];
  y->size[0] = nfft;
  y->size[1] = x->size[1];
  emxEnsureCapacity_creal_T(y, i);
  y_data = y->data;
  if (nfft > x->size[0]) {
    i = y->size[0] * y->size[1];
    y->size[0] = nfft;
    y->size[1] = x->size[1];
    emxEnsureCapacity_creal_T(y, i);
    y_data = y->data;
    b_y = nfft * x->size[1];
    for (i = 0; i < b_y; i++) {
      y_data[i].re = 0.0;
      y_data[i].im = 0.0;
    }
  }
  b_y = x->size[1];
#pragma omp parallel num_threads(omp_get_max_threads()) private(               \
        r, fy_data, fv_data, fv, fy, r1, xoff, ju, minNrowsNx, b_k,            \
            temp_re_tmp, temp_re, temp_im, iy, j, nRowsD2, b_i, tst, twid_re,  \
            twid_im, ihi)
  {
    emxInit_creal_T(&fv, 1);
    emxInit_creal_T(&fy, 1);
    emxInit_creal_T(&r1, 1);
#pragma omp for nowait
    for (chan = 0; chan < b_y; chan++) {
      xoff = chan * nInt2m1;
      ju = r1->size[0];
      r1->size[0] = nfft;
      emxEnsureCapacity_creal_T(r1, ju);
      r = r1->data;
      if (nfft > x->size[0]) {
        ju = r1->size[0];
        r1->size[0] = nfft;
        emxEnsureCapacity_creal_T(r1, ju);
        r = r1->data;
        for (ju = 0; ju < nfft; ju++) {
          r[ju].re = 0.0;
          r[ju].im = 0.0;
        }
      }
      minNrowsNx = x->size[0];
      if (nfft <= minNrowsNx) {
        minNrowsNx = nfft;
      }
      for (b_k = 0; b_k < minNrowsNx; b_k++) {
        temp_re_tmp = (nfft + b_k) - 1;
        temp_re = wwc_data[temp_re_tmp].re;
        temp_im = wwc_data[temp_re_tmp].im;
        ju = xoff + b_k;
        r[b_k].re = temp_re * x_data[ju].re + temp_im * x_data[ju].im;
        r[b_k].im = temp_re * x_data[ju].im - temp_im * x_data[ju].re;
      }
      ju = minNrowsNx + 1;
      for (b_k = ju; b_k <= nfft; b_k++) {
        r[b_k - 1].re = 0.0;
        r[b_k - 1].im = 0.0;
      }
      ju = fy->size[0];
      fy->size[0] = n2blue;
      emxEnsureCapacity_creal_T(fy, ju);
      fy_data = fy->data;
      if (n2blue > r1->size[0]) {
        ju = fy->size[0];
        fy->size[0] = n2blue;
        emxEnsureCapacity_creal_T(fy, ju);
        fy_data = fy->data;
        for (ju = 0; ju < n2blue; ju++) {
          fy_data[ju].re = 0.0;
          fy_data[ju].im = 0.0;
        }
      }
      iy = r1->size[0];
      j = n2blue;
      if (iy <= n2blue) {
        j = iy;
      }
      minNrowsNx = n2blue - 2;
      nRowsD2 = (int)((unsigned int)n2blue >> 1);
      b_k = nRowsD2 / 2;
      iy = 0;
      ju = 0;
      for (b_i = 0; b_i <= j - 2; b_i++) {
        fy_data[iy] = r[b_i];
        xoff = n2blue;
        tst = true;
        while (tst) {
          xoff >>= 1;
          ju ^= xoff;
          tst = ((ju & xoff) == 0);
        }
        iy = ju;
      }
      if (j - 2 < 0) {
        xoff = 0;
      } else {
        xoff = j - 1;
      }
      fy_data[iy] = r[xoff];
      if (n2blue > 1) {
        for (b_i = 0; b_i <= minNrowsNx; b_i += 2) {
          temp_re = fy_data[b_i + 1].re;
          temp_im = fy_data[b_i + 1].im;
          twid_re = fy_data[b_i].re;
          twid_im = fy_data[b_i].im;
          fy_data[b_i + 1].re = fy_data[b_i].re - fy_data[b_i + 1].re;
          fy_data[b_i + 1].im = fy_data[b_i].im - fy_data[b_i + 1].im;
          twid_re += temp_re;
          twid_im += temp_im;
          fy_data[b_i].re = twid_re;
          fy_data[b_i].im = twid_im;
        }
      }
      xoff = 2;
      minNrowsNx = 4;
      iy = ((b_k - 1) << 2) + 1;
      while (b_k > 0) {
        for (b_i = 0; b_i < iy; b_i += minNrowsNx) {
          temp_re_tmp = b_i + xoff;
          temp_re = fy_data[temp_re_tmp].re;
          temp_im = fy_data[temp_re_tmp].im;
          fy_data[temp_re_tmp].re = fy_data[b_i].re - temp_re;
          fy_data[temp_re_tmp].im = fy_data[b_i].im - temp_im;
          fy_data[b_i].re += temp_re;
          fy_data[b_i].im += temp_im;
        }
        ju = 1;
        for (j = b_k; j < nRowsD2; j += b_k) {
          twid_re = costab_data[j];
          twid_im = sintab_data[j];
          b_i = ju;
          ihi = ju + iy;
          while (b_i < ihi) {
            temp_re_tmp = b_i + xoff;
            temp_re = twid_re * fy_data[temp_re_tmp].re -
                      twid_im * fy_data[temp_re_tmp].im;
            temp_im = twid_re * fy_data[temp_re_tmp].im +
                      twid_im * fy_data[temp_re_tmp].re;
            fy_data[temp_re_tmp].re = fy_data[b_i].re - temp_re;
            fy_data[temp_re_tmp].im = fy_data[b_i].im - temp_im;
            fy_data[b_i].re += temp_re;
            fy_data[b_i].im += temp_im;
            b_i += minNrowsNx;
          }
          ju++;
        }
        b_k /= 2;
        xoff = minNrowsNx;
        minNrowsNx += minNrowsNx;
        iy -= xoff;
      }
      d_FFTImplementationCallback_r2b(wwc, n2blue, costab, sintab, fv);
      fv_data = fv->data;
      iy = fy->size[0];
      for (ju = 0; ju < iy; ju++) {
        twid_im =
            fy_data[ju].re * fv_data[ju].im + fy_data[ju].im * fv_data[ju].re;
        fy_data[ju].re =
            fy_data[ju].re * fv_data[ju].re - fy_data[ju].im * fv_data[ju].im;
        fy_data[ju].im = twid_im;
      }
      d_FFTImplementationCallback_r2b(fy, n2blue, costab, sintabinv, fv);
      fv_data = fv->data;
      if (fv->size[0] > 1) {
        twid_re = 1.0 / (double)fv->size[0];
        iy = fv->size[0];
        for (ju = 0; ju < iy; ju++) {
          fv_data[ju].re *= twid_re;
          fv_data[ju].im *= twid_re;
        }
      }
      ju = wwc->size[0];
      for (b_k = nfft; b_k <= ju; b_k++) {
        iy = b_k - nfft;
        r[iy].re = wwc_data[b_k - 1].re * fv_data[b_k - 1].re +
                   wwc_data[b_k - 1].im * fv_data[b_k - 1].im;
        r[iy].im = wwc_data[b_k - 1].re * fv_data[b_k - 1].im -
                   wwc_data[b_k - 1].im * fv_data[b_k - 1].re;
      }
      iy = y->size[0];
      for (ju = 0; ju < iy; ju++) {
        y_data[ju + y->size[0] * chan] = r[ju];
      }
    }
    emxFree_creal_T(&r1);
    emxFree_creal_T(&fy);
    emxFree_creal_T(&fv);
  }
  emxFree_creal_T(&wwc);
}

/*
 * Arguments    : const emxArray_creal_T *x
 *                int n1_unsigned
 *                const emxArray_real_T *costab
 *                const emxArray_real_T *sintab
 *                emxArray_creal_T *y
 * Return Type  : void
 */
void c_FFTImplementationCallback_r2b(const emxArray_creal_T *x, int n1_unsigned,
                                     const emxArray_real_T *costab,
                                     const emxArray_real_T *sintab,
                                     emxArray_creal_T *y)
{
  emxArray_creal_T *r1;
  const creal_T *x_data;
  creal_T *r;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  double temp_im;
  double temp_re;
  double twid_im;
  double twid_re;
  int b_i;
  int chan;
  int i;
  int iDelta2;
  int ihi;
  int istart;
  int iy;
  int ju;
  int k;
  int loop_ub_tmp;
  int nRowsD2;
  int nrows;
  int temp_re_tmp;
  int xoff;
  boolean_T tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  nrows = x->size[0];
  i = y->size[0] * y->size[1];
  y->size[0] = n1_unsigned;
  y->size[1] = x->size[1];
  emxEnsureCapacity_creal_T(y, i);
  y_data = y->data;
  if (n1_unsigned > x->size[0]) {
    i = y->size[0] * y->size[1];
    y->size[0] = n1_unsigned;
    y->size[1] = x->size[1];
    emxEnsureCapacity_creal_T(y, i);
    y_data = y->data;
    loop_ub_tmp = n1_unsigned * x->size[1];
    for (i = 0; i < loop_ub_tmp; i++) {
      y_data[i].re = 0.0;
      y_data[i].im = 0.0;
    }
  }
  loop_ub_tmp = x->size[1];
#pragma omp parallel num_threads(omp_get_max_threads()) private(               \
        r, r1, xoff, ju, iy, istart, iDelta2, nRowsD2, k, b_i, tst, temp_re,   \
            temp_im, twid_re, twid_im, temp_re_tmp, ihi)
  {
    emxInit_creal_T(&r1, 1);
#pragma omp for nowait
    for (chan = 0; chan < loop_ub_tmp; chan++) {
      xoff = chan * nrows;
      ju = r1->size[0];
      r1->size[0] = n1_unsigned;
      emxEnsureCapacity_creal_T(r1, ju);
      r = r1->data;
      if (n1_unsigned > x->size[0]) {
        ju = r1->size[0];
        r1->size[0] = n1_unsigned;
        emxEnsureCapacity_creal_T(r1, ju);
        r = r1->data;
        for (ju = 0; ju < n1_unsigned; ju++) {
          r[ju].re = 0.0;
          r[ju].im = 0.0;
        }
      }
      iy = x->size[0];
      istart = n1_unsigned;
      if (iy <= n1_unsigned) {
        istart = iy;
      }
      iDelta2 = n1_unsigned - 2;
      nRowsD2 = (int)((unsigned int)n1_unsigned >> 1);
      k = nRowsD2 / 2;
      iy = 0;
      ju = 0;
      for (b_i = 0; b_i <= istart - 2; b_i++) {
        r[iy] = x_data[xoff + b_i];
        iy = n1_unsigned;
        tst = true;
        while (tst) {
          iy >>= 1;
          ju ^= iy;
          tst = ((ju & iy) == 0);
        }
        iy = ju;
      }
      if (istart - 2 >= 0) {
        xoff = (xoff + istart) - 1;
      }
      r[iy] = x_data[xoff];
      if (n1_unsigned > 1) {
        for (b_i = 0; b_i <= iDelta2; b_i += 2) {
          temp_re = r[b_i + 1].re;
          temp_im = r[b_i + 1].im;
          twid_re = r[b_i].re;
          twid_im = r[b_i].im;
          r[b_i + 1].re = r[b_i].re - r[b_i + 1].re;
          r[b_i + 1].im = r[b_i].im - r[b_i + 1].im;
          twid_re += temp_re;
          twid_im += temp_im;
          r[b_i].re = twid_re;
          r[b_i].im = twid_im;
        }
      }
      iy = 2;
      iDelta2 = 4;
      ju = ((k - 1) << 2) + 1;
      while (k > 0) {
        for (b_i = 0; b_i < ju; b_i += iDelta2) {
          temp_re_tmp = b_i + iy;
          temp_re = r[temp_re_tmp].re;
          temp_im = r[temp_re_tmp].im;
          r[temp_re_tmp].re = r[b_i].re - temp_re;
          r[temp_re_tmp].im = r[b_i].im - temp_im;
          r[b_i].re += temp_re;
          r[b_i].im += temp_im;
        }
        istart = 1;
        for (xoff = k; xoff < nRowsD2; xoff += k) {
          twid_re = costab_data[xoff];
          twid_im = sintab_data[xoff];
          b_i = istart;
          ihi = istart + ju;
          while (b_i < ihi) {
            temp_re_tmp = b_i + iy;
            temp_re = twid_re * r[temp_re_tmp].re - twid_im * r[temp_re_tmp].im;
            temp_im = twid_re * r[temp_re_tmp].im + twid_im * r[temp_re_tmp].re;
            r[temp_re_tmp].re = r[b_i].re - temp_re;
            r[temp_re_tmp].im = r[b_i].im - temp_im;
            r[b_i].re += temp_re;
            r[b_i].im += temp_im;
            b_i += iDelta2;
          }
          istart++;
        }
        k /= 2;
        iy = iDelta2;
        iDelta2 += iDelta2;
        ju -= iy;
      }
      iy = y->size[0];
      for (ju = 0; ju < iy; ju++) {
        y_data[ju + y->size[0] * chan] = r[ju];
      }
    }
    emxFree_creal_T(&r1);
  }
}

/*
 * File trailer for FFTImplementationCallback.c
 *
 * [EOF]
 */
