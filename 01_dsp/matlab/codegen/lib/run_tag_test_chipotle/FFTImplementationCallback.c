/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: FFTImplementationCallback.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
  double re;
  double temp_im;
  double temp_re;
  int b_i;
  int i;
  int iDelta;
  int iDelta2;
  int iheight;
  int istart;
  int iy;
  int j;
  int ju;
  int k;
  int nRowsD2;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  istart = y->size[0];
  y->size[0] = unsigned_nRows;
  emxEnsureCapacity_creal_T(y, istart);
  y_data = y->data;
  if (unsigned_nRows > x->size[0]) {
    istart = y->size[0];
    y->size[0] = unsigned_nRows;
    emxEnsureCapacity_creal_T(y, istart);
    y_data = y->data;
    for (i = 0; i < unsigned_nRows; i++) {
      y_data[i].re = 0.0;
      y_data[i].im = 0.0;
    }
  }
  j = x->size[0];
  if (j > unsigned_nRows) {
    j = unsigned_nRows;
  }
  iDelta = unsigned_nRows - 2;
  nRowsD2 = (int)((unsigned int)unsigned_nRows >> 1);
  k = (int)((unsigned int)nRowsD2 >> 1);
  iy = 0;
  ju = 0;
  for (i = 0; i <= j - 2; i++) {
    bool tst;
    y_data[iy] = x_data[i];
    istart = unsigned_nRows;
    tst = true;
    while (tst) {
      istart >>= 1;
      ju ^= istart;
      tst = ((ju & istart) == 0);
    }
    iy = ju;
  }
  if (j - 2 < 0) {
    istart = 0;
  } else {
    istart = j - 1;
  }
  y_data[iy] = x_data[istart];
  if (unsigned_nRows > 1) {
    for (i = 0; i <= iDelta; i += 2) {
      temp_re = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      re = y_data[i].re;
      im = y_data[i].im;
      y_data[i + 1].re = re - temp_re;
      y_data[i + 1].im = y_data[i].im - y_data[i + 1].im;
      re += temp_re;
      im += temp_im;
      y_data[i].re = re;
      y_data[i].im = im;
    }
  }
  iDelta = 2;
  iDelta2 = 4;
  iheight = ((k - 1) << 2) + 1;
  while (k > 0) {
    for (b_i = 0; b_i < iheight; b_i += iDelta2) {
      istart = b_i + iDelta;
      temp_re = y_data[istart].re;
      temp_im = y_data[istart].im;
      y_data[istart].re = y_data[b_i].re - temp_re;
      y_data[istart].im = y_data[b_i].im - temp_im;
      y_data[b_i].re += temp_re;
      y_data[b_i].im += temp_im;
    }
    istart = 1;
    for (j = k; j < nRowsD2; j += k) {
      double twid_im;
      double twid_re;
      twid_re = costab_data[j];
      twid_im = sintab_data[j];
      b_i = istart;
      iy = istart + iheight;
      while (b_i < iy) {
        ju = b_i + iDelta;
        re = y_data[ju].im;
        im = y_data[ju].re;
        temp_re = twid_re * im - twid_im * re;
        temp_im = twid_re * re + twid_im * im;
        y_data[ju].re = y_data[b_i].re - temp_re;
        y_data[ju].im = y_data[b_i].im - temp_im;
        y_data[b_i].re += temp_re;
        y_data[b_i].im += temp_im;
        b_i += iDelta2;
      }
      istart++;
    }
    k = (int)((unsigned int)k >> 1);
    iDelta = iDelta2;
    iDelta2 += iDelta2;
    iheight -= iDelta;
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
  double im;
  double nt_im;
  double nt_re;
  double re;
  double temp_im;
  double temp_re;
  double twid_im;
  double twid_re;
  int b_k;
  int b_y;
  int c_k;
  int chan;
  int i;
  int iDelta;
  int iDelta2;
  int iheight;
  int istart;
  int iy;
  int k;
  int minNrowsNx;
  int nInt2;
  int nInt2m1;
  int nRowsD2;
  int rt;
  int xoff;
  bool tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  nInt2m1 = (nfft + nfft) - 1;
  emxInit_creal_T(&wwc, 1);
  b_y = wwc->size[0];
  wwc->size[0] = nInt2m1;
  emxEnsureCapacity_creal_T(wwc, b_y);
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
    nt_re = cos(nt_im);
    nt_im = sin(nt_im);
    b_y = (nfft - k) - 2;
    wwc_data[b_y].re = nt_re;
    wwc_data[b_y].im = -nt_im;
  }
  b_y = nInt2m1 - 1;
  for (k = b_y; k >= nfft; k--) {
    wwc_data[k] = wwc_data[(nInt2m1 - k) - 1];
  }
  rt = x->size[0];
  b_y = y->size[0] * y->size[1];
  y->size[0] = nfft;
  y->size[1] = x->size[1];
  emxEnsureCapacity_creal_T(y, b_y);
  y_data = y->data;
  if (nfft > x->size[0]) {
    b_y = y->size[0] * y->size[1];
    y->size[0] = nfft;
    y->size[1] = x->size[1];
    emxEnsureCapacity_creal_T(y, b_y);
    y_data = y->data;
    b_y = nfft * x->size[1];
    for (k = 0; k < b_y; k++) {
      y_data[k].re = 0.0;
      y_data[k].im = 0.0;
    }
  }
  b_y = x->size[1];
#pragma omp parallel num_threads(omp_get_max_threads()) private(               \
        r, fy_data, fv_data, fv, fy, r1, xoff, istart, b_k, minNrowsNx,        \
            temp_re, temp_im, iy, re, im, iDelta, nRowsD2, c_k, tst, iDelta2,  \
            iheight, i, twid_re, twid_im)
  {
    emxInit_creal_T(&fv, 1);
    emxInit_creal_T(&fy, 1);
    emxInit_creal_T(&r1, 1);
#pragma omp for nowait
    for (chan = 0; chan < b_y; chan++) {
      xoff = chan * rt;
      istart = r1->size[0];
      r1->size[0] = nfft;
      emxEnsureCapacity_creal_T(r1, istart);
      r = r1->data;
      if (nfft > x->size[0]) {
        istart = r1->size[0];
        r1->size[0] = nfft;
        emxEnsureCapacity_creal_T(r1, istart);
        r = r1->data;
        for (b_k = 0; b_k < nfft; b_k++) {
          r[b_k].re = 0.0;
          r[b_k].im = 0.0;
        }
      }
      minNrowsNx = x->size[0];
      if (nfft <= minNrowsNx) {
        minNrowsNx = nfft;
      }
      for (b_k = 0; b_k < minNrowsNx; b_k++) {
        istart = (nfft + b_k) - 1;
        temp_re = wwc_data[istart].re;
        temp_im = wwc_data[istart].im;
        iy = xoff + b_k;
        re = x_data[iy].im;
        im = x_data[iy].re;
        r[b_k].re = temp_re * im + temp_im * re;
        r[b_k].im = temp_re * re - temp_im * im;
      }
      istart = minNrowsNx + 1;
      for (b_k = istart; b_k <= nfft; b_k++) {
        r[b_k - 1].re = 0.0;
        r[b_k - 1].im = 0.0;
      }
      istart = fy->size[0];
      fy->size[0] = n2blue;
      emxEnsureCapacity_creal_T(fy, istart);
      fy_data = fy->data;
      if (n2blue > r1->size[0]) {
        istart = fy->size[0];
        fy->size[0] = n2blue;
        emxEnsureCapacity_creal_T(fy, istart);
        fy_data = fy->data;
        for (b_k = 0; b_k < n2blue; b_k++) {
          fy_data[b_k].re = 0.0;
          fy_data[b_k].im = 0.0;
        }
      }
      istart = r1->size[0];
      xoff = n2blue;
      if (istart <= n2blue) {
        xoff = istart;
      }
      iDelta = n2blue - 2;
      nRowsD2 = (int)((unsigned int)n2blue >> 1);
      c_k = (int)((unsigned int)nRowsD2 >> 1);
      iy = 0;
      minNrowsNx = 0;
      for (b_k = 0; b_k <= xoff - 2; b_k++) {
        fy_data[iy] = r[b_k];
        istart = n2blue;
        tst = true;
        while (tst) {
          istart >>= 1;
          minNrowsNx ^= istart;
          tst = ((minNrowsNx & istart) == 0);
        }
        iy = minNrowsNx;
      }
      if (xoff - 2 < 0) {
        istart = 0;
      } else {
        istart = xoff - 1;
      }
      fy_data[iy] = r[istart];
      if (n2blue > 1) {
        for (b_k = 0; b_k <= iDelta; b_k += 2) {
          temp_re = fy_data[b_k + 1].re;
          temp_im = fy_data[b_k + 1].im;
          re = fy_data[b_k].re;
          im = fy_data[b_k].im;
          fy_data[b_k + 1].re = re - temp_re;
          fy_data[b_k + 1].im = fy_data[b_k].im - fy_data[b_k + 1].im;
          re += temp_re;
          im += temp_im;
          fy_data[b_k].re = re;
          fy_data[b_k].im = im;
        }
      }
      iDelta = 2;
      iDelta2 = 4;
      iheight = ((c_k - 1) << 2) + 1;
      while (c_k > 0) {
        for (i = 0; i < iheight; i += iDelta2) {
          istart = i + iDelta;
          temp_re = fy_data[istart].re;
          temp_im = fy_data[istart].im;
          fy_data[istart].re = fy_data[i].re - temp_re;
          fy_data[istart].im = fy_data[i].im - temp_im;
          fy_data[i].re += temp_re;
          fy_data[i].im += temp_im;
        }
        istart = 1;
        for (minNrowsNx = c_k; minNrowsNx < nRowsD2; minNrowsNx += c_k) {
          twid_re = costab_data[minNrowsNx];
          twid_im = sintab_data[minNrowsNx];
          i = istart;
          xoff = istart + iheight;
          while (i < xoff) {
            iy = i + iDelta;
            re = fy_data[iy].im;
            im = fy_data[iy].re;
            temp_re = twid_re * im - twid_im * re;
            temp_im = twid_re * re + twid_im * im;
            fy_data[iy].re = fy_data[i].re - temp_re;
            fy_data[iy].im = fy_data[i].im - temp_im;
            fy_data[i].re += temp_re;
            fy_data[i].im += temp_im;
            i += iDelta2;
          }
          istart++;
        }
        c_k = (int)((unsigned int)c_k >> 1);
        iDelta = iDelta2;
        iDelta2 += iDelta2;
        iheight -= iDelta;
      }
      d_FFTImplementationCallback_r2b(wwc, n2blue, costab, sintab, fv);
      fv_data = fv->data;
      istart = fy->size[0];
      for (b_k = 0; b_k < istart; b_k++) {
        re = fy_data[b_k].re;
        im = fv_data[b_k].im;
        twid_re = fy_data[b_k].im;
        twid_im = fv_data[b_k].re;
        fy_data[b_k].re = re * twid_im - twid_re * im;
        fy_data[b_k].im = re * im + twid_re * twid_im;
      }
      d_FFTImplementationCallback_r2b(fy, n2blue, costab, sintabinv, fv);
      fv_data = fv->data;
      if (fv->size[0] > 1) {
        re = 1.0 / (double)fv->size[0];
        istart = fv->size[0];
        for (b_k = 0; b_k < istart; b_k++) {
          fv_data[b_k].re *= re;
          fv_data[b_k].im *= re;
        }
      }
      istart = wwc->size[0];
      for (b_k = nfft; b_k <= istart; b_k++) {
        re = wwc_data[b_k - 1].re;
        im = fv_data[b_k - 1].im;
        twid_re = wwc_data[b_k - 1].im;
        twid_im = fv_data[b_k - 1].re;
        iy = b_k - nfft;
        r[iy].re = re * twid_im + twid_re * im;
        r[iy].im = re * im - twid_re * twid_im;
      }
      istart = y->size[0];
      for (b_k = 0; b_k < istart; b_k++) {
        y_data[b_k + y->size[0] * chan] = r[b_k];
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
  double im;
  double re;
  double temp_im;
  double temp_re;
  double twid_im;
  double twid_re;
  int b_i;
  int c_i;
  int chan;
  int i;
  int iDelta2;
  int iheight;
  int ihi;
  int iy;
  int ju;
  int k;
  int loop_ub;
  int nRowsD2;
  int nRowsM2;
  int nrows;
  int xoff;
  bool tst;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  nrows = x->size[0];
  loop_ub = y->size[0] * y->size[1];
  y->size[0] = n1_unsigned;
  y->size[1] = x->size[1];
  emxEnsureCapacity_creal_T(y, loop_ub);
  y_data = y->data;
  if (n1_unsigned > x->size[0]) {
    loop_ub = y->size[0] * y->size[1];
    y->size[0] = n1_unsigned;
    y->size[1] = x->size[1];
    emxEnsureCapacity_creal_T(y, loop_ub);
    y_data = y->data;
    loop_ub = n1_unsigned * x->size[1];
    for (i = 0; i < loop_ub; i++) {
      y_data[i].re = 0.0;
      y_data[i].im = 0.0;
    }
  }
  loop_ub = x->size[1];
#pragma omp parallel num_threads(omp_get_max_threads()) private(               \
        r, r1, xoff, iy, b_i, ihi, nRowsM2, nRowsD2, k, ju, tst, temp_re,      \
            temp_im, re, im, iDelta2, iheight, c_i, twid_re, twid_im)
  {
    emxInit_creal_T(&r1, 1);
#pragma omp for nowait
    for (chan = 0; chan < loop_ub; chan++) {
      xoff = chan * nrows;
      iy = r1->size[0];
      r1->size[0] = n1_unsigned;
      emxEnsureCapacity_creal_T(r1, iy);
      r = r1->data;
      if (n1_unsigned > x->size[0]) {
        iy = r1->size[0];
        r1->size[0] = n1_unsigned;
        emxEnsureCapacity_creal_T(r1, iy);
        r = r1->data;
        for (b_i = 0; b_i < n1_unsigned; b_i++) {
          r[b_i].re = 0.0;
          r[b_i].im = 0.0;
        }
      }
      iy = x->size[0];
      ihi = n1_unsigned;
      if (iy <= n1_unsigned) {
        ihi = iy;
      }
      nRowsM2 = n1_unsigned - 2;
      nRowsD2 = (int)((unsigned int)n1_unsigned >> 1);
      k = (int)((unsigned int)nRowsD2 >> 1);
      iy = 0;
      ju = 0;
      for (b_i = 0; b_i <= ihi - 2; b_i++) {
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
      if (ihi - 2 >= 0) {
        xoff = (xoff + ihi) - 1;
      }
      r[iy] = x_data[xoff];
      if (n1_unsigned > 1) {
        for (b_i = 0; b_i <= nRowsM2; b_i += 2) {
          temp_re = r[b_i + 1].re;
          temp_im = r[b_i + 1].im;
          re = r[b_i].re;
          im = r[b_i].im;
          r[b_i + 1].re = re - temp_re;
          r[b_i + 1].im = r[b_i].im - r[b_i + 1].im;
          re += temp_re;
          im += temp_im;
          r[b_i].re = re;
          r[b_i].im = im;
        }
      }
      xoff = 2;
      iDelta2 = 4;
      iheight = ((k - 1) << 2) + 1;
      while (k > 0) {
        for (c_i = 0; c_i < iheight; c_i += iDelta2) {
          iy = c_i + xoff;
          temp_re = r[iy].re;
          temp_im = r[iy].im;
          r[iy].re = r[c_i].re - temp_re;
          r[iy].im = r[c_i].im - temp_im;
          r[c_i].re += temp_re;
          r[c_i].im += temp_im;
        }
        iy = 1;
        for (ju = k; ju < nRowsD2; ju += k) {
          twid_re = costab_data[ju];
          twid_im = sintab_data[ju];
          c_i = iy;
          ihi = iy + iheight;
          while (c_i < ihi) {
            nRowsM2 = c_i + xoff;
            re = r[nRowsM2].im;
            im = r[nRowsM2].re;
            temp_re = twid_re * im - twid_im * re;
            temp_im = twid_re * re + twid_im * im;
            r[nRowsM2].re = r[c_i].re - temp_re;
            r[nRowsM2].im = r[c_i].im - temp_im;
            r[c_i].re += temp_re;
            r[c_i].im += temp_im;
            c_i += iDelta2;
          }
          iy++;
        }
        k = (int)((unsigned int)k >> 1);
        xoff = iDelta2;
        iDelta2 += iDelta2;
        iheight -= xoff;
      }
      iy = y->size[0];
      for (b_i = 0; b_i < iy; b_i++) {
        y_data[b_i + y->size[0] * chan] = r[b_i];
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
