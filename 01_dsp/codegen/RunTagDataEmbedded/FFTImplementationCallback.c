/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: FFTImplementationCallback.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "FFTImplementationCallback.h"
#include "RunTagDataEmbedded_emxutil.h"
#include "RunTagDataEmbedded_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void c_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
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
static void c_FFTImplementationCallback_r2b(const emxArray_creal_T *x,
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
    bool tst;
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
  emxArray_creal_T *r;
  emxArray_creal_T *wwc;
  const creal_T *x_data;
  creal_T *fv_data;
  creal_T *fy_data;
  creal_T *r1;
  creal_T *wwc_data;
  creal_T *y_data;
  const double *costab_data;
  const double *sintab_data;
  double nt_im;
  int b_i;
  int chan;
  int i;
  int i1;
  int istart;
  int j;
  int k;
  int minNrowsNx;
  int nChan_tmp;
  int nInt2;
  int nRowsD2;
  int nRowsD4 = 0;
  int nRowsM2;
  int rt;
  int xidx;
  sintab_data = sintab->data;
  costab_data = costab->data;
  x_data = x->data;
  nChan_tmp = x->size[1];
  istart = (nfft + nfft) - 1;
  emxInit_creal_T(&wwc, 1);
  j = wwc->size[0];
  wwc->size[0] = istart;
  emxEnsureCapacity_creal_T(wwc, j);
  wwc_data = wwc->data;
  rt = 0;
  wwc_data[nfft - 1].re = 1.0;
  wwc_data[nfft - 1].im = 0.0;
  nInt2 = nfft << 1;
  for (k = 0; k <= nfft - 2; k++) {
    xidx = ((k + 1) << 1) - 1;
    if (nInt2 - rt <= xidx) {
      rt += xidx - nInt2;
    } else {
      rt += xidx;
    }
    nt_im = -3.1415926535897931 * (double)rt / (double)nfft;
    j = (nfft - k) - 2;
    wwc_data[j].re = cos(nt_im);
    wwc_data[j].im = -sin(nt_im);
  }
  j = istart - 1;
  for (k = j; k >= nfft; k--) {
    wwc_data[k] = wwc_data[(istart - k) - 1];
  }
  j = y->size[0] * y->size[1];
  y->size[0] = nfft;
  y->size[1] = x->size[1];
  emxEnsureCapacity_creal_T(y, j);
  y_data = y->data;
  if (nfft > x->size[0]) {
    j = y->size[0] * y->size[1];
    y->size[0] = nfft;
    y->size[1] = x->size[1];
    emxEnsureCapacity_creal_T(y, j);
    y_data = y->data;
    xidx = nfft * x->size[1];
    for (j = 0; j < xidx; j++) {
      y_data[j].re = 0.0;
      y_data[j].im = 0.0;
    }
  }
  if (x->size[1] - 1 >= 0) {
    minNrowsNx = x->size[0];
    if (nfft <= minNrowsNx) {
      minNrowsNx = nfft;
    }
    i = minNrowsNx + 1;
    nRowsM2 = n2blue - 2;
    nRowsD2 = (int)((unsigned int)n2blue >> 1);
    nRowsD4 = nRowsD2 / 2;
    i1 = istart;
  }
  emxInit_creal_T(&r, 1);
  emxInit_creal_T(&fy, 1);
  emxInit_creal_T(&fv, 1);
  for (chan = 0; chan < nChan_tmp; chan++) {
    double b_nt_re_tmp;
    double im;
    double nt_re;
    double re_tmp;
    double twid_im;
    double twid_re;
    int nt_re_tmp;
    j = r->size[0];
    r->size[0] = nfft;
    emxEnsureCapacity_creal_T(r, j);
    r1 = r->data;
    if (nfft > x->size[0]) {
      j = r->size[0];
      r->size[0] = nfft;
      emxEnsureCapacity_creal_T(r, j);
      r1 = r->data;
      for (j = 0; j < nfft; j++) {
        r1[j].re = 0.0;
        r1[j].im = 0.0;
      }
    }
    xidx = chan * x->size[0];
    for (k = 0; k < minNrowsNx; k++) {
      nt_re_tmp = (nfft + k) - 1;
      nt_re = wwc_data[nt_re_tmp].re;
      nt_im = wwc_data[nt_re_tmp].im;
      j = xidx + k;
      twid_im = x_data[j].im;
      re_tmp = x_data[j].re;
      r1[k].re = nt_re * re_tmp + nt_im * twid_im;
      r1[k].im = nt_re * twid_im - nt_im * re_tmp;
    }
    for (k = i; k <= nfft; k++) {
      r1[k - 1].re = 0.0;
      r1[k - 1].im = 0.0;
    }
    j = fy->size[0];
    fy->size[0] = n2blue;
    emxEnsureCapacity_creal_T(fy, j);
    fy_data = fy->data;
    if (n2blue > r->size[0]) {
      j = fy->size[0];
      fy->size[0] = n2blue;
      emxEnsureCapacity_creal_T(fy, j);
      fy_data = fy->data;
      for (j = 0; j < n2blue; j++) {
        fy_data[j].re = 0.0;
        fy_data[j].im = 0.0;
      }
    }
    xidx = r->size[0];
    nInt2 = n2blue;
    if (xidx <= n2blue) {
      nInt2 = xidx;
    }
    xidx = 0;
    rt = 0;
    for (b_i = 0; b_i <= nInt2 - 2; b_i++) {
      bool tst;
      fy_data[xidx] = r1[b_i];
      xidx = n2blue;
      tst = true;
      while (tst) {
        xidx >>= 1;
        rt ^= xidx;
        tst = ((rt & xidx) == 0);
      }
      xidx = rt;
    }
    if (nInt2 - 2 < 0) {
      istart = 0;
    } else {
      istart = nInt2 - 1;
    }
    fy_data[xidx] = r1[istart];
    if (n2blue > 1) {
      for (b_i = 0; b_i <= nRowsM2; b_i += 2) {
        b_nt_re_tmp = fy_data[b_i + 1].re;
        nt_im = fy_data[b_i + 1].im;
        re_tmp = fy_data[b_i].re;
        im = fy_data[b_i].im;
        fy_data[b_i + 1].re = re_tmp - b_nt_re_tmp;
        fy_data[b_i + 1].im = fy_data[b_i].im - fy_data[b_i + 1].im;
        im += nt_im;
        fy_data[b_i].re = re_tmp + b_nt_re_tmp;
        fy_data[b_i].im = im;
      }
    }
    xidx = 2;
    rt = 4;
    k = nRowsD4;
    nInt2 = ((nRowsD4 - 1) << 2) + 1;
    while (k > 0) {
      for (b_i = 0; b_i < nInt2; b_i += rt) {
        nt_re_tmp = b_i + xidx;
        nt_re = fy_data[nt_re_tmp].re;
        nt_im = fy_data[nt_re_tmp].im;
        fy_data[nt_re_tmp].re = fy_data[b_i].re - nt_re;
        fy_data[nt_re_tmp].im = fy_data[b_i].im - nt_im;
        fy_data[b_i].re += nt_re;
        fy_data[b_i].im += nt_im;
      }
      istart = 1;
      for (j = k; j < nRowsD2; j += k) {
        int ihi;
        twid_re = costab_data[j];
        twid_im = sintab_data[j];
        b_i = istart;
        ihi = istart + nInt2;
        while (b_i < ihi) {
          nt_re_tmp = b_i + xidx;
          b_nt_re_tmp = fy_data[nt_re_tmp].im;
          im = fy_data[nt_re_tmp].re;
          nt_re = twid_re * im - twid_im * b_nt_re_tmp;
          nt_im = twid_re * b_nt_re_tmp + twid_im * im;
          fy_data[nt_re_tmp].re = fy_data[b_i].re - nt_re;
          fy_data[nt_re_tmp].im = fy_data[b_i].im - nt_im;
          fy_data[b_i].re += nt_re;
          fy_data[b_i].im += nt_im;
          b_i += rt;
        }
        istart++;
      }
      k /= 2;
      xidx = rt;
      rt += rt;
      nInt2 -= xidx;
    }
    c_FFTImplementationCallback_r2b(wwc, n2blue, costab, sintab, fv);
    fv_data = fv->data;
    xidx = fy->size[0];
    for (j = 0; j < xidx; j++) {
      re_tmp = fy_data[j].re;
      im = fv_data[j].im;
      twid_re = fy_data[j].im;
      twid_im = fv_data[j].re;
      fy_data[j].re = re_tmp * twid_im - twid_re * im;
      fy_data[j].im = re_tmp * im + twid_re * twid_im;
    }
    c_FFTImplementationCallback_r2b(fy, n2blue, costab, sintabinv, fv);
    fv_data = fv->data;
    if (fv->size[0] > 1) {
      im = 1.0 / (double)fv->size[0];
      xidx = fv->size[0];
      for (j = 0; j < xidx; j++) {
        fv_data[j].re *= im;
        fv_data[j].im *= im;
      }
    }
    for (k = nfft; k <= i1; k++) {
      twid_im = wwc_data[k - 1].re;
      re_tmp = fv_data[k - 1].im;
      im = wwc_data[k - 1].im;
      twid_re = fv_data[k - 1].re;
      j = k - nfft;
      r1[j].re = twid_im * twid_re + im * re_tmp;
      r1[j].im = twid_im * re_tmp - im * twid_re;
    }
    xidx = y->size[0];
    for (j = 0; j < xidx; j++) {
      y_data[j + y->size[0] * chan] = r1[j];
    }
  }
  emxFree_creal_T(&fv);
  emxFree_creal_T(&fy);
  emxFree_creal_T(&r);
  emxFree_creal_T(&wwc);
}

/*
 * File trailer for FFTImplementationCallback.c
 *
 * [EOF]
 */
