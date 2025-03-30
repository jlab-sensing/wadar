/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fft.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 04:43:50
 */

/* Include Files */
#include "fft.h"
#include "FFTImplementationCallback.h"
#include "WIP_LiveFFT_data.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double x[1024000]
 *                creal_T y[1024000]
 * Return Type  : void
 */
void fft(const double x[1024000], creal_T y[1024000])
{
  static creal_T b_y[1024000];
  static double b_x[1024000];
  creal_T wwc[1999];
  int k;
  int rt;
  for (rt = 0; rt < 512; rt++) {
    for (k = 0; k < 2000; k++) {
      b_x[k + 2000 * rt] = x[rt + (k << 9)];
    }
  }
  rt = 0;
  wwc[999].re = 1.0;
  wwc[999].im = 0.0;
  for (k = 0; k < 999; k++) {
    double nt_im;
    int c_y;
    c_y = ((k + 1) << 1) - 1;
    if (2000 - rt <= c_y) {
      rt = (c_y + rt) - 2000;
    } else {
      rt += c_y;
    }
    nt_im = -3.1415926535897931 * (double)rt / 1000.0;
    wwc[998 - k].re = cos(nt_im);
    wwc[998 - k].im = -sin(nt_im);
  }
  for (k = 998; k >= 0; k--) {
    wwc[k + 1000] = wwc[998 - k];
  }
  for (rt = 0; rt < 512; rt++) {
    c_FFTImplementationCallback_doH(b_x, rt * 2000, &b_y[2000 * rt], wwc, dv);
  }
  for (rt = 0; rt < 2000; rt++) {
    for (k = 0; k < 512; k++) {
      y[k + (rt << 9)] = b_y[rt + 2000 * k];
    }
  }
}

/*
 * File trailer for fft.c
 *
 * [EOF]
 */
