/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: WIP_LiveFFT.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 04:43:50
 */

/* Include Files */
#include "WIP_LiveFFT.h"
#include "abs.h"
#include "fft.h"
#include "minOrMax.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double frameTotFlat[1024000]
 *                double outFFT[2000]
 * Return Type  : void
 */
void WIP_LiveFFT(const double frameTotFlat[1024000], double outFFT[2000])
{
  static creal_T dcv[1024000];
  static double b_dv[1024000];
  int i;
  fft(frameTotFlat, dcv);
  b_abs(dcv, b_dv);
  maximum(b_dv, outFFT);
  for (i = 0; i < 100; i++) {
    outFFT[i] = 0.0;
    outFFT[i + 1900] = 0.0;
  }
}

/*
 * File trailer for WIP_LiveFFT.c
 *
 * [EOF]
 */
