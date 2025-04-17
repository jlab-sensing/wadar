/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: WIP_LiveFFT.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 02:59:58
 */

/* Include Files */
#include "WIP_LiveFFT.h"
#include "WIP_LiveFFT_data.h"
#include "WIP_LiveFFT_initialize.h"
#include "abs.h"
#include "fft.h"
#include "minOrMax.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static double framesBB[1024000];

static double n;

/* Function Definitions */
/*
 * Arguments    : const double newFrameBB[512]
 *                double *peakStrength
 *                double *peakLocation
 * Return Type  : void
 */
void WIP_LiveFFT(const double newFrameBB[512], double *peakStrength,
                 double *peakLocation)
{
  static creal_T dcv[1024000];
  static double b_dv[1024000];
  int iindx;
  if (!isInitialized_WIP_LiveFFT) {
    WIP_LiveFFT_initialize();
  }
  if (n > 2000.0) {
    n = 1.0;
  }
  for (iindx = 0; iindx < 512; iindx++) {
    framesBB[iindx + (((int)n - 1) << 9)] = newFrameBB[iindx];
  }
  *peakStrength = -1.0;
  *peakLocation = -1.0;
  if (b_mod(n) == 0.0) {
    double liveOutput[2000];
    fft(framesBB, dcv);
    b_abs(dcv, b_dv);
    maximum(b_dv, liveOutput);
    for (iindx = 0; iindx < 100; iindx++) {
      liveOutput[iindx] = 0.0;
      liveOutput[iindx + 1900] = 0.0;
    }
    *peakStrength = b_maximum(liveOutput, &iindx);
    *peakLocation = (double)iindx / 10.0;
  }
  n++;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void WIP_LiveFFT_init(void)
{
  n = 1.0;
  memset(&framesBB[0], 0, 1024000U * sizeof(double));
}

/*
 * File trailer for WIP_LiveFFT.c
 *
 * [EOF]
 */
