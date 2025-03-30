/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: WIP_LiveFFT.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 02:59:58
 */

#ifndef WIP_LIVEFFT_H
#define WIP_LIVEFFT_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void WIP_LiveFFT(const double newFrameBB[512], double *peakStrength,
                        double *peakLocation);

void WIP_LiveFFT_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for WIP_LiveFFT.h
 *
 * [EOF]
 */
