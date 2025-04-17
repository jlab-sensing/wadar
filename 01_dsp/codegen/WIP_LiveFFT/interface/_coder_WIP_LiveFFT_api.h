/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_WIP_LiveFFT_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 02:59:58
 */

#ifndef _CODER_WIP_LIVEFFT_API_H
#define _CODER_WIP_LIVEFFT_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void WIP_LiveFFT(real_T newFrameBB[512], real_T *peakStrength,
                 real_T *peakLocation);

void WIP_LiveFFT_api(const mxArray *prhs, int32_T nlhs, const mxArray *plhs[2]);

void WIP_LiveFFT_atexit(void);

void WIP_LiveFFT_initialize(void);

void WIP_LiveFFT_terminate(void);

void WIP_LiveFFT_xil_shutdown(void);

void WIP_LiveFFT_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_WIP_LiveFFT_api.h
 *
 * [EOF]
 */
