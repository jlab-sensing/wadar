/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_RunTagDataEmbedded_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

#ifndef _CODER_RUNTAGDATAEMBEDDED_API_H
#define _CODER_RUNTAGDATAEMBEDDED_API_H

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
void RunTagDataEmbedded(char_T captureName_data[], int32_T captureName_size[2],
                        char_T localDataPath_data[],
                        int32_T localDataPath_size[2], real_T tagHz,
                        real_T *peakBin, real_T *SNRdB);

void RunTagDataEmbedded_api(const mxArray *const prhs[3], int32_T nlhs,
                            const mxArray *plhs[2]);

void RunTagDataEmbedded_atexit(void);

void RunTagDataEmbedded_initialize(void);

void RunTagDataEmbedded_terminate(void);

void RunTagDataEmbedded_xil_shutdown(void);

void RunTagDataEmbedded_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_RunTagDataEmbedded_api.h
 *
 * [EOF]
 */
