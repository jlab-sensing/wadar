/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_RunTagDataEmbedded_mex.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

#ifndef _CODER_RUNTAGDATAEMBEDDED_MEX_H
#define _CODER_RUNTAGDATAEMBEDDED_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_RunTagDataEmbedded_mexFunction(int32_T nlhs, mxArray *plhs[2],
                                           int32_T nrhs,
                                           const mxArray *prhs[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_RunTagDataEmbedded_mex.h
 *
 * [EOF]
 */
