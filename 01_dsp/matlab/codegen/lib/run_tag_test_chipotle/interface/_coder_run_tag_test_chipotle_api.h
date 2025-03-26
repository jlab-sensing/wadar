/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_run_tag_test_chipotle_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

#ifndef _CODER_RUN_TAG_TEST_CHIPOTLE_API_H
#define _CODER_RUN_TAG_TEST_CHIPOTLE_API_H

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
void run_tag_test_chipotle(char_T captureName_data[],
                           int32_T captureName_size[2],
                           char_T localDataPath_data[],
                           int32_T localDataPath_size[2], real_T tagHz,
                           real_T *peakBin, real_T *SNRdB);

void run_tag_test_chipotle_api(const mxArray *const prhs[3], int32_T nlhs,
                               const mxArray *plhs[2]);

void run_tag_test_chipotle_atexit(void);

void run_tag_test_chipotle_initialize(void);

void run_tag_test_chipotle_terminate(void);

void run_tag_test_chipotle_xil_shutdown(void);

void run_tag_test_chipotle_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_run_tag_test_chipotle_api.h
 *
 * [EOF]
 */
