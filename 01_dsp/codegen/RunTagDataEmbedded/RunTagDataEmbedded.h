/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RunTagDataEmbedded.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

#ifndef RUNTAGDATAEMBEDDED_H
#define RUNTAGDATAEMBEDDED_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void RunTagDataEmbedded(const char captureName_data[],
                               const int captureName_size[2],
                               const char localDataPath_data[],
                               const int localDataPath_size[2], double tagHz,
                               double *peakBin, double *SNRdB);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for RunTagDataEmbedded.h
 *
 * [EOF]
 */
