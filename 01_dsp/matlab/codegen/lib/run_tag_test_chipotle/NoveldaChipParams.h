/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: NoveldaChipParams.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

#ifndef NOVELDACHIPPARAMS_H
#define NOVELDACHIPPARAMS_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double NoveldaChipParams(const char chipSet_Value_data[],
                         const int chipSet_Value_size[2],
                         const double PGen_data[], int PGen_size, double *bw,
                         double *bwr, double *vp, double *n, double *bw_hz,
                         double *pwr_dBm, double *fs_hz);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for NoveldaChipParams.h
 *
 * [EOF]
 */
