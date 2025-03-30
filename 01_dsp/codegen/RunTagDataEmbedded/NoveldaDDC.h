/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: NoveldaDDC.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

#ifndef NOVELDADDC_H
#define NOVELDADDC_H

/* Include Files */
#include "RunTagDataEmbedded_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void NoveldaDDC(const emxArray_real_T *rfSignal,
                const char chipSet_Value_data[],
                const int chipSet_Value_size[2], const double PGen_data[],
                int PGen_size, double Fs, emxArray_creal_T *basebandSignal);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for NoveldaDDC.h
 *
 * [EOF]
 */
