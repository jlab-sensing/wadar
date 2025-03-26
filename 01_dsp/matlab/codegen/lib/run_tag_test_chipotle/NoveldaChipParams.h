/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: NoveldaChipParams.h
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
