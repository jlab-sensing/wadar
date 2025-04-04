/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fileManager.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

#ifndef FILEMANAGER_H
#define FILEMANAGER_H

/* Include Files */
#include "RunTagDataEmbedded_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int cfclose(double fid);

signed char cfopen(const emxArray_char_T *cfilename);

int fileManager(void);

void filedata_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for fileManager.h
 *
 * [EOF]
 */
