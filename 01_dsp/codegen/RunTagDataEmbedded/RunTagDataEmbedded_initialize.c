/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RunTagDataEmbedded_initialize.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "RunTagDataEmbedded_initialize.h"
#include "RunTagDataEmbedded_data.h"
#include "fileManager.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void RunTagDataEmbedded_initialize(void)
{
  filedata_init();
  isInitialized_RunTagDataEmbedded = true;
}

/*
 * File trailer for RunTagDataEmbedded_initialize.c
 *
 * [EOF]
 */
