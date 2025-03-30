/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: WIP_LiveFFT_initialize.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 30-Mar-2025 02:59:58
 */

/* Include Files */
#include "WIP_LiveFFT_initialize.h"
#include "WIP_LiveFFT.h"
#include "WIP_LiveFFT_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void WIP_LiveFFT_initialize(void)
{
  WIP_LiveFFT_init();
  isInitialized_WIP_LiveFFT = true;
}

/*
 * File trailer for WIP_LiveFFT_initialize.c
 *
 * [EOF]
 */
