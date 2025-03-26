/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: run_tag_test_chipotle_terminate.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "run_tag_test_chipotle_terminate.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_data.h"
#include "omp.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void run_tag_test_chipotle_terminate(void)
{
  omp_destroy_nest_lock(&run_tag_test_chipotle_nestLockGlobal);
  isInitialized_run_tag_test_chipotle = false;
}

/*
 * File trailer for run_tag_test_chipotle_terminate.c
 *
 * [EOF]
 */
