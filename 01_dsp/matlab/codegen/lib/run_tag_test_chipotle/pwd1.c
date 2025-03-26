/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: pwd1.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "pwd1.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include "coder_fileops.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_char_T *wd
 * Return Type  : void
 */
void pwd(emxArray_char_T *wd)
{
  emxArray_char_T *wdTemp;
  int fileNameMax;
  int i;
  char *wdTemp_data;
  char *wd_data;
  fileNameMax = coderGetLenghtOfCwd() + 1;
  emxInit_char_T(&wdTemp);
  i = wdTemp->size[0] * wdTemp->size[1];
  wdTemp->size[0] = 1;
  wdTemp->size[1] = fileNameMax;
  emxEnsureCapacity_char_T(wdTemp, i);
  wdTemp_data = wdTemp->data;
  fileNameMax = coderGetCurrentDirectory(&wdTemp_data[0], fileNameMax);
  if (fileNameMax < 1) {
    fileNameMax = 0;
  }
  i = wd->size[0] * wd->size[1];
  wd->size[0] = 1;
  wd->size[1] = fileNameMax;
  emxEnsureCapacity_char_T(wd, i);
  wd_data = wd->data;
  for (i = 0; i < fileNameMax; i++) {
    wd_data[i] = wdTemp_data[i];
  }
  emxFree_char_T(&wdTemp);
}

/*
 * File trailer for pwd1.c
 *
 * [EOF]
 */
