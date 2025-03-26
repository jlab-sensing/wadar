/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: pwd1.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
  int len;
  char *wdTemp_data;
  char *wd_data;
  fileNameMax = coderGetLenghtOfCwd() + 1;
  emxInit_char_T(&wdTemp);
  len = wdTemp->size[0] * wdTemp->size[1];
  wdTemp->size[0] = 1;
  wdTemp->size[1] = fileNameMax;
  emxEnsureCapacity_char_T(wdTemp, len);
  wdTemp_data = wdTemp->data;
  len = coderGetCurrentDirectory(&wdTemp_data[0], fileNameMax);
  if (len < 1) {
    len = 0;
  }
  fileNameMax = wd->size[0] * wd->size[1];
  wd->size[0] = 1;
  wd->size[1] = len;
  emxEnsureCapacity_char_T(wd, fileNameMax);
  wd_data = wd->data;
  for (i = 0; i < len; i++) {
    wd_data[i] = wdTemp_data[i];
  }
  emxFree_char_T(&wdTemp);
}

/*
 * File trailer for pwd1.c
 *
 * [EOF]
 */
