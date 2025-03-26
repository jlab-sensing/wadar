/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: strcmp.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "strcmp.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const char b[3]
 * Return Type  : boolean_T
 */
boolean_T b_strcmp(const char b[3])
{
  static const char a[3] = {'4', 'm', 'm'};
  int ret;
  ret = memcmp(&a[0], &b[0], 3);
  return ret == 0;
}

/*
 * Arguments    : const char b[3]
 * Return Type  : boolean_T
 */
boolean_T c_strcmp(const char b[3])
{
  static const char a[3] = {'8', 'm', 'm'};
  int ret;
  ret = memcmp(&a[0], &b[0], 3);
  return ret == 0;
}

/*
 * Arguments    : const char b[3]
 * Return Type  : boolean_T
 */
boolean_T d_strcmp(const char b[3])
{
  static const char a[3] = {'4', 'c', 'm'};
  int ret;
  ret = memcmp(&a[0], &b[0], 3);
  return ret == 0;
}

/*
 * File trailer for strcmp.c
 *
 * [EOF]
 */
