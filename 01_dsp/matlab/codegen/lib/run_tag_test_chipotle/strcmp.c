/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: strcmp.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "strcmp.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const char b[3]
 * Return Type  : bool
 */
bool b_strcmp(const char b[3])
{
  static const char a[3] = {'4', 'm', 'm'};
  int ret;
  ret = memcmp(&a[0], &b[0], 3);
  return ret == 0;
}

/*
 * Arguments    : const char b[3]
 * Return Type  : bool
 */
bool c_strcmp(const char b[3])
{
  static const char a[3] = {'8', 'm', 'm'};
  int ret;
  ret = memcmp(&a[0], &b[0], 3);
  return ret == 0;
}

/*
 * Arguments    : const char b[3]
 * Return Type  : bool
 */
bool d_strcmp(const char b[3])
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
