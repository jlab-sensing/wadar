/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: string1.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "string1.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_data.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const char obj2_Value_data[]
 *                const int obj2_Value_size[2]
 * Return Type  : bool
 */
bool b_string_eq(const char obj2_Value_data[], const int obj2_Value_size[2])
{
  static const char b_cv[7] = {'x', '1', '-', 'i', 'p', 'g', '1'};
  bool equal;
  equal = false;
  if (obj2_Value_size[1] == 7) {
    int kstr;
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 7) {
        if (b_cv[kstr] != obj2_Value_data[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        equal = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return equal;
}

/*
 * Arguments    : const char obj2_Value_data[]
 *                const int obj2_Value_size[2]
 * Return Type  : bool
 */
bool c_string_eq(const char obj2_Value_data[], const int obj2_Value_size[2])
{
  static const char b_cv[2] = {'x', '2'};
  bool equal;
  equal = false;
  if (obj2_Value_size[1] == 2) {
    int kstr;
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 2) {
        if (b_cv[kstr] != obj2_Value_data[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        equal = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return equal;
}

/*
 * Arguments    : const char obj2_Value_data[]
 *                const int obj2_Value_size[2]
 * Return Type  : bool
 */
bool d_string_eq(const char obj2_Value_data[], const int obj2_Value_size[2])
{
  static const char b_cv[2] = {'x', '4'};
  bool equal;
  equal = false;
  if (obj2_Value_size[1] == 2) {
    int kstr;
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 2) {
        if (b_cv[kstr] != obj2_Value_data[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        equal = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return equal;
}

/*
 * Arguments    : const char obj2_Value_data[]
 *                const int obj2_Value_size[2]
 * Return Type  : bool
 */
bool string_eq(const char obj2_Value_data[], const int obj2_Value_size[2])
{
  static const char b_cv[7] = {'x', '1', '-', 'i', 'p', 'g', '0'};
  bool equal;
  equal = false;
  if (obj2_Value_size[1] == 7) {
    int kstr;
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 7) {
        if (b_cv[kstr] != obj2_Value_data[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        equal = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return equal;
}

/*
 * Arguments    : const char obj_Value_data[]
 *                const int obj_Value_size[2]
 *                char y_Value_data[]
 *                int y_Value_size[2]
 * Return Type  : void
 */
void string_lower(const char obj_Value_data[], const int obj_Value_size[2],
                  char y_Value_data[], int y_Value_size[2])
{
  int i;
  int k;
  y_Value_size[0] = 1;
  i = obj_Value_size[1];
  y_Value_size[1] = obj_Value_size[1];
  for (k = 0; k < i; k++) {
    y_Value_data[k] = cv[(int)obj_Value_data[k]];
  }
}

/*
 * File trailer for string1.c
 *
 * [EOF]
 */
