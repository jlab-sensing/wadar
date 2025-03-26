/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: hasIRIPrefix.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "hasIRIPrefix.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_char_T *path
 * Return Type  : bool
 */
bool hasIRIPrefix(const emxArray_char_T *path)
{
  static const bool bv[128] = {
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, true,  true,  true,  true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
      true,  false, false, false, false, false, false, true,  true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
      true,  true,  true,  false, false, false, false, false};
  static const bool bv1[128] = {
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  false, false,
      false, false, false, false, false, true,  true,  true,  true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
      true,  false, false, false, false, false, false, true,  true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
      true,  true,  true,  true,  true,  true,  true,  true,  true,  true,
      true,  true,  true,  false, false, false, false, false};
  emxArray_char_T *b_path;
  int b_i;
  int pthSz;
  const char *path_data;
  char *b_path_data;
  bool b;
  path_data = path->data;
  pthSz = path->size[1];
  emxInit_char_T(&b_path);
  if (pthSz < 4) {
    b = false;
  } else if (!bv[(int)((unsigned char)path_data[0] & 127U)]) {
    b = false;
  } else if (bv1[(int)((unsigned char)path_data[1] & 127U)] ||
             (path_data[1] == '.') || (path_data[1] == '-') ||
             (path_data[1] == '+')) {
    int colIdx;
    int i;
    bool guard1;
    colIdx = 0;
    i = 2;
    guard1 = false;
    int exitg1;
    do {
      exitg1 = 0;
      if (i + 1 <= pthSz) {
        if (bv1[(int)((unsigned char)path_data[i] & 127U)] ||
            (path_data[i] == '.') || (path_data[i] == '-') ||
            (path_data[i] == '+')) {
          i++;
          guard1 = false;
        } else {
          if (path_data[i] != ':') {
            b = false;
          } else {
            colIdx = i + 1;
            guard1 = true;
          }
          exitg1 = 1;
        }
      } else {
        guard1 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
    if (guard1) {
      if (colIdx == 0) {
        b = false;
      } else if (colIdx + 1 >= pthSz) {
        b = false;
      } else if (path_data[colIdx] == '/') {
        b = true;
      } else {
        if (colIdx + 1 > path->size[1]) {
          colIdx = 0;
          pthSz = 0;
        }
        i = b_path->size[0] * b_path->size[1];
        b_path->size[0] = 1;
        pthSz -= colIdx;
        b_path->size[1] = pthSz;
        emxEnsureCapacity_char_T(b_path, i);
        b_path_data = b_path->data;
        for (b_i = 0; b_i < pthSz; b_i++) {
          b_path_data[b_i] = path_data[colIdx + b_i];
        }
        b = hasIRIPrefix(b_path);
      }
    }
  } else {
    b = false;
  }
  emxFree_char_T(&b_path);
  return b;
}

/*
 * File trailer for hasIRIPrefix.c
 *
 * [EOF]
 */
