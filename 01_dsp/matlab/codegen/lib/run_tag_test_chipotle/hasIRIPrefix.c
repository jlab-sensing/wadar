/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: hasIRIPrefix.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
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
 * Return Type  : boolean_T
 */
boolean_T hasIRIPrefix(const emxArray_char_T *path)
{
  static const boolean_T bv[128] = {
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
  static const boolean_T bv1[128] = {
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
  int pthSz_tmp;
  const char *path_data;
  char *b_path_data;
  boolean_T b;
  path_data = path->data;
  pthSz_tmp = path->size[1];
  emxInit_char_T(&b_path);
  if (pthSz_tmp < 4) {
    b = false;
  } else if (!bv[(int)((unsigned char)path_data[0] & 127U)]) {
    b = false;
  } else if (bv1[(int)((unsigned char)path_data[1] & 127U)] ||
             (path_data[1] == '.') || (path_data[1] == '-') ||
             (path_data[1] == '+')) {
    int colIdx;
    int i;
    boolean_T guard1;
    colIdx = 0;
    i = 2;
    guard1 = false;
    int exitg1;
    do {
      exitg1 = 0;
      if (i + 1 <= pthSz_tmp) {
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
      } else if (colIdx + 1 >= pthSz_tmp) {
        b = false;
      } else if (path_data[colIdx] == '/') {
        b = true;
      } else {
        if (colIdx + 1 > path->size[1]) {
          colIdx = 0;
          pthSz_tmp = 0;
        }
        b_i = b_path->size[0] * b_path->size[1];
        b_path->size[0] = 1;
        i = pthSz_tmp - colIdx;
        b_path->size[1] = i;
        emxEnsureCapacity_char_T(b_path, b_i);
        b_path_data = b_path->data;
        for (b_i = 0; b_i < i; b_i++) {
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
