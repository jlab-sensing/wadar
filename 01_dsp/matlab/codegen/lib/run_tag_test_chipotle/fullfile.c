/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: fullfile.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "fullfile.h"
#include "hasIRIPrefix.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include "coder_platform.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_char_T *varargin_1
 *                const char varargin_2_data[]
 *                const int varargin_2_size[2]
 *                emxArray_char_T *fOut
 * Return Type  : void
 */
void fullfile(const emxArray_char_T *varargin_1, const char varargin_2_data[],
              const int varargin_2_size[2], emxArray_char_T *fOut)
{
  static const char b_cv[4] = {'\\', '\\', '?', '\\'};
  static const char cv1[4] = {'\\', '\\', '.', '\\'};
  emxArray_char_T *fTemp;
  double k;
  int i;
  unsigned int j;
  int loop_ub;
  int t;
  const char *varargin_1_data;
  char c;
  char fs;
  char *fOut_data;
  char *fTemp_data;
  bool exitg1;
  bool guard1;
  bool isIRI;
  bool p;
  varargin_1_data = varargin_1->data;
  emxInit_char_T(&fTemp);
  isIRI = false;
  if (!hasIRIPrefix(varargin_1)) {
    t = coderIsPC();
    if (t != 0) {
      fs = '\\';
    } else {
      fs = '/';
    }
  } else {
    fs = '/';
    isIRI = true;
  }
  if (varargin_2_size[1] != 0) {
    guard1 = false;
    if (varargin_1->size[1] != 0) {
      c = varargin_1_data[varargin_1->size[1] - 1];
      if (c != fs) {
        t = coderIsPC();
        if ((t != 0) && (c == '/')) {
          guard1 = true;
        } else {
          t = fTemp->size[0] * fTemp->size[1];
          fTemp->size[0] = 1;
          fTemp->size[1] = varargin_1->size[1] + 1;
          emxEnsureCapacity_char_T(fTemp, t);
          fTemp_data = fTemp->data;
          t = varargin_1->size[1];
          for (i = 0; i < t; i++) {
            fTemp_data[i] = varargin_1_data[i];
          }
          fTemp_data[varargin_1->size[1]] = fs;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      t = fTemp->size[0] * fTemp->size[1];
      fTemp->size[0] = 1;
      loop_ub = varargin_1->size[1];
      fTemp->size[1] = varargin_1->size[1];
      emxEnsureCapacity_char_T(fTemp, t);
      fTemp_data = fTemp->data;
      for (i = 0; i < loop_ub; i++) {
        fTemp_data[i] = varargin_1_data[i];
      }
    }
  } else {
    t = fTemp->size[0] * fTemp->size[1];
    fTemp->size[0] = 1;
    loop_ub = varargin_1->size[1];
    fTemp->size[1] = varargin_1->size[1];
    emxEnsureCapacity_char_T(fTemp, t);
    fTemp_data = fTemp->data;
    for (i = 0; i < loop_ub; i++) {
      fTemp_data[i] = varargin_1_data[i];
    }
  }
  t = fOut->size[0] * fOut->size[1];
  fOut->size[0] = 1;
  loop_ub = fTemp->size[1] + varargin_2_size[1];
  fOut->size[1] = loop_ub;
  emxEnsureCapacity_char_T(fOut, t);
  fOut_data = fOut->data;
  t = fTemp->size[1];
  for (i = 0; i < t; i++) {
    fOut_data[i] = fTemp_data[i];
  }
  t = varargin_2_size[1];
  for (i = 0; i < t; i++) {
    fOut_data[i + fTemp->size[1]] = varargin_2_data[i];
  }
  if (fs != '/') {
    for (i = 0; i < loop_ub; i++) {
      if (fOut_data[i] == '/') {
        fOut_data[i] = fs;
      }
    }
  }
  t = fTemp->size[0] * fTemp->size[1];
  fTemp->size[0] = 1;
  fTemp->size[1] = loop_ub;
  emxEnsureCapacity_char_T(fTemp, t);
  fTemp_data = fTemp->data;
  for (i = 0; i < loop_ub; i++) {
    fTemp_data[i] = ' ';
  }
  k = 1.0;
  j = 1U;
  t = coderIsPC();
  guard1 = false;
  if ((t != 0) && (fOut->size[1] >= 4)) {
    p = true;
    t = 0;
    exitg1 = false;
    while ((!exitg1) && (t < 4)) {
      if (fOut_data[t] != b_cv[t]) {
        p = false;
        exitg1 = true;
      } else {
        t++;
      }
    }
    if (!p) {
      p = true;
      t = 0;
      exitg1 = false;
      while ((!exitg1) && (t < 4)) {
        if (fOut_data[t] != cv1[t]) {
          p = false;
          exitg1 = true;
        } else {
          t++;
        }
      }
      if (p) {
        fTemp_data[0] = fOut_data[0];
        fTemp_data[1] = fOut_data[1];
        fTemp_data[2] = fOut_data[2];
        fTemp_data[3] = fOut_data[3];
        j = 4U;
        k = 4.0;
      }
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    while (j <= (unsigned int)fOut->size[1]) {
      c = fOut_data[(int)j - 1];
      if (c == fs) {
        j++;
        while ((j < (unsigned int)fOut->size[1]) &&
               (fOut_data[(int)j - 1] == '.') && (fOut_data[(int)j] == fs)) {
          j += 2U;
        }
        fTemp_data[(int)k - 1] = c;
      } else {
        fTemp_data[(int)k - 1] = c;
        j++;
      }
      k++;
    }
    if (k - 1.0 < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = (int)(k - 1.0);
    }
    t = fOut->size[0] * fOut->size[1];
    fOut->size[0] = 1;
    fOut->size[1] = loop_ub;
    emxEnsureCapacity_char_T(fOut, t);
    fOut_data = fOut->data;
    for (i = 0; i < loop_ub; i++) {
      fOut_data[i] = fTemp_data[i];
    }
  }
  if (!isIRI) {
    j = 1U;
    t = fTemp->size[0] * fTemp->size[1];
    fTemp->size[0] = 1;
    loop_ub = fOut->size[1];
    fTemp->size[1] = fOut->size[1];
    emxEnsureCapacity_char_T(fTemp, t);
    fTemp_data = fTemp->data;
    for (i = 0; i < loop_ub; i++) {
      fTemp_data[i] = ' ';
    }
    k = 1.0;
    t = coderIsPC();
    guard1 = false;
    if (t != 0) {
      if (fOut_data[0] == '\\') {
        j = 2U;
        k = 2.0;
        fTemp_data[0] = fOut_data[0];
        guard1 = true;
      } else if (fOut->size[1] >= 4) {
        p = true;
        t = 0;
        exitg1 = false;
        while ((!exitg1) && (t < 4)) {
          if (fOut_data[t] != b_cv[t]) {
            p = false;
            exitg1 = true;
          } else {
            t++;
          }
        }
        if (!p) {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      while (j <= (unsigned int)fOut->size[1]) {
        c = fOut_data[(int)j - 1];
        if (c == fs) {
          j++;
          while ((j <= (unsigned int)fOut->size[1]) &&
                 (fOut_data[(int)j - 1] == fs)) {
            j++;
          }
          fTemp_data[(int)k - 1] = fs;
        } else {
          fTemp_data[(int)k - 1] = c;
          j++;
        }
        k++;
      }
      if (k - 1.0 < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = (int)(k - 1.0);
      }
      t = fOut->size[0] * fOut->size[1];
      fOut->size[0] = 1;
      fOut->size[1] = loop_ub;
      emxEnsureCapacity_char_T(fOut, t);
      fOut_data = fOut->data;
      for (i = 0; i < loop_ub; i++) {
        fOut_data[i] = fTemp_data[i];
      }
    }
  }
  emxFree_char_T(&fTemp);
}

/*
 * File trailer for fullfile.c
 *
 * [EOF]
 */
