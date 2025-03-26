/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: smoothdata.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "smoothdata.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *A
 * Return Type  : void
 */
void smoothdata(emxArray_real_T *A)
{
  emxArray_real_T *Bd;
  double *A_data;
  double *Bd_data;
  int i;
  int ib;
  int ii;
  int k;
  A_data = A->data;
  emxInit_real_T(&Bd, 1);
  if (A->size[0] >= 2) {
    int b_tmp;
    int inds_data;
    int windowEnd;
    int windowStart;
    b_tmp = A->size[0];
    i = Bd->size[0];
    Bd->size[0] = b_tmp;
    emxEnsureCapacity_real_T(Bd, i);
    Bd_data = Bd->data;
    windowStart = 0;
    windowEnd = 1;
    i = A->size[0] - 1;
    for (ii = 0; ii <= i; ii++) {
      double y;
      int counts;
      int currentDimValue;
      int indexInDimCount;
      int lastBlockLength;
      boolean_T exitg1;
      while ((windowStart + 1 < ii + 1) && (ii - windowStart > 5)) {
        windowStart++;
      }
      if (windowEnd < ii + 1) {
        windowEnd = ii + 1;
      }
      while ((windowEnd < b_tmp) && (windowEnd - ii < 5)) {
        windowEnd++;
      }
      currentDimValue = 1;
      indexInDimCount = 1;
      lastBlockLength = 0;
      exitg1 = false;
      while ((!exitg1) && (lastBlockLength <= A->size[0] - 1)) {
        if (indexInDimCount > 1) {
          if (currentDimValue == b_tmp) {
            currentDimValue = 1;
          } else {
            currentDimValue++;
          }
        }
        if (currentDimValue == ii + 1) {
          inds_data = lastBlockLength + 1;
          exitg1 = true;
        } else {
          indexInDimCount = 2;
          lastBlockLength++;
        }
      }
      currentDimValue = windowEnd - windowStart;
      if (currentDimValue == 0) {
        y = 0.0;
        counts = 0;
      } else {
        int nblocks;
        int xoffset;
        if (currentDimValue <= 1024) {
          indexInDimCount = currentDimValue;
          lastBlockLength = 0;
          nblocks = 1;
        } else {
          indexInDimCount = 1024;
          nblocks = currentDimValue / 1024;
          lastBlockLength = currentDimValue - (nblocks << 10);
          if (lastBlockLength > 0) {
            nblocks++;
          } else {
            lastBlockLength = 1024;
          }
        }
        if (rtIsNaN(A_data[windowStart])) {
          y = 0.0;
          counts = 0;
        } else {
          y = A_data[windowStart];
          counts = 1;
        }
        for (k = 2; k <= indexInDimCount; k++) {
          xoffset = (windowStart + k) - 1;
          if (!rtIsNaN(A_data[xoffset])) {
            y += A_data[xoffset];
            counts++;
          }
        }
        for (ib = 2; ib <= nblocks; ib++) {
          double bsum;
          currentDimValue = windowStart + ((ib - 1) << 10);
          if (rtIsNaN(A_data[currentDimValue])) {
            bsum = 0.0;
          } else {
            bsum = A_data[currentDimValue];
            counts++;
          }
          if (ib == nblocks) {
            indexInDimCount = lastBlockLength;
          } else {
            indexInDimCount = 1024;
          }
          for (k = 2; k <= indexInDimCount; k++) {
            xoffset = (currentDimValue + k) - 1;
            if (!rtIsNaN(A_data[xoffset])) {
              bsum += A_data[xoffset];
              counts++;
            }
          }
          y += bsum;
        }
      }
      if (counts == 0) {
        y = rtNaN;
      } else {
        y /= (double)counts;
      }
      Bd_data[inds_data - 1] = y;
    }
    for (i = 0; i < b_tmp; i++) {
      A_data[i] = Bd_data[i];
    }
  }
  emxFree_real_T(&Bd);
}

/*
 * File trailer for smoothdata.c
 *
 * [EOF]
 */
