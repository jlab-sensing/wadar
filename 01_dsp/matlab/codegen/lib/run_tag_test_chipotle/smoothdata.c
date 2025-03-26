/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: smoothdata.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
  int b_k;
  int ii;
  int k;
  A_data = A->data;
  emxInit_real_T(&Bd, 1);
  if (A->size[0] >= 2) {
    int b;
    int currentDimValue;
    int i;
    int inds_data;
    int windowEnd;
    int windowStart;
    b = A->size[0];
    currentDimValue = Bd->size[0];
    Bd->size[0] = b;
    emxEnsureCapacity_real_T(Bd, currentDimValue);
    Bd_data = Bd->data;
    windowStart = 0;
    windowEnd = 1;
    i = A->size[0] - 1;
    for (ii = 0; ii <= i; ii++) {
      double y;
      int b_ii;
      int indexInDimCount;
      bool exitg1;
      while ((windowStart + 1 < ii + 1) && (ii - windowStart > 5)) {
        windowStart++;
      }
      if (windowEnd < ii + 1) {
        windowEnd = ii + 1;
      }
      while ((windowEnd < b) && (windowEnd - ii < 5)) {
        windowEnd++;
      }
      currentDimValue = 1;
      indexInDimCount = 1;
      b_ii = 0;
      exitg1 = false;
      while ((!exitg1) && (b_ii <= A->size[0] - 1)) {
        if (indexInDimCount > 1) {
          if (currentDimValue == b) {
            currentDimValue = 1;
          } else {
            currentDimValue++;
          }
        }
        if (currentDimValue == ii + 1) {
          inds_data = b_ii + 1;
          exitg1 = true;
        } else {
          indexInDimCount = 2;
          b_ii++;
        }
      }
      currentDimValue = windowEnd - windowStart;
      if (currentDimValue == 0) {
        y = 0.0;
        b_ii = 0;
      } else {
        int lastBlockLength;
        int nblocks;
        int xoffset;
        if (currentDimValue <= 1024) {
          indexInDimCount = currentDimValue;
          lastBlockLength = 0;
          nblocks = 1;
        } else {
          indexInDimCount = 1024;
          nblocks = (int)((unsigned int)currentDimValue >> 10);
          lastBlockLength = currentDimValue - (nblocks << 10);
          if (lastBlockLength > 0) {
            nblocks++;
          } else {
            lastBlockLength = 1024;
          }
        }
        if (rtIsNaN(A_data[windowStart])) {
          y = 0.0;
          b_ii = 0;
        } else {
          y = A_data[windowStart];
          b_ii = 1;
        }
        for (k = 2; k <= indexInDimCount; k++) {
          xoffset = (windowStart + k) - 1;
          if (!rtIsNaN(A_data[xoffset])) {
            y += A_data[xoffset];
            b_ii++;
          }
        }
        for (k = 2; k <= nblocks; k++) {
          double bsum;
          currentDimValue = windowStart + ((k - 1) << 10);
          if (rtIsNaN(A_data[currentDimValue])) {
            bsum = 0.0;
          } else {
            bsum = A_data[currentDimValue];
            b_ii++;
          }
          if (k == nblocks) {
            indexInDimCount = lastBlockLength;
          } else {
            indexInDimCount = 1024;
          }
          for (b_k = 2; b_k <= indexInDimCount; b_k++) {
            xoffset = (currentDimValue + b_k) - 1;
            if (!rtIsNaN(A_data[xoffset])) {
              bsum += A_data[xoffset];
              b_ii++;
            }
          }
          y += bsum;
        }
      }
      if (b_ii == 0) {
        y = rtNaN;
      } else {
        y /= (double)b_ii;
      }
      Bd_data[inds_data - 1] = y;
    }
    for (k = 0; k < b; k++) {
      A_data[k] = Bd_data[k];
    }
  }
  emxFree_real_T(&Bd);
}

/*
 * File trailer for smoothdata.c
 *
 * [EOF]
 */
