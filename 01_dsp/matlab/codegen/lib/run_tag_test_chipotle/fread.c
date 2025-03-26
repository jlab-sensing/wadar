/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: fread.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "fread.h"
#include "coderFread.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_data.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double fileID
 *                double A_data[]
 * Return Type  : int
 */
int b_fread(double fileID, double A_data[])
{
  FILE *filestar;
  int A_size;
  int k;
  filestar = getFileStar(fileID);
  if (filestar == NULL) {
    A_size = 0;
  } else {
    size_t numReadSizeT;
    unsigned int Atmp;
    unsigned int buf;
    Atmp = 0U;
    numReadSizeT = fread(&buf, sizeof(unsigned int), (size_t)1, filestar);
    A_size = (int)numReadSizeT;
    for (k = 0; k < A_size; k++) {
      Atmp = buf;
    }
    A_size = (int)numReadSizeT + 1;
    for (k = A_size; k < 2; k++) {
      Atmp = 0U;
    }
    if ((int)numReadSizeT < 1) {
      A_size = 0;
    } else {
      A_size = 1;
      A_data[0] = Atmp;
    }
  }
  return A_size;
}

/*
 * Arguments    : double fileID
 *                double A_data[]
 * Return Type  : int
 */
int c_fread(double fileID, double A_data[])
{
  FILE *filestar;
  int A_size;
  int k;
  filestar = getFileStar(fileID);
  if (filestar == NULL) {
    A_size = 0;
  } else {
    size_t numReadSizeT;
    int Atmp;
    int numRead;
    Atmp = 0;
    numReadSizeT = fread(&A_size, sizeof(int), (size_t)1, filestar);
    numRead = (int)numReadSizeT;
    for (k = 0; k < numRead; k++) {
      Atmp = A_size;
    }
    A_size = (int)numReadSizeT + 1;
    for (k = A_size; k < 2; k++) {
      Atmp = 0;
    }
    if ((int)numReadSizeT < 1) {
      A_size = 0;
    } else {
      A_size = 1;
      A_data[0] = Atmp;
    }
  }
  return A_size;
}

/*
 * Arguments    : double fileID
 *                double A_data[]
 * Return Type  : int
 */
int d_fread(double fileID, double A_data[])
{
  FILE *filestar;
  int A_size;
  int k;
  filestar = getFileStar(fileID);
  if (filestar == NULL) {
    A_size = 0;
  } else {
    size_t numReadSizeT;
    double Atmp;
    float buf;
    Atmp = 0.0;
    numReadSizeT = fread(&buf, sizeof(float), (size_t)1, filestar);
    A_size = (int)numReadSizeT;
    for (k = 0; k < A_size; k++) {
      Atmp = buf;
    }
    A_size = (int)numReadSizeT + 1;
    for (k = A_size; k < 2; k++) {
      Atmp = 0.0;
    }
    if ((int)numReadSizeT < 1) {
      A_size = 0;
    } else {
      A_size = 1;
      A_data[0] = Atmp;
    }
  }
  return A_size;
}

/*
 * Arguments    : double fileID
 *                double A_data[]
 * Return Type  : int
 */
int e_fread(double fileID, double A_data[])
{
  FILE *filestar;
  int A_size;
  int k;
  filestar = getFileStar(fileID);
  if (filestar == NULL) {
    A_size = 0;
  } else {
    size_t numReadSizeT;
    double Atmp;
    numReadSizeT = fread(&Atmp, sizeof(double), (size_t)1, filestar);
    A_size = (int)numReadSizeT + 1;
    for (k = A_size; k < 2; k++) {
      Atmp = 0.0;
    }
    if ((int)numReadSizeT < 1) {
      A_size = 0;
    } else {
      A_size = 1;
      A_data[0] = Atmp;
    }
  }
  return A_size;
}

/*
 * Arguments    : double fileID
 *                double sizeA
 *                emxArray_real_T *A
 * Return Type  : void
 */
void f_fread(double fileID, double sizeA, emxArray_real_T *A)
{
  FILE *filestar;
  size_t nBytes;
  double *A_data;
  int dims_idx_0;
  int k;
  bool doEOF;
  if (sizeA >= 2.147483647E+9) {
    dims_idx_0 = 1024;
    doEOF = true;
  } else {
    dims_idx_0 = (int)sizeA;
    doEOF = false;
  }
  nBytes = sizeof(double);
  filestar = getFileStar(fileID);
  if (!doEOF) {
    if (filestar == NULL) {
      A->size[0] = 0;
    } else {
      int bytesOut;
      int numRead;
      numRead = A->size[0];
      A->size[0] = (int)sizeA;
      emxEnsureCapacity_real_T(A, numRead);
      A_data = A->data;
      bytesOut = 0;
      numRead = 1;
      while ((bytesOut < dims_idx_0) && (numRead > 0)) {
        size_t numReadSizeT;
        numReadSizeT = fread(&A_data[bytesOut], nBytes,
                             (size_t)(dims_idx_0 - bytesOut), filestar);
        numRead = (int)numReadSizeT;
        bytesOut += (int)numReadSizeT;
      }
      numRead = bytesOut + 1;
      dims_idx_0 = A->size[0];
      for (k = numRead; k <= dims_idx_0; k++) {
        A_data[k - 1] = 0.0;
      }
      if (bytesOut < sizeA) {
        numRead = A->size[0];
        if (bytesOut < 1) {
          A->size[0] = 0;
        } else {
          A->size[0] = bytesOut;
        }
        emxEnsureCapacity_real_T(A, numRead);
      }
    }
  } else {
    A->size[0] = 0;
    if (!(filestar == NULL)) {
      int c;
      c = 1;
      while (c > 0) {
        double tbuf[1024];
        int bytesOut;
        int numRead;
        c = 0;
        numRead = 1;
        while ((c < 1024) && (numRead > 0)) {
          size_t numReadSizeT;
          numReadSizeT = fread(&tbuf[c], nBytes, (size_t)(1024 - c), filestar);
          numRead = (int)numReadSizeT;
          c += (int)numReadSizeT;
        }
        if (c < 1) {
          dims_idx_0 = 0;
        } else {
          dims_idx_0 = c;
        }
        bytesOut = A->size[0];
        numRead = A->size[0];
        A->size[0] += dims_idx_0;
        emxEnsureCapacity_real_T(A, numRead);
        A_data = A->data;
        for (k = 0; k < dims_idx_0; k++) {
          A_data[bytesOut + k] = tbuf[k];
        }
      }
    }
  }
}

/*
 * Arguments    : double fileID
 *                double sizeA
 *                emxArray_real_T *A
 * Return Type  : void
 */
void g_fread(double fileID, double sizeA, emxArray_real_T *A)
{
  FILE *filestar;
  size_t nBytes;
  double *A_data;
  int bdims_idx_0;
  int k;
  bool doEOF;
  if (sizeA >= 2.147483647E+9) {
    bdims_idx_0 = 1024;
    doEOF = true;
  } else {
    bdims_idx_0 = (int)sizeA;
    doEOF = false;
  }
  nBytes = sizeof(unsigned int);
  filestar = getFileStar(fileID);
  if (!doEOF) {
    if (filestar == NULL) {
      A->size[0] = 0;
    } else {
      int bytesOut;
      int i;
      int numRequested;
      int other2Read;
      numRequested = bdims_idx_0;
      i = (int)sizeA;
      other2Read = A->size[0];
      A->size[0] = (int)sizeA;
      emxEnsureCapacity_real_T(A, other2Read);
      A_data = A->data;
      if (((int)sizeA == 0) || (bdims_idx_0 == 0)) {
        bytesOut = 0;
      } else {
        int b_numRead;
        if (bdims_idx_0 > 1024) {
          bdims_idx_0 = 1024;
        }
        bytesOut = 0;
        b_numRead = 1;
        while ((bytesOut < numRequested) && (b_numRead > 0)) {
          unsigned int buf_data[1024];
          int num2Read;
          int numRead;
          num2Read = bdims_idx_0;
          other2Read = numRequested - bytesOut;
          if (bdims_idx_0 > other2Read) {
            num2Read = other2Read;
          }
          b_numRead = 0;
          numRead = 1;
          while ((b_numRead < num2Read) && (numRead > 0)) {
            size_t numReadSizeT;
            numReadSizeT = fread(&buf_data[b_numRead], nBytes,
                                 (size_t)(num2Read - b_numRead), filestar);
            numRead = (int)numReadSizeT;
            b_numRead += (int)numReadSizeT;
          }
          for (k = 0; k < b_numRead; k++) {
            A_data[k + bytesOut] = buf_data[k];
          }
          bytesOut += b_numRead;
        }
        other2Read = bytesOut + 1;
        for (k = other2Read; k <= i; k++) {
          A_data[k - 1] = 0.0;
        }
      }
      if (bytesOut < sizeA) {
        other2Read = A->size[0];
        if (bytesOut < 1) {
          A->size[0] = 0;
        } else {
          A->size[0] = bytesOut;
        }
        emxEnsureCapacity_real_T(A, other2Read);
      }
    }
  } else {
    A->size[0] = 0;
    if (!(filestar == NULL)) {
      int num2Read;
      num2Read = 1;
      while (num2Read > 0) {
        unsigned int buf_data[1024];
        int b_numRead;
        int numRead;
        int other2Read;
        num2Read = 0;
        other2Read = 1;
        while ((num2Read < 1024) && (other2Read > 0)) {
          size_t numReadSizeT;
          numReadSizeT = fread(&buf_data[num2Read], nBytes,
                               (size_t)(1024 - num2Read), filestar);
          other2Read = (int)numReadSizeT;
          num2Read += (int)numReadSizeT;
        }
        if (num2Read < 1) {
          numRead = 0;
        } else {
          numRead = num2Read;
        }
        b_numRead = A->size[0];
        other2Read = A->size[0];
        A->size[0] += numRead;
        emxEnsureCapacity_real_T(A, other2Read);
        A_data = A->data;
        for (k = 0; k < numRead; k++) {
          A_data[b_numRead + k] = buf_data[k];
        }
      }
    }
  }
}

/*
 * Arguments    : double fileID
 *                emxArray_real_T *A
 * Return Type  : double
 */
double h_fread(double fileID, emxArray_real_T *A)
{
  FILE *filestar;
  size_t nBytes;
  double *A_data;
  int bytesOut;
  int i1;
  nBytes = sizeof(unsigned char);
  filestar = getFileStar(fileID);
  A->size[0] = 0;
  if (filestar == NULL) {
    bytesOut = 0;
  } else {
    int c;
    c = 1;
    bytesOut = 0;
    while (c > 0) {
      int i;
      int loop_ub;
      int numRead;
      unsigned char tbuf[1024];
      c = 0;
      numRead = 1;
      while ((c < 1024) && (numRead > 0)) {
        size_t numReadSizeT;
        numReadSizeT = fread(&tbuf[c], nBytes, (size_t)(1024 - c), filestar);
        numRead = (int)numReadSizeT;
        c += (int)numReadSizeT;
      }
      if (c < 1) {
        loop_ub = 0;
      } else {
        loop_ub = c;
      }
      i = A->size[0];
      numRead = A->size[0];
      A->size[0] += loop_ub;
      emxEnsureCapacity_real_T(A, numRead);
      A_data = A->data;
      for (i1 = 0; i1 < loop_ub; i1++) {
        A_data[i + i1] = tbuf[i1];
      }
      bytesOut += c;
    }
  }
  return bytesOut;
}

/*
 * File trailer for fread.c
 *
 * [EOF]
 */
