/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fread.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
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
  int i;
  boolean_T doEOF;
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
      int c;
      int numRead;
      numRead = A->size[0];
      A->size[0] = (int)sizeA;
      emxEnsureCapacity_real_T(A, numRead);
      A_data = A->data;
      c = 0;
      numRead = 1;
      while ((c < dims_idx_0) && (numRead > 0)) {
        size_t numReadSizeT;
        numReadSizeT =
            fread(&A_data[c], nBytes, (size_t)(dims_idx_0 - c), filestar);
        numRead = (int)numReadSizeT;
        c += (int)numReadSizeT;
      }
      numRead = c + 1;
      i = A->size[0];
      for (dims_idx_0 = numRead; dims_idx_0 <= i; dims_idx_0++) {
        A_data[dims_idx_0 - 1] = 0.0;
      }
      if (c < sizeA) {
        numRead = A->size[0];
        if (c < 1) {
          A->size[0] = 0;
        } else {
          A->size[0] = c;
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
        numRead = A->size[0];
        i = A->size[0];
        A->size[0] += dims_idx_0;
        emxEnsureCapacity_real_T(A, i);
        A_data = A->data;
        for (i = 0; i < dims_idx_0; i++) {
          A_data[numRead + i] = tbuf[i];
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
  int c;
  int num2Read;
  boolean_T doEOF;
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
      numRequested = bdims_idx_0;
      i = (int)sizeA;
      num2Read = A->size[0];
      A->size[0] = (int)sizeA;
      emxEnsureCapacity_real_T(A, num2Read);
      A_data = A->data;
      if (((int)sizeA == 0) || (bdims_idx_0 == 0)) {
        bytesOut = 0;
      } else {
        int numRead;
        if (bdims_idx_0 > 1024) {
          bdims_idx_0 = 1024;
        }
        bytesOut = 0;
        numRead = 1;
        while ((bytesOut < numRequested) && (numRead > 0)) {
          unsigned int tbuf[1024];
          num2Read = bdims_idx_0;
          numRead = numRequested - bytesOut;
          if (bdims_idx_0 > numRead) {
            num2Read = numRead;
          }
          numRead = 0;
          c = 1;
          while ((numRead < num2Read) && (c > 0)) {
            size_t numReadSizeT;
            numReadSizeT = fread(&tbuf[numRead], nBytes,
                                 (size_t)(num2Read - numRead), filestar);
            c = (int)numReadSizeT;
            numRead += (int)numReadSizeT;
          }
          for (c = 0; c < numRead; c++) {
            A_data[c + bytesOut] = tbuf[c];
          }
          bytesOut += numRead;
        }
        num2Read = bytesOut + 1;
        for (c = num2Read; c <= i; c++) {
          A_data[c - 1] = 0.0;
        }
      }
      if (bytesOut < sizeA) {
        i = A->size[0];
        if (bytesOut < 1) {
          A->size[0] = 0;
        } else {
          A->size[0] = bytesOut;
        }
        emxEnsureCapacity_real_T(A, i);
      }
    }
  } else {
    A->size[0] = 0;
    if (!(filestar == NULL)) {
      c = 1;
      while (c > 0) {
        unsigned int tbuf[1024];
        int i;
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
          numRead = 0;
        } else {
          numRead = c;
        }
        i = A->size[0];
        num2Read = A->size[0];
        A->size[0] += numRead;
        emxEnsureCapacity_real_T(A, num2Read);
        A_data = A->data;
        for (num2Read = 0; num2Read < numRead; num2Read++) {
          A_data[i + num2Read] = tbuf[num2Read];
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
        numRead = 0;
      } else {
        numRead = c;
      }
      i = A->size[0];
      i1 = A->size[0];
      A->size[0] += numRead;
      emxEnsureCapacity_real_T(A, i1);
      A_data = A->data;
      for (i1 = 0; i1 < numRead; i1++) {
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
