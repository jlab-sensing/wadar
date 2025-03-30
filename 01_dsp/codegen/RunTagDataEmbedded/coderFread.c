/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: coderFread.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "coderFread.h"
#include "RunTagDataEmbedded_data.h"
#include "rt_nonfinite.h"
#include <stdio.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double fileID
 * Return Type  : FILE *
 */
FILE *getFileStar(double fileID)
{
  FILE *filestar;
  signed char fileid;
  fileid = (signed char)fileID;
  if (((signed char)fileID < 0) || (fileID != (signed char)fileID)) {
    fileid = -1;
  }
  if (fileid >= 3) {
    filestar = eml_openfiles[fileid - 3];
  } else if (fileid == 0) {
    filestar = stdin;
  } else if (fileid == 1) {
    filestar = stdout;
  } else if (fileid == 2) {
    filestar = stderr;
  } else {
    filestar = NULL;
  }
  if ((!(fileID != 0.0)) || (!(fileID != 1.0)) || (!(fileID != 2.0))) {
    filestar = NULL;
  }
  return filestar;
}

/*
 * File trailer for coderFread.c
 *
 * [EOF]
 */
