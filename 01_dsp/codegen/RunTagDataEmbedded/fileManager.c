/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fileManager.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "fileManager.h"
#include "RunTagDataEmbedded_data.h"
#include "RunTagDataEmbedded_emxutil.h"
#include "RunTagDataEmbedded_types.h"
#include "rt_nonfinite.h"
#include <stdio.h>
#include <string.h>

/* Function Declarations */
static signed char filedata(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : signed char
 */
static signed char filedata(void)
{
  int k;
  signed char f;
  bool exitg1;
  f = 0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 20)) {
    if (eml_openfiles[k] == NULL) {
      f = (signed char)(k + 1);
      exitg1 = true;
    } else {
      k++;
    }
  }
  return f;
}

/*
 * Arguments    : double fid
 * Return Type  : int
 */
int cfclose(double fid)
{
  FILE *f;
  int st;
  signed char b_fileid;
  signed char fileid;
  st = -1;
  fileid = (signed char)fid;
  if (((signed char)fid < 0) || (fid != (signed char)fid)) {
    fileid = -1;
  }
  b_fileid = fileid;
  if (fileid < 0) {
    b_fileid = -1;
  }
  if (b_fileid >= 3) {
    f = eml_openfiles[b_fileid - 3];
  } else if (b_fileid == 0) {
    f = stdin;
  } else if (b_fileid == 1) {
    f = stdout;
  } else if (b_fileid == 2) {
    f = stderr;
  } else {
    f = NULL;
  }
  if ((f != NULL) && (fileid >= 3)) {
    int cst;
    cst = fclose(f);
    if (cst == 0) {
      st = 0;
      eml_openfiles[fileid - 3] = NULL;
    }
  }
  return st;
}

/*
 * Arguments    : const emxArray_char_T *cfilename
 * Return Type  : signed char
 */
signed char cfopen(const emxArray_char_T *cfilename)
{
  FILE *filestar;
  emxArray_char_T *ccfilename;
  int i;
  const char *cfilename_data;
  signed char fileid;
  signed char j;
  char *ccfilename_data;
  cfilename_data = cfilename->data;
  fileid = -1;
  j = filedata();
  if (j >= 1) {
    int loop_ub_tmp;
    emxInit_char_T(&ccfilename);
    i = ccfilename->size[0] * ccfilename->size[1];
    ccfilename->size[0] = 1;
    ccfilename->size[1] = cfilename->size[1] + 1;
    emxEnsureCapacity_char_T(ccfilename, i);
    ccfilename_data = ccfilename->data;
    loop_ub_tmp = cfilename->size[1];
    for (i = 0; i < loop_ub_tmp; i++) {
      ccfilename_data[i] = cfilename_data[i];
    }
    ccfilename_data[cfilename->size[1]] = '\x00';
    filestar = fopen(&ccfilename_data[0], "rb");
    emxFree_char_T(&ccfilename);
    if (filestar != NULL) {
      eml_openfiles[j - 1] = filestar;
      i = j + 2;
      if (j + 2 > 127) {
        i = 127;
      }
      fileid = (signed char)i;
    }
  }
  return fileid;
}

/*
 * Arguments    : void
 * Return Type  : int
 */
int fileManager(void)
{
  int f;
  int j;
  f = 0;
  for (j = 0; j < 20; j++) {
    if (eml_openfiles[j] != NULL) {
      int cst;
      cst = fclose(eml_openfiles[j]);
      if (cst == 0) {
        eml_openfiles[j] = NULL;
      } else {
        f = -1;
      }
    }
  }
  return f;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void filedata_init(void)
{
  int i;
  for (i = 0; i < 20; i++) {
    eml_openfiles[i] = NULL;
  }
}

/*
 * File trailer for fileManager.c
 *
 * [EOF]
 */
