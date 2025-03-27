/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: eml_setop.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 27-Mar-2025 00:17:05
 */

/* Include Files */
#include "eml_setop.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_int32_T *a
 *                const emxArray_int32_T *b
 *                emxArray_int32_T *c
 *                emxArray_int32_T *ia
 *                emxArray_int32_T *ib
 * Return Type  : void
 */
void do_vectors(const emxArray_int32_T *a, const emxArray_int32_T *b,
                emxArray_int32_T *c, emxArray_int32_T *ia, emxArray_int32_T *ib)
{
  const int *a_data;
  const int *b_data;
  int b_ialast;
  int b_iblast;
  int iafirst;
  int ialast;
  int ibfirst;
  int iblast;
  int na_tmp;
  int nb_tmp;
  int nc;
  int ncmax;
  int nia;
  int nib;
  int *c_data;
  int *ia_data;
  int *ib_data;
  b_data = b->data;
  a_data = a->data;
  na_tmp = a->size[0];
  nb_tmp = b->size[0];
  ncmax = a->size[0] + b->size[0];
  iafirst = c->size[0];
  c->size[0] = ncmax;
  emxEnsureCapacity_int32_T(c, iafirst);
  c_data = c->data;
  iafirst = ia->size[0];
  ia->size[0] = a->size[0];
  emxEnsureCapacity_int32_T(ia, iafirst);
  ia_data = ia->data;
  iafirst = ib->size[0];
  ib->size[0] = b->size[0];
  emxEnsureCapacity_int32_T(ib, iafirst);
  ib_data = ib->data;
  nc = -1;
  nia = -1;
  nib = 0;
  iafirst = 1;
  ialast = 0;
  ibfirst = 0;
  iblast = 0;
  while ((ialast + 1 <= na_tmp) && (iblast + 1 <= nb_tmp)) {
    int ak;
    int bk;
    b_ialast = ialast + 1;
    ak = a_data[ialast];
    while ((b_ialast < a->size[0]) && (a_data[b_ialast] == ak)) {
      b_ialast++;
    }
    ialast = b_ialast - 1;
    b_iblast = iblast + 1;
    bk = b_data[iblast];
    while ((b_iblast < b->size[0]) && (b_data[b_iblast] == bk)) {
      b_iblast++;
    }
    iblast = b_iblast - 1;
    if (ak == bk) {
      nc++;
      c_data[nc] = ak;
      nia++;
      ia_data[nia] = iafirst;
      ialast = b_ialast;
      iafirst = b_ialast + 1;
      iblast = b_iblast;
      ibfirst = b_iblast;
    } else if (ak < bk) {
      nc++;
      nia++;
      c_data[nc] = ak;
      ia_data[nia] = iafirst;
      ialast = b_ialast;
      iafirst = b_ialast + 1;
    } else {
      nc++;
      nib++;
      c_data[nc] = bk;
      ib_data[nib - 1] = ibfirst + 1;
      iblast = b_iblast;
      ibfirst = b_iblast;
    }
  }
  while (ialast + 1 <= na_tmp) {
    b_ialast = ialast + 1;
    while ((b_ialast < a->size[0]) && (a_data[b_ialast] == a_data[ialast])) {
      b_ialast++;
    }
    nc++;
    nia++;
    c_data[nc] = a_data[ialast];
    ia_data[nia] = iafirst;
    ialast = b_ialast;
    iafirst = b_ialast + 1;
  }
  while (iblast + 1 <= nb_tmp) {
    b_iblast = iblast + 1;
    while ((b_iblast < b->size[0]) && (b_data[b_iblast] == b_data[iblast])) {
      b_iblast++;
    }
    nc++;
    nib++;
    c_data[nc] = b_data[iblast];
    ib_data[nib - 1] = ibfirst + 1;
    iblast = b_iblast;
    ibfirst = b_iblast;
  }
  if (a->size[0] > 0) {
    iafirst = ia->size[0];
    if (nia + 1 < 1) {
      ia->size[0] = 0;
    } else {
      ia->size[0] = nia + 1;
    }
    emxEnsureCapacity_int32_T(ia, iafirst);
  }
  if (b->size[0] > 0) {
    iafirst = ib->size[0];
    if (nib < 1) {
      ib->size[0] = 0;
    } else {
      ib->size[0] = nib;
    }
    emxEnsureCapacity_int32_T(ib, iafirst);
  }
  if (ncmax > 0) {
    iafirst = c->size[0];
    if (nc + 1 < 1) {
      c->size[0] = 0;
    } else {
      c->size[0] = nc + 1;
    }
    emxEnsureCapacity_int32_T(c, iafirst);
  }
}

/*
 * File trailer for eml_setop.c
 *
 * [EOF]
 */
