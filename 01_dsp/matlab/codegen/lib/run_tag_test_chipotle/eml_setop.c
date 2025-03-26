/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: eml_setop.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
  int iafirst;
  int ialast;
  int ibfirst;
  int iblast;
  int na;
  int nb;
  int nc;
  int ncmax;
  int nia;
  int nib;
  int *c_data;
  int *ia_data;
  int *ib_data;
  b_data = b->data;
  a_data = a->data;
  na = a->size[0];
  nb = b->size[0];
  ncmax = a->size[0] + b->size[0];
  ialast = c->size[0];
  c->size[0] = ncmax;
  emxEnsureCapacity_int32_T(c, ialast);
  c_data = c->data;
  ialast = ia->size[0];
  ia->size[0] = a->size[0];
  emxEnsureCapacity_int32_T(ia, ialast);
  ia_data = ia->data;
  ialast = ib->size[0];
  ib->size[0] = b->size[0];
  emxEnsureCapacity_int32_T(ib, ialast);
  ib_data = ib->data;
  nc = -1;
  nia = -1;
  nib = 0;
  iafirst = 1;
  b_ialast = 0;
  ibfirst = 0;
  iblast = 0;
  while ((b_ialast + 1 <= na) && (iblast + 1 <= nb)) {
    int ak;
    int b_iblast;
    int bk;
    ialast = b_ialast + 1;
    ak = a_data[b_ialast];
    while ((ialast < a->size[0]) && (a_data[ialast] == ak)) {
      ialast++;
    }
    b_ialast = ialast - 1;
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
      b_ialast = ialast;
      iafirst = ialast + 1;
      iblast = b_iblast;
      ibfirst = b_iblast;
    } else if (ak < bk) {
      nc++;
      nia++;
      c_data[nc] = ak;
      ia_data[nia] = iafirst;
      b_ialast = ialast;
      iafirst = ialast + 1;
    } else {
      nc++;
      nib++;
      c_data[nc] = bk;
      ib_data[nib - 1] = ibfirst + 1;
      iblast = b_iblast;
      ibfirst = b_iblast;
    }
  }
  while (b_ialast + 1 <= na) {
    ialast = b_ialast + 1;
    while ((ialast < a->size[0]) && (a_data[ialast] == a_data[b_ialast])) {
      ialast++;
    }
    nc++;
    nia++;
    c_data[nc] = a_data[b_ialast];
    ia_data[nia] = iafirst;
    b_ialast = ialast;
    iafirst = ialast + 1;
  }
  while (iblast + 1 <= nb) {
    ialast = iblast + 1;
    while ((ialast < b->size[0]) && (b_data[ialast] == b_data[iblast])) {
      ialast++;
    }
    nc++;
    nib++;
    c_data[nc] = b_data[iblast];
    ib_data[nib - 1] = ibfirst + 1;
    iblast = ialast;
    ibfirst = ialast;
  }
  if (a->size[0] > 0) {
    ialast = ia->size[0];
    if (nia + 1 < 1) {
      ia->size[0] = 0;
    } else {
      ia->size[0] = nia + 1;
    }
    emxEnsureCapacity_int32_T(ia, ialast);
  }
  if (b->size[0] > 0) {
    ialast = ib->size[0];
    if (nib < 1) {
      ib->size[0] = 0;
    } else {
      ib->size[0] = nib;
    }
    emxEnsureCapacity_int32_T(ib, ialast);
  }
  if (ncmax > 0) {
    ialast = c->size[0];
    if (nc + 1 < 1) {
      c->size[0] = 0;
    } else {
      c->size[0] = nc + 1;
    }
    emxEnsureCapacity_int32_T(c, ialast);
  }
}

/*
 * File trailer for eml_setop.c
 *
 * [EOF]
 */
