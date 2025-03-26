/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: findpeaks.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "findpeaks.h"
#include "eml_setop.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *Yin
 *                double varargin_2
 *                emxArray_real_T *Ypk
 *                emxArray_real_T *Xpk
 * Return Type  : void
 */
void findpeaks(const emxArray_real_T *Yin, double varargin_2,
               emxArray_real_T *Ypk, emxArray_real_T *Xpk)
{
  emxArray_int32_T *c;
  emxArray_int32_T *fPk;
  emxArray_int32_T *iInfinite;
  emxArray_int32_T *iPk;
  emxArray_int32_T *idx;
  emxArray_int32_T *y;
  const double *Yin_data;
  double ykfirst;
  double *Xpk_data;
  double *Ypk_data;
  int b_nPk;
  int k;
  int kfirst;
  int nInf;
  int nPk;
  int *c_data;
  int *fPk_data;
  int *iPk_data;
  int *idx_data;
  char dir;
  bool isinfykfirst;
  Yin_data = Yin->data;
  emxInit_int32_T(&fPk, 1);
  nPk = Yin->size[0];
  kfirst = fPk->size[0];
  fPk->size[0] = Yin->size[0];
  emxEnsureCapacity_int32_T(fPk, kfirst);
  fPk_data = fPk->data;
  emxInit_int32_T(&iInfinite, 1);
  kfirst = iInfinite->size[0];
  iInfinite->size[0] = Yin->size[0];
  emxEnsureCapacity_int32_T(iInfinite, kfirst);
  iPk_data = iInfinite->data;
  emxInit_int32_T(&idx, 1);
  b_nPk = 0;
  nInf = 0;
  dir = 'n';
  kfirst = 0;
  ykfirst = rtInf;
  isinfykfirst = true;
  for (k = 1; k <= nPk; k++) {
    double yk;
    bool isinfyk;
    yk = Yin_data[k - 1];
    if (rtIsNaN(yk)) {
      yk = rtInf;
      isinfyk = true;
    } else if (rtIsInf(yk) && (yk > 0.0)) {
      isinfyk = true;
      nInf++;
      iPk_data[nInf - 1] = k;
    } else {
      isinfyk = false;
    }
    if (yk != ykfirst) {
      char previousdir;
      previousdir = dir;
      if (isinfyk || isinfykfirst) {
        dir = 'n';
      } else if (yk < ykfirst) {
        dir = 'd';
        if (previousdir == 'i') {
          b_nPk++;
          fPk_data[b_nPk - 1] = kfirst;
        }
      } else {
        dir = 'i';
      }
      ykfirst = yk;
      kfirst = k;
      isinfykfirst = isinfyk;
    }
  }
  if (b_nPk < 1) {
    b_nPk = 0;
  }
  kfirst = fPk->size[0];
  fPk->size[0] = b_nPk;
  emxEnsureCapacity_int32_T(fPk, kfirst);
  fPk_data = fPk->data;
  kfirst = iInfinite->size[0];
  if (nInf < 1) {
    iInfinite->size[0] = 0;
  } else {
    iInfinite->size[0] = nInf;
  }
  emxEnsureCapacity_int32_T(iInfinite, kfirst);
  emxInit_int32_T(&iPk, 1);
  kfirst = iPk->size[0];
  iPk->size[0] = b_nPk;
  emxEnsureCapacity_int32_T(iPk, kfirst);
  iPk_data = iPk->data;
  nPk = 0;
  for (k = 0; k < b_nPk; k++) {
    ykfirst = Yin_data[fPk_data[k] - 1];
    if ((ykfirst > varargin_2) &&
        (ykfirst - fmax(Yin_data[fPk_data[k] - 2], Yin_data[fPk_data[k]]) >=
         0.0)) {
      nPk++;
      iPk_data[nPk - 1] = fPk_data[k];
    }
  }
  kfirst = iPk->size[0];
  iPk->size[0] = nPk;
  emxEnsureCapacity_int32_T(iPk, kfirst);
  emxInit_int32_T(&c, 1);
  do_vectors(iPk, iInfinite, c, idx, fPk);
  c_data = c->data;
  emxFree_int32_T(&iInfinite);
  emxFree_int32_T(&iPk);
  nPk = c->size[0];
  emxInit_int32_T(&y, 2);
  kfirst = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = c->size[0];
  emxEnsureCapacity_int32_T(y, kfirst);
  iPk_data = y->data;
  if (c->size[0] > 0) {
    iPk_data[0] = 1;
    kfirst = 1;
    for (k = 2; k <= nPk; k++) {
      kfirst++;
      iPk_data[k - 1] = kfirst;
    }
  }
  kfirst = idx->size[0];
  idx->size[0] = c->size[0];
  emxEnsureCapacity_int32_T(idx, kfirst);
  idx_data = idx->data;
  for (k = 0; k < nPk; k++) {
    idx_data[k] = iPk_data[k];
  }
  emxFree_int32_T(&y);
  if (idx->size[0] > Yin->size[0]) {
    kfirst = fPk->size[0];
    fPk->size[0] = Yin->size[0];
    emxEnsureCapacity_int32_T(fPk, kfirst);
    kfirst = idx->size[0];
    idx->size[0] = Yin->size[0];
    emxEnsureCapacity_int32_T(idx, kfirst);
    idx_data = idx->data;
  } else {
    kfirst = fPk->size[0];
    fPk->size[0] = c->size[0];
    emxEnsureCapacity_int32_T(fPk, kfirst);
  }
  nPk = fPk->size[0];
  kfirst = fPk->size[0];
  fPk->size[0] = nPk;
  emxEnsureCapacity_int32_T(fPk, kfirst);
  fPk_data = fPk->data;
  for (k = 0; k < nPk; k++) {
    fPk_data[k] = c_data[idx_data[k] - 1];
  }
  emxFree_int32_T(&c);
  emxFree_int32_T(&idx);
  kfirst = Ypk->size[0];
  Ypk->size[0] = nPk;
  emxEnsureCapacity_real_T(Ypk, kfirst);
  Ypk_data = Ypk->data;
  kfirst = Xpk->size[0];
  Xpk->size[0] = nPk;
  emxEnsureCapacity_real_T(Xpk, kfirst);
  Xpk_data = Xpk->data;
  for (k = 0; k < nPk; k++) {
    Ypk_data[k] = Yin_data[fPk_data[k] - 1];
    Xpk_data[k] = (unsigned int)fPk_data[k];
  }
  emxFree_int32_T(&fPk);
}

/*
 * File trailer for findpeaks.c
 *
 * [EOF]
 */
