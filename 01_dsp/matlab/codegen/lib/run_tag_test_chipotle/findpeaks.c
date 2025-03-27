/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: findpeaks.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 27-Mar-2025 00:17:05
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
  int i;
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
  i = Yin->size[0];
  nPk = fPk->size[0];
  fPk->size[0] = Yin->size[0];
  emxEnsureCapacity_int32_T(fPk, nPk);
  fPk_data = fPk->data;
  emxInit_int32_T(&iInfinite, 1);
  nPk = iInfinite->size[0];
  iInfinite->size[0] = Yin->size[0];
  emxEnsureCapacity_int32_T(iInfinite, nPk);
  iPk_data = iInfinite->data;
  emxInit_int32_T(&idx, 1);
  nPk = 0;
  nInf = 0;
  dir = 'n';
  kfirst = 0;
  ykfirst = rtInf;
  isinfykfirst = true;
  for (k = 1; k <= i; k++) {
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
          nPk++;
          fPk_data[nPk - 1] = kfirst;
        }
      } else {
        dir = 'i';
      }
      ykfirst = yk;
      kfirst = k;
      isinfykfirst = isinfyk;
    }
  }
  if (nPk < 1) {
    i = 0;
  } else {
    i = nPk;
  }
  nPk = fPk->size[0];
  fPk->size[0] = i;
  emxEnsureCapacity_int32_T(fPk, nPk);
  fPk_data = fPk->data;
  nPk = iInfinite->size[0];
  if (nInf < 1) {
    iInfinite->size[0] = 0;
  } else {
    iInfinite->size[0] = nInf;
  }
  emxEnsureCapacity_int32_T(iInfinite, nPk);
  emxInit_int32_T(&iPk, 1);
  nPk = iPk->size[0];
  iPk->size[0] = i;
  emxEnsureCapacity_int32_T(iPk, nPk);
  iPk_data = iPk->data;
  nPk = 0;
  for (k = 0; k < i; k++) {
    ykfirst = Yin_data[fPk_data[k] - 1];
    if ((ykfirst > varargin_2) &&
        (ykfirst - fmax(Yin_data[fPk_data[k] - 2], Yin_data[fPk_data[k]]) >=
         0.0)) {
      nPk++;
      iPk_data[nPk - 1] = fPk_data[k];
    }
  }
  i = iPk->size[0];
  iPk->size[0] = nPk;
  emxEnsureCapacity_int32_T(iPk, i);
  emxInit_int32_T(&c, 1);
  do_vectors(iPk, iInfinite, c, idx, fPk);
  c_data = c->data;
  emxFree_int32_T(&iInfinite);
  emxFree_int32_T(&iPk);
  emxInit_int32_T(&y, 2);
  kfirst = c->size[0];
  i = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = c->size[0];
  emxEnsureCapacity_int32_T(y, i);
  iPk_data = y->data;
  if (c->size[0] > 0) {
    iPk_data[0] = 1;
    nPk = 1;
    for (k = 2; k <= kfirst; k++) {
      nPk++;
      iPk_data[k - 1] = nPk;
    }
  }
  i = idx->size[0];
  idx->size[0] = c->size[0];
  emxEnsureCapacity_int32_T(idx, i);
  idx_data = idx->data;
  for (i = 0; i < kfirst; i++) {
    idx_data[i] = iPk_data[i];
  }
  emxFree_int32_T(&y);
  if (idx->size[0] > Yin->size[0]) {
    i = fPk->size[0];
    fPk->size[0] = Yin->size[0];
    emxEnsureCapacity_int32_T(fPk, i);
    i = idx->size[0];
    idx->size[0] = Yin->size[0];
    emxEnsureCapacity_int32_T(idx, i);
    idx_data = idx->data;
  } else {
    i = fPk->size[0];
    fPk->size[0] = c->size[0];
    emxEnsureCapacity_int32_T(fPk, i);
  }
  nPk = fPk->size[0];
  i = fPk->size[0];
  fPk->size[0] = nPk;
  emxEnsureCapacity_int32_T(fPk, i);
  fPk_data = fPk->data;
  for (i = 0; i < nPk; i++) {
    fPk_data[i] = c_data[idx_data[i] - 1];
  }
  emxFree_int32_T(&c);
  emxFree_int32_T(&idx);
  i = Ypk->size[0];
  Ypk->size[0] = nPk;
  emxEnsureCapacity_real_T(Ypk, i);
  Ypk_data = Ypk->data;
  i = Xpk->size[0];
  Xpk->size[0] = nPk;
  emxEnsureCapacity_real_T(Xpk, i);
  Xpk_data = Xpk->data;
  for (i = 0; i < nPk; i++) {
    Ypk_data[i] = Yin_data[fPk_data[i] - 1];
    Xpk_data[i] = (unsigned int)fPk_data[i];
  }
  emxFree_int32_T(&fPk);
}

/*
 * File trailer for findpeaks.c
 *
 * [EOF]
 */
