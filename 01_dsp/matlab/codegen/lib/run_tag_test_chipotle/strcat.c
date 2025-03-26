/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: strcat.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

/* Include Files */
#include "strcat.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <string.h>

/* Function Declarations */
static void sanitizeInputAndConvertToChar(emxArray_char_T *arg);

/* Function Definitions */
/*
 * Arguments    : emxArray_char_T *arg
 * Return Type  : void
 */
static void sanitizeInputAndConvertToChar(emxArray_char_T *arg)
{
  static const boolean_T bv[128] = {
      false, false, false, false, false, false, false, false, false, true,
      true,  true,  true,  true,  false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, true,  true,
      true,  true,  true,  false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false, false};
  char *arg_data;
  arg_data = arg->data;
  if (arg->size[1] != 0) {
    unsigned char u;
    u = (unsigned char)arg_data[arg->size[1] - 1];
    if ((u == 0) || bv[(int)(u & 127U)]) {
      int i;
      int ncols;
      boolean_T exitg1;
      ncols = arg->size[1] - 1;
      exitg1 = false;
      while ((!exitg1) && (ncols + 1 > 0)) {
        u = (unsigned char)arg_data[ncols];
        if ((u != 0) && (!bv[(int)(u & 127U)])) {
          exitg1 = true;
        } else {
          ncols--;
        }
      }
      i = arg->size[0] * arg->size[1];
      arg->size[0] = 1;
      if (ncols + 1 < 1) {
        ncols = -1;
      }
      arg->size[1] = ncols + 1;
      emxEnsureCapacity_char_T(arg, i);
    }
  }
}

/*
 * Arguments    : const emxArray_char_T *varargin_1
 *                const char varargin_2_data[]
 *                const int varargin_2_size[2]
 *                emxArray_char_T *t
 * Return Type  : void
 */
void b_strcat(const emxArray_char_T *varargin_1, const char varargin_2_data[],
              const int varargin_2_size[2], emxArray_char_T *t)
{
  emxArray_char_T *input;
  int i;
  int i1;
  int loop_ub;
  unsigned int unnamed_idx_1;
  int varargin_2_tmp;
  const char *varargin_1_data;
  char *input_data;
  char *t_data;
  varargin_1_data = varargin_1->data;
  unnamed_idx_1 =
      (unsigned int)varargin_1->size[1] + (unsigned int)varargin_2_size[1];
  i = t->size[0] * t->size[1];
  t->size[0] = 1;
  t->size[1] = (int)unnamed_idx_1;
  emxEnsureCapacity_char_T(t, i);
  t_data = t->data;
  loop_ub = (int)unnamed_idx_1;
  for (i = 0; i < loop_ub; i++) {
    t_data[i] = ' ';
  }
  emxInit_char_T(&input);
  i = input->size[0] * input->size[1];
  input->size[0] = 1;
  loop_ub = varargin_1->size[1];
  input->size[1] = varargin_1->size[1];
  emxEnsureCapacity_char_T(input, i);
  input_data = input->data;
  for (i = 0; i < loop_ub; i++) {
    input_data[i] = varargin_1_data[i];
  }
  sanitizeInputAndConvertToChar(input);
  input_data = input->data;
  varargin_2_tmp = input->size[1];
  for (i = 0; i < varargin_2_tmp; i++) {
    t_data[i] = input_data[i];
  }
  i = input->size[0] * input->size[1];
  input->size[0] = 1;
  loop_ub = varargin_2_size[1];
  input->size[1] = varargin_2_size[1];
  emxEnsureCapacity_char_T(input, i);
  input_data = input->data;
  for (i = 0; i < loop_ub; i++) {
    input_data[i] = varargin_2_data[i];
  }
  sanitizeInputAndConvertToChar(input);
  input_data = input->data;
  if (input->size[1] < 1) {
    i = 0;
  } else {
    i = varargin_2_tmp;
  }
  loop_ub = input->size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    t_data[i + i1] = input_data[i1];
  }
  i = t->size[0] * t->size[1];
  t->size[1] =
      (int)((unsigned int)varargin_2_tmp + (unsigned int)input->size[1]);
  emxFree_char_T(&input);
  emxEnsureCapacity_char_T(t, i);
}

/*
 * File trailer for strcat.c
 *
 * [EOF]
 */
