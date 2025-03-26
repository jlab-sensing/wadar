/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: strcat.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
  static const bool bv[128] = {
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
      bool exitg1;
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
  int b_loop_ub;
  int i;
  int loop_ub;
  unsigned int unnamed_idx_1;
  int varargin_2;
  const char *varargin_1_data;
  char *input_data;
  char *t_data;
  varargin_1_data = varargin_1->data;
  unnamed_idx_1 =
      (unsigned int)varargin_1->size[1] + (unsigned int)varargin_2_size[1];
  loop_ub = t->size[0] * t->size[1];
  t->size[0] = 1;
  t->size[1] = (int)unnamed_idx_1;
  emxEnsureCapacity_char_T(t, loop_ub);
  t_data = t->data;
  loop_ub = (int)unnamed_idx_1;
  for (i = 0; i < loop_ub; i++) {
    t_data[i] = ' ';
  }
  emxInit_char_T(&input);
  loop_ub = input->size[0] * input->size[1];
  input->size[0] = 1;
  b_loop_ub = varargin_1->size[1];
  input->size[1] = varargin_1->size[1];
  emxEnsureCapacity_char_T(input, loop_ub);
  input_data = input->data;
  for (i = 0; i < b_loop_ub; i++) {
    input_data[i] = varargin_1_data[i];
  }
  sanitizeInputAndConvertToChar(input);
  input_data = input->data;
  varargin_2 = input->size[1];
  for (i = 0; i < varargin_2; i++) {
    t_data[i] = input_data[i];
  }
  loop_ub = input->size[0] * input->size[1];
  input->size[0] = 1;
  b_loop_ub = varargin_2_size[1];
  input->size[1] = varargin_2_size[1];
  emxEnsureCapacity_char_T(input, loop_ub);
  input_data = input->data;
  for (i = 0; i < b_loop_ub; i++) {
    input_data[i] = varargin_2_data[i];
  }
  sanitizeInputAndConvertToChar(input);
  input_data = input->data;
  if (input->size[1] < 1) {
    loop_ub = 0;
  } else {
    loop_ub = varargin_2;
  }
  b_loop_ub = input->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    t_data[loop_ub + i] = input_data[i];
  }
  loop_ub = t->size[0] * t->size[1];
  t->size[1] = (int)((unsigned int)varargin_2 + (unsigned int)input->size[1]);
  emxFree_char_T(&input);
  emxEnsureCapacity_char_T(t, loop_ub);
}

/*
 * File trailer for strcat.c
 *
 * [EOF]
 */
