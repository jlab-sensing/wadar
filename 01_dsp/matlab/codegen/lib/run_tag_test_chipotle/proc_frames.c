/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: proc_frames.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "proc_frames.h"
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle_emxutil.h"
#include "run_tag_test_chipotle_types.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *in1
 *                const emxArray_real_T *in2
 *                const double in3_data[]
 *                const double in4_data[]
 *                const int *in4_size
 *                const double in5_data[]
 *                const double in6_data[]
 * Return Type  : void
 */
void binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                      const double in3_data[], const double in4_data[],
                      const int *in4_size, const double in5_data[],
                      const double in6_data[])
{
  emxArray_real_T *b_in2;
  emxArray_real_T *c_in2;
  const double *in2_data;
  double *b_in2_data;
  double *in1_data;
  int i;
  int i1;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in2_data = in2->data;
  stride_1_0 = in2->size[0];
  loop_ub = in2->size[1];
  emxInit_real_T(&b_in2, 1);
  stride_0_0 = b_in2->size[0];
  b_in2->size[0] = stride_1_0;
  emxEnsureCapacity_real_T(b_in2, stride_0_0);
  b_in2_data = b_in2->data;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < stride_1_0; i1++) {
      b_in2_data[i1] = in2_data[i1] * in3_data[0];
    }
  }
  emxInit_real_T(&c_in2, 1);
  if (*in4_size == 1) {
    loop_ub = b_in2->size[0];
  } else {
    loop_ub = *in4_size;
  }
  stride_0_0 = c_in2->size[0];
  c_in2->size[0] = loop_ub;
  emxEnsureCapacity_real_T(c_in2, stride_0_0);
  in1_data = c_in2->data;
  stride_0_0 = (b_in2->size[0] != 1);
  stride_1_0 = (*in4_size != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in2_data[i * stride_0_0] + in4_data[i * stride_1_0];
  }
  stride_0_0 = b_in2->size[0];
  b_in2->size[0] = loop_ub;
  emxEnsureCapacity_real_T(b_in2, stride_0_0);
  b_in2_data = b_in2->data;
  for (i = 0; i < loop_ub; i++) {
    b_in2_data[i] = in1_data[i];
  }
  emxFree_real_T(&c_in2);
  stride_1_0 = (int)in5_data[0];
  loop_ub = (int)in6_data[0];
  stride_0_0 = in1->size[0] * in1->size[1];
  in1->size[0] = stride_1_0;
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(in1, stride_0_0);
  in1_data = in1->data;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < stride_1_0; i1++) {
      in1_data[i1 + in1->size[0] * i] = b_in2_data[i1 + stride_1_0 * i];
    }
  }
  emxFree_real_T(&b_in2);
}

/*
 * File trailer for proc_frames.c
 *
 * [EOF]
 */
