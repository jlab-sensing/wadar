/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ProcessFrames.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "ProcessFrames.h"
#include "RunTagDataEmbedded_emxutil.h"
#include "RunTagDataEmbedded_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *in1
 *                const emxArray_real_T *in2
 *                const double in3_data[]
 *                const int *in3_size
 *                const double in4_data[]
 *                const double in5_data[]
 * Return Type  : void
 */
void binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                      const double in3_data[], const int *in3_size,
                      const double in4_data[], const double in5_data[])
{
  emxArray_real_T *b_in2;
  const double *in2_data;
  double *b_in2_data;
  double *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in2_data = in2->data;
  emxInit_real_T(&b_in2, 1);
  if (*in3_size == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = *in3_size;
  }
  i = b_in2->size[0];
  b_in2->size[0] = loop_ub;
  emxEnsureCapacity_real_T(b_in2, i);
  b_in2_data = b_in2->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (*in3_size != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in2_data[i] = in2_data[i * stride_0_0] + in3_data[i * stride_1_0];
  }
  loop_ub = (int)in4_data[0];
  stride_0_0 = (int)in5_data[0];
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  in1->size[1] = stride_0_0;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  for (i = 0; i < stride_0_0; i++) {
    for (stride_1_0 = 0; stride_1_0 < loop_ub; stride_1_0++) {
      in1_data[stride_1_0 + in1->size[0] * i] =
          b_in2_data[stride_1_0 + loop_ub * i];
    }
  }
  emxFree_real_T(&b_in2);
}

/*
 * File trailer for ProcessFrames.c
 *
 * [EOF]
 */
