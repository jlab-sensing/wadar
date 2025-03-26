/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: string1.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 26-Mar-2025 15:29:23
 */

#ifndef STRING1_H
#define STRING1_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T b_string_eq(const char obj2_Value_data[],
                      const int obj2_Value_size[2]);

boolean_T c_string_eq(const char obj2_Value_data[],
                      const int obj2_Value_size[2]);

boolean_T d_string_eq(const char obj2_Value_data[],
                      const int obj2_Value_size[2]);

boolean_T string_eq(const char obj2_Value_data[], const int obj2_Value_size[2]);

void string_lower(const char obj_Value_data[], const int obj_Value_size[2],
                  char y_Value_data[], int y_Value_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for string1.h
 *
 * [EOF]
 */
