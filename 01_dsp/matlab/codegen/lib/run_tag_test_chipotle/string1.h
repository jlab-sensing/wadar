/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: string1.h
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
bool b_string_eq(const char obj2_Value_data[], const int obj2_Value_size[2]);

bool c_string_eq(const char obj2_Value_data[], const int obj2_Value_size[2]);

bool d_string_eq(const char obj2_Value_data[], const int obj2_Value_size[2]);

bool string_eq(const char obj2_Value_data[], const int obj2_Value_size[2]);

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
