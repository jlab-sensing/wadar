/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: ifWhileCond.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/* Include Files */
#include "ifWhileCond.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const bool x_data[]
 *                int x_size
 * Return Type  : bool
 */
bool ifWhileCond(const bool x_data[], int x_size)
{
  bool y;
  y = (x_size != 0);
  if (y) {
    int k;
    bool exitg1;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= x_size - 1)) {
      if (!x_data[0]) {
        y = false;
        exitg1 = true;
      } else {
        k = 1;
      }
    }
  }
  return y;
}

/*
 * File trailer for ifWhileCond.c
 *
 * [EOF]
 */
