/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: rtGetInf.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
 */

/*
 * Abstract:
 *       MATLAB for code generation function to initialize non-finite, Inf and
 * MinusInf
 */
/* Include Files */
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/*
 * Function: rtGetInf
 * ================================================================== Abstract:
 *  Initialize rtInf needed by the generated code.
 */
real_T rtGetInf(void)
{
  return rtInf;
}

/*
 * Function: rtGetInfF
 * ================================================================= Abstract:
 *  Initialize rtInfF needed by the generated code.
 */
real32_T rtGetInfF(void)
{
  return rtInfF;
}

/*
 * Function: rtGetMinusInf
 * ============================================================= Abstract:
 *  Initialize rtMinusInf needed by the generated code.
 */
real_T rtGetMinusInf(void)
{
  return rtMinusInf;
}

/*
 * Function: rtGetMinusInfF
 * ============================================================ Abstract:
 *  Initialize rtMinusInfF needed by the generated code.
 */
real32_T rtGetMinusInfF(void)
{
  return rtMinusInfF;
}

/*
 * File trailer for rtGetInf.c
 *
 * [EOF]
 */
