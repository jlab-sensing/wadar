/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "RunTagDataEmbedded.h"
#include "RunTagDataEmbedded_terminate.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void argInit_1xd100_char_T(char result_data[], int result_size[2]);

static char argInit_char_T(void);

static double argInit_real_T(void);

/* Function Definitions */
/*
 * Arguments    : char result_data[]
 *                int result_size[2]
 * Return Type  : void
 */
static void argInit_1xd100_char_T(char result_data[], int result_size[2])
{
  int idx1;
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result_size[0] = 1;
  result_size[1] = 2;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 2; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result_data[idx1] = argInit_char_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : char
 */
static char argInit_char_T(void)
{
  return '?';
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_RunTagDataEmbedded();
  /* Terminate the application.
You do not need to do this more than one time. */
  RunTagDataEmbedded_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_RunTagDataEmbedded(void)
{
  double SNRdB;
  double peakBin;
  int captureName_size[2];
  int localDataPath_size[2];
  char captureName_data[100];
  char localDataPath_data[100];
  /* Initialize function 'RunTagDataEmbedded' input arguments. */
  /* Initialize function input argument 'captureName'. */
  argInit_1xd100_char_T(captureName_data, captureName_size);
  /* Initialize function input argument 'localDataPath'. */
  argInit_1xd100_char_T(localDataPath_data, localDataPath_size);
  /* Call the entry-point 'RunTagDataEmbedded'. */
  RunTagDataEmbedded(captureName_data, captureName_size, localDataPath_data,
                     localDataPath_size, argInit_real_T(), &peakBin, &SNRdB);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
