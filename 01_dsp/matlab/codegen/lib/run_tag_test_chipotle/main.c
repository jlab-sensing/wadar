/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: main.c
 *
 * MATLAB Coder version            : 25.1
 * C/C++ source code generated on  : 26-Mar-2025 16:20:55
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
#include "rt_nonfinite.h"
#include "run_tag_test_chipotle.h"
#include "run_tag_test_chipotle_initialize.h"
#include "run_tag_test_chipotle_terminate.h"
#include <string.h>
#include <stdio.h>

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
  return 80;
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
  /* Initialize the application.
You do not need to do this more than one time. */
  run_tag_test_chipotle_initialize();
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_run_tag_test_chipotle();
  /* Terminate the application.
You do not need to do this more than one time. */
  run_tag_test_chipotle_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_run_tag_test_chipotle(void)
{
  double SNRdB;
  double peakBin;
  int captureName_size[2];
  int localDataPath_size[2];
  char captureName_data[100];
  char localDataPath_data[100];

  /* Initialize function 'run_tag_test_chipotle' input arguments. */
  /* Set the size of the arrays. */
  captureName_size[0] = 1;
  captureName_size[1] = strlen("2025-2-25_64105_assembly_tag_1_C1.frames");
  localDataPath_size[0] = 1;
  localDataPath_size[1] = strlen("/data");

  /* Set the values of the arrays. */
  strcpy(captureName_data, "2025-2-25_64105_assembly_tag_1_C1.frames");
  strcpy(localDataPath_data, "/data");
  double tagHz = 105;
  
  /* Initialize function 'run_tag_test_chipotle' input arguments. */
  /* Initialize function input argument 'captureName'. */
  // argInit_1xd100_char_T(captureName_data, captureName_size);
  /* Initialize function input argument 'localDataPath'. */
  // argInit_1xd100_char_T(localDataPath_data, localDataPath_size);
  /* Call the entry-point 'run_tag_test_chipotle'. */

  printf("Parameters sent in\n");
  printf("captureName_data: %s\n", captureName_data);
  printf("localDataPath_data: %s\n", localDataPath_data);

  printf("argInit_real_T(): %f\n", tagHz);

  run_tag_test_chipotle(captureName_data, captureName_size, localDataPath_data,
                        localDataPath_size, tagHz, &peakBin, &SNRdB);

  printf("Peak Bin: %f\n", peakBin);
  printf("SNRdB: %f\n", SNRdB);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
