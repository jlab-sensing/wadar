/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 28-Mar-2025 10:33:50
 */

/* Include Files */
#include "main.h"
#include "RunTagDataEmbedded.h"
#include "RunTagDataEmbedded_terminate.h"
#include "rt_nonfinite.h"
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
  return 0.0;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char *argv[])
{
  // (void)argc;
  // (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */

  int captureName_size[2];
  int localDataPath_size[2];
  char captureName_data[100];
  char localDataPath_data[100];
  double tagHz;

  if (argc < 7) {
    printf("Usage: ./main -captureName <captureName> -localDataPath <localDataPath> -tagHz <tagHz>\n");
    return 1;
  }

  int i;
  for (i = 0; i < argc; i++) {
    if (strcmp(argv[i], "-captureName") == 0) {
      int j;
      for (j = 0; j < strlen(argv[i + 1]); j++) {
        captureName_data[j] = argv[i + 1][j];
      }
      captureName_size[0] = 1;
      captureName_size[1] = strlen(argv[i + 1]);
    }
    if (strcmp(argv[i], "-localDataPath") == 0) {
      int j;
      for (j = 0; j < strlen(argv[i + 1]); j++) {
        localDataPath_data[j] = argv[i + 1][j];
      }
      localDataPath_size[0] = 1;
      localDataPath_size[1] = strlen(argv[i + 1]);
    }
    if (strcmp(argv[i], "-tagHz") == 0) {
      tagHz = atof(argv[i + 1]);
    }
  }

  double SNRdB;
  double peakBin;

  /* From when I hardcoded the values */
  // char captureName_data[] = "105Tag.frames";
  // char localDataPath_data[] = "/data";
  // int captureName_size[2];
  // int localDataPath_size[2];
  // captureName_size[0] = 1;
  // captureName_size[1] = 17;
  // localDataPath_size[0] = 1;
  // localDataPath_size[1] = 5;
  // double tagHz = 105;

  printf("Input arguments:\n");
  printf("captureName: %s\n", captureName_data);
  printf("localDataPath: %s\n", localDataPath_data);
  printf("tagHz: %f\n", tagHz);

  /* Call the entry-point 'RunTagDataEmbedded'. */
  RunTagDataEmbedded(captureName_data, captureName_size, 
  localDataPath_data, localDataPath_size, tagHz, &peakBin, &SNRdB);

  printf("Output arguments:\n");
  printf("peakBin: %f\n", peakBin);
  printf("SNRdB: %f\n", SNRdB);
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
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
