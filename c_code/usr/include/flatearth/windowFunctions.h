/**
   @file windowFunctions.h

   Functions to generate window functions

   @copyright 2017 by FlatEarth, Inc

   @par Environment
   BeagleBone Black + ancho cape

   @par Compiler
   GNU GCC

   @author Raymond Weber
*/

#ifndef RADARDSP_WINDOWFN_h
#define RADARDSP_WINDOWFN_h

#ifdef __cplusplus
extern "C"
{
#endif

void bartlettWindow(double *W, int N);
void welchWindow(double *W, int N);
void hannWindow(double *W, int N);
void hammingWindow(double *W, int N);
void bohmanWindow(double *W, int N);

void blackmanWindow(double *W, int N);
void blackmanHarrisWindow(double *W, int N);
void blackmanNuttallWindow(double *W, int N);

void gaussianWindow(double *W, int N);
void flattopWindow(double *W, int N);

#ifdef __cplusplus
}
#endif
#endif
