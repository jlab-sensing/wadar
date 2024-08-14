#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fftw3.h>
#include <stdbool.h>
#include <complex.h>
#include <string.h>
#include "salsa.h"

// Function to apply a digital downcovert (DDC) to a high frequency radar
// signal. Brings signal to baseband frequencies and provides an analytic
// signal (i.e. I & Q, in-phase & quadrature, outputs)
void NoveldaDDC(double *rfSignal, double complex *basebandSignal);

// Returns the N-point symmetric Hamming window in a column vector
void hamming(double *window, int M);

// Smooth noisy data by averaging over each window of windowSize.
void smoothData(double *data, int length, int windowSize);

void computeFFT(double complex *framesBB, double complex *captureFT, int numFrames, int numOfSamplers);

// Find local peaks in data
int* findPeaks(double *arr, int size, int *numPeaks, double minPeakHeight);