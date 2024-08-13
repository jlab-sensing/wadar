#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fftw3.h>
#include <stdbool.h>
#include <complex.h>
#include <string.h>
#include "salsa.h"

void smoothData(double *data, int length, int windowSize);

int procCaptureCWT(double *tagFT, bool displayFlag);

double calculateSNR(double complex *captureFT, int freqTag, int peakBin);

// bool procRadarFrames(const char* localDataPath, const char* captureName, double tagHz,
//                      bool* procSuccess, double** captureFT, double** tagFT,
//                      int* peakBin, double* SNRdB);