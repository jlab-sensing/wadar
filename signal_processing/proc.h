#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fftw3.h>
#include <stdbool.h>
#include <complex.h>
#include <string.h>
#include "salsa.h"

typedef struct
{
    bool procSuccess;
    double complex *captureFT;
    double *tagFT;
    int peakBin;
    int SNRdB;
} CaptureData;

// Function processes radar frames for various purposes
CaptureData *procRadarFrames(const char *localDataPath, const char *captureName, double tagHz);

// Find the bin corresponding to the largest peak
int procLargestPeak(double *tagFT);

int procCaptureCWT(double *tagFT, int length);