#include "proc.h"
#include "salsa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>

#define PI 3.14159265358979323846

#define PROC_TEST

void NoveldaDDC(double *rfSignal, complex float *basebandSignal)
{

    double CF;
    double Fs;
    int frameSize;

    CF = 1.8E9;
    Fs = 3.9E10;
    frameSize = 512;

    // Digital Down-Convert parameters (normalized frequency index)
    double freqIndex = CF / Fs * frameSize;

    // Generate the complex sinusoid LO (local oscillator)
    complex float LO[frameSize];
    for (int i = 0; i < frameSize; i++)
    {
        float t = (float)i / (frameSize - 1);
        LO[i] = sin(2 * PI * freqIndex * t) + I * cos(2 * PI * freqIndex * t);
    }

    // Digital Downconvert via direct multiplication
    complex float mean_rfSignal = 0.0f;
    for (int i = 0; i < frameSize; i++)
    {
        mean_rfSignal += rfSignal[i];
    }
    mean_rfSignal /= frameSize;
    for (int i = 0; i < frameSize; i++)
    {
        basebandSignal[i] = (rfSignal[i] - mean_rfSignal) * LO[i];
    }

    // Digital Downconvert (the DDC) via direct multiplication
    // subtracting the mean removes DC offset
    int M = 20;
    float window[M + 1];
    hamming(window, M);

    float filterWeights[M + 1];
    float sum = 0.0;
    for (int i = 0; i <= M / 2; i++)
    {
        sum += window[i];
    }

    for (int i = 0; i <= M; i++)
    {
        filterWeights[i] = window[i] / sum;
    }

    // Baseband signal using convolution (provides downcoverted, filtered analytic signal)
    complex float tempSignal[frameSize];
    int filterSize = M + 1;
    for (int i = 0; i < frameSize; i++)
    {
        tempSignal[i] = 0.0 + 0.0 * I; // Initialize the output to zero
        for (int j = 0; j < filterSize; j++)
        {
            int inputIndex = i + j - ((filterSize) / 2);
            if (inputIndex >= 0 && inputIndex < frameSize)
            {
                tempSignal[i] += basebandSignal[inputIndex] * filterWeights[j];
            }
        }
    }

    for (int i = 0; i < frameSize; i++)
    {
        basebandSignal[i] = tempSignal[i];
    }
}

void hamming(float *window, int M)
{
    for (int n = 0; n <= M; n++)
    {
        window[n] = 0.54 - 0.46 * cos(2.0 * PI * n / M);
    }
}

#ifdef PROC_TEST
int main()
{
    RadarData *radarData = salsaLoad("/home/ericdvet/jlab/wadar/signal_processing/testFile.frames");
    complex float *basebandSignal;
    double *rfSignal;

    rfSignal = (double *)malloc(512 * sizeof(double));
    basebandSignal = (complex float *)malloc(512 * sizeof(complex float));

    for (int i = 0; i < 512; i++)
    {
        rfSignal[i] = radarData->frameTot[i];
    }
    NoveldaDDC(rfSignal, basebandSignal);

    for (int i = 0; i < 512; i++)
    {
        printf("%f + %f\n", creal(basebandSignal[i]), cimag(basebandSignal[i]));
    }

    return 0;
}
#endif