#include "utils.h"
#include "salsa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>

#define PI 3.14159265358979323846

// #define UTILS_TEST

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

void smoothData(float *data, int length, int windowSize)
{
    float *temp = (float *)malloc(length * sizeof(double));
    float sum = 0.0;
    int halfWindow = windowSize / 2;

    for (int i = 0; i < length; i++)
    {
        sum = 0.0;
        int count = 0;

        for (int j = i - halfWindow; j <= i + halfWindow; j++)
        {
            if (j >= 0 && j < length)
            {
                sum += data[j];
                count++;
            }
        }

        temp[i] = sum / count;
    }

    // Copy the smoothed data back to the original array
    for (int i = 0; i < length; i++)
    {
        data[i] = temp[i];
    }

    // Free the temporary array
    free(temp);
}

void computeFFT(complex float *framesBB, complex float *captureFT, int numFrames, int numOfSamplers)
{
    fftwf_plan plan;

    fftwf_complex *in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * numFrames);
    fftwf_complex *out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * numFrames);
    plan = fftwf_plan_dft_1d(numFrames, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    
    for (int j = 0; j < numOfSamplers; j++)
    {
        for (int i = 0; i < numFrames; i++)
        {
            in[i][0] = crealf(framesBB[j + i * numOfSamplers]);
            in[i][1] = cimagf(framesBB[j + i * numOfSamplers]);
        }

        // Execute the FFT
        fftwf_execute(plan);

        // Convert FFTW's complex type to complex float
        for (int i = 0; i < numFrames; ++i)
        {
            captureFT[j + i * numOfSamplers] = out[i][0] + I * out[i][1];
        }
    }

    fftwf_destroy_plan(plan);
    fftwf_free(in);
    fftwf_free(out);
}


// Find local peaks in data
int* findPeaks(float *arr, int size, int *numPeaks, double minPeakHeight) {
    int *peaks = (int *)malloc(size * sizeof(int));
    *numPeaks = 0;
    
    for (int i = 1; i < size - 1; i++) {
        // printf("Peak @ %d: %f\n", i, arr[i]);
        if (arr[i] > arr[i - 1] && arr[i] > arr[i + 1] && arr[i] > minPeakHeight) {
            // printf("Peak @ %d: %f\n", i, arr[i]);
            peaks[*numPeaks] = i;
            (*numPeaks)++;
        }
    }
    return peaks;
}

#ifdef UTILS_TEST
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

    // for (int i = 0; i < 512; i++) {
    //     printf("%f\n", rfSignal[i]);
    // }

    smoothData(rfSignal, 512, 10);

    // for (int i = 0; i < 512; i++) {
    //     printf("%f\n", rfSignal[i]);
    // }

    free(rfSignal);
    free(basebandSignal);
    freeRadarData(radarData);
    return 0;
}
#endif