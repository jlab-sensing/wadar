/*
 * File:   utils.h
 * Author: ericdvet
 *
 * Utility functions to assist with WaDAR's data and signal processing functions
 */

#include "utils.h"
#include "salsa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>

#define PI 3.14159265358979323846

/**
 * @function NoveldaDDC(double *rfSignal, double complex *basebandSignal)
 * @param rfSignal - Raw radar frames
 * @param basebandSignal - Resulting digitally downcoverted radar frames
 * @return None
 * @brief Function to apply a digital downcovert (DDC) to a high frequency radar
 *      signal. Brings signal to baseband frequencies and provides an analytic
 *      signal (i.e. I & Q, in-phase & quadrature, outputs)
 * @author ericdvet */
void NoveldaDDC(double *rfSignal, double complex *basebandSignal)
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
    double complex LO[frameSize];
    for (int i = 0; i < frameSize; i++)
    {
        double t = (double)i / (frameSize - 1);
        LO[i] = sin(2 * PI * freqIndex * t) + I * cos(2 * PI * freqIndex * t);
    }

    // Digital Downconvert via direct multiplication
    double complex mean_rfSignal = 0.0;
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
    double window[M + 1];
    hamming(window, M);

    double filterWeights[M + 1];
    double sum = 0.0;
    for (int i = 0; i <= M / 2; i++)
    {
        sum += window[i];
    }

    for (int i = 0; i <= M; i++)
    {
        filterWeights[i] = window[i] / sum;
    }

    // Baseband signal using convolution (provides downcoverted, filtered analytic signal)
    double complex tempSignal[frameSize];
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

/**
 * @function hamming(double *window, int M)
 * @param window - Resulting hamming window
 * @param M - Size of window
 * @return None
 * @brief Returns the N-point symmetric Hamming window in a column vector
 * @author ericdvet */
void hamming(double *window, int M)
{
    for (int n = 0; n <= M; n++)
    {
        window[n] = 0.54 - 0.46 * cos(2.0 * PI * n / M);
    }
}

/**
 * @function smoothData(double *data, int length, int windowSize)
 * @param *data - Data to smooth
 * @param length - Length of data
 * @param windowSize - Length of smoothing window
 * @return None
 * @brief Smooth noisy data by averaging over each window of windowSize.
 * @author ericdvet */
void smoothData(double *data, int length, int windowSize)
{
    double *temp = (double *)malloc(length * sizeof(double));
    double sum = 0.0;
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

/**
 * @function computeFFT(double complex *framesBB, double complex *captureFT, int numFrames, int numOfSamplers)
 * @param *framesBB - Input of FFT
 * @param captureFT - Output of FFT
 * @param numFrames - Number of frames (total columns)
 * @param numOfSamplers - Number of samplers (total rows)
 * @return None
 * @brief Fast Fourier transform (FFT) of input
 *      The FFT block computes the fast Fourier transform (FFT) across the first
 *      dimension of an N-D input array, u.
 * @author ericdvet */
void computeFFT(double complex *framesBB, double complex *captureFT, int numFrames, int numOfSamplers)
{
    fftw_plan plan;

    fftw_complex *in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * numFrames);
    fftw_complex *out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * numFrames);
    plan = fftw_plan_dft_1d(numFrames, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    for (int j = 0; j < numOfSamplers; j++)
    {

        for (int i = 0; i < numFrames; i++)
        {
            in[i][0] = creal(framesBB[j + i * numOfSamplers]);
            in[i][1] = cimag(framesBB[j + i * numOfSamplers]);
        }

        // Execute the FFT
        fftw_execute(plan);

        // Convert FFTW's complex type to double complex
        for (int i = 0; i < numFrames; ++i)
        {
            captureFT[j + i * numOfSamplers] = out[i][0] + I * out[i][1];
        }
    }

    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);
}

/**
 * @function findPeaks(double *arr, int size, int *numPeaks, double minPeakHeight)
 * @param *arr - Array to find peaks from
 * @param size - Length of array
 * @param *numPeaks - Resulting number of peaks
 * @param minPeakHeight - Minimum peak height
 * @return int
 * @brief Returns local peaks in data array
 * @author ericdvet */
int *findPeaks(double *arr, int size, int *numPeaks, double minPeakHeight)
{
    int *peaks = (int *)malloc(size * sizeof(int));
    *numPeaks = 0;

    for (int i = 1; i < size - 1; i++)
    {
        // printf("Peak @ %d: %f\n", i, arr[i]);
        if (arr[i] > arr[i - 1] && arr[i] > arr[i + 1] && arr[i] > minPeakHeight)
        {
            // printf("Peak @ %d: %f\n", i, arr[i]);
            peaks[*numPeaks] = i;
            (*numPeaks)++;
        }
    }
    return peaks;
}

/**
 * @function calculateSNR(double complex *captureFT, int numOfSamplers, int freqTag, int peakBin)
 * @param *captureFT - FT of radar frames
 * @param numOfSamplers - Number of sampelrs (rows)
 * @param freqTag - FT isolation of backscatter tag
 * @param peakBin - Determined peak bin location of backscatter tag
 * @return float
 * @brief Returns signal to noise ratio. Calculates ratio of peak bin amplitude at desired 
 *      frequency vs an average of a few irrelevant frequencies 
 * @author ericdvet */
double calculateSNR(double complex *captureFT, int numOfSamplers, int freqTag, int peakBin)
{
    double signalMag;
    signalMag = cabs(captureFT[peakBin + numOfSamplers * (freqTag - 1)]);


    int noiseFreqLowBound;
    int noiseFreqHighBound;
    noiseFreqLowBound = (int)freqTag * 0.945;
    noiseFreqHighBound = (int)freqTag * 0.955;

    double noiseMag;
    noiseMag = 0;
    for (int j = noiseFreqLowBound; j < noiseFreqHighBound; j++) {
        noiseMag += cabs(captureFT[peakBin + numOfSamplers * (j-1)]);
    }

    noiseMag = noiseMag / (noiseFreqHighBound - noiseFreqLowBound);

    double SNR;
    SNR = signalMag / noiseMag;
    return (10 * log10(SNR));
}

/**
 * @function compare(const void *a, const void *b)
 * @param *a - First number to compare
 * @param *b - Second number to compare
 * @return int
 * @brief Compare function for qsort() to sort in ascending order
 * @author ericdvet */
int compare(const void *a, const void *b) {
    double arg1 = *(const double *)a;
    double arg2 = *(const double *)b;
    if (arg1 < arg2) return -1;
    if (arg1 > arg2) return 1;
    return 0;
}

/**
 * @function median(double *arr, int length)
 * @param *arr - Array to find median from
 * @param length - Length of array
 * @return double
 * @brief Returns the median of an array of numbers
 * @author ericdvet */
double median(double *arr, int length) {
    qsort(arr, length, sizeof(double), compare);
    if (length % 2 == 0) {
        return (arr[length/2 - 1] + arr[length/2]) / 2.0;
    } else {
        return arr[length/2];
    }
}

// #define UTILS_TEST

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