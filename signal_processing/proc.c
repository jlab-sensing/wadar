/*
 * File:   proc.h
 * Author: ericdvet
 *
 * Functions to process .frames data to obtain tag information
 */

#include "proc.h"
#include "utils.h"
#include "salsa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include "wavelib/header/wavelib.h"

/**
 * @function procRadarFrames(const char *localDataPath, const char *captureName, double tagHz)
 * @param localDataPath - Local file path to radar capture
 * @param captureName - Name of radar capture file
 * @param captureName - Frequency at which tag is oscillating in Hz
 * @return CaptureData *
 * @brief Function processes radar frames for various purposes
 * @author ericdvet */
CaptureData *procRadarFrames(const char *localDataPath, const char *captureName, double tagHz)
{

    CaptureData *captureData = (CaptureData *)malloc(sizeof(CaptureData));

    // Processing parameters
    int frameRate = 200;
    int numOfSamplers = 512;

    // Load Capture
    char fullPath[1024];
    sprintf(fullPath, "%s/%s", localDataPath, captureName);
    RadarData *radarData = salsaLoad(fullPath);
    if (radarData == NULL)
    {
        return false;
    }

    double *rfSignal;
    double complex *framesBB;
    double complex *temp;

    rfSignal = (double *)malloc(numOfSamplers * sizeof(double));
    framesBB = (double complex *)malloc((radarData->numFrames) * numOfSamplers * sizeof(double complex));
    temp = (double complex *)malloc(numOfSamplers * sizeof(double complex));

    // Baseband Conversion
    for (int i = 0; i < radarData->numFrames; i++)
    {
        for (int j = 0; j < numOfSamplers; j++)
        {
            rfSignal[j] = radarData->frameTot[j + i * numOfSamplers];
        }
        NoveldaDDC(rfSignal, temp);
        for (int j = 0; j < numOfSamplers; j++)
        {
            framesBB[j + i * numOfSamplers] = temp[j];
            // printf("%d ", j + i * numOfSamplers);
        }
    }

    // for (int i = 500; i < numOfSamplers; i++) {
    //     printf("%f ", creal(framesBB[i]));
    // }
    // printf("\n");

    // Find Tag FT
    int freqTag = (int)(tagHz / frameRate * radarData->numFrames);

    captureData->captureFT = (double complex *)malloc(radarData->numFrames * numOfSamplers * sizeof(double complex));

    computeFFT(framesBB, captureData->captureFT, radarData->numFrames, numOfSamplers);

    // for (int i = 0; i < numOfSamplers; i++) {
    //     printf("%f\n", creal(captureData->captureFT[i]));
    // }

    captureData->tagFT = (double *)malloc(numOfSamplers * sizeof(double *));

    double maxFTPeak;
    int idx_maxFTPeak;
    maxFTPeak = 0;

    for (int j = freqTag - 2; j <= freqTag + 2; j++)
    {
        for (int i = 0; i < numOfSamplers; i++)
        {
            if (cabs(captureData->captureFT[i + numOfSamplers * (j - 1)]) > maxFTPeak)
            {
                maxFTPeak = cabs(captureData->captureFT[i + numOfSamplers * (j - 1)]);
                idx_maxFTPeak = j;
            }
        }
    }
    freqTag = idx_maxFTPeak;

    for (int i = 0; i < numOfSamplers; i++)
    {
        captureData->tagFT[i] = (double)cabs(captureData->captureFT[i + numOfSamplers * (idx_maxFTPeak - 1)]);
        // printf("%f\n", captureData->tagFT[i]);
    }

    // smoothData(captureData->tagFT, numOfSamplers, 10);

    for (int i = 0; i < numOfSamplers; i++)
    {
        // captureData->tagFT[i] = (float) cabs(captureData->captureFT[i + numOfSamplers * (idx_maxFTPeak)]);
        // printf("%d: %f\n", i, captureData->tagFT[i]);
    }

    int peakBin;
    peakBin = procLargestPeak(captureData->tagFT);
    // peakBin = procCaptureCWT(captureData->tagFT, 512);

    // printf("\nPeak of %f at %d\n", captureData->tagFT[peakBin], peakBin);

    double SNR;
    SNR = calculateSNR(captureData->captureFT, numOfSamplers, freqTag, peakBin);

    // printf("SNR of %f\n", SNR);

    procCaptureCWT(captureData->tagFT);

    free(rfSignal);
    free(framesBB);
    free(temp);
    freeRadarData(radarData);

    return captureData;
}

/**
 * @function freeCaptureData(CaptureData *captureData)
 * @param captureData - CaptureData struct to free
 * @return None
 * @brief Free RadarData constructed by procRadarFrames()
 * @author ericdvet */
void freeCaptureData(CaptureData *captureData)
{
    if (captureData)
    {
        free(captureData->tagFT);
        free(captureData->captureFT);
        free(captureData);
    }
}

/**
 * @function procLargestPeak(double *tagFT)
 * @param *tagFT - pointer to FT of the tag's frequency isolated
 * @return int
 * @brief Returns bin corresponding to the largest peak
 * @author ericdvet */
int procLargestPeak(double *tagFT)
{
    double maxVal = -1;
    int size = 512;

    for (int i = 0; i < size; i++)
    {
        if (tagFT[i] > maxVal)
        {
            maxVal = tagFT[i];
        }
    }

    double minPeakHeight = maxVal * 0.9;
    int numPeaks = 0;
    int *peaks = findPeaks(tagFT, size, &numPeaks, minPeakHeight);

    int peakBin = -1;

    if (numPeaks > 1)
    {
        if ((peaks[1] - peaks[0]) < 50)
        {
            peakBin = peaks[0] + round((peaks[1] - peaks[0]) / 2.0) + round((tagFT[peaks[1]] - tagFT[peaks[0]]) / maxVal * (peaks[1] - peaks[0]));
        }
        else
        {
            peakBin = peaks[0];
        }
    }
    else if (numPeaks == 1)
    {
        peakBin = peaks[0];
    }

    free(peaks);
    return peakBin;
}

int procCaptureCWT(double *tagFT)
{

    // Control variables
    int gapThreshold = 5;
    double slidingWindowThreshold = 0.3; // 0.5;
    float SNRThreshold = 3.0;
    double *scales;
    scales = (double *)malloc(32 * sizeof(double));
    for (int i = 1; i < 64; i += 2)
    {
        scales[i] = (double)i;
        // printf("%f ", scales[i]);
    }
    float ridgeLengthThreshold = 12.0;

    // //     %% Continuous Wavelet Transform
    // // scales = [scales; 1:length(scales)];
    // // scale_idxs = 1:length(scales);
    // // cwtInfo = cwtft(tagFT, 'wavelet', 'mexh', 'scales', scales);
    // // cwtCoeffs = cwtInfo.cfs;
    // // amplThreshold = 0.1 * max(max(cwtCoeffs));

    cwt_object cwtInfo;
    cwtInfo = cwt_init("dog", 2, 512, 1, 32);
    // setCWTScaleVector(cwtInfo, scales, 32, 1, 1);
    setCWTScales(cwtInfo, 1, 2, "linear", 1);

    cwt(cwtInfo, tagFT);

    double amplThreshold;
    amplThreshold = 0;
    for (int i = 0; i < cwtInfo->J * cwtInfo->siglength; i++)
    {
        if (cwtInfo->output[i].re > amplThreshold)
        {
            amplThreshold = cwtInfo->output[i].re;
        }
    }
    amplThreshold = 0.1 * amplThreshold;

    double *cwtScaleCoeffs;
    int *numPeaks;
    int **peaks;

    peaks = (int **)malloc(cwtInfo->J * sizeof(int *));
    numPeaks = (int *)malloc(cwtInfo->J * sizeof(int));
    cwtScaleCoeffs = (double *)malloc(512 * sizeof(double));

    for (int i = 0; i < cwtInfo->J * cwtInfo->siglength; i++)
    {
        cwtScaleCoeffs[i % 512] = fabs(cwtInfo->output[i].re);
        if (i % 512 == 0 && i != 0)
        {
            printf("%d: ", (int)i / 512);
            peaks[i / 512] = findPeaks(cwtScaleCoeffs, 512, &(numPeaks[i / 512]), 0);
            // printf("%d entries: ", numPeaks[i / 512]);
            for (int j = 0; j < numPeaks[i / 512]; j++)
            {
                printf("%d ", peaks[i / 512][j]);
            }
            printf("\n");
        }
    }
    peaks[(cwtInfo->J * cwtInfo->siglength) / 512] = findPeaks(cwtScaleCoeffs, 512, &(numPeaks[(cwtInfo->J * cwtInfo->siglength) / 512]), 0);

    
    //     for (int scale = M-1; scale >= 0; scale--) {
    //     for (int i = 0; i < N; i++) {
    //         if (locMax[scale][i] == 1) {
    //             RidgeLine ridgeLine;
    //             ridgeLine.length = 1;
    //             ridgeLine.points[0] = (RidgePoint){scale, i};

    //             int gap = 0;
    //             for (int s = scale-1; s >= 0 && gap <= gapThreshold; s--) {
    //                 int closestMaxIdx = -1;
    //                 double closestGap = slidingWindowThreshold * s;
    //                 for (int j = 0; j < N; j++) {
    //                     if (locMax[s][j] == 1) {
    //                         double currentGap = fabs(i - j);
    //                         if (currentGap < closestGap) {
    //                             closestGap = currentGap;
    //                             closestMaxIdx = j;
    //                         }
    //                     }
    //                 }
    //                 if (closestMaxIdx != -1) {
    //                     ridgeLine.points[ridgeLine.length++] = (RidgePoint){s, closestMaxIdx};
    //                     locMax[s][closestMaxIdx] = 0; // Remove the maximum
    //                     gap = 0;
    //                 } else {
    //                     gap++;
    //                 }
    //             }
    //             if (ridgeLine.length > 1) {
    //                 ridgeLines[*numRidgeLines] = ridgeLine;
    //                 (*numRidgeLines)++;
    //             }
    //         }
    //     }
    // }

    // // Find all ridge lines
    // RidgeLine ridgeLines[MAX_RIDGE_LINES];
    // int numRidgeLines = 0;
    // findRidgeLines(locMax, ridgeLines, &numRidgeLines);

    // // Process ridges and find the peak
    // int tagPeakBin = findPeak(tagFT, cwtCoeffs, ridgeLines, numRidgeLines);

    /////

    // // printf("%d x %d\n", cwtInfo->J, cwtInfo->siglength);
    // for (int i = 0; i < cwtInfo->J; i ++) {
    //     // printf("%f ", cwtInfo->scale[i]);
    // }
    // for (int i = 0; i < 512; i++) {
    //     // printf("%f\n", sqrt(cwtInfo->output[i].re * cwtInfo->output[i].re + cwtInfo->output[i].im * cwtInfo->output[i].im));
    //     // printf("%f + i%f\n", cwtInfo->output[i].re, cwtInfo->output[i].im);
    // }

    // FILE *file = fopen("cwt_output.dat", "w");
    // for (int i = 0; i < 512; i++) {
    //     fprintf(file, "%d %f\n", i, cwtInfo->output[i].re);
    // }
    // fclose(file);

    // cwt_summary(cwtInfo);
}

#define PROC_TEST

#ifdef PROC_TEST
int main()
{
    CaptureData *captureData;
    captureData = procRadarFrames("/home/ericdvet/jlab/wadar/signal_processing/", "testFile.frames", 80);

    freeCaptureData(captureData);

    return 0;
}
#endif