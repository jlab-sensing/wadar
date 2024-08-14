#include "proc.h"
#include "utils.h"
#include "salsa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>

bool procRadarFrames(const char *localDataPath, const char *captureName, double tagHz)
{
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

    double complex *captureFT;
    captureFT = (double complex *)malloc(radarData->numFrames * numOfSamplers * sizeof(double complex));

    computeFFT(framesBB, captureFT, radarData->numFrames, numOfSamplers);

    // for (int i = 0; i < numOfSamplers; i++) {
    //     printf("%f\n", creal(captureFT[i]));
    // }

    double *tagFT;
    tagFT = (double *)malloc(numOfSamplers * sizeof(double *));

    double maxFTPeak;
    int idx_maxFTPeak;
    maxFTPeak = 0;

    for (int j = freqTag-2; j <= freqTag+2; j++) {
        for (int i = 0; i < numOfSamplers; i++)
        {
            if (cabs(captureFT[i + numOfSamplers * (j - 1)]) > maxFTPeak) {
                maxFTPeak = cabs(captureFT[i + numOfSamplers * (j - 1)]);
                idx_maxFTPeak = j;
            }
        }
    }
    freqTag = idx_maxFTPeak;

    for (int i = 0; i < numOfSamplers; i++)
    {
        tagFT[i] = (double) cabs(captureFT[i + numOfSamplers * (idx_maxFTPeak - 1)]);
        // printf("%f\n", tagFT[i]);
    }


    // smoothData(tagFT, numOfSamplers, 10);

    for (int i = 0; i < numOfSamplers; i++)
    {
        // tagFT[i] = (float) cabs(captureFT[i + numOfSamplers * (idx_maxFTPeak)]);
        printf("%d: %f\n", i, tagFT[i]);
    }

    int peakBin;
    peakBin = procLargestPeak(tagFT);
    // peakBin = procCaptureCWT(tagFT, 512);

    printf("\nPeak of %f at %d\n\n", tagFT[peakBin], peakBin);

    free(tagFT);
    free(rfSignal);
    free(framesBB);
    free(temp);
    free(captureFT);
    freeRadarData(radarData);

    return true;
}

// int findPeakBin(double *tagFT, int size) {
// Find the bin corresponding to the largest peak
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

#define PROC_TEST

#ifdef PROC_TEST
int main()
{
    procRadarFrames("/home/ericdvet/jlab/wadar/signal_processing/", "testFile.frames", 80);

    return 0;
}
#endif