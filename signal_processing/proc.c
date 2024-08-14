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
    complex float *framesBB;
    complex float *temp;

    rfSignal = (double *)malloc(numOfSamplers * sizeof(double));
    framesBB = (complex float *)malloc((radarData->numFrames) * numOfSamplers * sizeof(complex float));
    temp = (complex float *)malloc(numOfSamplers * sizeof(complex float));

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

    // Find Tag FT
    int freqTag = (int)(tagHz / frameRate * radarData->numFrames);

    complex float *captureFT;
    captureFT = (complex float *)malloc(radarData->numFrames * numOfSamplers * sizeof(complex float *));

    computeFFT(framesBB, captureFT, radarData->numFrames, numOfSamplers);

    // for (int i = 0; i < numOfSamplers * radarData->numFrames; i++) {
    //     printf("%f ", creal(captureFT[i]));
    // }

    float *tagFT;
    tagFT = (float *)malloc(numOfSamplers * sizeof(float *));

    float maxFTPeak;
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
        tagFT[i] = cabs(captureFT[i + numOfSamplers * (idx_maxFTPeak - 1)]);
        // printf("%f\n", tagFT[i]);
    }


    smoothData(tagFT, numOfSamplers, 10);

    int peakBin;
    
    peakBin = procLargestPeak(tagFT);

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
int procLargestPeak(float *tagFT)
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