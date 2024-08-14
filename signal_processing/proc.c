#include "proc.h"
#include "utils.h"
#include "salsa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>

bool procRadarFrames(const char *localDataPath, const char *captureName, double tagHz)
{
    double *rawFrames;

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

    float *tagFT;
    tagFT = (float *)malloc(numOfSamplers * sizeof(float *));

    for (int i = 0; i < numOfSamplers; i++) {
        tagFT[i] = cabs(captureFT[i + numOfSamplers * (freqTag - 1)]);
        printf("%d: %f\n", i + numOfSamplers * freqTag, tagFT[i]);
        // printf("%d\n", freqTag);
    }


    // for (int i = 0; i < radarData->numFrames; i++)
    // {
    //     (*captureFT)[i] = (complex float *)malloc(radarData->numFrames * sizeof(complex float));
    //     // Apply FFT to framesBB[i] and store in captureFT[i]
    //     // [Fill in your FFT implementation here]
    // }

    free(tagFT);
    free(rfSignal);
    free(framesBB);
    free(temp);
    free(captureFT);
    freeRadarData(radarData);

    return true;
}

#define PROC_TEST

#ifdef PROC_TEST
int main()
{
    procRadarFrames("/home/ericdvet/jlab/wadar/signal_processing/", "testFile.frames", 80);

    return 0;
}
#endif