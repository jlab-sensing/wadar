#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "proc.h"

// Capture parameters
#define FRAME_RATE 200
#define RADAR_TYPE "Chipotle"
#define SOIL_TYPE "farm"

double wadar(char *localDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount, double tagDepth) {

    double volumetricWaterContent = 0.0;

    // Process Air Capture
    CaptureData* airCapture = procRadarFrames(localDataPath, airFramesName, tagHz);
    if (!airCapture || !airCapture->procSuccess) {
        printf("ERROR: Air Frames Invalid\n");
        return -1;
    }
    int airPeakBin = airCapture->peakBin;

    // Load soil capture
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char captureName[256];
    snprintf(captureName, sizeof(captureName), "%d-%02d-%02d_%dmmDepth_%s_C", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, (int)(tagDepth * 1000), trialName);

    // Frame logger command
    char frameLoggerCommand[1024];
    snprintf(frameLoggerCommand, sizeof(frameLoggerCommand),
             "ssh root@192.168.7.2 \"screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger -s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s \" &",
             captureName, frameCount, captureCount, FRAME_RATE, RADAR_TYPE, localDataPath);
    system(frameLoggerCommand);

    // Load and Process Captures
    double SNRdB[captureCount];
    double peakMagnitudes[captureCount];
    int peakBin[captureCount];
    double vwc[captureCount];
    int failedCaptures[captureCount];
    int failedCount = 0;

    for (int i = 0; i < captureCount; i++) {
        printf("Please wait. Capture %d is proceeding\n", i + 1);
        usleep((frameCount / FRAME_RATE) * 1000000);
        printf("Waiting for data to be transferred...\n");

        char wetFramesName[256];
        snprintf(wetFramesName, sizeof(wetFramesName), "%s%d.frames", captureName, i + 1);
        CaptureData* wetCapture = procRadarFrames(localDataPath, wetFramesName, tagHz);
        if (!wetCapture || !wetCapture->procSuccess) {
            failedCaptures[failedCount++] = i;
            printf("Capture %d will not be processed due to an issue.\n", i + 1);
            continue;
        }

        peakBin[i] = wetCapture->peakBin;
        peakMagnitudes[i] = wetCapture->tagFT[peakBin[i]];
        vwc[i] = procSoilMoisture(peakBin[i], airPeakBin, SOIL_TYPE, tagDepth);

        freeCaptureData(wetCapture);
    }

    // Free air capture data
    freeCaptureData(airCapture);

    // Remove failed captures
    for (int i = 0; i < failedCount; i++) {
        int idx = failedCaptures[i];
        for (int j = idx; j < captureCount - 1; j++) {
            SNRdB[j] = SNRdB[j + 1];
            peakMagnitudes[j] = peakMagnitudes[j + 1];
            peakBin[j] = peakBin[j + 1];
            vwc[j] = vwc[j + 1];
        }
        captureCount--;
    }

    volumetricWaterContent = median(vwc, captureCount);

    return volumetricWaterContent;

}
