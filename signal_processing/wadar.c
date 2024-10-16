/*
 * File:   wadar.h
 * Author: ericdvet
 *
 * High level WaDAR functions. Functions to automatically capture and process radar data to determine soil moisture content.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "proc.h"
#include <unistd.h>
#include "utils.h"
#include <curl/curl.h>
#include <netdb.h>

// Capture parameters
#define FRAME_RATE 200
#define RADAR_TYPE "Chipotle"
#define SOIL_TYPE "farm"

/**
 * @function wadar(char *localDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount, double tagDepth)
 * @param localDataPath - Local file path to radar capture
 * @param airFramesName - Name of radar capture file with tag uncovered with soil
 * @param trialName - Trial name for file documenting purposes
 * @param tagHz - Oscillation frequency of tag being captured
 * @param captureCount - Number of captures desired
 * @param tagDepth - Depth at which tag is buried measured in meters
 * @return double
 * @brief Function calculates the volumetric water content of the soil from the capture
 * @author ericdvet */
double wadar(char *localDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount, double tagDepth)
{

    double volumetricWaterContent = 0.0;

    // Process Air Capture
    CaptureData *airCapture = procRadarFrames(localDataPath, airFramesName, tagHz);
    if (!airCapture || !airCapture->procSuccess)
    {
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
    char fullDataPath[256];
    sprintf(fullDataPath, "%s@192.168.7.1:%s", "ericdvet", localDataPath);
    char frameLoggerCommand[1024];
    snprintf(frameLoggerCommand, sizeof(frameLoggerCommand),
             "ssh root@192.168.7.2 \"screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger -s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s \" &",
             captureName, frameCount, captureCount, FRAME_RATE, RADAR_TYPE, fullDataPath);
    system(frameLoggerCommand);

    // Load and Process Captures
    double SNRdB[captureCount];
    double peakMagnitudes[captureCount];
    int peakBin[captureCount];
    double vwc[captureCount];
    int failedCaptures[captureCount];
    int failedCount = 0;

    for (int i = 0; i < captureCount; i++)
    {
        printf("Please wait. Capture %d is proceeding\n", i + 1);
        usleep((frameCount / FRAME_RATE) * 1000000);
        printf("Waiting for data to be transferred...\n");
        usleep((frameCount / FRAME_RATE) * 1000000 * 0.5);

        char wetFramesName[1000];
        snprintf(wetFramesName, sizeof(wetFramesName), "%s%d.frames", captureName, i + 1);
        CaptureData *wetCapture = procRadarFrames(localDataPath, wetFramesName, tagHz);
        if (!wetCapture || !wetCapture->procSuccess)
        {
            failedCaptures[failedCount++] = i;
            printf("Capture %d will not be processed due to an issue.\n", i + 1);
            continue;
        }

        peakBin[i] = wetCapture->peakBin;
        SNRdB[i] = wetCapture->SNRdB;
        peakMagnitudes[i] = wetCapture->tagFT[peakBin[i]];
        vwc[i] = procSoilMoisture(peakBin[i], airPeakBin, SOIL_TYPE, tagDepth);

        freeCaptureData(wetCapture);
    }

    freeCaptureData(airCapture);

    // Remove failed captures
    for (int i = 0; i < failedCount; i++)
    {
        int idx = failedCaptures[i];
        for (int j = idx; j < captureCount - 1; j++)
        {
            SNRdB[j] = SNRdB[j + 1];
            peakMagnitudes[j] = peakMagnitudes[j + 1];
            peakBin[j] = peakBin[j + 1];
            vwc[j] = vwc[j + 1];
        }
        captureCount--;
    }

    volumetricWaterContent = median(vwc, captureCount);

    printf("The Volumetric Water Content is: %.2f\n", volumetricWaterContent);

    return volumetricWaterContent;
}

/**
 * @function wadarAirCapture(char *localDataPath, char *airFramesName, double tagHz, int frameCount, int captureCount)
 * @param localDataPath - Local file path to radar capture
 * @param airFramesName - Name of radar capture file with tag uncovered with soil
 * @param tagHz - Oscillation frequency of tag being captured
 * @param captureCount - Number of captures desired
 * @return void 
 * @brief Function captures radar frame of tag uncovered with air to determine air tag peak bin
 * @author ericdvet */
void wadarAirCapture(char *localDataPath, char *airFramesName, double tagHz, int frameCount, int captureCount) {

    // Load soil capture
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char captureName[256];
    snprintf(captureName, sizeof(captureName), "%d-%02d-%02d_Air_C", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);

    // Frame logger command
    char fullDataPath[256];
    sprintf(fullDataPath, "%s@192.168.7.1:%s", "ericdvet", localDataPath);
    char frameLoggerCommand[1024];
    snprintf(frameLoggerCommand, sizeof(frameLoggerCommand),
             "ssh root@192.168.7.2 \"screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger -s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s \" &",
             captureName, frameCount, captureCount, FRAME_RATE, RADAR_TYPE, fullDataPath);
    system(frameLoggerCommand);

    // Load and Process Captures
    double SNRdB[captureCount];
    double peakMagnitudes[captureCount];
    int peakBin[captureCount];
    int failedCaptures[captureCount];
    int failedCount = 0;

    for (int i = 0; i < captureCount; i++)
    {
        printf("Please wait. Capture %d is proceeding\n", i + 1);
        usleep((frameCount / FRAME_RATE) * 1000000);
        printf("Waiting for data to be transferred...\n");
        usleep((frameCount / FRAME_RATE) * 1000000 * 0.5);

        char airFramesName[1000];
        snprintf(airFramesName, sizeof(airFramesName), "%s%d.frames", captureName, i + 1);
        CaptureData *airCapture = procRadarFrames(localDataPath, airFramesName, tagHz);
        if (!airCapture || !airCapture->procSuccess)
        {
            failedCaptures[failedCount++] = i;
            printf("Capture %d will not be processed due to an issue.\n", i + 1);
            continue;
        }

        peakBin[i] = airCapture->peakBin;
        SNRdB[i] = airCapture->SNRdB;
        peakMagnitudes[i] = airCapture->tagFT[peakBin[i]];

        freeCaptureData(airCapture);
    }

    // Remove failed captures
    for (int i = 0; i < failedCount; i++)
    {
        int idx = failedCaptures[i];
        for (int j = idx; j < captureCount - 1; j++)
        {
            SNRdB[j] = SNRdB[j + 1];
            peakMagnitudes[j] = peakMagnitudes[j + 1];
            peakBin[j] = peakBin[j + 1];
        }
        captureCount--;
    }
}


/**
 * @function wadarTagTest(char *fullDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount)
 * @param fullDataPath - Full data file path to radar capture. Must be in the format "user@ip:path"
 * @param trialName - Trial name for file documenting purposes
 * @param tagHz - Oscillation frequency of tag being captured
 * @param captureCount - Number of captures desired
 * @return double
 * @brief Function captures radar frames to test tag SNR
 * @author ericdvet */
double wadarTagTest(char *fullDataPath, char *trialName, double tagHz, int frameCount, int captureCount)
{

    // Load soil capture
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char captureName[256];
    snprintf(captureName, sizeof(captureName), "%d-%02d-%02d__%s_C", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, trialName);

    // Frame logger command
    char frameLoggerCommand[1024];
    snprintf(frameLoggerCommand, sizeof(frameLoggerCommand),
             "ssh -q root@192.168.7.2 \"screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger -s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s \" &",
             captureName, frameCount, captureCount, FRAME_RATE, RADAR_TYPE, fullDataPath);
    system(frameLoggerCommand);

    // Load and Process Captures
    double SNRdB[captureCount];
    int failedCaptures[captureCount];
    int failedCount = 0;

    printf("Connecting to radar...\n");
    usleep(10 * 1000000);

    for (int i = 0; i < captureCount; i++)
    {
        printf("Please wait. Capture %d is proceeding\n", i + 1);
        usleep((frameCount / FRAME_RATE) * 1000000);
        printf("Waiting for data to be transferred...\n");
        usleep((frameCount / FRAME_RATE) * 1000000 * 0.5 + 5000000);

        char fileName[1000];
        snprintf(fileName, sizeof(fileName), "%s%d.frames", captureName, i + 1);
        SNRdB[i] = procTagTest(fullDataPath, fileName, tagHz);
        if (SNRdB[i] == -1)
        {
            failedCaptures[failedCount++] = i;
            printf("Capture %d will not be processed due to an issue.\n", i + 1);
            continue;
        }
    }

    // Remove failed captures
    for (int i = 0; i < failedCount; i++)
    {
        int idx = failedCaptures[i];
        for (int j = idx; j < captureCount - 1; j++)
        {
            SNRdB[j] = SNRdB[j + 1];
        }
        captureCount--;
    }

    double medianSNRdB = median(SNRdB, captureCount);

    wadarSaveData(fullDataPath, trialName, medianSNRdB);

    printf("Tag Test Complete\n");
    printf("Median SNR: %f\n", medianSNRdB);
    // system("python3 plotRadarCapture.py");
    printf("Please run python3 plotRadarCapture.py to view the plots\n");

    return medianSNRdB;
}

/**
 * @function wadarTwoTag(char *localDataPath, char *airFramesName, char *trialName, double tag1Hz, double tag2Hz, int frameCount, int captureCount, double tagDiff)
 * @param localDataPath - Local file path to radar capture
 * @param airFramesName - Name of radar capture file with tag uncovered with soil
 * @param trialName - Trial name for file documenting purposes
 * @param tag1Hz - Oscillation frequency of top tag being captured
 * @param tag2Hz - Oscillation frequency of bottom tag being captured
 * @param captureCount - Number of captures desired
 * @param tagDiff - Distance between tags measured in meters
 * @return double
 * @brief Function captures radar frames with two tags to calculate the volumetric water content of the soil
 * @author ericdvet */
double wadarTwoTag(char *localDataPath, char *trialName, double tag1Hz, double tag2Hz, int frameCount, int captureCount, double tagDiff)
{

    double volumetricWaterContent = 0.0;

    // Load soil capture
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char captureName[256];
    snprintf(captureName, sizeof(captureName), "%d-%02d-%02d_DualTag%d%d_%s_C", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, (int) tag1Hz, (int) tag2Hz, trialName);

    // Frame logger command
    char fullDataPath[256];
    sprintf(fullDataPath, "%s@192.168.7.1:%s", "ericdvet", localDataPath);
    char frameLoggerCommand[1024];
    snprintf(frameLoggerCommand, sizeof(frameLoggerCommand),
             "ssh root@192.168.7.2 \"screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger -s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s \" &",
             captureName, frameCount, captureCount, FRAME_RATE, RADAR_TYPE, fullDataPath);
    system(frameLoggerCommand);

    // Load and Process Captures
    double SNRdB1[captureCount];
    double SNRdB2[captureCount];
    double peakMagnitudes1[captureCount];
    double peakMagnitudes2[captureCount];
    int peakBin1[captureCount];
    int peakBin2[captureCount];
    double vwc[captureCount];
    int failedCaptures[captureCount];
    int failedCount = 0;

    for (int i = 0; i < captureCount; i++)
    {
        printf("Please wait. Capture %d is proceeding\n", i + 1);
        usleep((frameCount / FRAME_RATE) * 1000000);
        printf("Waiting for data to be transferred...\n");
        usleep((frameCount / FRAME_RATE) * 1000000 * 0.5);

        char wetFramesName[1000];
        snprintf(wetFramesName, sizeof(wetFramesName), "%s%d.frames", captureName, i + 1);
        CaptureData *wetCapture = procTwoTag(localDataPath, wetFramesName, tag1Hz, tag2Hz);
        if (!wetCapture || !wetCapture->procSuccess)
        {
            failedCaptures[failedCount++] = i;
            printf("Capture %d will not be processed due to an issue.\n", i + 1);
            continue;
        }

        peakBin1[i] = wetCapture->peakBin;
        SNRdB1[i] = wetCapture->SNRdB;
        peakMagnitudes1[i] = wetCapture->tagFT[peakBin1[i]];
        peakBin2[i] = wetCapture->peakBin2;
        SNRdB2[i] = wetCapture->SNRdB2;
        peakMagnitudes2[i] = wetCapture->tagFT2[peakBin2[i]];
        vwc[i] = procSoilMoisture(peakBin1[i], peakBin2[i], SOIL_TYPE, tagDiff);

        freeCaptureData(wetCapture);
    }

    // Remove failed captures
    for (int i = 0; i < failedCount; i++)
    {
        int idx = failedCaptures[i];
        for (int j = idx; j < captureCount - 1; j++)
        {
            SNRdB1[j] = SNRdB1[j + 1];
            peakMagnitudes1[j] = peakMagnitudes1[j + 1];
            peakBin1[j] = peakBin1[j + 1];
            SNRdB2[j] = SNRdB2[j + 1];
            peakMagnitudes2[j] = peakMagnitudes2[j + 1];
            peakBin2[j] = peakBin2[j + 1];
            vwc[j] = vwc[j + 1];
        }
        captureCount--;
    }

    volumetricWaterContent = median(vwc, captureCount);

    printf("The Volumetric Water Content is: %.2f\n", volumetricWaterContent);

    return volumetricWaterContent;
}

/**
 * @function wadarSaveData(char *fullDataPath, char *name, char *dataName, double data)
 * @param fullDataPath - Full data file path to radar capture. Must be in the format "user@ip:path". Example: "ericdvet@192.168.7.1:/home/ericdvet/hare-lab/dev_ws/src/wadar/signal_processing/data".
 * @param name - Name of data
 * @param data - Data to save
 * @return void
 * @brief Function saves data to a CSV file in the local data path directory with the current time stamp
 */
void wadarSaveData(char *fullDataPath, char *name, double data) {
    FILE *file;
    char filePath[512];
    snprintf(filePath, sizeof(filePath), "%s/data.csv", fullDataPath);
    const char *colon = strchr(filePath, ':');
    if (colon != NULL) {
        memmove(filePath, colon + 1, strlen(colon));
    }
    file = fopen(filePath, "a");
    if (file == NULL)
    {
        printf("Error opening file %s\n", filePath);
        return;
    }

    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char timeStr[100];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &tm);

    fprintf(file, "%s,%s,%.2f\n", timeStr, name, data);
    fclose(file);
}

// Function to post data to the URL
void wadar2dirtviz(const char *url, double vwc)
{
    char message[1000];
    sprintf(message, "python temp.py %f", vwc);
    system(message);
}

#define WADAR_TEST
#ifdef WADAR_TEST
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Run wadar for measuring soil moisture content or wadarTagTest for testing the tag\n");
        printf("Usage: %s wadar -s <fullDataPath> -b <airFramesName> -t <trialName> -f <tagHz> -c <frameCount> -n <captureCount> -d <tagDepth>\n", argv[0]);
        printf("Usage: %s wadarAirCapture -s <fullDataPath> -b <airFramesName> -f <tagHz> -c <frameCount> -n <captureCount>\n", argv[0]);
        printf("Usage: %s wadarTagTest -s <fullDataPath> -t <trialName> -f <tagHz> -c <frameCount> -n <captureCount> -d <tagDepth>\n", argv[0]);
        printf("Usage: %s wadarTwoTag -s <fullDataPath> -t <trialName> -f <tag1Hz> -g <tag2Hz> -c <frameCount> -n <captureCount> -d <tagDiff>\n", argv[0]);
        return -1;
    }

    char *fullDataPath = NULL;
    char *airFramesName = NULL;
    char *trialName = NULL;
    double tagHz = 0.0;
    double tag2Hz = 0.0;
    int frameCount = 0;
    int captureCount = 0;
    double tagDepth = 0.0;
    double tagDiff = 0.0;

    // Case: "wadar"
    if (strcmp(argv[1], "wadar") == 0)
    {
        for (int i = 2; i < argc; i++)
        {
            if (strcmp(argv[i], "-s") == 0)
            {
                fullDataPath = argv[++i];
            }
            else if (strcmp(argv[i], "-b") == 0)
            {
                airFramesName = argv[++i];
            }
            else if (strcmp(argv[i], "-t") == 0)
            {
                trialName = argv[++i];
            }
            else if (strcmp(argv[i], "-f") == 0)
            {
                tagHz = atof(argv[++i]);
            }
            else if (strcmp(argv[i], "-c") == 0)
            {
                frameCount = atoi(argv[++i]);
            }
            else if (strcmp(argv[i], "-n") == 0)
            {
                captureCount = atoi(argv[++i]);
            }
            else if (strcmp(argv[i], "-d") == 0)
            {
                tagDepth = atof(argv[++i]);
            }
            else
            {
                printf("Unknown argument: %s\n", argv[i]);
                return -1;
            }
        }
        if (!fullDataPath || !airFramesName || !trialName || tagHz == 0.0 || frameCount == 0 || captureCount == 0 || tagDepth == 0.0)
        {
            printf("Missing arguments. Usage: %s wadar -s <fullDataPath> -b <airFramesName> -t <trialName> -f <tagHz> -c <frameCount> -n <captureCount> -d <tagDepth>\n", argv[0]);
            return -1;
        }

        // Call the wadar function with the parsed arguments
        double VWC = wadar(fullDataPath, airFramesName, trialName, tagHz, frameCount, captureCount, tagDepth);
        wadar2dirtviz("https://dirtviz.jlab.ucsc.edu/api/teros/", VWC);
        return 0;
    }

    // Case: "wadarAirCapture"
    if (strcmp(argv[1], "wadarAirCapture") == 0) {
        for (int i = 2; i < argc; i++)
        {
            if (strcmp(argv[i], "-s") == 0)
            {
                fullDataPath = argv[++i];
            }
            else if (strcmp(argv[i], "-b") == 0)
            {
                airFramesName = argv[++i];
            }
            else if (strcmp(argv[i], "-f") == 0)
            {
                tagHz = atof(argv[++i]);
            }
            else if (strcmp(argv[i], "-c") == 0)
            {
                frameCount = atoi(argv[++i]);
            }
            else if (strcmp(argv[i], "-n") == 0)
            {
                captureCount = atoi(argv[++i]);
            }
            else
            {
                printf("Unknown argument: %s\n", argv[i]);
                return -1;
            }
        }
        if (!fullDataPath || !airFramesName || tagHz == 0.0 || frameCount == 0 || captureCount == 0)
        {
            printf("Missing arguments. Usage: %s wadarAirCapture -s <localDataPath> -b <airFramesName> -f <tagHz> -c <frameCount> -n <captureCount>\n", argv[0]);
            return -1;
        }

        // Call the wadar function with the parsed arguments
        wadarAirCapture(fullDataPath, airFramesName, tagHz, frameCount, captureCount);
        return 0;
    }

    // Case: "wadarTagTest"
    if (strcmp(argv[1], "wadarTagTest") == 0)
    {
        for (int i = 2; i < argc; i++)
        {
            if (strcmp(argv[i], "-s") == 0)
            {
                fullDataPath = argv[++i];
            }
            else if (strcmp(argv[i], "-t") == 0)
            {
                trialName = argv[++i];
            }
            else if (strcmp(argv[i], "-f") == 0)
            {
                tagHz = atof(argv[++i]);
            }
            else if (strcmp(argv[i], "-c") == 0)
            {
                frameCount = atoi(argv[++i]);
            }
            else if (strcmp(argv[i], "-n") == 0)
            {
                captureCount = atoi(argv[++i]);
            }
            else
            {
                printf("Unknown argument: %s\n", argv[i]);
                return -1;
            }
        }
        if (!fullDataPath || !trialName || tagHz == 0.0 || frameCount == 0 || captureCount == 0)
        {
            printf("Missing arguments. Usage: %s wadarTagTest -s <fullDataPath> -t <trialName> -f <tagHz> -c <frameCount> -n <captureCount> -d <tagDepth>\n", argv[0]);
            return -1;
        }

        // Call the wadar function with the parsed arguments
        wadarTagTest(fullDataPath, trialName, tagHz, frameCount, captureCount);
        return 0;
    }

    // Case: "wadarTwoTag"
    if (strcmp(argv[1], "wadarTwoTag") == 0)
    {
        for (int i = 2; i < argc; i++)
        {
            if (strcmp(argv[i], "-s") == 0)
            {
                fullDataPath = argv[++i];
            }
            else if (strcmp(argv[i], "-t") == 0)
            {
                trialName = argv[++i];
            }
            else if (strcmp(argv[i], "-f") == 0)
            {
                tagHz = atof(argv[++i]);
            }
            else if (strcmp(argv[i], "-g") == 0)
            {
                tag2Hz = atof(argv[++i]);
            }
            else if (strcmp(argv[i], "-c") == 0)
            {
                frameCount = atoi(argv[++i]);
            }
            else if (strcmp(argv[i], "-n") == 0)
            {
                captureCount = atoi(argv[++i]);
            }
            else if (strcmp(argv[i], "-d") == 0)
            {
                tagDiff = atof(argv[++i]);
            }
            else
            {
                printf("Unknown argument: %s\n", argv[i]);
                return -1;
            }
        }
        if (!fullDataPath || !trialName || tagHz == 0.0 || tag2Hz == 0.0 || frameCount == 0 || captureCount == 0 || tagDiff == 0.0)
        {
            printf("Missing arguments. Usage: %s wadarTwoTag -s <fullDataPath> -t <trialName> -f <tag1Hz> -g <tag2Hz> -c <frameCount> -n <captureCount> -d <tagDiff>\n", argv[0]);
            return -1;
        }

        // Call the wadar function with the parsed arguments
        double VWC = wadarTwoTag(fullDataPath, trialName, tagHz, tag2Hz, frameCount, captureCount, tagDiff);
        wadar2dirtviz("https://dirtviz.jlab.ucsc.edu/api/teros/", VWC);
        return 0;
    }

    // Invalid function message
    printf("Run wadar for measuring soil moisture content or wadarTagTest for testing the tag\n");
    printf("Usage: %s wadar -s <fullDataPath> -b <airFramesName> -t <trialName> -f <tagHz> -c <frameCount> -n <captureCount> -d <tagDepth>\n", argv[0]);
    printf("Usage: %s wadarAirCapture -s <fullDataPath> -b <airFramesName> -f <tagHz> -c <frameCount> -n <captureCount>\n", argv[0]);
    printf("Usage: %s wadarTagTest -s <fullDataPath> -b <airFramesName> -t <trialName> -f <tagHz> -c <frameCount> -n <captureCount>\n", argv[0]);
    printf("Usage: %s wadarTwoTag -s <fullDataPath> -t <trialName> -f <tag1Hz> -g <tag2Hz> -c <frameCount> -n <captureCount> -d <tagDiff>\n", argv[0]);
    return -1;
}
#endif

// #define DIRTVIZ_TEST
#ifdef DIRTVIZ_TEST
int main(void)
{
    wadar2dirtviz("https://dirtviz.jlab.ucsc.edu/api/teros/", 0.3);
    return 0;
}

#endif