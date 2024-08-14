/*
 * File:   salsa.c
 * Author: ericdvet
 *
 * Replication of Sensor Logic's SalsaLab MATLAB signal processing functions in C
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "salsa.h"

#define FRAME_LOGGER_MAGIC_NUM 0xFEFE00A2

/**
 * @function salsaLoad(const char *fileName)
 * @param fileName - Name of radar capture to load
 * @return RadarData *
 * @brief Load radar data from a binary file (captured from frameLogger.c on BBB)
 * @author ericdvet */
RadarData *salsaLoad(const char *fileName)
{

    FILE *fid = fopen(fileName, "rb");
    if (!fid)
    {
        fprintf(stderr, "ERROR: File not available");
        return NULL;
    }

    uint32_t magic;
    if (fread(&magic, sizeof(uint32_t), 1, fid) != 1)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        fclose(fid);
        return NULL;
    }
    if (magic != FRAME_LOGGER_MAGIC_NUM)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        fclose(fid);
        return NULL;
    }

    int iterations, pps, dacMin, dacMax, dacStep;
    if (fread(&iterations, sizeof(int), 1, fid) != 1 || fread(&pps, sizeof(int), 1, fid) != 1 ||
        fread(&dacMin, sizeof(int), 1, fid) != 1 || fread(&dacMax, sizeof(int), 1, fid) != 1 ||
        fread(&dacStep, sizeof(int), 1, fid) != 1)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        fclose(fid);
        return NULL;
    }

    int radarSpecifier;
    if (fread(&radarSpecifier, sizeof(int), 1, fid) != 1)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        fclose(fid);
        return NULL;
    }

    float samplesPerSecond = 0;
    int samplingRate = 0, clkDivider = 0;
    float sampleDelayToReference = -1;
    float offsetDistance = -1;

    RadarData *radarData = (RadarData *)malloc(sizeof(RadarData));
    if (!radarData)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        fclose(fid);
        return NULL;
    }

    char chipSet[10];
    int pgen;

    switch (radarSpecifier)
    {
    case 2:
        strcpy(chipSet, "X2");
        if (fread(&samplesPerSecond, sizeof(float), 1, fid) != 1 || fread(&pgen, sizeof(int), 1, fid) != 1 ||
            fread(&offsetDistance, sizeof(float), 1, fid) != 1 || fread(&sampleDelayToReference, sizeof(float), 1, fid) != 1)
        {
            fprintf(stderr, "ERROR: .frames file formatting");
            free(radarData);
            fclose(fid);
            return NULL;
        }
        break;
    case 10:
    case 11:
        if (radarSpecifier == 10)
            strcpy(chipSet, "X1-IPGO");
        if (radarSpecifier == 11)
            strcpy(chipSet, "X1-IPG1");
        if (fread(&samplesPerSecond, sizeof(double), 1, fid) != 1 || fread(&pgen, sizeof(int), 1, fid) != 1 ||
            fread(&samplingRate, sizeof(int), 1, fid) != 1 || fread(&clkDivider, sizeof(int), 1, fid) != 1)
        {
            fprintf(stderr, "ERROR: .frames file formatting");
            free(radarData);
            fclose(fid);
            return NULL;
        }
        break;
    default:
        fprintf(stderr, "ERROR: .frames file formatting");
        free(radarData);
        fclose(fid);
        return NULL;
    }

    int numberOfSamplers, numRuns;
    if (fread(&numberOfSamplers, sizeof(int), 1, fid) != 1 || fread(&(radarData->numFrames), sizeof(int), 1, fid) != 1 ||
        fread(&numRuns, sizeof(int), 1, fid) != 1 || fread(&(radarData->frameRate), sizeof(int), 1, fid) != 1)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        free(radarData);
        fclose(fid);
        return NULL;
    }

    size_t frameTotSize = (radarData->numFrames) * numberOfSamplers * sizeof(uint32_t);
    uint32_t *frameTotRaw = (uint32_t *)malloc(frameTotSize);
    if (!frameTotRaw)
    {
        fprintf(stderr, "ERROR: Memory allocation failure");
        free(radarData);
        fclose(fid);
        return NULL;
    }

    radarData->times = (double *)malloc((radarData->numFrames) * sizeof(double));
    radarData->frameTot = (double *)malloc((radarData->numFrames) * numberOfSamplers * sizeof(double));
    if (!radarData->times || !radarData->frameTot)
    {
        fprintf(stderr, "ERROR: Memory allocation failure");
        free(frameTotRaw);
        free(radarData->times);
        free(radarData->frameTot);
        free(radarData);
        fclose(fid);
        return NULL;
    }

    for (int i = 0; i < (radarData->numFrames); i++)
    {
        if (fread(&radarData->times[i], sizeof(double), 1, fid) != 1)
        {
            fprintf(stderr, "ERROR: .frames file formatting");
            free(frameTotRaw);
            free(radarData->times);
            free(radarData->frameTot);
            free(radarData);
            fclose(fid);
            return NULL;
        }
    }

    if (fread(frameTotRaw, sizeof(uint32_t), (radarData->numFrames) * numberOfSamplers, fid) != (radarData->numFrames) * numberOfSamplers)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        free(frameTotRaw);
        free(radarData);
        fclose(fid);
        return NULL;
    }

    for (int i = 0; i < (radarData->numFrames) * numberOfSamplers; i++)
    {
        radarData->frameTot[i] = (double)frameTotRaw[i] / (pps * iterations) * dacStep + dacMin;
    }
    free(frameTotRaw);

    // Process out the weird spike
    for (int i = 0; i < (radarData->numFrames); i++)
    {
        double maxVal = radarData->frameTot[i * numberOfSamplers];
        for (int j = 1; j < numberOfSamplers; j++)
        {
            if (radarData->frameTot[i * numberOfSamplers + j] > maxVal)
            {
                maxVal = radarData->frameTot[i * numberOfSamplers + j];
            }
        }
        if (maxVal > 8191)
        {
            if (i > 0)
            {
                memcpy(&radarData->frameTot[i * numberOfSamplers], &radarData->frameTot[(i - 1) * numberOfSamplers], numberOfSamplers * sizeof(double));
            }
            else
            {
                memcpy(&radarData->frameTot[i * numberOfSamplers], &radarData->frameTot[(i + 1) * numberOfSamplers], numberOfSamplers * sizeof(double));
            }
        }
    }

    float fpsEst;
    if (fread(&fpsEst, sizeof(float), 1, fid) != 1)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        free(radarData->times);
        free(radarData->frameTot);
        free(radarData);
        fclose(fid);
        return NULL;
    }

    fseek(fid, 0, SEEK_END);
    long remainingData = ftell(fid) - ftell(fid);
    if (remainingData > 0)
    {
        fprintf(stderr, "ERROR: .frames file formatting");
        free(radarData->times);
        free(radarData->frameTot);
        free(radarData);
        fclose(fid);
        return NULL;
    }

    fclose(fid);
    return radarData;
}

/**
 * @function freeRadarData(RadarData *radarData)
 * @param radarData - RadarData struct to free
 * @return None
 * @brief Free RadarData constructed by salsaLoad()
 * @author ericdvet */
void freeRadarData(RadarData *radarData)
{
    if (radarData)
    {
        free(radarData->times);
        free(radarData->frameTot);
        free(radarData);
    }
}

// #define SALSA_TEST

#ifdef SALSA_TEST
int main()
{
    RadarData *radarData = salsaLoad("/home/ericdvet/jlab/wadar/signal_processing/testFile.frames");
    if (radarData)
    {
        freeRadarData(radarData);
    }
    return 0;
}
#endif
