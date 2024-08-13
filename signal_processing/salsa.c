#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "salsa.h"

#define FRAME_LOGGER_MAGIC_NUM 0xFEFE00A2

typedef struct {
    char chipSet[10];
    float* times;
    double* frameTot;
    double fs_hz;
    int pgen;
} RadarData;

RadarData* salsaLoad(const char* fileName) {
    printf("Loading saved data from %s...\n", fileName);

    FILE* fid = fopen(fileName, "rb");
    if (!fid) {
        perror("Error opening file");
        return NULL;
    }

    uint32_t magic;
    if (fread(&magic, sizeof(uint32_t), 1, fid) != 1) {
        perror("Error reading magic number");
        fclose(fid);
        return NULL;
    }
    if (magic != FRAME_LOGGER_MAGIC_NUM) {
        printf("Wrong data format: %s!\n", fileName);
        fclose(fid);
        return NULL;
    }

    int iterations, pps, dacMin, dacMax, dacStep;
    if (fread(&iterations, sizeof(int), 1, fid) != 1 ||
        fread(&pps, sizeof(int), 1, fid) != 1 ||
        fread(&dacMin, sizeof(int), 1, fid) != 1 ||
        fread(&dacMax, sizeof(int), 1, fid) != 1 ||
        fread(&dacStep, sizeof(int), 1, fid) != 1) {
        perror("Error reading settings");
        fclose(fid);
        return NULL;
    }

    int radarSpecifier;
    if (fread(&radarSpecifier, sizeof(int), 1, fid) != 1) {
        perror("Error reading radar specifier");
        fclose(fid);
        return NULL;
    }

    float samplesPerSecond = 0;
    int samplingRate = 0, clkDivider = 0;
    float sampleDelayToReference = -1;
    float offsetDistance = -1;

    RadarData* radarData = (RadarData*)malloc(sizeof(RadarData));
    if (!radarData) {
        perror("Memory allocation failed");
        fclose(fid);
        return NULL;
    }

    switch (radarSpecifier) {
        case 2:
            strcpy(radarData->chipSet, "X2");
            if (fread(&samplesPerSecond, sizeof(float), 1, fid) != 1 ||
                fread(&radarData->pgen, sizeof(int), 1, fid) != 1 ||
                fread(&offsetDistance, sizeof(float), 1, fid) != 1 ||
                fread(&sampleDelayToReference, sizeof(float), 1, fid) != 1) {
                perror("Error reading X2 settings");
                free(radarData);
                fclose(fid);
                return NULL;
            }
            break;
        case 10:
        case 11:
            if (radarSpecifier == 10) strcpy(radarData->chipSet, "X1-IPGO");
            if (radarSpecifier == 11) strcpy(radarData->chipSet, "X1-IPG1");
            if (fread(&samplesPerSecond, sizeof(double), 1, fid) != 1 ||
                fread(&radarData->pgen, sizeof(int), 1, fid) != 1 ||
                fread(&samplingRate, sizeof(int), 1, fid) != 1 ||
                fread(&clkDivider, sizeof(int), 1, fid) != 1) {
                perror("Error reading X1 settings");
                free(radarData);
                fclose(fid);
                return NULL;
            }
            break;
        default:
            printf("Unknown radar specifier!\n");
            free(radarData);
            fclose(fid);
            return NULL;
    }

    int numberOfSamplers, numFrames, numRuns, frameRate;
    if (fread(&numberOfSamplers, sizeof(int), 1, fid) != 1 ||
        fread(&numFrames, sizeof(int), 1, fid) != 1 ||
        fread(&numRuns, sizeof(int), 1, fid) != 1 ||
        fread(&frameRate, sizeof(int), 1, fid) != 1) {
        perror("Error reading frame settings");
        free(radarData);
        fclose(fid);
        return NULL;
    }

    size_t frameTotSize = numFrames * numberOfSamplers * sizeof(uint32_t);
    uint32_t* frameTotRaw = (uint32_t*)malloc(frameTotSize);
    if (!frameTotRaw) {
        perror("Memory allocation failed for frameTotRaw");
        free(radarData);
        fclose(fid);
        return NULL;
    }

    if (fread(frameTotRaw, sizeof(uint32_t), numFrames * numberOfSamplers, fid) != numFrames * numberOfSamplers) {
        perror("Error reading radar frames");
        free(frameTotRaw);
        free(radarData);
        fclose(fid);
        return NULL;
    }

    radarData->times = (float*)malloc(numFrames * sizeof(float));
    radarData->frameTot = (double*)malloc(numFrames * numberOfSamplers * sizeof(double));
    if (!radarData->times || !radarData->frameTot) {
        perror("Memory allocation failed for times or frameTot");
        free(frameTotRaw);
        free(radarData->times);
        free(radarData->frameTot);
        free(radarData);
        fclose(fid);
        return NULL;
    }

    for (int i = 0; i < numFrames; i++) {
        if (fread(&radarData->times[i], sizeof(float), 1, fid) != 1) {
            perror("Error reading times");
            free(frameTotRaw);
            free(radarData->times);
            free(radarData->frameTot);
            free(radarData);
            fclose(fid);
            return NULL;
        }
    }

    for (int i = 0; i < numFrames * numberOfSamplers; i++) {
        radarData->frameTot[i] = (double)frameTotRaw[i] / (pps * iterations) * dacStep + dacMin;
    }
    free(frameTotRaw);

    for (int i = 0; i < numFrames; i++) {
        double maxVal = radarData->frameTot[i * numberOfSamplers];
        for (int j = 1; j < numberOfSamplers; j++) {
            if (radarData->frameTot[i * numberOfSamplers + j] > maxVal) {
                maxVal = radarData->frameTot[i * numberOfSamplers + j];
            }
        }
        if (maxVal > 8191) {
            if (i > 0) {
                memcpy(&radarData->frameTot[i * numberOfSamplers], &radarData->frameTot[(i - 1) * numberOfSamplers], numberOfSamplers * sizeof(double));
            } else {
                memcpy(&radarData->frameTot[i * numberOfSamplers], &radarData->frameTot[(i + 1) * numberOfSamplers], numberOfSamplers * sizeof(double));
            }
        }
    }

    float fpsEst;
    if (fread(&fpsEst, sizeof(float), 1, fid) != 1) {
        perror("Error reading estimated FPS");
        free(radarData->times);
        free(radarData->frameTot);
        free(radarData);
        fclose(fid);
        return NULL;
    }

    fseek(fid, 0, SEEK_END);
    long remainingData = ftell(fid) - ftell(fid);
    if (remainingData > 0) {
        printf("FILE READ ERROR: %ld data remains! Check that file format matches read code\n", remainingData);
        free(radarData->times);
        free(radarData->frameTot);
        free(radarData);
        fclose(fid);
        return NULL;
    }

    fclose(fid);
    return radarData;
}

void freeRadarData(RadarData* radarData) {
    if (radarData) {
        free(radarData->times);
        free(radarData->frameTot);
        free(radarData);
    }
}

int main() {
    RadarData* radarData = salsaLoad("/home/ericdvet/jlab/wadar/signal_processing/2024-7-3_3264_BackParkingLot_C1.frames");
    if (radarData) {
        // Use radarData
        freeRadarData(radarData);
    }
    return 0;
}
