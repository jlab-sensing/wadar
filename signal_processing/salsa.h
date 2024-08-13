#ifndef SALSA_LOAD_H
#define SALSA_LOAD_H

#include <stdio.h>
#include <stdint.h>

typedef struct SalsaDataObj *SalsaData;

typedef struct SalsaDataObj {
    int iterations;
    int pps;
    int dacMin;
    int dacMax;
    int dacStep;
    char *chipSet;
    int radarSpecifier;
    float samplesPerSecond;
    int pgen;
    float offsetDistance;
    float sampleDelayToReference;
    int samplingRate;
    int clkDivider;
    int numberOfSamplers;
    int numFrames;
    int numRuns;
    int frameRate;
    double *times;
    double *frameTot;
    float fpsEst;
} SalsaDataObj;

SalsaData salsa_load(const char *fileName);
void free_salsa_data(SalsaData data);

#endif // SALSA_LOAD_H
