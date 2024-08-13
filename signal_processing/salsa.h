#ifndef SALSA_LOAD_H
#define SALSA_LOAD_H

#include <stdio.h>
#include <stdint.h>

typedef struct
{
    double *times;
    double *frameTot;
    int frameRate;
    int numFrames;
} RadarData;

// USAGE: Load radar data from a binary file (captured from frameLogger.c on BBB)
RadarData *salsaLoad(const char *fileName);

// free data
void freeRadarData(RadarData *radarData);

double frameTot(RadarData *data, int frame, int sampler);

#endif // SALSA_LOAD_H
