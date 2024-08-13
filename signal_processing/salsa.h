#ifndef SALSA_LOAD_H
#define SALSA_LOAD_H

#include <stdio.h>
#include <stdint.h>

typedef struct {
    char chipSet[10];
    float* times;
    double* frameTot;
    double fs_hz;
    int pgen;
} RadarData;

RadarData* salsaLoad(const char* fileName);
void freeRadarData(RadarData* radarData);

#endif // SALSA_LOAD_H
