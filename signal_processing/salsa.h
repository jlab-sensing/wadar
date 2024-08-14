/*
 * File:   salsa.h
 * Author: ericdvet
 *
 * Replication of Sensor Logic's SalsaLab MATLAB signal processing functions in C
 */

#ifndef SALSA_LOAD_H
#define SALSA_LOAD_H

#include <stdio.h>
#include <stdint.h>

/**
 * @struct RadarData
 * @brief Stores important data collected by salsaLoad()
 * @author ericdvet */
typedef struct
{
    double *times;
    double *frameTot;
    int frameRate;
    int numFrames;
} RadarData;

/**
 * @function salsaLoad(const char *fileName)
 * @param fileName - Name of radar capture to load
 * @return RadarData *
 * @brief Load radar data from a binary file (captured from frameLogger.c on BBB)
 * @author ericdvet */
RadarData *salsaLoad(const char *fileName);

/**
 * @function freeRadarData(RadarData *radarData)
 * @param radarData - RadarData struct to free
 * @return None
 * @brief Free RadarData constructed by salsaLoad()
 * @author ericdvet */
void freeRadarData(RadarData *radarData);

#endif // SALSA_LOAD_H
