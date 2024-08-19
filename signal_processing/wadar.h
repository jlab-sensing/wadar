/*
 * File:   wadar.h
 * Author: ericdvet
 *
 * High level WaDAR functions. Functions to automatically capture and process radar data to determine soil moisture content. 
 */

#ifndef WADAR_H
#define WADAH_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "proc.h"
#include "utils.h"

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
double wadar(char *localDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount, double tagDepth);

#endif WADAR_H