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
 * @function wadarSaveData(char *localDataPath, char *name, double data)
 * @param localDataPath - Local file path to radar capture
 * @param name - Name of data
 * @param data - Data to save
 * @return void
 * @brief Function saves data to a CSV file in the local data path directory with the current time stamp
 */
void wadarSaveData(char *localDataPath, char *name, double data);

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

/**
 * @function wadarAirCapture(char *localDataPath, char *airFramesName, double tagHz, int frameCount, int captureCount)
 * @param localDataPath - Local file path to radar capture
 * @param airFramesName - Name of radar capture file with tag uncovered with soil
 * @param tagHz - Oscillation frequency of tag being captured
 * @param captureCount - Number of captures desired
 * @return void 
 * @brief Function captures radar frame of tag uncovered with air to determine air tag peak bin
 * @author ericdvet */
void wadarAirCapture(char *localDataPath, char *airFramesName, double tagHz, int frameCount, int captureCount);

/**
 * @function wadarTagTest(char *localDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount)
 * @param localDataPath - Local file path to radar capture
 * @param airFramesName - Name of radar capture file with tag uncovered with soil
 * @param trialName - Trial name for file documenting purposes
 * @param tagHz - Oscillation frequency of tag being captured
 * @param captureCount - Number of captures desired
 * @return double
 * @brief Function captures radar frames to test tag SNR
 * @author ericdvet */
double wadarTagTest(char *localDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount);

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
double wadarTwoTag(char *localDataPath, char *trialName, double tag1Hz, double tag2Hz, int frameCount, int captureCount, double tagDiff);

#endif WADAR_H