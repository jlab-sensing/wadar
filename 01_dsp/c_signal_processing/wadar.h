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
 * @function wadarSaveData(char *fullDataPath, char *name, char *dataName, double vwc, double snr, int peakBin)
 * @param fullDataPath - Full data file path to radar capture. Must be in the format "user@ip:path". Example: "ericdvet@192.168.7.1:/home/ericdvet/hare-lab/dev_ws/src/wadar/signal_processing/data".
 * @param name - Name of data
 * @param vwc - Volumetric Water Content
 * @param snr - Signal-to-Noise Ratio
 * @param peakBin - Peak bin value
 * @return void
 * @brief Function saves data to a CSV file in the local data path directory with the current time stamp
 */
void wadarSaveData(char *fullDataPath, char *name, double vwc, double snr, int peakBin);

/**
 * @function wadar(char *fullDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount, double tagDepth)
 * @param fullDataPath - Full data file path to radar capture. Must be in the format "user@ip:path". Example: "ericdvet@192.168.7.1:/home/ericdvet/hare-lab/dev_ws/src/wadar/signal_processing/data"
 * @param airFramesName - Name of radar capture file with tag uncovered with soil
 * @param trialName - Trial name for file documenting purposes
 * @param tagHz - Oscillation frequency of tag being captured
 * @param captureCount - Number of captures desired
 * @param tagDepth - Depth at which tag is buried measured in meters
 * @return double 
 * @brief Function calculates the volumetric water content of the soil from the capture
 * @author ericdvet */
double wadar(char *fullDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount, double tagDepth);

/**
 * @function wadarAirCapture(char *fullDataPath, char *airFramesName, double tagHz, int frameCount, int captureCount)
 * @param fullDataPath - Full data file path to radar capture. Must be in the format "user@ip:path". Example: "ericdvet@192.168.7.1:/home/ericdvet/hare-lab/dev_ws/src/wadar/signal_processing/data".
 * @param airFramesName - Name of radar capture file with tag uncovered with soil
 * @param tagHz - Oscillation frequency of tag being captured
 * @param captureCount - Number of captures desired
 * @return void 
 * @brief Function captures radar frame of tag uncovered with air to determine air tag peak bin
 * @author ericdvet */
void wadarAirCapture(char *fullDataPath, char *airFramesName, double tagHz, int frameCount, int captureCount);

/**
 * @function wadarTagTest(char *fullDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount)
 * @param fullDataPath - Full data file path to radar capture. Must be in the format "user@ip:path". Example: "ericdvet@192.168.7.1:/home/ericdvet/hare-lab/dev_ws/src/wadar/signal_processing/data".
 * @param trialName - Trial name for file documenting purposes
 * @param tagHz - Oscillation frequency of tag being captured
 * @param captureCount - Number of captures desired
 * @return double
 * @brief Function captures radar frames to test tag SNR
 * @author ericdvet */
double wadarTagTest(char *fullDataPath, char *trialName, double tagHz, int frameCount, int captureCount);

/**
 * @function wadarTwoTag(char *fullDataPath, char *airFramesName, char *trialName, double tag1Hz, double tag2Hz, int frameCount, int captureCount, double tagDiff)
 * @param fullDataPath - Full data file path to radar capture. Must be in the format "user@ip:path". Example: "ericdvet@192.168.7.1:/home/ericdvet/hare-lab/dev_ws/src/wadar/signal_processing/data".
 * @param airFramesName - Name of radar capture file with tag uncovered with soil
 * @param trialName - Trial name for file documenting purposes
 * @param tag1Hz - Oscillation frequency of top tag being captured
 * @param tag2Hz - Oscillation frequency of bottom tag being captured
 * @param captureCount - Number of captures desired
 * @param tagDiff - Distance between tags measured in meters
 * @return double
 * @brief Function captures radar frames with two tags to calculate the volumetric water content of the soil
 * @author ericdvet */
double wadarTwoTag(char *fullDataPath, char *trialName, double tag1Hz, double tag2Hz, int frameCount, int captureCount, double tagDiff);

#endif 