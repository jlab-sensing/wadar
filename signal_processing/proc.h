/*
 * File:   proc.h
 * Author: ericdvet
 *
 * Functions to process .frames data to obtain tag information
 */

#ifndef PROC_LOAD_H
#define PROC_LOAD_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fftw3.h>
#include <stdbool.h>
#include <complex.h>
#include <string.h>
#include "salsa.h"

/**
 * @struct CaptureData
 * @brief Stores important data collected by procRadarFrames()
 * @author ericdvet */
typedef struct
{
    bool procSuccess;
    double complex *captureFT;
    double *tagFT;
    double *tagFT2;
    int peakBin;
    int peakBin2;
    int SNRdB;
    int SNRdB2;
    int numFrames;
} CaptureData;

/**
 * @struct RidgeLine
 * @brief Stores ridge line information for procCaptureCWT()
 * @author ericdvet */
typedef struct
{
    int *pointScales;
    int *pointLocations;
    int length;
} RidgeLine;

/**
 * @function procRadarFrames(const char *localDataPath, const char *captureName, double tagHz)
 * @param localDataPath - Local file path to radar capture
 * @param captureName - Name of radar capture file
 * @param tagHz - Frequency at which tag is oscillating in Hz
 * @return CaptureData *
 * @brief Function processes radar frames for various purposes
 * @author ericdvet */
CaptureData *procRadarFrames(const char *localDataPath, const char *captureName, double tagHz);

/**
 * @function procTagTest(const char *localDataPath, const char *captureName, double tagHz)
 * @param localDataPath - Local file path to radar capture
 * @param captureName - Name of radar capture file
 * @param captureName - Frequency at which tag is oscillating in Hz
 * @return double
 * @brief Function prints capture FT and tag FT to CSV files and returns SNR in dB
 * @author ericdvet */
double procTagTest(const char *localDataPath, const char *captureName, double tagHz);

/**
 * @function procTwoTag(const char *localDataPath, const char *captureName, double tag1Hz, double tag2Hz)
 * @param localDataPath - Local file path to radar capture
 * @param captureName - Name of radar capture file
 * @param tag1Hz - Frequency at which tag 1 is oscillating in Hz
 * @param tag2Hz - Frequency at which tag 2 is oscillating in Hz
 * @return CaptureData *
 * @brief Function processes radar frames for various purposes
 * @author ericdvet */
CaptureData *procTwoTag(const char *localDataPath, const char *captureName, double tag1Hz, double tag2Hz);

/**
 * @function freeCaptureData(CaptureData *captureData)
 * @param captureData - CaptureData struct to free
 * @return None
 * @brief Free RadarData constructed by procRadarFrames()
 * @author ericdvet */
void freeCaptureData(CaptureData *captureData);

/**
 * @function procLargestPeak(double *tagFT)
 * @param *tagFT - pointer to FT of the tag's frequency isolated
 * @return int
 * @brief Returns bin corresponding to the largest peak
 * @author ericdvet */
int procLargestPeak(double *tagFT);

/**
 * @function procCaptureCWT(double *tagFT)
 * @param *tagFT - pointer to FT of the tag's frequency isolated
 * @return int
 * @brief Returns bin corresponding to the peak most similar to the ricker wavelet based
 * @author ericdvet */
int procCaptureCWT(double *tagFT);

/**
 * @function procSoilMoisture(double wetPeakBin, double airPeakBin, const char* soilType, double distance)
 * @param wetPeakBin - peak bin of backscatter tag covered by wet soil
 * @param airPeakBin - peak bin of backscatter tag uncovered by soil
 * @param soilType - type of soil
 * @param distance - distance between backscatter tag and surface in meters
 * @return double 
 * @brief Returns VWC calculated based on ToF and teros-12 sensor calibrations
 * @author ericdvet */
double procSoilMoisture(double wetPeakBin, double airPeakBin, const char* soilType, double distance);

#endif