/*
 * File:   proc.h
 * Author: ericdvet
 *
 * Functions to process .frames data to obtain tag information
 */

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
 * @brief Stores important data collected by procradarFrames()
 * @author ericdvet */
typedef struct
{
    bool procSuccess;
    double complex *captureFT;
    double *tagFT;
    int peakBin;
    int SNRdB;
} CaptureData;

/**
 * @function procRadarFrames(const char *localDataPath, const char *captureName, double tagHz)
 * @param localDataPath - Local file path to radar capture
 * @param captureName - Name of radar capture file
 * @param captureName - Frequency at which tag is oscillating in Hz
 * @return CaptureData *
 * @brief Function processes radar frames for various purposes
 * @author ericdvet */
CaptureData *procRadarFrames(const char *localDataPath, const char *captureName, double tagHz);

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

// int procCaptureCWT(double *tagFT, int length);