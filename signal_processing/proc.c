/*
 * File:   proc.h
 * Author: ericdvet
 *
 * Functions to process .frames data to obtain tag information
 */

#include "proc.h"
#include "utils.h"
#include "salsa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include "wavelib/header/wavelib.h"

/**
 * @function procRadarFrames(const char *localDataPath, const char *captureName, double tagHz)
 * @param localDataPath - Local file path to radar capture
 * @param captureName - Name of radar capture file
 * @param captureName - Frequency at which tag is oscillating in Hz
 * @return CaptureData *
 * @brief Function processes radar frames for various purposes
 * @author ericdvet */
CaptureData *procRadarFrames(const char *localDataPath, const char *captureName, double tagHz)
{

    CaptureData *captureData = (CaptureData *)malloc(sizeof(CaptureData));

    // Processing parameters
    int frameRate = 200;
    int numOfSamplers = 512;

    // Load Capture
    char fullPath[1024];
    const char *colon = strchr(localDataPath, ':');
    if (colon != NULL) {
        localDataPath = colon + 1;
    }
    sprintf(fullPath, "%s/%s", localDataPath, captureName);
    RadarData *radarData = salsaLoad(fullPath);
    if (radarData == NULL)
    {
        return false;
    }

    double *rfSignal;
    double complex *framesBB;
    double complex *temp;

    rfSignal = (double *)malloc(numOfSamplers * sizeof(double));
    framesBB = (double complex *)malloc((radarData->numFrames) * numOfSamplers * sizeof(double complex));
    temp = (double complex *)malloc(numOfSamplers * sizeof(double complex));

    // Baseband Conversion
    for (int i = 0; i < radarData->numFrames; i++)
    {
        for (int j = 0; j < numOfSamplers; j++)
        {
            rfSignal[j] = radarData->frameTot[j + i * numOfSamplers];
        }
        NoveldaDDC(rfSignal, temp);
        for (int j = 0; j < numOfSamplers; j++)
        {
            framesBB[j + i * numOfSamplers] = temp[j];
            // printf("%d ", j + i * numOfSamplers);
        }
    }

    // for (int i = 500; i < numOfSamplers; i++) {
    //     printf("%f ", creal(framesBB[i]));
    // }
    // printf("\n");

    // Find Tag FT
    int freqTag = (int)(tagHz / frameRate * radarData->numFrames);

    captureData->captureFT = (double complex *)malloc(radarData->numFrames * numOfSamplers * sizeof(double complex));

    computeFFT(framesBB, captureData->captureFT, radarData->numFrames, numOfSamplers);

    // for (int i = 0; i < numOfSamplers; i++) {
    //     printf("%f\n", creal(captureData->captureFT[i]));
    // }

    captureData->tagFT = (double *)malloc(numOfSamplers * sizeof(double *));

    double maxFTPeak;
    int idx_maxFTPeak;
    maxFTPeak = 0;

    for (int j = freqTag - 2; j <= freqTag + 2; j++)
    {
        for (int i = 0; i < numOfSamplers; i++)
        {
            if (cabs(captureData->captureFT[i + numOfSamplers * (j - 1)]) > maxFTPeak)
            {
                maxFTPeak = cabs(captureData->captureFT[i + numOfSamplers * (j - 1)]);
                idx_maxFTPeak = j;
            }
        }
    }
    freqTag = idx_maxFTPeak;

    for (int i = 0; i < numOfSamplers; i++)
    {
        captureData->tagFT[i] = (double)cabs(captureData->captureFT[i + numOfSamplers * (idx_maxFTPeak - 1)]);
        // printf("%f\n", captureData->tagFT[i]);
    }

    // smoothData(captureData->tagFT, numOfSamplers, 10);

    for (int i = 0; i < numOfSamplers; i++)
    {
        // captureData->tagFT[i] = (float) cabs(captureData->captureFT[i + numOfSamplers * (idx_maxFTPeak)]);
        // printf("%d: %f\n", i, captureData->tagFT[i]);
    }

    // peakBin = procLargestPeak(captureData->tagFT);
    captureData->peakBin = procCaptureCWT(captureData->tagFT);

    // printf("\nPeak of %f at %d\n", captureData->tagFT[captureData->peakBin], captureData->peakBin);

    captureData->SNRdB = calculateSNR(captureData->captureFT, numOfSamplers, freqTag, captureData->peakBin);

    // printf("SNR of %f\n", SNR);

    captureData->numFrames = radarData->numFrames;
    captureData->procSuccess = true;

    free(rfSignal);
    free(framesBB);
    free(temp);
    freeRadarData(radarData);

    return captureData;
}

/**
 * @function procTagTest(const char *localDataPath, const char *captureName, double tagHz)
 * @param localDataPath - Local file path to radar capture
 * @param captureName - Name of radar capture file
 * @param captureName - Frequency at which tag is oscillating in Hz
 * @return None
 * @brief Function prints capture FT and tag FT to CSV files
 * @author ericdvet */
double procTagTest(const char *localDataPath, const char *captureName, double tagHz) {
    CaptureData *captureData;
    captureData = procRadarFrames(localDataPath, captureName, tagHz);

    FILE *fileTagFT = fopen("tagFT.csv", "w");
    if (fileTagFT == NULL) {
        perror("Error opening file");
        return -1;
    }


    for (int i = 0; i < 512; i++) {
        fprintf(fileTagFT, "%.2f\n", captureData->tagFT[i]);  // Write to file in CSV format
    }

    FILE *fileCaptureFT = fopen("captureFT.csv", "w");
    if (fileCaptureFT == NULL) {
        perror("Error opening file");
        return -1;
    }

    for (int j = 0; j < 512; j++)
    {
        for (int i = 0; i < captureData->numFrames - 1; i++)
        {
            fprintf(fileCaptureFT, "%.2f, ", fabs(captureData->captureFT[j + i * 512]));
        }
        fprintf(fileCaptureFT, "%.2f\n", fabs(captureData->captureFT[j + (captureData->numFrames-1) * 512]));
    }

    double SNR = captureData->SNRdB;

    printf("SNR of %f\n", SNR);

    freeCaptureData(captureData);
    
    return SNR;
}

CaptureData *procTwoTag(const char *localDataPath, const char *captureName, double tag1Hz, double tag2Hz) {
    CaptureData *captureData = (CaptureData *)malloc(sizeof(CaptureData));

    // Processing parameters
    int frameRate = 200;
    int numOfSamplers = 512;

    // Load Capture
    char fullPath[1024];
    sprintf(fullPath, "%s/%s", localDataPath, captureName);
    RadarData *radarData = salsaLoad(fullPath);
    if (radarData == NULL)
    {
        return false;
    }

    double *rfSignal;
    double complex *framesBB;
    double complex *temp;

    rfSignal = (double *)malloc(numOfSamplers * sizeof(double));
    framesBB = (double complex *)malloc((radarData->numFrames) * numOfSamplers * sizeof(double complex));
    temp = (double complex *)malloc(numOfSamplers * sizeof(double complex));

    // Baseband Conversion
    for (int i = 0; i < radarData->numFrames; i++)
    {
        for (int j = 0; j < numOfSamplers; j++)
        {
            rfSignal[j] = radarData->frameTot[j + i * numOfSamplers];
        }
        NoveldaDDC(rfSignal, temp);
        for (int j = 0; j < numOfSamplers; j++)
        {
            framesBB[j + i * numOfSamplers] = temp[j];
            // printf("%d ", j + i * numOfSamplers);
        }
    }

    // for (int i = 500; i < numOfSamplers; i++) {
    //     printf("%f ", creal(framesBB[i]));
    // }
    // printf("\n");

    // Find Tag FT
    int freqTag1 = (int)(tag1Hz / frameRate * radarData->numFrames);
    int freqTag2 = (int)(tag2Hz / frameRate * radarData->numFrames);

    captureData->captureFT = (double complex *)malloc(radarData->numFrames * numOfSamplers * sizeof(double complex));

    computeFFT(framesBB, captureData->captureFT, radarData->numFrames, numOfSamplers);

    // for (int i = 0; i < numOfSamplers; i++) {
    //     printf("%f\n", creal(captureData->captureFT[i]));
    // }

    captureData->tagFT = (double *)malloc(numOfSamplers * sizeof(double *));
    captureData->tagFT2 = (double *)malloc(numOfSamplers * sizeof(double *));

    double maxFTPeak1, maxFTPeak2;
    int idx_maxFTPeak1, idx_maxFTPeak2;
    maxFTPeak1 = 0;
    maxFTPeak2 = 0;

    for (int j = freqTag1 - 2; j <= freqTag1 + 2; j++)
    {
        for (int i = 0; i < numOfSamplers; i++)
        {
            if (cabs(captureData->captureFT[i + numOfSamplers * (j - 1)]) > maxFTPeak1)
            {
                maxFTPeak1 = cabs(captureData->captureFT[i + numOfSamplers * (j - 1)]);
                idx_maxFTPeak1 = j;
            }
        }
    }
    freqTag1 = idx_maxFTPeak1;

    for (int j = freqTag2 - 2; j <= freqTag2 + 2; j++)
    {
        for (int i = 0; i < numOfSamplers; i++)
        {
            if (cabs(captureData->captureFT[i + numOfSamplers * (j - 1)]) > maxFTPeak2)
            {
                maxFTPeak2 = cabs(captureData->captureFT[i + numOfSamplers * (j - 1)]);
                idx_maxFTPeak2 = j;
            }
        }
    }
    freqTag2 = idx_maxFTPeak2;

    for (int i = 0; i < numOfSamplers; i++)
    {
        captureData->tagFT[i] = (double)cabs(captureData->captureFT[i + numOfSamplers * (idx_maxFTPeak1 - 1)]);
        captureData->tagFT2[i] = (double)cabs(captureData->captureFT[i + numOfSamplers * (idx_maxFTPeak2 - 1)]);
        // printf("%f\n", captureData->tagFT[i]);
    }

    smoothData(captureData->tagFT, numOfSamplers, 10);

    for (int i = 0; i < numOfSamplers; i++)
    {
        // captureData->tagFT[i] = (float) cabs(captureData->captureFT[i + numOfSamplers * (idx_maxFTPeak)]);
        // printf("%d: %f\n", i, captureData->tagFT[i]);
    }

    // peakBin = procLargestPeak(captureData->tagFT);
    captureData->peakBin = procCaptureCWT(captureData->tagFT);
    captureData->peakBin2 = procCaptureCWT(captureData->tagFT2);

    printf("\nPeak of %f at %d\n", captureData->tagFT[captureData->peakBin], captureData->peakBin);
    printf("\nPeak 2 of %f at %d\n", captureData->tagFT[captureData->peakBin2], captureData->peakBin2);

    captureData->SNRdB = calculateSNR(captureData->captureFT, numOfSamplers, freqTag1, captureData->peakBin);
    captureData->SNRdB2 = calculateSNR(captureData->captureFT, numOfSamplers, freqTag2, captureData->peakBin2);     



    // printf("SNR of %f\n", SNR);

    captureData->numFrames = radarData->numFrames;
    captureData->procSuccess = true;

    free(rfSignal);
    free(framesBB);
    free(temp);
    freeRadarData(radarData);

    return captureData;
}

/**
 * @function freeCaptureData(CaptureData *captureData)
 * @param captureData - CaptureData struct to free
 * @return None
 * @brief Free RadarData constructed by procRadarFrames()
 * @author ericdvet */
void freeCaptureData(CaptureData *captureData)
{
    if (captureData)
    {
        free(captureData->tagFT);
        free(captureData->captureFT);
        free(captureData);
    }
}

/**
 * @function procLargestPeak(double *tagFT)
 * @param *tagFT - pointer to FT of the tag's frequency isolated
 * @return int
 * @brief Returns bin corresponding to the largest peak
 * @author ericdvet */
int procLargestPeak(double *tagFT)
{
    double maxVal = -1;
    int size = 512;

    for (int i = 0; i < size; i++)
    {
        if (tagFT[i] > maxVal)
        {
            maxVal = tagFT[i];
        }
    }

    double minPeakHeight = maxVal * 0.9;
    int numPeaks = 0;
    int *peaks = findPeaks(tagFT, size, &numPeaks, minPeakHeight);

    int peakBin = -1;

    if (numPeaks > 1)
    {
        if ((peaks[1] - peaks[0]) < 50)
        {
            peakBin = peaks[0] + round((peaks[1] - peaks[0]) / 2.0) + round((tagFT[peaks[1]] - tagFT[peaks[0]]) / maxVal * (peaks[1] - peaks[0]));
        }
        else
        {
            peakBin = peaks[0];
        }
    }
    else if (numPeaks == 1)
    {
        peakBin = peaks[0];
    }

    free(peaks);
    return peakBin;
}

/**
 * @function procCaptureCWT(double *tagFT)
 * @param *tagFT - pointer to FT of the tag's frequency isolated
 * @return int
 * @brief Returns bin corresponding to the peak most similar to the ricker wavelet based
 * @author ericdvet */
int procCaptureCWT(double *tagFT)
{

    // Control variables
    int gapThreshold = 5;
    double slidingWindowThreshold = 0.3; // 0.5;
    // float SNRThreshold = 3.0;
    float ridgeLengthThreshold = 5;

    // Continuous Wavelet Transform
    cwt_object cwtInfo;
    cwtInfo = cwt_init("dog", 2, 512, 1, 32);
    setCWTScales(cwtInfo, 1, 2, "linear", 1);   
    cwt(cwtInfo, tagFT);

    // More control variables
    double amplThreshold;
    amplThreshold = 0;
    for (int i = 0; i < cwtInfo->J * cwtInfo->siglength; i++)
    {
        if (cwtInfo->output[i].re > amplThreshold)
        {
            amplThreshold = cwtInfo->output[i].re;
        }
    }
    amplThreshold = 0.1 * amplThreshold;


    // Find local maximums in each cwt
    double *cwtScaleCoeffs;
    int *numPeaks;
    int **peaks;
    peaks = (int **)malloc(cwtInfo->J * sizeof(int *));
    numPeaks = (int *)malloc(cwtInfo->J * sizeof(int));
    cwtScaleCoeffs = (double *)malloc(512 * sizeof(double));
    for (int i = 0; i < cwtInfo->J * cwtInfo->siglength; i++)
    {
        cwtScaleCoeffs[i % 512] = fabs(cwtInfo->output[i].re);
        if (i % 512 == 0 && i != 0)
        {
            peaks[(i / 512) - 1] = findPeaks(cwtScaleCoeffs, 512, &(numPeaks[(i / 512) - 1]), 0); // Fuck this line of code for the 2 hours of seg fault headaches
            for (int j = 0; j < numPeaks[(i / 512) - 1]; j++)
            {
            }
        }
    }
    peaks[((cwtInfo->J * cwtInfo->siglength) / 512) - 1] = findPeaks(cwtScaleCoeffs, 512, &(numPeaks[((cwtInfo->J * cwtInfo->siglength) / 512) - 1]), 0);

    // Find all ridge lines
    RidgeLine *ridgeLines;
    ridgeLines = (RidgeLine *)malloc(1000 * sizeof(RidgeLine));
    int numRidgeLines = 0;
    for (int scale = cwtInfo->J - 1; scale >= 0; scale--)
    {
        // printf("test: %d @ %d\n", numPeaks[scale], scale);
        for (int i = 0; i < numPeaks[scale]; i++)
        {
            int peak = peaks[scale][i];
            RidgeLine ridgeLine;
            ridgeLine.pointScales = (int *)malloc(cwtInfo->J * sizeof(int));
            ridgeLine.pointLocations = (int *)malloc(cwtInfo->J * sizeof(int));
            if (ridgeLine.pointScales == NULL || ridgeLine.pointLocations == NULL)
            {
                return -1;
            }
            ridgeLine.length = 1;
            ridgeLine.pointScales[0] = scale;
            ridgeLine.pointLocations[0] = peak;

            int gap = 0;
            for (int s = scale - 1; s >= 0 && gap <= gapThreshold; s--)
            {
                int closestMaxIdx = -1;
                double closestGap = slidingWindowThreshold * s;
                for (int j = 0; j < numPeaks[s]; j++)
                {
                    double currentGap = fabs(peak - peaks[s][j]);
                    if (currentGap < closestGap)
                    {
                        closestGap = currentGap;
                        closestMaxIdx = j;
                    }
                }
                if (closestMaxIdx != -1)
                {
                    ridgeLine.pointScales[ridgeLine.length] = s;
                    ridgeLine.pointLocations[ridgeLine.length] = peaks[s][closestMaxIdx];
                    ridgeLine.length++;
                    numPeaks[s]--;
                    gap = 0;
                }
                else
                {
                    gap++;
                }
            }

            if (ridgeLine.length > 1)
            {
                ridgeLines[numRidgeLines] = ridgeLine;
                numRidgeLines++;
            }
            else
            {
                free(ridgeLine.pointScales);
                free(ridgeLine.pointLocations);
            }
        }
    }

    // Process ridges
    int peakBin = -1;
    double maxAmplitude = 0;

    for (int i = 0; i < numRidgeLines; i++)
    {
        RidgeLine ridgeLine = ridgeLines[i];
        // Ridge length > threshold
        if (ridgeLine.length >= ridgeLengthThreshold)
        {
            for (int j = 0; j < ridgeLine.length; j++)
            {
                double amplitude = tagFT[ridgeLine.pointLocations[j]];
                // Scale with max amplitude > certain range
                if (amplitude > maxAmplitude)
                {
                    maxAmplitude = amplitude;
                    peakBin = ridgeLine.pointLocations[j];
                }
            }
        }
    }

    free(numPeaks);
    free(cwtScaleCoeffs);
    for (int i = 0; i < cwtInfo->J; i++)
    {
        free(peaks[i]);
    }
    free(peaks);
    for (int i = 0; i < numRidgeLines; i++) {
        free(ridgeLines[i].pointScales);
        free(ridgeLines[i].pointLocations);
    }
    free(ridgeLines);
    cwt_free(cwtInfo);

    return peakBin;
}

/**
 * @function procSoilMoisture(double wetPeakBin, double airPeakBin, const char* soilType, double distance)
 * @param wetPeakBin - peak bin of backscatter tag covered by wet soil
 * @param airPeakBin - peak bin of backscatter tag uncovered by soil
 * @param soilType - type of soil
 * @param distance - distance between backscatter tag and surface in meters
 * @return double 
 * @brief Returns VWC calculated based on ToF and teros-12 sensor calibrations
 * @author ericdvet */
double procSoilMoisture(double wetPeakBin, double airPeakBin, const char* soilType, double distance) {
    double t = ((wetPeakBin - airPeakBin + distance / 0.003790984152165) * 0.003790984152165) / 299792458.0;

    double radar_perm = pow((299792458.0 * t) / distance, 2);

    double perm_to_RAW = (0.01018 * pow(radar_perm, 3)) - (1.479 * pow(radar_perm, 2)) + (77.47 * radar_perm) + 1711;

    double VWC;

    // Soil type calibration
    if (strcmp(soilType, "farm") == 0) {
        VWC = (5.12018081e-10 * pow(perm_to_RAW, 3)) - (0.000003854251138 * pow(perm_to_RAW, 2)) + (0.009950433112 * perm_to_RAW) - 8.508168835941;
    } else if (strcmp(soilType, "stanfordFarm") == 0) {
        VWC = (9.079e-10 * pow(perm_to_RAW, 3)) - (6.626e-6 * pow(perm_to_RAW, 2)) + (1.643e-2 * perm_to_RAW) - 1.354e1;
    } else if (strcmp(soilType, "stanfordSilt") == 0) {
        VWC = (-3.475e-10 * pow(perm_to_RAW, 3)) + (2.263e-6 * pow(perm_to_RAW, 2)) - (4.515e-3 * perm_to_RAW) + 2.85e0;
    } else if (strcmp(soilType, "stanfordClay") == 0) {
        VWC = (5.916e-10 * pow(perm_to_RAW, 3)) - (4.536e-6 * pow(perm_to_RAW, 2)) + (1.183e-2 * perm_to_RAW) - 1.017e1;
    } else {
        fprintf(stderr, "ERROR: Need a legitimate soil type (farm, stanfordFarm, stanfordSilt, stanfordClay)\n");
        return -1.0;  // Return an error code
    }

    return VWC;
}

// #define PROC_TEST

#ifdef PROC_TEST
int main()
{
    // CaptureData *captureData;
    // captureData = procRadarFrames("/home/ericdvet/jlab/wadar/signal_processing/", "testFile.frames", 80);
    procTagTest("/home/ericdvet/hare-lab/dev_ws/src/wadar/signal_processing/data", "2024-10-10__testNoTag_C1.frames", 64);
    // procTwoTag("/home/ericdvet/jlab/wadar/signal_processing/", "testFile.frames", 79, 80);

    // freeCaptureData(captureData);

    return 0;
}
#endif