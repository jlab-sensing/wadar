/*
 * File:   wadar.h
 * Author: ericdvet
 *
 * High level WaDAR functions. Functions to automatically capture and process radar data to determine soil moisture content. 
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "proc.h"
#include <unistd.h>
#include "utils.h"
#include <curl/curl.h>

// Capture parameters
#define FRAME_RATE 200
#define RADAR_TYPE "Chipotle"
#define SOIL_TYPE "farm"

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
double wadar(char *localDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount, double tagDepth) {

    double volumetricWaterContent = 0.0;

    // Process Air Capture
    CaptureData* airCapture = procRadarFrames(localDataPath, airFramesName, tagHz);
    if (!airCapture || !airCapture->procSuccess) {
        printf("ERROR: Air Frames Invalid\n");
        return -1;
    }
    int airPeakBin = airCapture->peakBin;

    // Load soil capture
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char captureName[256];
    snprintf(captureName, sizeof(captureName), "%d-%02d-%02d_%dmmDepth_%s_C", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, (int)(tagDepth * 1000), trialName);

    // Frame logger command
    char frameLoggerCommand[1024];
    snprintf(frameLoggerCommand, sizeof(frameLoggerCommand),
             "ssh root@192.168.7.2 \"screen -dmS radar -m bash -c && cd FlatEarth/Demos/Common/FrameLogger && nice -n -20 ./frameLogger -s ../data/captureSettings -l ../data/%s -n %d -r %d -f %d -t %s -c %s \" &",
             captureName, frameCount, captureCount, FRAME_RATE, RADAR_TYPE, localDataPath);
    system(frameLoggerCommand);

    // Load and Process Captures
    double SNRdB[captureCount];
    double peakMagnitudes[captureCount];
    int peakBin[captureCount];
    double vwc[captureCount];
    int failedCaptures[captureCount];
    int failedCount = 0;

    for (int i = 0; i < captureCount; i++) {
        printf("Please wait. Capture %d is proceeding\n", i + 1);
        usleep((frameCount / FRAME_RATE) * 1000000);
        printf("Waiting for data to be transferred...\n");
        usleep((frameCount / FRAME_RATE) * 1000000 * 0.5);

        char wetFramesName[1000];
        snprintf(wetFramesName, sizeof(wetFramesName), "%s%d.frames", captureName, i + 1);
        CaptureData* wetCapture = procRadarFrames(localDataPath, wetFramesName, tagHz);
        if (!wetCapture || !wetCapture->procSuccess) {
            failedCaptures[failedCount++] = i;
            printf("Capture %d will not be processed due to an issue.\n", i + 1);
            continue;
        }

        peakBin[i] = wetCapture->peakBin;
        peakMagnitudes[i] = wetCapture->tagFT[peakBin[i]];
        vwc[i] = procSoilMoisture(peakBin[i], airPeakBin, SOIL_TYPE, tagDepth);

        freeCaptureData(wetCapture);
    }

    // Free air capture data
    freeCaptureData(airCapture);

    // Remove failed captures
    for (int i = 0; i < failedCount; i++) {
        int idx = failedCaptures[i];
        for (int j = idx; j < captureCount - 1; j++) {
            SNRdB[j] = SNRdB[j + 1];
            peakMagnitudes[j] = peakMagnitudes[j + 1];
            peakBin[j] = peakBin[j + 1];
            vwc[j] = vwc[j + 1];
        }
        captureCount--;
    }

    volumetricWaterContent = median(vwc, captureCount);

    return volumetricWaterContent;

}

// Function to post data to the URL
void wadar2dirtviz(const char *url) {
    CURL *curl;
    CURLcode res;
    struct curl_slist *headers = NULL;

    // Variables for JSON data
    long ts = 12345678;
    const char *cell = "dummy";
    double vwc = 0.4;
    int raw_vwc = 3;
    int temp = 2;
    int ec = 2;
    int water_pot = 1;

    // Create JSON data
    char json_data[512];
    snprintf(json_data, sizeof(json_data),
        "{\"ts\":%ld,\"cell\":\"%s\",\"vwc\":%.2f,\"raw_vwc\":%d,\"temp\":%d,\"ec\":%d,\"water_pot\":%d}",
        ts, cell, vwc, raw_vwc, temp, ec, water_pot
    );

    // Initialize CURL
    curl = curl_easy_init();
    if(curl) {
        // Set URL for the POST request
        curl_easy_setopt(curl, CURLOPT_URL, url);

        // Set the JSON data to be sent
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data);

        // Set the content type to application/json
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // Perform the POST request
        res = curl_easy_perform(curl);

        // Check for errors
        if(res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        }

        // Cleanup
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }
}



#define WADAR_TEST
#ifdef WADAR_TEST
int main()
{
    // CaptureData *captureData;
    // captureData = procRadarFrames("/home/ericdvet/jlab/wadar/signal_processing/", "testFile.frames", 80);
    // procTagTest("/home/ericdvet/jlab/wadar/signal_processing/", "testFile.frames", 80);

    // wadar("/home/ericdvet/jlab/wadar/signal_processing/", "testFile.frames", "testFile.frames", 80, 200, 1, 0.1);

    wadar2dirtviz("https://dirtviz.jlab.ucsc.edu/dev/data-collection/frontend/api/teros/");

    return 0;
}
#endif