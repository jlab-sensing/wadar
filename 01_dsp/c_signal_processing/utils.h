#ifndef UTILS_H
#define UTILS_H

/*
 * File:   utils.h
 * Author: ericdvet
 *
 * Utility functions to assist with WaDAR's data and signal processing functions
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
 * @function NoveldaDDC(double *rfSignal, double complex *basebandSignal)
 * @param rfSignal - Raw radar frames
 * @param basebandSignal - Resulting digitally downcoverted radar frames
 * @return None
 * @brief Function to apply a digital downcovert (DDC) to a high frequency radar
 *      signal. Brings signal to baseband frequencies and provides an analytic
 *      signal (i.e. I & Q, in-phase & quadrature, outputs)
 * @author ericdvet */
void NoveldaDDC(double *rfSignal, double complex *basebandSignal);

/**
 * @function hamming(double *window, int M)
 * @param window - Resulting hamming window
 * @param M - Size of window
 * @return None
 * @brief Returns the N-point symmetric Hamming window in a column vector
 * @author ericdvet */
void hamming(double *window, int M);

/**
 * @function smoothData(double *data, int length, int windowSize)
 * @param *data - Data to smooth
 * @param length - Length of data
 * @param windowSize - Length of smoothing window
 * @return None
 * @brief Smooth noisy data by averaging over each window of windowSize.
 * @author ericdvet */
void smoothData(double *data, int length, int windowSize);

/**
 * @function computeFFT(double complex *framesBB, double complex *captureFT, int numFrames, int numOfSamplers)
 * @param *framesBB - Input of FFT
 * @param captureFT - Output of FFT
 * @param numFrames - Number of frames (total columns)
 * @param numOfSamplers - Number of samplers (total rows)
 * @return None
 * @brief Fast Fourier transform (FFT) of input
 *      The FFT block computes the fast Fourier transform (FFT) across the first
 *      dimension of an N-D input array, u.
 * @author ericdvet */
void computeFFT(double complex *framesBB, double complex *captureFT, int numFrames, int numOfSamplers);

/**
 * @function findPeaks(double *arr, int size, int *numPeaks, double minPeakHeight)
 * @param *arr - Array to find peaks from
 * @param size - Length of array
 * @param *numPeaks - Resulting number of peaks
 * @param minPeakHeight - Minimum peak height
 * @return int
 * @brief Returns local peaks in data array
 * @author ericdvet */
int *findPeaks(double *arr, int size, int *numPeaks, double minPeakHeight);

/**
 * @function calculateSNR(double complex *captureFT, int numOfSamplers, int freqTag, int peakBin)
 * @param *captureFT - FT of radar frames
 * @param numOfSamplers - Number of sampelrs (rows)
 * @param freqTag - FT isolation of backscatter tag
 * @param peakBin - Determined peak bin location of backscatter tag
 * @return float
 * @brief Returns signal to noise ratio. Calculates ratio of peak bin amplitude at desired 
 *      frequency vs an average of a few irrelevant frequencies 
 * @author ericdvet */
double calculateSNR(double complex *captureFT, int numOfSamplers, int freqTag, int peakBin);

/**
 * @function compare(const void *a, const void *b)
 * @param *a - First number to compare
 * @param *b - Second number to compare
 * @return int
 * @brief Compare function for qsort() to sort in ascending order
 * @author ericdvet */
int compare(const void *a, const void *b);

/**
 * @function median(double *arr, int length)
 * @param *arr - Array to find median from
 * @param length - Length of array
 * @return double
 * @brief Returns the median of an array of numbers
 * @author ericdvet */
double median(double *arr, int length);

#endif