/**
   @file detection.h

   The detection library provides functions for assisting in determining if a return is a radar detection or noise.  This includes basic functions for pruning signals below a threhsold value, as well as CFAR which builds an adaptive threshold map on a sampler by sampler basis.
   @ingroup detection

   Copyright 2015 by FlatEarth, Inc

   @par Environment
   Environment Independent

   @par Compiler
   Compiler Independent

   @author Raymond Weber
*/

#ifndef DETECTION_h
#define DETECTION_h

#include "CFAR.h"

#ifdef __cplusplus
extern "C"
{
#endif


/**
  Prunes input signal by user determined dB amount

  The maximum signal amplitude is determined and values 'x' dB below this max
  are set to 0.
  @param [in] input   The input signal
  @param [in] output  The pruned output signal
  @param [in] length  The length of the signal
  @param [in] prune   Pruning threshold (in dB)
  @ingroup detection
*/
int detection_Prune(double *input, double *output, unsigned int length, double prune);



/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup detection
*/
int detection_thresholdIndex(int *sequence, int *threshold, int *returnSequence, int length);

/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup detection
*/
int detection_sp_thresholdIndex(float *sequence, float *threshold, int *returnSequence, int length);

/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup detection
*/
int detection_dp_thresholdIndex(double *sequence, double *threshold, int *returnSequence, int length);



/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup detection
*/
int detection_thresholdValues(int *sequence, int *threshold, int *returnSequence, int length);

/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup detection
*/
int detection_sp_thresholdValues(float *sequence, float *threshold, float *returnSequence, int length);

/** Find the points where the waveform is above a threshold and filter out the low samples in the return sequence
  @param sequence The input waveform
  @param threshold A threshold value for each samples
  @param returnSequence The thresholder result, sequence can be the same array
  @param length The length of the vector to operate on
  @ingroup detection
*/
int detection_dp_thresholdValues(double *sequence, double *threshold, double *returnSequence, int length);



/**
  @defgroup detection Detection
  Functions related to determining detection ratios in radar sigals

  This functions are split into different catagories
  - General Thresholding functions
  - Fixed thresholds
  - Constant False Alarm Rate
  - Functions to adaptivly detemine the thresholds during runtime

  @ingroup salsaLib
*/


/**
  @defgroup detection_thresholds Fixed Threshold Functions

  @ingroup detection
*/




#ifdef __cplusplus
}
#endif
#endif
