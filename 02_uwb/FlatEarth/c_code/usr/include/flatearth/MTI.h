/**
   @file MTI.h

   Definitions for external functions in cluttermap library

   MTI Filters are a computationally simple method for removing stationary targets and clutter from radar signals.  They are commonly used as a pre-processing step for Pulse Doppler processing algorithms.

   The MTI filter works by keeping a short history of the last N radar frames in memory and applying a low pass filter across the each sampler in the long time direction.


   Copyright 2015 by FlatEarth, Inc

   @author Raymond Weber
*/

#ifndef CLUTTERMAP_MTI_h
#define CLUTTERMAP_MTI_h



#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAXLENGTH_MTI 6144        /**< Largest radar frame size allowed */


/** Structure to hold the setup, temporary variables, and output of an MTI Filter
  @ingroup cluttermap_MTI
*/
typedef struct MTIClutterMapConfig
{
  int MTIType;                          /**< What type of MTI is this, Fixed 3 pulse, fixed 5 pulse, custom */
  int MTILength;                        /**< Number of pulses for the MTI filter (custom) */
  int MTIInd;                           /**< Interval variable for where the next frame will be place in a circular buffer */
  int length;                           /**< Number of samplers in the radar frame*/
  double windowFunction[10];            /**< Custom coefficients */
  uint32_t MTIData[MAXLENGTH_MTI][10];  /**< Circular buffer to store the last N frames */
  int valid;                            /**< Have MTILenth number of frames occured?*/
} MTIClutterMapConfig;


/** Initialize a MTI Cluttermap Structure
  @param [in] type Type of the MTI Cluttermap routine to use (1==3 pulse, 2==5 pulse, 3==custom)
  @param [in] length Length of the cluttermap (number of samplers)
  @param config Pointer to a MTIClutterMapConfig structure
  @ingroup cluttermap_MTI
*/
int clutter_mti_initialize(int type, int length, MTIClutterMapConfig *config);


/** Apply the MTI Filter to a signal and return a decluttred signal
  @param [in] newSignal Raw signal to process
  @param [out] returnSignal Decluttered signal (can be the same as newSignal)
  @param config Pointer to a MTIClutterMapConfig structure
  @ingroup cluttermap_MTI
*/
int clutter_mti_process(uint32_t *newSignal, int32_t *returnSignal, MTIClutterMapConfig *config);

/** Initialize a single precision MTI Cluttermap structure
  @param type Number of pulses in the cluttermap
  @param length Number of samplers coming from the radar into the clutter processing
  @param config a pointer to a MTICluttermap structure to store use for the processing
  @returns SUCCESSS or FAILURE
  @ingroup cluttermap_MTI
*/
int clutter_mti_sp_initialize(int type, int length, MTIClutterMapConfig *config);

/** Run the MTI cluttermap on a new radar frame and return a result, single precision version
  @param newSignal The radar signal to declutter
  @param returnSignal The decluttered data
  @param config The initialized MTIClutterMapConfig structure with the parameters and data to use and update
  @returns SUCCESS or FAILURE
  @ingroup cluttermap_MTI
*/
int clutter_mti_sp_process(float *newSignal, float *returnSignal, MTIClutterMapConfig *config);

/** Initialize a double precision MTI Cluttermap structure
  @param type Number of pulses in the cluttermap
  @param length Number of samplers coming from the radar into the clutter processing
  @param config a pointer to a MTICluttermap structure to store use for the processing
  @returns SUCCESSS or FAILURE
*/
int clutter_mti_dp_initialize(int type, int length, MTIClutterMapConfig *config);

/** Run the MTI cluttermap on a new radar frame and return a result, double precision version
  @param newSignal The radar signal to declutter
  @param returnSignal The decluttered data
  @param config The initialized MTIClutterMapConfig structure with the parameters and data to use and update
  @returns SUCCESS or FAILURE
  @ingroup cluttermap_MTI
*/
int clutter_mti_dp_process(double *newSignal, double *returnSignal, MTIClutterMapConfig *config);

/**
  @defgroup cluttermap_MTI MTI
  MTI (Moving Target Indicator) Cluttermaps nearly instantly converge to remove any stationary objects as soon as they become stationary.

  To use these functions include:
  \code{.c}
  #include <MTI.h>
  \endcode

  and link in the makefile against the cluttermap library:
  \code{.c}
  -lcluttermap.so
  \endcode

  Example Usage:
  \code{.c}
  #include <MTI.h>

  int main()
  {
  //Create an adaptive cluttermap object
  MTIClutterMapConfig config;

  //Initialize the cluttermap w/ 512 samplers, 3 pulse MTI algorithm
  clutter_mti_initialize(1, 512, &config);

  while(1){
    //  Get a radar frame  //
    //  *****************  //

    //Subtract the cluttermap from the radarframe in-place
    clutter_mti_process(radarFrame,radarFrame,&config);

    // Use the radar frame for something //
    //  *******************************  //
  }
  return 0;
  }
  @endcode

  @ingroup cluttermaps
*/



#ifdef __cplusplus
}
#endif
#endif
