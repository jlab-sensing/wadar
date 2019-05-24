/**
  @file doppler.h

  Impulse doppler processing allows for highly sensitive detections to be made of moving objects.  This can allow for very simple detection of moving objects, respiration and heartbeat (most difficult).

  Impulse doppler operates by building a 2d circular buffer of n radar frames.  Radar frames are rotated in on the long time axes, and the oldest radar frame is rotated out.  An fft across the long axis (same sampler, different time) then results in a 2d plot of velocity vs range.

  The inputs to the 2d circular buffer should be analytic (IQ) signals.  If real signals are used the resulting matrix will be speed vs range (mirrored).

  @copyright Copyright 2015 by FlatEarth, Inc

  @par Environment
  Environment Independent

  @par Compiler
  Compiler Independent

  @author Raymond J. Weber
*/

#ifndef DOPPLER_h
#define DOPPLER_h

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


#ifdef USE_NE10
#include "NE10.h"
///Use the ne10 complex float datatypes
#define CFLOAT_TYPE ne10_fft_cpx_float32_t
#else
#include "kiss_fft.h"
///Use the KISSFFT complex float datatype
#define CFLOAT_TYPE kiss_fft_cpx
#endif

/** Structure type to store complex numbers */
typedef struct
{
  int r;        //!< Real component
  int i;        //!< Imaginary component
} int_complex;


/** Structure to hold the Doppler configuration and data
  @ingroup doppler
*/
typedef struct DopplerConfig
{
  int length;            //!< Length of the radar frame (fast time)
  int dopplerTime;       //!< Number of time steps (long time)
  int dopplerTimeFFT;    //!< Length of the long time FFT's (Must be greater or equal to dopplerTime)
  int windowType;        //!< Type of window to apply on long time axis
  CFLOAT_TYPE **fin;     //!< 2d FIFO complex buffer of the radar frames to process
  CFLOAT_TYPE **fout;    //!< 2d FIFO complex buffer of the doppler output matrix
  CFLOAT_TYPE **foutInv; //!< 2d FIFO complex buffer of the inverse doppler output matrix
#ifdef USE_NE10
  ne10_fft_cfg_float32_t cfg;
#endif
} dopplerConfig;


/** Structure to hold the Doppler configuration and data
  @ingroup doppler
*/
typedef struct dopplerConfig_i
{
  int length;            /**< Length of the radar frame (fast time) */
  int dopplerTime;       /**< Number of time steps (long time) */
  int windowType;        /**< Type of window to apply on long time axis */
  int_complex **fin;     /**< 2d FIFO complex buffer of the radar frames to process */
  int_complex **fout;    /**< 2d FIFO complex buffer of the doppler output matrix */
} dopplerConfig_i;


/** Initialize the doppler processing routine
  @param dopplerLength Number of fast time steps (the length of the radar frame)
  @param dopplerTime Number of long time steps (radar frames to use)
  @param config The DopplerFilter structure to initialize
  @ingroup doppler
  @note Will be replaced by doppler_dp_initialize().  This function will likely change datatypes in the future.
*/
int doppler_initialize(int dopplerLength, int dopplerTime, dopplerConfig *config);

/** Initialize the doppler processing routine
  @param dopplerLength Number of fast time steps (the length of the radar frame)
  @param dopplerTime Number of long time steps (radar frames to use)
  @param dopplerTimeFFT Length of the long time FFT's (Must be greater or equal to dopplerTime)
  @param config The DopplerFilter structure to initialize
  @ingroup doppler

  @note
  In this version, the dopplerTime and dopplerTimeFFT values are independent. In
  use, the dopplerTime would represent the number of columns in the FFT matrix,
  16 for example. So the matrix is storing the last 16 signals. However, we can
  zero pad the column to a longer length to increase the frequency resolution.
  For example, we could do a 128 point FFT, so the last 112 values would be set
  to 0.

  @note Will be replaced by doppler_dp_initializePadded() soon.  This function will likely change datatypes in the future.
*/
int doppler_initializePadded(int dopplerLength, int dopplerTime, int dopplerTimeFFT, dopplerConfig *config);

/** Add a new radar frame to the doppler processing
  @param [in] newFrameRe Real Component of the radar frame to add to the doppler filter
  @param [in] newFrameIm Imaginary Component of the radar frame to add
  @param [in] config The Doppler filter to use
  @ingroup doppler

  @note Will be replaced by doppler_dp_addFrame() soon.  This function will likely change datatypes in the future.
*/
int doppler_addFrame(double *newFrameRe, double *newFrameIm, dopplerConfig *config);

/** Perform the doppler processing.  This is seperate as it may be desirable to add multiple frame and then process as this step could be slow.
  @param config The doppler filter to run the processing step on
  @ingroup doppler
  @note Will be replaced by doppler_dp_processWindow() soon.  This function will likely change datatypes in the future.
*/
int doppler_processWindow(dopplerConfig *config);

/** Output the last window of doppler processed data
  @param [out] dopplerWindowRe Real components of the doppler output
  @param [out] dopplerWindowIm Imaginary components of the doppler output
  @param config The structure to pull the results from
  @ingroup doppler
  @note Will be replaced by doppler_dp_getLastWindow() soon.  This function will likely change datatypes in the future.
*/
int doppler_getLastWindow(double *dopplerWindowRe, double *dopplerWindowIm, dopplerConfig *config);

/** Output the last window of doppler processed data
  @param [out] dopplerWindowRe Real components of the doppler output
  @param [out] dopplerWindowIm Imaginary components of the doppler output
  @param config The structure to pull the results from
  @ingroup doppler
  @deprecated May be removed in future releases
*/
int doppler_getLastWindowF(float *dopplerWindowRe, float *dopplerWindowIm, dopplerConfig *config);

/** Output the last window of doppler processed data
  @param [out] dopplerWindowZ Doppler output as complex doubles
  @param config The structure to pull the results from
  @ingroup doppler
  @deprecated May be removed in future releases
*/
int doppler_getLastWindowC(float *dopplerWindowZ, dopplerConfig *config);

/** Output the last window of doppler processed data
  @param [out] dopplerWindowZ Doppler output as complex doubles
  @param config The structure to pull the results from
  @ingroup doppler
  @note Will be replaced by doppler_dpz_getLastWindow() soon?  This function will likely change datatypes in the future.
*/
int doppler_getLastWindowZ(double *dopplerWindowZ, dopplerConfig *config);

/** Set the window function for long time to reduce sidelobes
  @param windowType Type of the window function to use (defined in radarDSP.h)
  @param config The structure to pull the results from
  @ingroup doppler
  @note Will be replaced by doppler_dp_setWindow() soon.  This function will likely change datatypes in the future.
*/
int doppler_setWindow(int windowType, dopplerConfig *config);

/** Free the memory associated with the stucture to prevent memory leaks when the program is finished with the structure.
  @note This is not currently needed (The current implementation uses static allocation instead, future version may begin using dynamic allocation)
  @param config The structure to pull the results from
  @ingroup doppler
  @note Will be replaced by doppler_dp_free() soon.  This function will likely change datatypes in the future.
*/
int doppler_free(dopplerConfig *config);













/** Initialize the doppler processing routine
  @param dopplerLength Number of fast time steps (the length of the radar frame)
  @param dopplerTime Number of long time steps (radar frames to use)
  @param config The DopplerFilter structure to initialize
  @ingroup doppler
  @deprecated Will be removed in a future version and replaced by doppler_initialize()
*/
int doppler_i_initialize(int dopplerLength, int dopplerTime, dopplerConfig_i *config);

/** Add a new radar frame to the doppler processing
  @param [in] newFrameRe Real Component of the radar frame to add to the doppler filter
  @param [in] newFrameIm Imaginary Component of the radar frame to add
  @param [in] config The Doppler filter to use
  @ingroup doppler
  @deprecated Will be removed in a future version and replaced by doppler_addframe()
*/
int doppler_i_addFrame(int *newFrameRe, int *newFrameIm, dopplerConfig_i *config);

/** Perform the doppler processing.  This is seperate as it may be desirable to add multiple frame and then process as this step could be slow.
  @param config The doppler filter to run the processing step on
  @ingroup doppler
  @deprecated Will be removed in a future version and replaced with a doppler_processWindow()
*/
int doppler_i_processWindow(dopplerConfig_i *config);

/** Output the last window of doppler processed data
  @param [out] dopplerWindowRe Real components of the doppler output
  @param [out] dopplerWindowIm Imaginary components of the doppler output
  @param config The structure to pull the results from
  @ingroup doppler
  @deprecated Will be removed in a future version and replaced with doppler_getLastWindow()
*/
int doppler_i_getLastWindow(int *dopplerWindowRe, int *dopplerWindowIm, dopplerConfig_i *config);

/** Output the last window of doppler processed data
  @param [out] dopplerWindowRe Real components of the doppler output
  @param [out] dopplerWindowIm Imaginary components of the doppler output
  @param config The structure to pull the results from
  @ingroup doppler
  @deprecated Will be removed in a future version
*/
int doppler_i_getLastWindowF(int *dopplerWindowRe, int *dopplerWindowIm, dopplerConfig_i *config);

/** Output the last window of doppler processed data
  @param [out] dopplerWindowZ Doppler output as complex doubles
  @param config The structure to pull the results from
  @ingroup doppler
  @deprecated Will be removed in a future version and replaced by maybe doppler_z_getLastWindow()
*/
int doppler_i_getLastWindowC(int *dopplerWindowZ, dopplerConfig_i *config);

/** Output the last window of doppler processed data
  @param [out] dopplerWindowZ Doppler output as complex doubles
  @param config The structure to pull the results from
  @ingroup doppler
  @deprecated Will be removed in a future version
*/
int doppler_i_getLastWindowZ(int *dopplerWindowZ, dopplerConfig_i *config);

/** Set the window function for long time to reduce sidelobes
  @param windowType Type of the window function to use (defined in radarDSP.h)
  @param config The structure to pull the results from
  @ingroup doppler
  @deprecated Will be removed in a future version
*/
int doppler_i_setWindow(int windowType, dopplerConfig_i *config);

/** Free the memory associated with the stucture to prevent memory leaks when the program is finished with the structure.
  @note This is not currently needed (The current implementation uses static allocation instead, future version may begin using dynamic allocation)
  @param config The structure to pull the results from
  @ingroup doppler
  @deprecated Will be removed in a future version
*/
int doppler_i_free(dopplerConfig_i *config);














/**
  @warning Beta
*/
int doppler_dp_getLastWindow(double *dopplerWindowRe, double *dopplerWindowIm, dopplerConfig *config);

/**
  @warning Beta
*/
int doppler_dp_addFrame(double *newFrameRe, double *newFrameIm, dopplerConfig *config);

/**
  @defgroup doppler Impulse Doppler
  Impulse doppler processing is a Radar Technique to greatly enhance sensitivity of moving targets.

  Impulse doppler can also improve target resolution by allowing to clearly distinguishing targets at the same or similar range as long as they are travelling at different velocities.

  To use these functions include the header with:
  \code{.c}
  #include <flatearth/Doppler.h>
  \endcode

  and link the library with
  \code{.c}
  -ldoppler
  \endcode



  @ingroup salsaLib
*/




#ifdef __cplusplus
}
#endif
#endif
