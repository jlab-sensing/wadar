
/**
   @file doppler_adv.h

   These are the more internal functions that doppler uses.  This will allow for the exact FFT library to be specified instead of using the default library for the platform.  In some situations this could result in better performance.

   @warning Depending on the platform and settings at build, not all libraries will be avalible for all

   @copyright Copyright 2015 by FlatEarth, Inc

   @par Environment
   Environment Independent

   @par Compiler
   Compiler Independent

   @author Raymond J. Weber
*/


#ifndef DOPPLER_ADV_h
#define DOPPLER_ADV_h


#ifdef __cplusplus
extern "C"
{
#endif



#ifdef USE_NE10
#include "NE10.h"
///Use the ne10 complex float datatype
#define CFLOAT_TYPE ne10_fft_cpx_float32_t
#else
#include "kiss_fft.h"
///Use the KISSFFT complex float datatype
#define CFLOAT_TYPE kiss_fft_cpx
#endif






/** Initialize a doppler structure
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_ne10
*/
int doppler_initialize_ne10(dopplerConfig *config, int dopplerLength, int dopplerTime);

/** Initialize a doppler structure with zero padding
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @param dopplerTimeFFT Length of the FFTs to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_ne10
*/
int doppler_initializePadded_ne10(dopplerConfig *config, int dopplerLength, int dopplerTime, int dopplerTimeFFT);

/** Do the actual doppler processing using NE10 FFTs (ARM only)
  @param config Config containing the data to process
  @return SUCCESS or error code
  @ingroup doppler_adv_ne10
*/
int doppler_processWindow_ne10(dopplerConfig *config);

/** Release memory used by the NE10 doppler functions
  @param config Config to release the memory related too
  @returns SUCCESS or error code
  @ingroup doppler_adv_ne10
*/
int doppler_free_ne10(dopplerConfig *config);






/** Initialize a doppler structure
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_kissfft
*/
int doppler_initialize_kissfft(dopplerConfig *config, int dopplerLength, int dopplerTime);

/** Initialize a doppler structure with zero padding
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @param dopplerTimeFFT Length of the FFTs to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_kissfft
*/
int doppler_initializePadded_kissfft(dopplerConfig *config, int dopplerLength, int dopplerTime, int dopplerTimeFFT);

/** Do the actual doppler processing using NE10 FFTs (ARM only)
  @param config Config containing the data to process
  @return SUCCESS or error code
  @ingroup doppler_adv_kissfft
*/
int doppler_processWindow_kissfft(dopplerConfig *config);

/** Release memory used by the NE10 doppler functions
  @param config Config to release the memory related too
  @returns SUCCESS or error code
  @ingroup doppler_adv_kissfft
*/
int doppler_free_kissfft(dopplerConfig *config);




/** Initialize a doppler structure
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_initialize_ooura(dopplerConfig *config, int dopplerLength, int dopplerTime);

/** Initialize a doppler structure with zero padding
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @param dopplerTimeFFT Length of the FFTs to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_initializePadded_ooura(dopplerConfig *config, int dopplerLength, int dopplerTime, int dopplerTimeFFT);

/** Do the actual doppler processing using NE10 FFTs (ARM only)
  @param config Config containing the data to process
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_processWindow_ooura(dopplerConfig *config);

/** Release memory used by the NE10 doppler functions
  @param config Config to release the memory related too
  @returns SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_free_ooura(dopplerConfig *config);







/** Initialize a doppler structure
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_sp_initialize_ooura(dopplerConfig *config, int dopplerLength, int dopplerTime);

/** Initialize a doppler structure with zero padding
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @param dopplerTimeFFT Length of the FFTs to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_sp_initializePadded_ooura(dopplerConfig *config, int dopplerLength, int dopplerTime, int dopplerTimeFFT);

/** Do the actual doppler processing using NE10 FFTs (ARM only)
  @param config Config containing the data to process
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_sp_processWindow_ooura(dopplerConfig *config);

/** Release memory used by the NE10 doppler functions
  @param config Config to release the memory related too
  @returns SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_sp_free_ooura(dopplerConfig *config);





/** Initialize a doppler structure
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_dp_initialize_ooura(dopplerConfig *config, int dopplerLength, int dopplerTime);

/** Initialize a doppler structure with zero padding
  @param config Config containing the data to create
  @param dopplerLength Length of the frames that will be processed
  @param dopplerTime Number of frames to use in the doppler processing step
  @param dopplerTimeFFT Length of the FFTs to use in the doppler processing step
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_dp_initializePadded_ooura(dopplerConfig *config, int dopplerLength, int dopplerTime, int dopplerTimeFFT);

/** Do the actual doppler processing using ooura FFTs
  @param config Config containing the data to process
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_dp_processWindow_ooura(dopplerConfig *config);

/** Do the inverse doppler processing using ooura FFTs
  This is used for doppler CFAR based methods to get from the doppler domain back into the time domain
  @param config Config containing the data to process
  @return SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_dp_iProcessWindow_ooura(dopplerConfig *config);



/** Release memory used by the NE10 doppler functions
  @param config Config to release the memory related too
  @returns SUCCESS or error code
  @ingroup doppler_adv_ooura
*/
int doppler_dp_free_ooura(dopplerConfig *config);









int doppler_sp_iProcessWindow_ooura(dopplerConfig *config);






int doppler_sp_initialize(int dopplerLength, int dopplerTime, dopplerConfig *config);
int doppler_sp_initializePadded(int dopplerLength, int dopplerTime, int dopplerTimeFFT, dopplerConfig *config);
int doppler_sp_setWindow(int windowType, dopplerConfig *config);
int doppler_sp_addFrame(float *newFrameRe, float *newFrameIm, dopplerConfig *config);
int doppler_spz_addFrame(float *newFrameRe, float *newFrameIm, dopplerConfig *config);
int doppler_sp_processWindow(dopplerConfig *config);
int doppler_sp_getLastWindow(float *dopplerWindowRe, float *dopplerWindowIm, dopplerConfig *config);
int doppler_spz_getLastWindow(float *dopplerWindow, dopplerConfig *config);
int doppler_sp_free(dopplerConfig *config);


/**
  @defgroup doppler_adv_ooura Ooura Doppler
  @ingroup doppler_adv
*/

/**
  @defgroup doppler_adv_kissfft Kissfft Doppler
  @ingroup doppler_adv
*/

/**
  @defgroup doppler_adv_ne10 NE10 Doppler
  @ingroup doppler_adv
*/

/**
  @defgroup doppler_adv Advanced Doppler Functions

  @ingroup doppler
*/




#ifdef __cplusplus
}
#endif
#endif
