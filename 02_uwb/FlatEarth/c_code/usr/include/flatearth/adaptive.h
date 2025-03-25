/**
  @file adaptive.h

  This library provides the functions for an adaptive cluttermap algorithm with adjustable adaptation (convergence) rates.  This algorithm is good for situations where the radar returns need clutter reduction for moving or near stationary returns.  Compared to the other clutter reduction algorithms this one has a slower convergence rate then MTI (eg: it can detection stationary objects for longer), but the adaptation allows it to self-compensate for environmental changes and new stationary reflectors better then a static cluttermap.

  @copyright 2015-2018 by FlatEarth, Inc

  @author Raymond Weber
  @author Justin Hadella
*/

#ifndef CLUTTERMAP_ADAPTIVE_h
#define CLUTTERMAP_ADAPTIVE_h

#include <stdint.h>


/** The structure holding setup and running parameters for an adaptive cluttermap
    @ingroup cluttermap_adaptive
*/
typedef struct adaptiveClutterMapConfig
{
  double beta;               /**< Adaption (convergence) Rate [0-1] - This adjusts how quickly the filter nulls non-moving signals.  Values greater generally seem to 0.9 work well in most situations */
  double alpha;              /**< Amount of the cluttermap to subtract [0 and 1] - Adjusting this value can sometimes allow for the detection of strong static signals even after cluttermap removal by only removing a portion of the clutter */
  int length;                /**< Number of samplers in the radar frame */
  float *cluttermap;         /**< Current cluttermap (internal signal, I can't foresee many situations where monitoring/modifying this signal manually would be useful) */
} adaptiveClutterMapConfig;


#ifdef __cplusplus
extern "C"
{
#endif


/** Initialize a cluttermap structure for an adaptive cluttermap
  @param config Pointer to an adaptiveClutterMapConfig structure
  @param length Number of samplers in the radar frames that will be used
  @param betaValue = Adaption (convergence) Rate [0-1] - This adjusts how quickly the filter nulls non-moving signals.  Values greater generally seem to 0.9 work well in most situations
  @param alphaValue = Subtraction Percentage [0-1], how much of the cluttermap to remove from the signal.  Adjusting this value can sometimes allow for the detection of strong static signals even after cluttermap removal by only removing a portion of the clutter
  @param config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_initializeACluttermap(int length, float betaValue, float alphaValue, adaptiveClutterMapConfig *config);

/**
  Clear content of existing adaptive cluttermap
  @param [in,out] *config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_clearACluttermap(adaptiveClutterMapConfig *config);

/** Update the cluttermap with a new signal without creating a decluttered version of it.
  @param newSignal A uint array containing the signal to declutter
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_updateACluttermap(int32_t *newSignal, adaptiveClutterMapConfig *config);

/** Apply the adaptive cluttermap to a signal
  @note To do this in place, newSignal and returnSignal can be set to the same array

  @param newSignal A uint array containing the signal to declutter
  @param returnSignal A uint array to contain the decluttered signal
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_removeAClutter(int32_t *newSignal, int32_t *returnSignal, adaptiveClutterMapConfig *config);

/** Update the adaptive "beta" value which controls the speed of adaptation
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_updateBeta(float betaValue, adaptiveClutterMapConfig *config);

/** Gets the current adaptive "beta" value
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_getBeta(float *betaValue, adaptiveClutterMapConfig *config);

/** Update the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [in] alphaValue Controls portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_updateAlpha(float alphaValue, adaptiveClutterMapConfig *config);

/** Gets the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] alphaValue Current portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_getAlpha(float *alphaValue, adaptiveClutterMapConfig *config);

/** Free memory used in in the cluttermap to prevent memory leaks when done with it for the final time
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_adaptiveFree(adaptiveClutterMapConfig *config);








/** Initialize a cluttermap structure for an adaptive cluttermap
  @param config Pointer to an adaptiveClutterMapConfig structure
  @param length Number of samplers in the radar frames that will be used
  @param betaValue = Adaption (convergence) Rate [0-1] - This adjusts how quickly the filter nulls non-moving signals.  Values greater generally seem to 0.9 work well in most situations
  @param alphaValue = Subtraction Percentage [0-1], how much of the cluttermap to remove from the signal.  Adjusting this value can sometimes allow for the detection of strong static signals even after cluttermap removal by only removing a portion of the clutter
  @param config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_z_initializeACluttermap(int length, float betaValue, float alphaValue, adaptiveClutterMapConfig *config);

/**
  Clear content of existing adaptive cluttermap
  @param [in,out] *config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_z_clearACluttermap(adaptiveClutterMapConfig *config);

/** Update the cluttermap with a new signal without creating a decluttered version of it.
  @param newSignal A uint array containing the signal to declutter
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_z_updateACluttermap(int *newSignal, adaptiveClutterMapConfig *config);

/** Apply the adaptive cluttermap to a signal
  @note To do this in place, newSignal and returnSignal can be set to the same array

  @param newSignal A uint array containing the signal to declutter
  @param returnSignal A uint array to contain the decluttered signal
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_z_removeAClutter(int *newSignal, int *returnSignal, adaptiveClutterMapConfig *config);

/** Update the adaptive "beta" value which controls the speed of adaptation
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_z_updateBeta(float betaValue, adaptiveClutterMapConfig *config);

/** Gets the current adaptive "beta" value
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_z_getBeta(float *betaValue, adaptiveClutterMapConfig *config);

/** Update the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [in] alphaValue Controls portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_z_updateAlpha(float alphaValue, adaptiveClutterMapConfig *config);

/** Gets the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] alphaValue Current portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_z_getAlpha(float *alphaValue, adaptiveClutterMapConfig *config);

/** Free memory used in in the cluttermap to prevent memory leaks when done with it for the final time
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_z_adaptiveFree(adaptiveClutterMapConfig *config);


/** Initialize a cluttermap structure for an adaptive cluttermap
  @param config Pointer to an adaptiveClutterMapConfig structure
  @param length Number of samplers in the radar frames that will be used
  @param betaValue = Adaption (convergence) Rate [0-1] - This adjusts how quickly the filter nulls non-moving signals.  Values greater generally seem to 0.9 work well in most situations
  @param alphaValue = Subtraction Percentage [0-1], how much of the cluttermap to remove from the signal.  Adjusting this value can sometimes allow for the detection of strong static signals even after cluttermap removal by only removing a portion of the clutter
  @param config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_sp_initializeACluttermap(int length, float betaValue, float alphaValue, adaptiveClutterMapConfig *config);

/**
  Clear content of existing adaptive cluttermap
  @param [in,out] *config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_sp_clearACluttermap(adaptiveClutterMapConfig *config);

/** Update the cluttermap with a new signal without creating a decluttered version of it.
  @param newSignal A uint array containing the signal to declutter
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_sp_updateACluttermap(float *newSignal, adaptiveClutterMapConfig *config);

/** Apply the adaptive cluttermap to a signal
  @note To do this in place, newSignal and returnSignal can be set to the same array

  @param newSignal A uint array containing the signal to declutter
  @param returnSignal A uint array to contain the decluttered signal
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_sp_removeAClutter(float *newSignal, float *returnSignal, adaptiveClutterMapConfig *config);

/** Update the adaptive "beta" value which controls the speed of adaptation
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_sp_updateBeta(float betaValue, adaptiveClutterMapConfig *config);

/** Gets the current adaptive "beta" value
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_sp_getBeta(float *betaValue, adaptiveClutterMapConfig *config);

/** Update the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [in] alphaValue Controls portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_sp_updateAlpha(float alphaValue, adaptiveClutterMapConfig *config);

/** Gets the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] alphaValue Current portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_sp_getAlpha(float *alphaValue, adaptiveClutterMapConfig *config);

/** Free memory used in in the cluttermap to prevent memory leaks when done with it for the final time
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_sp_adaptiveFree(adaptiveClutterMapConfig *config);


/** Initialize a cluttermap structure for an adaptive cluttermap
  @param config Pointer to an adaptiveClutterMapConfig structure
  @param length Number of samplers in the radar frames that will be used
  @param betaValue = Adaption (convergence) Rate [0-1] - This adjusts how quickly the filter nulls non-moving signals.  Values greater generally seem to 0.9 work well in most situations
  @param alphaValue = Subtraction Percentage [0-1], how much of the cluttermap to remove from the signal.  Adjusting this value can sometimes allow for the detection of strong static signals even after cluttermap removal by only removing a portion of the clutter
  @param config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_spz_initializeACluttermap(int length, float betaValue, float alphaValue, adaptiveClutterMapConfig *config);

/**
  Clear content of existing adaptive cluttermap
  @param [in,out] *config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_spz_clearACluttermap(adaptiveClutterMapConfig *config);

/** Update the cluttermap with a new signal without creating a decluttered version of it.
  @param newSignal A uint array containing the signal to declutter
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_spz_updateACluttermap(float *newSignal, adaptiveClutterMapConfig *config);

/** Apply the adaptive cluttermap to a signal
  @note To do this in place, newSignal and returnSignal can be set to the same array

  @param newSignal A uint array containing the signal to declutter
  @param returnSignal A uint array to contain the decluttered signal
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_spz_removeAClutter(float *newSignal, float *returnSignal, adaptiveClutterMapConfig *config);

/** Update the adaptive "beta" value which controls the speed of adaptation
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_spz_updateBeta(float betaValue, adaptiveClutterMapConfig *config);

/** Gets the current adaptive "beta" value
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_spz_getBeta(float *betaValue, adaptiveClutterMapConfig *config);

/** Update the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [in] alphaValue Controls portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_spz_updateAlpha(float alphaValue, adaptiveClutterMapConfig *config);

/** Gets the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] alphaValue Current portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_spz_getAlpha(float *alphaValue, adaptiveClutterMapConfig *config);

/** Free memory used in in the cluttermap to prevent memory leaks when done with it for the final time
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_spz_adaptiveFree(adaptiveClutterMapConfig *config);


/** Initialize a cluttermap structure for an adaptive cluttermap
  @param config Pointer to an adaptiveClutterMapConfig structure
  @param length Number of samplers in the radar frames that will be used
  @param betaValue = Adaption (convergence) Rate [0-1] - This adjusts how quickly the filter nulls non-moving signals.  Values greater generally seem to 0.9 work well in most situations
  @param alphaValue = Subtraction Percentage [0-1], how much of the cluttermap to remove from the signal.  Adjusting this value can sometimes allow for the detection of strong static signals even after cluttermap removal by only removing a portion of the clutter
  @param config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_dp_initializeACluttermap(int length, double betaValue, double alphaValue, adaptiveClutterMapConfig *config);

/**
  Clear content of existing adaptive cluttermap
  @param [in,out] *config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_dp_clearACluttermap(adaptiveClutterMapConfig *config);

/** Update the cluttermap with a new signal without creating a decluttered version of it.
  @param newSignal A uint array containing the signal to declutter
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_dp_updateACluttermap(double *newSignal, adaptiveClutterMapConfig *config);

/** Apply the adaptive cluttermap to a signal
  @note To do this in place, newSignal and returnSignal can be set to the same array

  @param newSignal A uint array containing the signal to declutter
  @param returnSignal A uint array to contain the decluttered signal
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_dp_removeAClutter(double *newSignal, double *returnSignal, adaptiveClutterMapConfig *config);

/** Update the adaptive "beta" value which controls the speed of adaptation
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_dp_updateBeta(double betaValue, adaptiveClutterMapConfig *config);

/** Gets the current adaptive "beta" value
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_dp_getBeta(double *betaValue, adaptiveClutterMapConfig *config);

/** Update the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [in] alphaValue Controls portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_dp_updateAlpha(double alphaValue, adaptiveClutterMapConfig *config);

/** Gets the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] alphaValue Current portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_dp_getAlpha(double *alphaValue, adaptiveClutterMapConfig *config);

/** Free memory used in in the cluttermap to prevent memory leaks when done with it for the final time
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_dp_adaptiveFree(adaptiveClutterMapConfig *config);


/** Initialize a cluttermap structure for an adaptive cluttermap
  @param config Pointer to an adaptiveClutterMapConfig structure
  @param length Number of samplers in the radar frames that will be used
  @param betaValue = Adaption (convergence) Rate [0-1] - This adjusts how quickly the filter nulls non-moving signals.  Values greater generally seem to 0.9 work well in most situations
  @param alphaValue = Subtraction Percentage [0-1], how much of the cluttermap to remove from the signal.  Adjusting this value can sometimes allow for the detection of strong static signals even after cluttermap removal by only removing a portion of the clutter
  @param config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_initializeACluttermap(int length, double betaValue, double alphaValue, adaptiveClutterMapConfig *config);

/**
  Clear content of existing adaptive cluttermap
  @param [in,out] *config Pointer to an adaptiveClutterMapConfig
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_clearACluttermap(adaptiveClutterMapConfig *config);

/** Update the cluttermap with a new signal without creating a decluttered version of it.
  @param newSignal A uint array containing the signal to declutter
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_updateACluttermap(double *newSignal, adaptiveClutterMapConfig *config);

/** Apply the adaptive cluttermap to a signal
  @note To do this in place, newSignal and returnSignal can be set to the same array

  @param newSignal A uint array containing the signal to declutter
  @param returnSignal A uint array to contain the decluttered signal
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_removeAClutter(double *newSignal, double *returnSignal, adaptiveClutterMapConfig *config);

/** Update the adaptive "beta" value which controls the speed of adaptation
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_updateBeta(double betaValue, adaptiveClutterMapConfig *config);

/** Gets the current adaptive "beta" value
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] betaValue Speed of the cluttermap to remove stationary signals (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_getBeta(double *betaValue, adaptiveClutterMapConfig *config);

/** Update the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [in] alphaValue Controls portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_updateAlpha(double alphaValue, adaptiveClutterMapConfig *config);

/** Gets the "alpha" value which controls the portion of clutter removed
  @param [in] config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @param [out] alphaValue Current portion of clutter to remove (0 to 1)
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_getAlpha(double *alphaValue, adaptiveClutterMapConfig *config);

/** Free memory used in in the cluttermap to prevent memory leaks when done with it for the final time
  @param config Pointer to an adaptiveClutterMapConfig with the cluttermap parameters and history
  @ingroup cluttermap_adaptive
*/
int clutter_dpz_adaptiveFree(adaptiveClutterMapConfig *config);


/**
  @defgroup cluttermap_adaptive Adaptive
  Adaptive cluttermaps slowly adapt to changes in the enviroment.  Stationary objects appearing are visible for a period of time, then dissappear.

  To use these functions include:
  \code{.c}
  #include <adaptive.h>
  \endcode

  and link in the makefile against the cluttermap library:
  \code{.c}
  -lcluttermap.so
  \endcode

  Example Usage:
  \code{.c}
  #include <adaptive.h>

  int main()
  {
  //Create an adaptive cluttermap object
  adaptiveClutterMapConfig config;

  //Initialize the cluttermap w/ 512 samplers, adaptation rate = .9, and subtract the full cluttermap
  clutter_initializeACluttermap(512, .9f, 1.0f, &config);
  while(1){
    //  Get a radar frame  //
    //  *****************  //

    //Update the cluttermap with the radarFrame
    clutter_updateACluttermap(radarFrame, &config);

    //Subtract the cluttermap from the radarframe in-place
    clutter_removeAClutter(radarFrame,radarFrame,&config);

    // Use the radar frame for something //
    //  *******************************  //
  }
  clutter_adaptiveFree(&config);

  return 0;
  }
  \endcode

  @ingroup cluttermaps
*/



#ifdef __cplusplus
}
#endif
#endif

