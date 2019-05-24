/**
   @file static.h

   A static cluttermap is the most basic clutter removal method.  A radar frame is collected at time N, and that signal is removed from any subsequent processed frame.  This can be highly effective IF: the cluttermap was collected with no targets in the field, there are no temperature variations, the static clutter does not change (eg: the radar is stationary).  Some of these issues can be mitigated by periodically rebuilding the static cluttermap.


   Copyright 2015 by FlatEarth, Inc

   @author Raymond Weber
*/


#ifndef CLUTTERMAP_STATIC_h
#define CLUTTERMAP_STATIC_h


#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif


//#define MAXLENGTH_STATIC 4096

/** Structure to hold the setup and current settings for a Static Cluttermap
  @ingroup cluttermap_static
*/
typedef struct staticClutterMapConfig
{
  int length;                    /**< Length of the radar frame */
  uint32_t *storedCluttermap;    /**< Array to hold a stored cluttermap */
  //uint32_t storedCluttermap[MAXLENGTH_STATIC];  /**< Array to hold a stored cluttermap */
} staticClutterMapConfig;

/** Create and a static cluttermap structure
  @param config Pointer to a staticClutterMapConfig structure
  @param length Length of the clutter signal
  @ingroup cluttermap_static
*/
int clutter_intializeSCluttermap(int length, staticClutterMapConfig *config);

/** Sets a static cluttermap
  @param newSignal A uint array containing the clutter signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_setSCluttermap(uint32_t *newSignal, staticClutterMapConfig *config);

/** Apply the static cluttermap to a signal
  @note newSignal and returnSignal can be the same array to use in place and overwrite the array with a decluttered version of it

  @param newSignal A uint array containing the signal to declutter
  @param returnSignal A uint array to contain the decluttered signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_removeSClutter(uint32_t *newSignal, int32_t *returnSignal, staticClutterMapConfig *config);

/** Free dynamically allocated memory in the staticClutterMapConfig structure to avoid memory leaks when done with the structure.  If the same struct is used later, it must be re-initialized before use.
  @param config Stucture to cleanup.
  @ingroup cluttermap_static
*/
int clutter_staticFree(staticClutterMapConfig *config);

/** Create and a static cluttermap structure
  @param config Pointer to a staticClutterMapConfig structure
  @param length Length of the clutter signal
  @ingroup cluttermap_static
*/
int clutter_sp_intializeSCluttermap(int length, staticClutterMapConfig *config);

/** Sets a static cluttermap
  @param newSignal A float array containing the clutter signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_sp_setSCluttermap(float *newSignal, staticClutterMapConfig *config);

/** Apply the static cluttermap to a signal
  @note newSignal and returnSignal can be the same array to use in place and overwrite the array with a decluttered version of it

  @param newSignal A float array containing the signal to declutter
  @param returnSignal A float array to contain the decluttered signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_sp_removeSClutter(float *newSignal, float *returnSignal, staticClutterMapConfig *config);

/** Free dynamically allocated memory in the staticClutterMapConfig structure to avoid memory leaks when done with the structure.  If the same struct is used later, it must be re-initialized before use.
  @param config Stucture to cleanup.
  @ingroup cluttermap_static
*/
int clutter_sp_staticFree(staticClutterMapConfig *config);

/** Create and a static cluttermap structure
  @param config Pointer to a staticClutterMapConfig structure
  @param length Length of the clutter signal
  @ingroup cluttermap_static
*/
int clutter_dp_intializeSCluttermap(int length, staticClutterMapConfig *config);

/** Sets a static cluttermap
  @param newSignal A double array containing the clutter signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_dp_setSCluttermap(double *newSignal, staticClutterMapConfig *config);

/** Apply the static cluttermap to a signal
  @note newSignal and returnSignal can be the same array to use in place and overwrite the array with a decluttered version of it

  @param newSignal A double array containing the signal to declutter
  @param returnSignal A double array to contain the decluttered signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_dp_removeSClutter(double *newSignal, double *returnSignal, staticClutterMapConfig *config);

/** Free dynamically allocated memory in the staticClutterMapConfig structure to avoid memory leaks when done with the structure.  If the same struct is used later, it must be re-initialized before use.
  @param config Stucture to cleanup.
  @ingroup cluttermap_static
*/
int clutter_dp_staticFree(staticClutterMapConfig *config);

























/** Create and a static cluttermap structure
  @param config Pointer to a staticClutterMapConfig structure
  @param length Length of the clutter signal
  @ingroup cluttermap_static
*/
int clutter_z_intializeSCluttermap(int length, staticClutterMapConfig *config);

/** Sets a static cluttermap
  @param newSignal A uint array containing the clutter signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_z_setSCluttermap(int *newSignal, staticClutterMapConfig *config);

/** Apply the static cluttermap to a signal
  @note newSignal and returnSignal can be the same array to use in place and overwrite the array with a decluttered version of it

  @param newSignal A uint array containing the signal to declutter
  @param returnSignal A uint array to contain the decluttered signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_z_removeSClutter(int *newSignal, int32_t *returnSignal, staticClutterMapConfig *config);

/** Free dynamically allocated memory in the staticClutterMapConfig structure to avoid memory leaks when done with the structure.  If the same struct is used later, it must be re-initialized before use.
  @param config Stucture to cleanup.
  @ingroup cluttermap_static
*/
int clutter_z_staticFree(staticClutterMapConfig *config);

/** Create and a static cluttermap structure
  @param config Pointer to a staticClutterMapConfig structure
  @param length Length of the clutter signal
  @ingroup cluttermap_static
*/
int clutter_spz_intializeSCluttermap(int length, staticClutterMapConfig *config);

/** Sets a static cluttermap
  @param newSignal A float array containing the clutter signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_spz_setSCluttermap(float *newSignal, staticClutterMapConfig *config);

/** Apply the static cluttermap to a signal
  @note newSignal and returnSignal can be the same array to use in place and overwrite the array with a decluttered version of it

  @param newSignal A float array containing the signal to declutter
  @param returnSignal A float array to contain the decluttered signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_spz_removeSClutter(float *newSignal, float *returnSignal, staticClutterMapConfig *config);

/** Free dynamically allocated memory in the staticClutterMapConfig structure to avoid memory leaks when done with the structure.  If the same struct is used later, it must be re-initialized before use.
  @param config Stucture to cleanup.
  @ingroup cluttermap_static
*/
int clutter_spz_staticFree(staticClutterMapConfig *config);

/** Create and a static cluttermap structure
  @param config Pointer to a staticClutterMapConfig structure
  @param length Length of the clutter signal
  @ingroup cluttermap_static
*/
int clutter_dpz_intializeSCluttermap(int length, staticClutterMapConfig *config);

/** Sets a static cluttermap
  @param newSignal A double array containing the clutter signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_dpz_setSCluttermap(double *newSignal, staticClutterMapConfig *config);

/** Apply the static cluttermap to a signal
  @note newSignal and returnSignal can be the same array to use in place and overwrite the array with a decluttered version of it

  @param newSignal A double array containing the signal to declutter
  @param returnSignal A double array to contain the decluttered signal
  @param config Pointer to a staticClutterMapConfig structure
  @ingroup cluttermap_static
*/
int clutter_dpz_removeSClutter(double *newSignal, double *returnSignal, staticClutterMapConfig *config);

/** Free dynamically allocated memory in the staticClutterMapConfig structure to avoid memory leaks when done with the structure.  If the same struct is used later, it must be re-initialized before use.
  @param config Stucture to cleanup.
  @ingroup cluttermap_static
*/
int clutter_dpz_staticFree(staticClutterMapConfig *config);

















/**
  @defgroup cluttermap_static Static
  Static cluttermaps are the most brittle, but can be very effective for detecting static targets appearing in the enviroment.

  Care must be taken however as the radar returns change slightly with chip temperature making this filter less effective the longer it is left.

  To use these functions include:
  \code{.c}
  #include <static.h>
  \endcode

  and link in the makefile against the cluttermap library:
  \code{.c}
  -lcluttermap.so
  \endcode


  Example Usage:
  \code{.c}
  //Create an adaptive cluttermap object
  #include <static.h>

  int main()
  {
  adaptiveClutterMapConfig config;

  //Initialize the cluttermap w/ 512 samplers
  clutter_intializeSCluttermap(512, &config);

  //  Get a radar frame  //
  //  *****************  //

  //Update the cluttermap with the radarFrame
  clutter_setSCluttermap(radarFrame, &config);

  while(1){
    //  Get a radar frame  //
    //  *****************  //

    //Subtract the cluttermap from the radarframe in-place
    clutter_removeSClutter(radarFrame,radarFrame,&config);

    // Use the radar frame for something //
    //  *******************************  //
  }
  clutter_staticFree(&config);

  return 0;
  }
  @endcode


  @ingroup cluttermaps
*/






#ifdef __cplusplus
}
#endif
#endif

