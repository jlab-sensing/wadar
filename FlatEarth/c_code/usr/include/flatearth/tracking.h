/**
  @file tracking.h

  Tracking is an API used to track a single target

  @copyright 2016 by FlatEarth, Inc

  @par Environment
  Environment Independent

  @par Compiler
  Compiler Independent

  @author Raymond Weber
  @author Justin Hadella
*/

#ifndef TRACKING_h
#define TRACKING_h

#ifdef __cplusplus
extern "C"
{
#endif

// -----------------------------------------------------------------------------
// Data Structure
// -----------------------------------------------------------------------------

/// Forward declaration for the tracking structure
typedef struct TrackingConfig_t *TrackingConfig;

// -----------------------------------------------------------------------------
// Definitions
// -----------------------------------------------------------------------------

/// Default value for update deltaT (100 ms)
#define DEFAULT_DELTA_T (0.1)

/// Maximum acquisition time value
#define MAX_ACQUISITION_TIME (30)

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/**
   Function used to initialize a tracking configuration

   @param [in] acquisitionTime  Controls number of tracking updates needed to transition from aquisition phase to tracking phase
   @param [in] coastTime        Controls number of updates in coasting phase before losing track
   @param [in] threshold        Controls spread of allowable location change

   @return Reference to a TrackingConfig (or null on failure)
  @ingroup tracking
*/
TrackingConfig track_initialize(int acquisitionTime, int coastTime, double threshold);

/**
   Function used to free memory allocated to given tracking config

   @param [out] config  Reference to memory to free up
  @ingroup tracking
*/
void track_free(TrackingConfig config);

/**
   Function to update a tracking config's deltaT value

   @param [out] config  The tracking configuration to update
   @param [in]  deltaT  The new deltaT value (in seconds)

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_setDeltaT(TrackingConfig config, double deltaT);

/**
   Function to get a tracking config's current deltaT value

   @param [in]   config  The tracking configuration to query
   @param [out]  deltaT  The deltaT value (in seconds)

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_getDeltaT(TrackingConfig config, double *deltaT);

/**
  Function to update a tracking config's acquisitionTime value

  @note
  In current version, 0 < acquisition time <= 10

  @param [out] config           The tracking configuration to update
  @param [in]  acquisitionTime  The new acquisitionTime value (# updates)

  @return Code indicating success/failure
  @ingroup tracking
*/
int track_setAcquisitionTime(TrackingConfig config, int acquisitionTime);

/**
   Function to get a tracking config's current acquisitionTime value

   @param [in]   config           The tracking configuration to query
   @param [out]  acquisitionTime  The acquisitionTime value (# updates)

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_getAcquisitionTime(TrackingConfig config, int *acquisitionTime);

/**
   Function to update a tracking config's coastTime value

   @param [out] config     The tracking configuration to update
   @param [in]  coastTime  The new coastTime value (# updates)

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_setCoastTime(TrackingConfig config, int coastTime);

/**
   Function to get a tracking config's current coastTime value

   @param [in]   config     The tracking configuration to query
   @param [out]  coastTime  The coastTime value (# updates)

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_getCoastTime(TrackingConfig config, int *coastTime);

/**
   Function to update a tracking config's current location using fixed deltaT

   @param [out] config    The tracking configuration to update
   @param [in]  location  The location

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_updateFixedDeltaT(TrackingConfig config, double location);

/**
   Function to update a tracking config's current location using given deltaT

   @note
   This version would be used when the deltaT is not fixed to a known value

   @param [out] config    The tracking configuration to update
   @param [in]  location  The location
   @param [in]  deltaT    The deltaT

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_update(TrackingConfig config, double location, double deltaT);

/**
   Function used to get a tracking config's current phase

   @param [in]  config  The tracking configuration to query
   @param [out] phase   The current phase

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_getCurrentPhase(TrackingConfig config, int *phase);

/**
   Function used to get a tracking config's current estimated distance

   @param [in]  config      The tracking configuration to query
   @param [out] estDistance The estimated distance to target

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_getEstDistance(TrackingConfig config, double *estDistance);

/**
   Function used to get a tracking config's current estimated next distance

   @param [in]  config           The tracking configuration to query
   @param [out] estNextDistance  The estimated next distance to target

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_getEstNextDistance(TrackingConfig config, double *estNextDistance);

/**
   Function used to get a tracking config's current estimated velocity

   @param [in]  config       The tracking configuration to query
   @param [out] estVelocity  The estimated velocity of target

   @return Code indicating success/failure
  @ingroup tracking
*/
int track_getEstVelocity(TrackingConfig config, double *estVelocity);







/**
  @defgroup tracking Tracking
  Function to track a single target though aquisition, tracking, and coast phases.

  To use these functions include:
  \code{.c}
  #include <flatearth/tracking.h>
  \endcode

  and link in the makefile against the cluttermap library:
  \code{.c}
  -ltracking.so
  \endcode


  In the tracking API there are several phases:
  - Init    - Resets tracking configuration (transitions to Acquire)
  - Acquire - Attempt to lock onto target (transitions to init on fail, Locked on success)
  - Locked  - Tracks a target (transitions to coast on fail, otherwise stays locked)
  - Coast   - Attempt to reacquire lost track (transition to delete on fail, stays on Coast or Locked on success)
  - Delete  - Delete target (transitions to Init)

  The tracking algorithm works using the following state-diagram:
  @verbatim
  INITIALIZE<-------+--------------+ phase 0
       |            ^              ^
  +---->+            |              |
  |     |            |              |
  |     V            |              |
  |  ACQUIRE  (here too long)       | phase 1
  |  |  |  |         |              |
  |  |  |  |         |              |
  +--+  |  +----------              |
       |                           |
  (found target)                    |
       |                           |
       +<--------------------+--+  |
       |                     ^  ^  |
       V                     |  |  |
     TRACK--->(good reading)-+  |  | phase 2
       |                        |  |
  (bad reading)                  |  |
       |                        |  |
  +--->+                        |  |
  |    |                        |  |
  |    V                        |  |
  |  COAST--->(found target)----+  | phase 3
  |  |   |                         |
  |  |   |                         |
  +--+   |                         |
         |                         |
  (here too long)                  |
         |                         |
         V                         |
    LOST/DELETE--------------------+ phase 4
  @endverbatim

  Example usage:
  @code{.c}
  TrackingConfig tracker;
  double estGndIdx;
  int currentPhase;
  ...

  tracker = track_initialize(5, 20, 40);

  // Set the default deltaT to ~ 20 ms
  track_setDeltaT(tracker, 0.02);

  ...

  // Update the tracker (where idx is index into signal)
  track_updateFixedDeltaT(tracker, idx);

  // Get the estimated distance and phase from the tracker
  track_getEstDistance(tracker, &estGndIdx);
  track_getCurrentPhase(tracker, &currentPhase);
  ...
  @endcode

  @ingroup salsaLib
*/






#ifdef __cplusplus
}
#endif
#endif

