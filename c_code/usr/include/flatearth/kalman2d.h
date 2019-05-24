/**

   @file kalman2d.h

   This is a naive Kalman filter which tracks a 2D distance to target (Untested)

   For reference:
   http://greg.czerniak.info/guides/kalman1/

   Copyright 2014 by Flat Earth, Inc

   @par Environment
   Code Composer Studio 4.2

   @par Compiler
   Code Composer Studio 4.2

   @author Justin Hadella
*/
#ifndef KALMAN2D_h
#define KALMAN2D_h

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------


#if defined _WIN32 || defined __CYGWIN__
#ifdef BUILDING_DLL
#ifdef __GNUC__
#define DLL_PUBLIC __attribute__ ((dllexport))
#else
#define DLL_PUBLIC __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
#endif
#else
#ifdef __GNUC__
#define DLL_PUBLIC __attribute__ ((dllimport))
#else
#define DLL_PUBLIC __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
#endif
#endif
#define DLL_LOCAL
#else
#if __GNUC__ >= 4
#define DLL_PUBLIC __attribute__ ((visibility ("default")))
#define DLL_LOCAL __attribute__ ((visibility ("hidden")))
#else
#define DLL_PUBLIC
#define DLL_LOCAL
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif



/**
   @struct SKalman2D
*/
typedef struct SKalman2D
{
  float af_A[16];            ///< State transition matrix

  float af_R[16];            ///< Estimated measurement error covariance
  float af_Q[16];            ///< Estimated process error covariance

  float af_P[16];            ///< Newest estimate of average error for each part of the state
  float af_P_predicted[16];  ///< Covariance prediction

  float af_S[16];            ///< Innovation covariance
  float af_K[16];            ///< Kalman gain

  float af_x[4];            ///< Newest estimate of the current "true" state
  float af_x_predicted[4];  ///< State prediction

  float af_y[4];            ///< Innovation

} SKalman2D;

/**
   Handle to SKalman2D data structure
*/
typedef struct SKalman2D *HKalman2D;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

/**
   Initializes the Kalman for use

   The "guts" of the Kalman data structure are initialized to known values in
   preparation for use.

   @note
   After init, the Q and R matrix are both initialized to 2 x 2 identity matrix,
   so after init, make sure to set Q and R.

   @param [in,out] k  The Kalman filter to initialize
*/
DLL_PUBLIC void kalman2d_vInit(HKalman2D k);

/**
   Initializes the R matrix

   @param [in,out] k     The Kalman filter instance to affect
   @param [in]     fR    The desrired R matrix
*/
DLL_PUBLIC void kalman2d_vSetR(HKalman2D k, float *fR);

/**
   Initializes the Q matrix

   @param [in,out] k     The Kalman filter instance to affect
   @param [in]     fQ    The desired Q matrix
*/
DLL_PUBLIC void kalman2d_vSetQ(HKalman2D k, float *fQ);

/**
   Update the Kalman with the latest data point

   @param [in,out] k          The Kalman filter instance to affect
   @param [in]     fDistance  The distance to the target
   @param [in]     fDeltaT    The time delta since the last update
*/
DLL_PUBLIC void kalman2d_vUpdate(HKalman2D k, float fDistance, float fDeltaT);

/**
   Get the latest distance estimate of the given Kalman filter

   @param [in,out] k  The Kalman filter instance to affect

   @return  The current distance estimate (filtered)
*/
DLL_PUBLIC float kalman2d_fGetDistanceEstimate(HKalman2D k);

/**
   Get the latest velocity estimate of the given Kalman filter

   @param [in,out] k  The Kalman filter instance to affect

   @return  The current velocity estimate (filtered)
*/
DLL_PUBLIC float kalman2d_fGetVelocityEstimate(HKalman2D k);

#ifdef __cplusplus
}
#endif


#endif // KALMAN2D_h

