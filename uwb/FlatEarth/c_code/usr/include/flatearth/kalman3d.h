/**

   @file kalman3d.h

   This is a naive Kalman filter which tracks a 3D distance to target (Untested)

   For reference:
   http://greg.czerniak.info/guides/kalman1/

   Copyright 2014 by Flat Earth, Inc

   @par Environment
   Code Composer Studio 4.2

   @par Compiler
   Code Composer Studio 4.2

   @author Justin Hadella
*/
#ifndef KALMAN3D_h
#define KALMAN3D_h

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
   @struct SKalman3D
*/
typedef struct SKalman3D
{
  float af_A[36];            ///< State transition matrix

  float af_R[36];            ///< Estimated measurement error covariance
  float af_Q[36];            ///< Estimated process error covariance

  float af_P[36];            ///< Newest estimate of average error for each part of the state
  float af_P_predicted[36];  ///< Covariance prediction

  float af_S[36];            ///< Innovation covariance
  float af_K[36];            ///< Kalman gain

  float af_x[6];            ///< Newest estimate of the current "true" state
  float af_x_predicted[6];  ///< State prediction

  float af_y[6];            ///< Innovation

} SKalman3D;

/**
   Handle to SKalman3D data structure
*/
typedef struct SKalman3D *HKalman3D;

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
DLL_PUBLIC void kalman3d_vInit(HKalman3D k);

/**
   Initializes the R matrix

   @param [in,out] k     The Kalman filter instance to affect
   @param [in]     fR    The desired Q matrix to use
*/
DLL_PUBLIC void kalman3d_vSetR(HKalman3D k, float *fR);

/**
   Initializes the Q matrix

   @param [in,out] k     The Kalman filter instance to affect
   @param [in]     fQ    The desired R matrix to use
*/
DLL_PUBLIC void kalman3d_vSetQ(HKalman3D k, float *fQ);

/**
   Update the Kalman with the latest data point

   @param [in,out] k          The Kalman filter instance to affect
   @param [in]     fDistance  The distance to the target
   @param [in]     fDeltaT    The time delta since the last update
*/
DLL_PUBLIC void kalman3d_vUpdate(HKalman3D k, float *fDistance, float fDeltaT);

/**
   Get the latest distance estimate of the given Kalman filter

   @param [in,out] k  The Kalman filter instance to affect

   @return  The current distance estimate (filtered)
*/
DLL_PUBLIC float kalman3d_fGetDistanceEstimate(HKalman3D k);

/**
   Get the latest velocity estimate of the given Kalman filter

   @param [in,out] k  The Kalman filter instance to affect

   @return  The current velocity estimate (filtered)
*/
DLL_PUBLIC float kalman3d_fGetVelocityEstimate(HKalman3D k);


#ifdef __cplusplus
}
#endif

#endif // KALMAN3D_h

