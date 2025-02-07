/**
   @file kalman.h

   This is a naive Kalman filter which tracks a 1D distance to target

   For reference:
   http://greg.czerniak.info/guides/kalman1/

   Copyright 2014 by Flat Earth, Inc

   @par Environment
   Code Composer Studio 4.2

   @par Compiler
   Code Composer Studio 4.2

   @author Justin Hadella
*/
#ifndef KALMAN1D_h
#define KALMAN1D_h

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




/**
   @struct SKalman1D
  @ingroup kalman
*/
typedef struct SKalman1D
{
  double af_A[4];            ///< State transition matrix

  double af_R[4];            ///< Estimated measurement error covariance
  double af_Q[4];            ///< Estimated process error covariance

  double af_P[4];            ///< Newest estimate of average error for each part of the state
  double af_P_predicted[4];  ///< Covariance prediction

  double af_S[4];            ///< Innovation covariance
  double af_K[4];            ///< Kalman gain

  double af_x[2];            ///< Newest estimate of the current "true" state
  double af_x_predicted[2];  ///< State prediction

  double af_y[2];            ///< Innovation

} SKalman1D;

/**
   Handle to SKalman1D data structure
*/
typedef struct SKalman1D *HKalman1D;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#endif


/**
   Initializes the Kalman for use

   The "guts" of the Kalman data structure are initialized to known values in
   preparation for use.

   @note
   After init, the Q and R matrix are both initialized to 2 x 2 identity matrix,
   so after init, make sure to set Q and R.

   @param [in,out] k  The Kalman filter to initialize
  @ingroup kalman
*/
DLL_PUBLIC void kalman1d_vInit(HKalman1D k);

/**
   Initializes the R matrix

   @param [in,out] k     The Kalman filter instance to affect
   @param [in]     fR11  The desrired row 1, col 1 parameter
   @param [in]     fR12  The desrired row 1, col 2 parameter
   @param [in]     fR21  The desrired row 2, col 1 parameter
   @param [in]     fR22  The desrired row 2, col 2 parameter
  @ingroup kalman
*/
DLL_PUBLIC void kalman1d_vSetR(HKalman1D k, double fR11, double fR12, double fR21, double fR22);

/**
   Initializes the Q matrix

   @param [in,out] k     The Kalman filter instance to affect
   @param [in]     fQ11  The desrired row 1, col 1 parameter
   @param [in]     fQ12  The desrired row 1, col 2 parameter
   @param [in]     fQ21  The desrired row 2, col 1 parameter
   @param [in]     fQ22  The desrired row 2, col 2 parameter
  @ingroup kalman
*/
DLL_PUBLIC void kalman1d_vSetQ(HKalman1D k, double fQ11, double fQ12, double fQ21, double fQ22);

DLL_PUBLIC int test_gemm();

/**
   Update the Kalman with the latest data point

   @param [in,out] k          The Kalman filter instance to affect
   @param [in]     fDistance  The distance to the target
   @param [in]     fDeltaT    The time delta since the last update
  @ingroup kalman
*/
DLL_PUBLIC void kalman1d_vUpdate(HKalman1D k, double fDistance, double fDeltaT);

/**
   Get the latest distance estimate of the given Kalman filter

   @param [in,out] k  The Kalman filter instance to affect

   @return  The current distance estimate (filtered)
  @ingroup kalman
*/
DLL_PUBLIC double kalman1d_fGetDistanceEstimate(HKalman1D k);

DLL_PUBLIC void kalman1d_vUpdate_orig(HKalman1D k, double fDistance, double fDeltaT);

/**
   Get the latest velocity estimate of the given Kalman filter

   @param [in,out] k  The Kalman filter instance to affect

   @return  The current velocity estimate (filtered)
  @ingroup kalman
*/
DLL_PUBLIC double kalman1d_fGetVelocityEstimate(HKalman1D k);


/**
  @defgroup kalman Kalman Filtering
  Functions implimenting a Kalman Filter

  @ingroup salsaLib
*/


#ifdef __cplusplus
}
#endif
#endif

