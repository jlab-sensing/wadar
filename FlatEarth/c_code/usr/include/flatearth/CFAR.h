/**
   @file CFAR.h

   CFAR (Constant False Alarm Rate) is a set of procedures that can be used to
   build a threshold on a sampler-by-sampler basis. This is done by looking at
   cells that are somewhat nearby to the cell under test (CUT) and determining
   either the mean of those cells or the order statistic (nth largest) of the
   cells.

   CFAR CA (Cell Averaging) is the simplest and conventional method. In this
   method you move n guard cells away from the cell under test in both
   directions, and average the m filter cells beyond.  The guard cells should
   be sufficient to get you away from any possible pulse without being so large
   as to be in a part of a radar frame with drastically different radar signal
   properties.

   CFAR OS (Order Statistic) was developed in response to issues with CFAR CA
   being slow to respond when the radar signal changes dramaticly in a short
   window when a return is not present.  Instead of computing the mean of the
   filter cells, this method sorts the filter cells and selects the n'th value
   (the order statistic) as the threshold for the CUT.

   @ingroup detection_cfar
   Copyright 2015 by FlatEarth, Inc

   @par Environment
   Environment Independent

   @par Compiler
   Compiler Independent

   @author Raymond Weber
*/

#ifndef DETECTION_CFAR_h
#define DETECTION_CFAR_h

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif



#define MAXLENGTH_CFAR 4096      /**< Maximum length of the radar frame that can be processed */



/** Structure to hold the parameters and result of a CFAR detector
  @ingroup detection_cfar
*/
typedef struct CFARConfig
{
  int CFARType;          /**< What type of CFAR is this, CA or SO (or Doppler CA/SO) */
  int guardCells;          /**< Number of cells to move away from the cell under test before looking for a threhsold */
  int filterCells;        /**< Number of cells to look at on each side of the cell under test */
  int orderStatistic;        /**< Order to use for CFAR SO */
  double falseAlarmRate;      /**< False alarm rate for CFAR CA */
  double alpha;          /**< ?? */
  int length;            /**< Number of samplers in the radar frame */
  int threshold[MAXLENGTH_CFAR];  /**< Stored threshold from the previous run */
} CFARConfig;



/** Intialize a Cell Averaged CFAR Filter
  @param guardCells_i Number of cells to skip (to get outside the current pulse) before computing the threshold
  @param filterCells_i Number of cells outside the guard range to use
  @param falseAlarmRate The percentage of false alarms expected
  @param length Number of samplers in the radar frame
  @param config CFARConfig structure to hold the setup and current state of the cfar algorithm
  @ingroup detection_cfar
*/
int detection_CFAR_CA_Initialize(int guardCells_i, int filterCells_i, double falseAlarmRate, int length, CFARConfig *config);

/** Intialize a Cell Averaged CFAR Filter
  @param guardCells_i Number of cells to skip (to get outside the current pulse) before computing the threshold
  @param filterCells_i Number of cells outside the guard range to use
  @param falseAlarmRate The percentage of false alarms expected
  @param length Number of samplers in the radar frame
  @param config CFARConfig structure to hold the setup and current state of the cfar algorithm
  @ingroup detection_cfar
*/
int detection_sp_CFAR_CA_Initialize(int guardCells_i, int filterCells_i, double falseAlarmRate, int length, CFARConfig *config);


/** Intialize a Cell Averaged CFAR Filter
  @param guardCells_i Number of cells to skip (to get outside the current pulse) before computing the threshold
  @param filterCells_i Number of cells outside the guard range to use
  @param falseAlarmRate The percentage of false alarms expected
  @param length Number of samplers in the radar frame
  @param config CFARConfig structure to hold the setup and current state of the cfar algorithm
  @ingroup detection_cfar
*/
int detection_dp_CFAR_CA_Initialize(int guardCells_i, int filterCells_i, double falseAlarmRate, int length, CFARConfig *config);


/** Run a Cell Averaged CFAR Filter on the data
  @param newSignal The radar frame to process
  @param returnSignal The CFAR threshold computed
  @param config The CFAR filter data structure
  @ingroup detection_cfar
*/
int detection_CFAR_CA_Process(int *newSignal, int *returnSignal, CFARConfig *config);

/** Run a Cell Averaged CFAR Filter on the data
  @param newSignal The radar frame to process
  @param returnSignal The CFAR threshold computed
  @param config The CFAR filter data structure
  @ingroup detection_cfar
*/
int detection_sp_CFAR_CA_Process(float *newSignal, float *returnSignal, CFARConfig *config);


/** Run a Cell Averaged CFAR Filter on the data
  @param newSignal The radar frame to process
  @param returnSignal The CFAR threshold computed
  @param config The CFAR filter data structure
  @ingroup detection_cfar
*/
int detection_dp_CFAR_CA_Process(double *newSignal, double *returnSignal, CFARConfig *config);

/** Intialize an Order Statistic CFAR Filter
  @param guardCells_i Number of cells to skip (to get outside the current pulse) before computing the threshold
  @param filterCells_i Number of cells outside the guard range to use
  @param orderStatistic_i Determines which filer cell to base thehreold off of
  @param alpha A scaler factor to multiply by to get the detection threshold
  @param length Number of samplers in the radar frame
  @param config CFARConfig structure to hold the setup and current state of the cfar algorithm
  @ingroup detection_cfar
*/
int detection_CFAR_OS_Initialize(int guardCells_i, int filterCells_i, int orderStatistic_i, int alpha, int length, CFARConfig *config);

/** Run an Order Statistic CFAR Filter on the data
  @param newSignal The radar frame to process
  @param returnSignal The CFAR threshold computed
  @param config The CFAR filter data structure
  @ingroup detection_cfar
*/
int detection_CFAR_OS_Process(int *newSignal, int *returnSignal, CFARConfig *config);

/** Intialize an Order Statistic CFAR Filter
  @param guardCells_i Number of cells to skip (to get outside the current pulse) before computing the threshold
  @param filterCells_i Number of cells outside the guard range to use
  @param orderStatistic_i Determines which filer cell to base thehreold off of
  @param alpha A scaler factor to multiply by to get the detection threshold
  @param length Number of samplers in the radar frame
  @param config CFARConfig structure to hold the setup and current state of the cfar algorithm
  @ingroup detection_cfar
*/
int detection_sp_CFAR_OS_Initialize(int guardCells_i, int filterCells_i, int orderStatistic_i, double alpha, int length, CFARConfig *config);

/** Run an Order Statistic CFAR Filter on the data
  @param newSignal The radar frame to process
  @param returnSignal The CFAR threshold computed
  @param config The CFAR filter data structure
  @ingroup detection_cfar
*/
int detection_sp_CFAR_OS_Process(float *newSignal, float *returnSignal, CFARConfig *config);

/** Intialize an Order Statistic CFAR Filter
  @param guardCells_i Number of cells to skip (to get outside the current pulse) before computing the threshold
  @param filterCells_i Number of cells outside the guard range to use
  @param orderStatistic_i Determines which filer cell to base thehreold off of
  @param alpha A scaler factor to multiply by to get the detection threshold
  @param length Number of samplers in the radar frame
  @param config CFARConfig structure to hold the setup and current state of the cfar algorithm
  @ingroup detection_cfar
*/
int detection_dp_CFAR_OS_Initialize(int guardCells_i, int filterCells_i, int orderStatistic_i, double alpha, int length, CFARConfig *config);

/** Run an Order Statistic CFAR Filter on the data
  @param newSignal The radar frame to process
  @param returnSignal The CFAR threshold computed
  @param config The CFAR filter data structure
  @ingroup detection_cfar
*/
int detection_dp_CFAR_OS_Process(double *newSignal, double *returnSignal, CFARConfig *config);

/**
  @defgroup detection_cfar CFAR Functions

  @ingroup detection
*/


#ifdef __cplusplus
}
#endif
#endif

