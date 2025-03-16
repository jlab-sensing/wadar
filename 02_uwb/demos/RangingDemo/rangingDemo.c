/**
  @anchor bbb_chipotle_ranging_demo

  # RangingDemo #
  This is a simple program which demonstrates simple ranging; this program is
  based on the _ClutterDemo_ project.

  ## Hardware Setup ##
  For this demo, a BeagleBone Black + a Chipotle cape + 5" antennas and AVA 0.8
  were used.

  ## Demonstration Code ##
  This program builds on code from the ClutterDemo project. In addition to the
  prior functionality, the program demonstrates the use of some DSP techniques.

  The radarHelper_getFrameRaw() function is used to get a raw radar frame. This
  "raw" frame consists of uint32_t DAC counts. The clutter map api also
  works on integer type values. However, typically floating-point signals are
  used when working with DSP functions.

  Once the clutter has been removed, the resultant signal is still consists of
  integer values. In the next step, this integer array is converted into a
  floating-point array. The signal mean is removed and the envelope is determined.
  This envelope signal is the one shown via Gnuplot. The maximum peak of this
  envelope is used to determine the distance to the target.

  ### Command-line Options ###
  There are several command-line options:
  + -c : execute the radar timing measurement
  + -o : save the JSON configuration to a file
  + -a : enable the adaptive clutter map

  ### Demo in Action ###

  The radar was placed on a table with the antennas facing sideways towards empty
  space. By default, the initial radar signal was used to set the static clutter
  map. Once the demo started running, a person moved towards and away from the
  radar.

  Console output:
  @verbatim
  swing= 410018, max * 1e3=   36.734, d= 1.31
  swing= 410685, max * 1e3=   36.969, d= 1.31
  swing= 410174, max * 1e3=   37.327, d= 1.31
  swing= 410124, max * 1e3=   37.250, d= 1.31
  swing= 409174, max * 1e3=   36.888, d= 1.31
  swing= 409353, max * 1e3=   36.792, d= 1.31
  swing= 409846, max * 1e3=   36.602, d= 1.31
  swing= 408058, max * 1e3=   35.450, d= 1.31
  swing= 410523, max * 1e3=   35.531, d= 1.31
  swing= 404404, max * 1e3=   35.104, d= 1.31
  swing= 409530, max * 1e3=   35.265, d= 1.31
  swing= 410228, max * 1e3=   34.826, d= 1.31
  swing= 410152, max * 1e3=   34.848, d= 1.31
  @endverbatim

  The envelope signal is plotted below. The maximum peak was used to determine the
  reported distance.

  @image html chipotle_demo_ranging.png "Signal Envelope"
  @image latex chipotle_demo_ranging.png "Signal Envelope" width=10cm

  By default, the static cluttermap is used. If the demo is started using the "-a"
  option, then the adaptic cluttermap is used instead.

  @image html chipotle_demo_ranging_adaptive.png "Signal Envelope"
  @image latex chipotle_demo_ranging_adaptive.png "Signal Envelope" width=10cm

  ### User Enhancements ###
  A simple 10-pt moving average filter was used to smooth out the envelope. This
  was simple to implement as an example, but a better filter should improve the
  distance estimation consistency. When viewing the pruned envelope, there will
  occasionally be a bouncing between two close peaks.

  In the demo's current form, nothing is done with processed signal max value.
  So, when no target is in view, the DSP will still estimate a distance to some
  peak. A simple modification would be to examine the max value against a user
  threshold to determine whether the distance estimate should be used or not.
  For example, experiment with and without targets in view and see how the max
  value changes.

  @par Environment
  BeagleBone Black + Chipotle cape

  @par Compiler
  GNU GCC

  @author Justin Hadella

  @copyright 2015 by FlatEarth, Inc
*/
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <unistd.h>
#include <stdbool.h>

// SalsaLib include
#include "chipotleHelper.h"
#include "radarHelper.h"
#include "cluttermap.h"
#include "radarDSP.h"

// Novelda radar API include
#include "Radarlib3.h"

// Local include
#include "../../Common/inc/utility.h"
#include "../../Common/inc/normalize.h"

// -----------------------------------------------------------------------------
// Special Flags
// -----------------------------------------------------------------------------

// These defines determine what should be output to user
#define SHOW_DEBUG_SETTINGS
#define SHOW_LIVE_SETTINGS

// Adjustments used to customized the clutter map
#define ADAPTIVE_ALPHA (0.99) /* Remove 99% of clutter */
#define ADAPTIVE_BETA  (0.30) /* Controls amount of new signal added to clutter */

// Initial value used to set the y-axis upper limit in Gnuplot
#define DEFAULT_Y_MIN  (5e-2)

// Controls how often the y-axis rescaling is checked
#define Y_AXIS_ADJUST_COUNTS (30)

// -----------------------------------------------------------------------------
// Demo Definitions
// -----------------------------------------------------------------------------

#define CAPE_NAME "Chipotle"

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

void Usage();

// -----------------------------------------------------------------------------
// Variables
// -----------------------------------------------------------------------------

// Connection string used when connecting to the radar
static char *radarConnectionStr = "BeagleBone!SPI device: 0!FE_Salsa!NVA6100";

// Number of samplers we want
static int numberOfSamplers;

// Pointers to arrays for signal storage
static uint32_t *radarCounters;
static int32_t   *radarClutterRemoved;
static double    *originalSignal;
static double    *processedSignal;

// Values computed during signal processing
static double   sampleResolution;
static double   offsetDistance;
static double   max = 0.0;
static int      maxIdx;
static double   distance;
static uint32_t maxCounters;
static uint32_t minCounters;
static int      dummy; // just a dummy variable (to satisfy function call...)

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------

void
Usage()
{
  printf("Usage: RangingDemo [-option(s)]\n");
  printf("where options include:\n");
  printf(" -%c %-18s - %-40s\n", 'c', "", "Calibrate timing circuits");
  printf(" -%c %-18s - %-40s\n", 'a', "", "Enable adaptive cluttermap");
  printf(" -%c %-18s - %-40s\n", 'o', "[output file]", "Specify configuration output file");
  printf("\n");
  printf("If the -o option is used, the output file will be a JSON file,\n");
  printf("but the .json extension will not be automatically added.\n\n");
}

// =============================================================================
// Main Program
// =============================================================================

static const double C = 299792458.0; // Speed of light in meters/second

int main(int argc, char **argv)
{
  // Radar handle used as a reference to a structure with information about the
  // connected radar
  RadarHandle_t rh = NULL;

  // Return status from radar helper calls
  int status;

  // Loop counters
  int i;

  // Used to determine when to quit (based on user keypress)
  int quit = 0;

  // Variables used to adjust Gnuplot scaling
  int signalScaleAdjustCount = 0;

  // Getopt Flags
  bool calibrate = false;
  bool saveOutputFile = false;
  bool useAdaptiveClutterMap = false;

  // Other flags
  bool isFirstFrame = true;

  // Used by the command-line parser
  int c;
  opterr = 0;

  // Data structure used to store the static cluttermap configuration
  staticClutterMapConfig staticClutter;

  // Data structure used to store the adaptive cluttermap configuration
  adaptiveClutterMapConfig adaptiveClutter;

  // Pointers to input/output files
  const char *inFile_stage1 = "stage1.json";
  const char *inFile_stage2 = "stage2.json";
  const char *outFile;

  //
  // Initiate a radar handle
  //
  status = radarHelper_open(&rh, radarConnectionStr);
  if (status) return 1;

  //
  // Process command-line arguments
  //
  while ((c = getopt(argc, argv, "aco:")) != -1) {
    switch (c) {
    case 'a':
      useAdaptiveClutterMap = true;
      break;

    case 'c':
      calibrate = true;
      break;

    case 'o':
      saveOutputFile = true;
      outFile = optarg;
      break;

    default:
      Usage();
      abort();
    }
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  //
  // Configure the radar using the Stage 1 configuration JSON file
  //
  status = radarHelper_configFromFile(rh, inFile_stage1, 1);
  if (status) return 1;

  // Set the low frequency pulse generator
  setIntValueByName(rh, "PulseGen", 1);

  //
  // Do radar timing measurements
  //
  if (calibrate) {
    fprintf(stderr, "Running radar timing calibration...\n");

    status = radarHelper_doAction(rh, "MeasureAll");
    if (status) return 1;

    // Note: Saving calibration results to FLASH is not supported yet...
    // Note: Restoring calibration results from FLASH is not supported yet...
  }

  //
  // Configure the radar using the Stage 2 configuration JSON file
  //
  status = radarHelper_configFromFile(rh, inFile_stage2, 2);
  if (status) return 1;

  //
  // Save radar parameters to file (optional)
  //
  if (saveOutputFile) {
    status = radarHelper_saveConfigToFile(rh, outFile);
    if (status) return 1;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Turn ON the Red LED
  chipotleHelper_setLED(LED_Red, 1);

  // Turn OFF the other LEDs
  chipotleHelper_setLED(LED_Blue,   0);
  chipotleHelper_setLED(LED_Green0, 0);
  chipotleHelper_setLED(LED_Green1, 0);

  // Determine the number of samplers in the radar frame
  numberOfSamplers = getIntValueByName(rh, "SamplersPerFrame");

  //
  // Allocate memory for signal storage
  //
  radarCounters = (uint32_t *)malloc(numberOfSamplers * sizeof (uint32_t));
  radarClutterRemoved = (int32_t *)malloc(numberOfSamplers * sizeof (int32_t));
  originalSignal = (double *)malloc(numberOfSamplers * sizeof (double));
  processedSignal = (double *)malloc(numberOfSamplers * sizeof (double));

  // On our first N frames, accumulate the radar counters
  static int frameCount = 0;
  static uint32_t *accumulatedCounters = NULL;
  static const int clutterFrames = 100; // Number of frames to accumulate for clutter map

  // Compute the resolution
  sampleResolution = (0.5 * C) / getFloatValueByName(rh, "SamplesPerSecond");

  // Compute the offset distance
  offsetDistance = getFloatValueByName(rh, "OffsetDistanceFromReference");



#ifdef SHOW_DEBUG_SETTINGS
  //
  // Echo User Settings (useful for testing)
  //
  printf("OffsetDistanceFromReference = %f m\n", offsetDistance);
  printf("Sample Delay = %f ns\n", getFloatValueByName(rh, "SampleDelay") / 1e-9);
  printf("SamplesPerSecond = %.0f\n", getFloatValueByName(rh, "SamplesPerSecond"));
  printf("Sample resolution = %.4f m\n", sampleResolution);
  printf("NumSamplers = %d\n", numberOfSamplers);
#endif



  //
  // Initialize clutter map
  //
  clutter_intializeSCluttermap(numberOfSamplers, &staticClutter);
  clutter_initializeACluttermap(numberOfSamplers, ADAPTIVE_BETA, ADAPTIVE_ALPHA, &adaptiveClutter);



  // Open a pipe to gnuplot
  FILE *gnuplotPipe = popen("gnuplot -persistant", "w");

  // Note: Connect with ssh -X to allow for GUI plots
  fprintf(gnuplotPipe, "set term x11 noraise \n");
  fprintf(gnuplotPipe, "set title \"%s - Ranging Demo\" \n", CAPE_NAME);
  fprintf(gnuplotPipe, "set xlabel \"Distance (m)\" \n");
  fprintf(gnuplotPipe, "set xrange [%f:%f] \n", offsetDistance, offsetDistance + (numberOfSamplers - 1) * sampleResolution);
  fprintf(gnuplotPipe, "set yrange [0:%f] \n", DEFAULT_Y_MIN);



  // Turn ON the Red LED
  chipotleHelper_setLED(LED_Red, 1);



  //
  // Operate the radar until uses quits
  //

  fprintf(stderr, "Starting radar loop... (hit ESC key to quit)\n");

  // This loops forever until user presses the ESC key to quit
  nonblock(NB_ENABLE);
  while (!quit) {
    // Get user input
    usleep(1);
    quit = kbhit();
    if (quit != 0) {
      c = fgetc(stdin);
      if (0x1B == c) {  /* ESC key is 0x1B */
        quit = 1;
      }
    }



    // Turn ON the Blue LED while getting a frame
    chipotleHelper_setLED(LED_Blue, 1);

    // Get a radar frame
    status = radarHelper_getFrameRaw(rh, radarCounters, numberOfSamplers);
    if (status) return 1;

    // Turn OFF the Blue LED while getting a frame
    chipotleHelper_setLED(LED_Blue, 0);



    // Turn ON the Green0 LED while doing DSP work
    chipotleHelper_setLED(LED_Green0, 1);

    if (frameCount == 0) {
      accumulatedCounters = (uint32_t *)calloc(numberOfSamplers, sizeof(uint32_t));
    }

    for (i = 0; i < numberOfSamplers; i++) {
      accumulatedCounters[i] += radarCounters[i];
    }

    frameCount++;

    if (frameCount < clutterFrames) {
      // Skip the rest of this iteration
      // continue;
    } else if (frameCount == clutterFrames) {
      // Compute the average and set the static cluttermap
      for (i = 0; i < numberOfSamplers; i++) {
      accumulatedCounters[i] /= clutterFrames;
      }
      clutter_setSCluttermap(accumulatedCounters, &staticClutter);
      free(accumulatedCounters);
      isFirstFrame = false;
      printf("Static clutter map is ready.\n");
    }

    // Determine signal swing (useful for testing)
    radarDSP_Min(radarCounters, numberOfSamplers, &minCounters, &dummy);
    radarDSP_Max(radarCounters, numberOfSamplers, &maxCounters, &dummy);

    //
    // Update the clutter map and remove clutter
    //
    if (useAdaptiveClutterMap) {
      clutter_updateACluttermap(radarCounters, &adaptiveClutter);
      clutter_removeAClutter(radarCounters, radarClutterRemoved, &adaptiveClutter);
    }
    else {
      clutter_removeSClutter(radarCounters, radarClutterRemoved, &staticClutter);
    }



    //
    // Normalize signal to volts before processing
    //

    // Convert counter signal into floating-point signal
    radarDSP_convertIntArray(radarClutterRemoved, originalSignal, numberOfSamplers);

    // Scale radar signal into voltage (DC removed)
    normalizeVolts(rh, originalSignal, originalSignal, numberOfSamplers, 1);



    // Process signal to produce envelope
    radarDSP_Envelope(originalSignal, processedSignal, numberOfSamplers);

    // 10-point moving average filter coefficients
    double b[] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    radarDSP_dp_FIR_Filter(b, 10, processedSignal, numberOfSamplers, processedSignal);

    // Prune the envelope by 3 dB
    //radarDSP_dp_Prune(processedSignal, processedSignal, numberOfSamplers, 3.0);



    //
    // Distance Estimation
    //

    // Find max of envelope
    radarDSP_dp_Max(processedSignal, numberOfSamplers, &max, &maxIdx);

    // Compute and display distance to max
    distance = offsetDistance + sampleResolution * maxIdx;

    // Turn OFF the Green0 LED when DSP is done
    chipotleHelper_setLED(LED_Green0, 0);



    //
    // Adjust y-axis scaling
    //

    signalScaleAdjustCount++;

    if (signalScaleAdjustCount > Y_AXIS_ADJUST_COUNTS) {
      // Reset counter
      signalScaleAdjustCount = 0;

      // Set the y-axis max value to 125% of the max value (or the min!)
      fprintf(gnuplotPipe, "set yrange [0:%f] \n", max > DEFAULT_Y_MIN ? 1.25 * max : DEFAULT_Y_MIN);
    }



#ifdef SHOW_LIVE_SETTINGS
    //
    // Echo dynamic settings
    //
    if (isFirstFrame && !useAdaptiveClutterMap) {
      continue;
    }
    else {
      fprintf(stderr, "swing= %6u, max * 1e3= %8.3lf, d= %.2f\n", maxCounters - minCounters, max * 1e3, distance);
    }
#endif // SHOW_LIVE_SETTINGS



    fprintf(gnuplotPipe, "plot '-' using 1:2 with lines \n");
    for (i = 0; i < numberOfSamplers - 1; i++) {
      fprintf(gnuplotPipe, "%lf %lf\n", (double)(offsetDistance + i * sampleResolution), processedSignal[i]);
    }
    fprintf(gnuplotPipe, "e\n");
    fflush(gnuplotPipe);
  }
  nonblock(NB_DISABLE);

  //
  // All done, clean up time...
  //
  pclose(gnuplotPipe);

  // Close radar connection
  status = radarHelper_close(&rh);
  if (status) return 1;

  // Free memory allocated to clutter maps
  clutter_staticFree(&staticClutter);
  clutter_adaptiveFree(&adaptiveClutter);

  // Free memory allocated for radar signals
  free (radarCounters);
  free (radarClutterRemoved);
  free (originalSignal);
  free (processedSignal);

  return 0;
}