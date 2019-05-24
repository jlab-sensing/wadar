/**
  @anchor bbb_ancho_frameviewer_demo

  # FrameViewer #
  FrameViewer is a simple program used to view (or post-process) binary data log
  files produced by the complementary program FrameLogger.

  ## Hardware Setup ##
  Using a BeagleBone Black + an Ancho cape + (user cables and antennas)

  ## Demonstration Code ##
  The data log is organized as:
  @verbatim
  [Magic#]
  [Iterations]
  [PulsesPerStep]
  [DACMin]
  [DACMax]
  [DACStep]
  [SamplesPerSecond]
  [PGSelect]
  [OffsetDistanceFromReference]
  [SampleDelayToReference]
  [#samples]
  [#frames]
  [temperature_0][frame_0]
  [temperature_1][frame_1]
  [temperature_2][frame_2]
  ...
  [temperature_N-1][frame_N-1]

  where...

  [Magic#]           - Code indicating data format
  [Iterations]       - Radar sweep controller setting which effect normalization
  [PulsesPerStep]    - ""
  [DACMin]           - ""
  [DACMax]           - ""
  [DACStep]          - ""
  [SamplesPerSecond] - Measured sampling rate
  [PGSelect]         - NVA6201 setting effects radar center frequency
  [OffsetDistanceFromReference] - The frame's OffsetDistanceFromReference value in meters (type is float)
  [SampleDelayToReference] - Internal bias (used for distance estimation)
  [#samples]         - Number of samples in a radar frame (type is int)
  [#frames]          - Total number of frames (type is int)
  [temperature_n]    - Temperature chip reading (degrees Celcius)
  [frame_n]          - Array of radar counter values, where the length is #samples (of type uint32_t)
  @endverbatim

  When experimenting, the user should make sure this and the corresponding
  frame logging program use the _same_ stage1.json and stage2.json files.

  @verbatim
  Usage: FrameViewer [-option(s)]
  where options include:
  -g                    - Enable Gnuplot of radar frames
  -l [log file]         - Specify dataLog filename
  @endverbatim

  Example user session:
  @verbatim
  # ./FrameViewer -l testLog1
  Starting radar loop: (#trials = 400)
  ........................................ 10.00%
  ........................................ 20.00%
  ........................................ 30.00%
  ........................................ 40.00%
  ........................................ 50.00%
  ........................................ 60.00%
  ........................................ 70.00%
  ........................................ 80.00%
  ........................................ 90.00%
  ........................................100.00%
  @endverbatim

  The example session reads in binary data log file, testLog1. Each frame is
  read from the file and then can be post-processed. There is an indicated
  area where the user could add any custom processing.

  This program is also intended to be used as a basis for _user_ customization.
  Often, collecting data is performed in the field; it can be difficult to
  produce realistic data in an office environment. Using the data log concept
  for field data collection and later post-processing playback can save time.

  In this demonstration, the only _user_ customization is that the signal output
  is normalized into the DAC range. The actual data logged is the radar counter
  values, but the signal is normalized internally so the Gnuplot display would
  display well (independent of settings changes).

  @par Environment
  BeagleBone Black + Ancho cape

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
#include <stdint.h>

// SalsaLib include
#include "anchoHelper.h"
#include "radarHelper.h"
#include "radarDSP.h"

// Novelda radar API include
#include "Radarlib3.h"

// -----------------------------------------------------------------------------
// Special Flags
// -----------------------------------------------------------------------------

// These defines determine what should be output to user
#define OUTPUT_DEBUG
//#define SHOW_FULL_Y_AXIS_RANGE

// -----------------------------------------------------------------------------
// Demo Definitions
// -----------------------------------------------------------------------------

// Magic# to aid in data parsing
#define FRAME_LOGGER_MAGIC_NUM (0xFEFE00A2)

// Once the radar counters are normalized, the y-values will range from 0-8191
// These define the range of the y-axis in Gnuplot.

#define GNUPLOT_SCALED_Y_MIN (0)
#define GNUPLOT_SCALED_Y_MAX (8191)

// Note: The practical DAC range is likely more limited. Use these definitions
// to tune the the range if desired. These values seem to work well using the
// Bowtie antennas with the Cayenne cape.

#define GNUPLOT_PRACTICAL_Y_MIN (1500)
#define GNUPLOT_PRACTICAL_Y_MAX (7000)

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

void Usage();

// -----------------------------------------------------------------------------
// Variables
// -----------------------------------------------------------------------------

// Number of samplers we want
static int numberOfSamplers;

// Pointers to arrays for signal storage
static uint32_t *radarCounters;
static double *radarScaled;
// Additional signals?
// Additional signals?

// Values computed during signal processing
static double offsetDistance;
// Additional values?
// Additional values?

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------

void Usage()
{
  printf("Usage: RadarLogger [-option(s)]\n");
  printf("where options include:\n");
  printf(" -%c %-18s - %-40s\n", 'g', "", "Enable Gnuplot of radar frames");
  printf(" -%c %-18s - %-40s\n", 'l', "[log file]", "Specify dataLog filename");
}

// =============================================================================
// Main Program
// =============================================================================

int main(int argc, char **argv)
{
  // Loop counters
  int i;
  int numTrials = 0;
  int trials;
  int trialNum;

  // Getopt Flags
  bool showGnuPlot = false;

  // Used by the command-line parser
  int c;
  opterr = 0;

  // Radar settings to LOG to the file
  uint32_t magic;
  int pgSelect;
  int iterations;
  int pps;
  int dacMin;
  int dacMax;
  int dacStep;
  float samplesPerSecond;
  float offsetDistance;
  float sampleDelayToReference;
  float temperature;

  // Pointers to input/output files
  const char *dataLogFile = NULL;

  // Pointer to datalog file
  FILE *dataLog;


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  //
  // Process command-line arguments
  //
  while ((c = getopt(argc, argv, "gl:")) != -1) {
    switch (c) {
    /* Enable Gnuplot of radar data */
    case 'g':
      showGnuPlot = true;
      break;

    /* Save DataLog to binary file */
    case 'l':
      dataLogFile = optarg;
      break;

    default:
      Usage();
      exit(0);
    }
  }

  //
  // Bail if there is no dataLog
  //
  if (dataLogFile == NULL) {
    printf("No dataLog specified!\n");
    exit(0);
  }


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  // Turn ON the Red LED
  anchoHelper_setLED(LED_Red, 1);

  // Turn OFF the other LEDs
  anchoHelper_setLED(LED_Blue,   0);
  anchoHelper_setLED(LED_Green0, 0);
  anchoHelper_setLED(LED_Green1, 0);

  //
  // Setup Gnuplot (if necessary)
  //
  FILE *gnuplotPipe;
  if (showGnuPlot) {
    // Open a pipe to gnuplot
    gnuplotPipe = popen("gnuplot -persistant", "w");
  }

  //
  // Setup datalog
  //
  dataLog = fopen(dataLogFile, "rb");
  if (!dataLog) {
    fprintf(stderr, "Unable to open %s!\n", dataLogFile);
    return 1;
  }


  // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


  // Turn ON the Red LED
  anchoHelper_setLED(LED_Red, 1);

  // The LOG begins with the magic#
  fread(&magic, sizeof (uint32_t), 1, dataLog);
  if (magic != FRAME_LOGGER_MAGIC_NUM) {
    fprintf(stderr, "Wrong data format: %s!\n", dataLogFile);
    fclose(dataLog);
    return 1;
  }

  // Next are the sweep controller settings
  fread(&iterations, sizeof (int), 1, dataLog);
  fread(&pps, sizeof (int), 1, dataLog);
  fread(&dacMin, sizeof (int), 1, dataLog);
  fread(&dacMax, sizeof (int), 1, dataLog);
  fread(&dacStep, sizeof (int), 1, dataLog);

  // The measured sampling rate is next
  fread(&samplesPerSecond, sizeof (float), 1, dataLog);

  // The NVA6201 specific settings are next
  fread(&pgSelect, sizeof (int), 1, dataLog);

  // The frame offset, reference delay
  fread(&offsetDistance, sizeof (float), 1, dataLog);
  fread(&sampleDelayToReference, sizeof (float), 1, dataLog);

  // Next is the #samplers in a frame
  fread(&numberOfSamplers, sizeof (int), 1, dataLog);

  // Next, determine the #frame in the dataLog
  fread(&numTrials, sizeof (int), 1, dataLog);



  if (showGnuPlot) {
    // Set up Gnuplot
    fprintf(gnuplotPipe, "set term x11 noraise\n");
    fprintf(gnuplotPipe, "set title \"FrameViewer Demo\" \n");
    fprintf(gnuplotPipe, "set xlabel \"sample#\" \n");
    fprintf(gnuplotPipe, "set xrange [0:%d] \n", numberOfSamplers - 1);
#ifdef SHOW_FULL_Y_AXIS_RANGE
    fprintf(gnuplotPipe, "set yrange [%d:%d] \n", GNUPLOT_SCALED_Y_MIN, GNUPLOT_SCALED_Y_MAX);
#else
    fprintf(gnuplotPipe, "set yrange [%d:%d] \n", GNUPLOT_PRACTICAL_Y_MIN, GNUPLOT_PRACTICAL_Y_MAX);
#endif
    fprintf(gnuplotPipe, "set ylabel \"Normalized DAC\" \n");
  }



  //
  // Allocate memory for signal storage
  //
  radarCounters = (uint32_t *)malloc(numberOfSamplers * sizeof (uint32_t));
  radarScaled = (double *)malloc(numberOfSamplers * sizeof (double));



  // Give user some feedback
  fprintf(stderr, "Starting radar loop: (#trials = %d)\n", numTrials);

  //
  // Run radar loop N times
  //

  trials = numTrials;
  trialNum = 0;

  while (trials > 0) {
    // Return value when using fread()
    int result;

    // Turn ON the Blue LED while getting a frame
    anchoHelper_setLED(LED_Blue, 1);

    // Get the offsetDistance
    result = fread(&temperature, sizeof (float), 1, dataLog);
    if (result != 1) {
      fprintf(stderr, "File read error: offsetDistance!");
      exit(1);
    }

    // Get a radar frame
    result = fread(radarCounters, sizeof (uint32_t), numberOfSamplers, dataLog);
    if (result != numberOfSamplers) {
      fprintf(stderr, "File read error: data frame!");
      exit(1);
    }

    // Turn OFF the Blue LED while getting a frame
    anchoHelper_setLED(LED_Blue, 0);



    // Turn ON the Green1 LED when doing any custom processing
    anchoHelper_setLED(LED_Green1, 1);

// ---> Add any customization here...
// ---> Add any customization here...

    // Do the DAC normalization
    for (i = 0; i < numberOfSamplers; i++) {
      radarScaled[i] = (double)radarCounters[i] / (1.0 * pps * iterations) * dacStep + dacMin;
    }

// ---> Add any customization here...
// ---> Add any customization here...

    // Turn OFF the Green1 LED when done with custom processing
    anchoHelper_setLED(LED_Green1, 0);



    //
    // Give some status feedback
    //
    printf(".");
    if ((++trialNum % 40) == 0) {
      printf("%6.2f%%\n", 100.0 * trialNum / numTrials);
    }
    fflush(stdout);

    //
    // Plot the radar frame (if enabled)
    //
    if (showGnuPlot) {
      fprintf(gnuplotPipe, "plot '-' using 1:2 with lines \n");
      for (i = 0; i < numberOfSamplers - 1; i++) {
        fprintf(gnuplotPipe, "%lf %lf\n", (double)i, radarScaled[i]);
      }
      fprintf(gnuplotPipe, "e\n");
      fflush(gnuplotPipe);
    }

    // Decrement loop counter
    trials--;
  }
  printf("\n");

  //
  // All done, clean up time...
  //

  // Close the Gnuplot pipe if necessary
  if (showGnuPlot) {
    pclose(gnuplotPipe);
  }

  // Close the dataLog
  fclose(dataLog);

  // Free memory allocated for radar signals
  free (radarCounters);
  free (radarScaled);

  return 0;
}
