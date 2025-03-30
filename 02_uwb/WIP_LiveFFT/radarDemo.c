/**
@anchor bbb_chipotle_radar_demo
  
# RadarDemo #
This is a simple demo program which demonstrates the use of the radar helper
and Radarlib3 API.

## Hardware Setup ##
For this demo, a BeagleBone Black + a Chipotle cape + 5" antennas and AVA 0.8 
were used. The orientation doesn't matter.

## Demonstration Code ##
Once a handle to the radar is opened, the timing circuits can be calibrated.
_This is optional but suggested_. Using the same settings, run the program with
and without the -c option to see the effect. There should be a difference in
the sampling rate, the sample resolution, and the signal will appear to be
shifted.

### Radar Configuration ###
The project directory contains some files with the extension ".json". These
files are used to alter the radar configuration. A few specific _X1_ variables
are highlighted below.

The 'ClkDivider' controls the radar's pulse repetition frequency (PRF). This
is set to 1, which sets the PRF to 50 MHz. This value is actually default,
but is set to explicitly indicate the desired PRF.

The 'SamplingRate' controls the spatial resolution of the radar frame bins.
The indicated setting should produce a resolution of ~ 8mm. If the timing
measurements are _not_ executed (the default program behavior) then reported
resolution will display as ~ 4mm (an incorrect value). 
 
### Command-line Options ###
There are several command-line options:
+ -c : execute the radar timing measurement
+ -o : save the JSON configuration to a file
+ -n : quit program after N trials

### Demo in Action ###

Once the radar has been configured a loop begins where the "raw" radar signal
is continually plotted via Gnuplot. In order to see the plot, make sure that
the -X option is used when making the SSH connection to the BeagleBone Black.

For example:
@verbatim
ssh -X root@192.168.7.2
@endverbatim

#### Example - Radar Counters ####

In this example, the radar was placed on a table with the antennas facing empty
space parallel to the floor. The _SamplingRate_ was set to 1, which results in 
a sample resolution of ~ 8 mm and a frame width of ~ 4 m.

Console output:
@verbatim
Running radar timing calibration...
OffsetDistanceFromReference = 0.000000 m
Sample Delay = 3.340000 ns
SamplesPerSecond = 18002020352
Sample resolution = 0.0083 m
NumSamplers = 512
Starting radar loop... (#trials = 100)
@endverbatim

In this image, a metal plate was placed ~ 1.8 m from the radar; its reflection
is approximately centered in the frame. There is a large direct-path reflection
between the two antennas.

@image html chipotle_demo_raw8mm.png "Radar Counters"
@image latex chipotle_demo_raw8mm.png "Radar Counters" width=10cm

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
#include <time.h>

// SalsaLib include
#include "chipotleHelper.h"
#include "radarHelper.h"

// Novelda radar API include
#include "Radarlib3.h"

// Local include
#include "../../Common/inc/utility.h"
#include "../../Common/inc/normalize.h"

// From WIP_LiveFFT.lib
#include "WIP_LiveFFT.h"
#include "WIP_LiveFFT_terminate.h"
#include "rt_nonfinite.h"

#define CLOCKID CLOCK_REALTIME

// -----------------------------------------------------------------------------
// Special Flags
// -----------------------------------------------------------------------------

// These defines determine what should be output to user
#define SHOW_DEBUG_SETTINGS
#define SHOW_FULL_Y_AXIS_RANGE

// -----------------------------------------------------------------------------
// Demo Definitions
// -----------------------------------------------------------------------------

#define CAPE_NAME "Chipotle"

// Once the radar counters are normalized, the y-values will range from 0-8191
// These define the range of the y-axis in Gnuplot.

#define GNUPLOT_SCALED_Y_MIN (0)
#define GNUPLOT_SCALED_Y_MAX (8191)

// Note: The practical DAC range is likely more limited. Use these definitions
// to tune the the range if desired. These values seem to work well using the
// TODO TODO antennas with the Chipotle cape.

#define GNUPLOT_PRACTICAL_Y_MIN (1000)
#define GNUPLOT_PRACTICAL_Y_MAX (7500)

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
static uint32_t * radarCounters;
static uint32_t * radarFrames;
static double *timedelta;

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------

void
Usage()
{
  printf("Usage: RadarDemo [-option(s)]\n");
  printf("where options include:\n");
  printf(" -%c %-18s - %-40s\n", 'c', "", "Calibrate timing circuits");
  printf(" -%c %-18s - %-40s\n", 'o', "[output file]", "Specify configuration output file");
  printf(" -%c %-18s - %-40s\n", 'n', "[num trials]", "Specify number of trials to run");
  printf("\n");
  printf("If the -o option is used, the output file will be a JSON file, ");
  printf("but the .json extension will not be automatically added.\n\n");
}

/* Returns the number of milliseconds elapsed */
inline
double ms_diff(struct timespec *end, struct timespec *start) {
  double ms = end->tv_nsec/1000000.0 - start->tv_nsec/1000000.0;
  ms += end->tv_sec*1000.0 - start->tv_sec*1000.0;
  return ms;
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
  int numTrials = 2000;

  // Used to determine when to quit (based on user keypress)
  int quit = 0;

  // Getopt Flags
  bool calibrate = false;
  bool saveOutputFile = false;
  bool showGnuPlot = true;
  bool quitAfterNTrials = false;

  // Used by the command-line parser
  int c;
  opterr = 0;

  // Pointers to input/output files
  const char *inFile_stage1 = "stage1.json";
  const char *inFile_stage2 = "stage2.json";
  const char *outFile;

  double b_dv[2000 * 512];  // Input to WIP_LiveFFT
  double outFFT[2000];      // Output from WIP_LiveFFT  

  int frameRate = 200;
  float fpsEst;

  //
  // Initiate a radar handle
  //  
  status = radarHelper_open(&rh, radarConnectionStr);
  if (status) return 1;

  //
  // Process command-line arguments
  //
  while ((c = getopt(argc, argv, "co:n:")) != -1) {
    switch (c) {
      case 'c':
        calibrate = true;
        break;

      case 'o':
        saveOutputFile = true;
        outFile = optarg;
        break;

      case 'n':
        numTrials = atoi(optarg);
        // Some simple user validation
        if (numTrials < 1) {
          fprintf(stderr, "Please make #trials an integer > 0\n");
          exit(0);
        }
        quitAfterNTrials = true;
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
  radarFrames = (uint32_t *)malloc(2000 * 512 * sizeof (uint32_t));
  timedelta = (double *) malloc(2000 * sizeof(double));



#ifdef SHOW_DEBUG_SETTINGS
  //
  // Echo User Settings (useful for testing)
  //
  printf("OffsetDistanceFromReference = %f m\n", getFloatValueByName(rh, "OffsetDistanceFromReference"));
  printf("Sample Delay = %f ns\n", getFloatValueByName(rh, "SampleDelay") / 1e-9);  
  printf("SamplesPerSecond = %.0f\n", getFloatValueByName(rh, "SamplesPerSecond"));
  printf("Sample resolution = %.4f m\n", (0.5 * C) / getFloatValueByName(rh, "SamplesPerSecond"));
  printf("NumSamplers = %d\n", numberOfSamplers);
#endif 



  //
  // Operate the radar until uses quits
  //
  if (quitAfterNTrials) {
    fprintf(stderr, "Starting radar loop... (#trials = %d)\n", numTrials);
  } else {
    fprintf(stderr, "Starting radar loop... (hit ESC key to quit)\n");
  }

  struct timespec now, start, tstart = {0};
  double ms_wait;
  clock_gettime(CLOCKID, &start);

  int t;
  for (t = 0; t < numTrials; t++) {
    clock_gettime(CLOCKID, &tstart);
    //printf("%f\n",ms_diff(&tstart, &end));
    timedelta[t] = (double)(ms_diff(&tstart, &start)/1000.0);

    // Get a radar frame
    status = radarHelper_getFrameRaw(rh, radarFrames + t * 512, 512);
    if (status) return 1;

    for (i = 0; i < 512; i++)
    {
      b_dv[i + t * 512] = (double)radarFrames[i + t * 512];
    }

    do {
      clock_gettime(CLOCKID, &now);
       ms_wait = (t+1) * 1000.0/frameRate - ms_diff(&now, &start);
   } while (ms_wait > 0);

  }

  //frames per second
  fpsEst = numTrials/(ms_diff(&now, &start)/1000.0);
  fprintf(stderr, "estimated fps: %f\n", fpsEst);

  /* Call the entry-point 'WIP_LiveFFT'. */
  WIP_LiveFFT(b_dv, outFFT);

  double maxOutFFT = 0;
  double minOutFFT = 100000000000;
  for (i = 100; i < 2000 - 100; i++)
  {
    if (outFFT[i] > maxOutFFT) maxOutFFT = outFFT[i];
    if (outFFT[i] < minOutFFT) minOutFFT = outFFT[i];
  }
  printf("maxOutFFT = %f\n", maxOutFFT);
  printf("minOutFFT = %f\n", minOutFFT);

  //
  // Setup Gnuplot if necessary
  //
  FILE * gnuplotPipe;
  if (showGnuPlot) {
    // Open a pipe to gnuplot
    gnuplotPipe = popen("gnuplot -persistant", "w");

    // Set up Gnuplot
    fprintf(gnuplotPipe, "set term x11 noraise \n");  
    fprintf(gnuplotPipe, "set title \"%s - Radar Demo\" \n", CAPE_NAME);
    fprintf(gnuplotPipe, "set xlabel \"sample#\" \n");
    fprintf(gnuplotPipe, "set ylabel \"Normalized DAC\" \n");
    fprintf(gnuplotPipe, "set xrange [0:%d] \n", 2000 - 1);
    fprintf(gnuplotPipe, "set yrange [%f:%f] \n", minOutFFT, maxOutFFT);
  }

  if (showGnuPlot) {
    // Plot the radar frame
    fprintf(gnuplotPipe, "plot '-' using 1:2 with lines \n");
    for (i = 0; i < 2000 - 1; i++)
    {
      fprintf(gnuplotPipe, "%lf %lf\n", (double)i, (double)outFFT[i]);
    }
    fprintf(gnuplotPipe, "e\n");
    fflush(gnuplotPipe);
  }

  nonblock(NB_DISABLE);

  //
  // All done, clean up time...
  //
  
  // Close the Gnuplot pipe if necessary
  if (showGnuPlot) {
    pclose(gnuplotPipe);
  }

  // Close radar connection
  status = radarHelper_close(&rh);
  if (status) return 1;

  // Free memory allocated for radar signals
  free (radarCounters);
  free (radarFrames);

  WIP_LiveFFT_terminate();

  return 0;
}
