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

// SalsaLib include
#include "chipotleHelper.h"
#include "radarHelper.h"

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
static double * radarScaled;

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
  int numTrials = 1;

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
  radarScaled = (double *)malloc(numberOfSamplers * sizeof (double));



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
    fprintf(gnuplotPipe, "set xrange [0:%d] \n", numberOfSamplers - 1);
#ifdef SHOW_FULL_Y_AXIS_RANGE
    fprintf(gnuplotPipe, "set yrange [%d:%d] \n", GNUPLOT_SCALED_Y_MIN, GNUPLOT_SCALED_Y_MAX);
#else
    fprintf(gnuplotPipe, "set yrange [%d:%d] \n", GNUPLOT_PRACTICAL_Y_MIN, GNUPLOT_PRACTICAL_Y_MAX);
#endif
  }



  //
  // Operate the radar until uses quits
  //
  if (quitAfterNTrials) {
    fprintf(stderr, "Starting radar loop... (#trials = %d)\n", numTrials);
  } else {
    fprintf(stderr, "Starting radar loop... (hit ESC key to quit)\n");
  }

  // This loops forever until user presses the ESC key to quit
  nonblock(NB_ENABLE);
  while(!quit)
  {
    // Get user input
    usleep(1);
    quit = kbhit();
    if (quit != 0)
    {
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
    
    // Convert radar counters to double before scaling
    for (i = 0; i < numberOfSamplers; i++) {
      radarScaled[i] = (double)radarCounters[i];
    }
    
    // Scale radar counters into DAC range (0 - 8191)
    normalizeDAC(rh, radarScaled, radarScaled, numberOfSamplers);

    // Turn OFF the Blue LED while getting a frame
    chipotleHelper_setLED(LED_Blue, 0);


    if (showGnuPlot) {
      // Plot the radar frame
      fprintf(gnuplotPipe, "plot '-' using 1:2 with lines \n");
      for (i = 0; i < numberOfSamplers - 1; i++)
      {
        fprintf(gnuplotPipe, "%lf %lf\n", (double)i, (double)radarScaled[i]);
      }
      fprintf(gnuplotPipe, "e\n");
      fflush(gnuplotPipe);
    }



    if (quitAfterNTrials) {
      numTrials--;

      // Break out of while loop if we've run enough trials
      if (numTrials == 0) break;
    }
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
  free (radarScaled);

  return 0;
}
