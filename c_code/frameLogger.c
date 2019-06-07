/**
  @anchor bbb_ancho_framelogger_demo

  # FrameLogger
  FrameLogger is a simple program used to produce binary data log files; in other
  words, this is a tool to help log radar frames for post-processing.

  ## Hardware Setup ##
  Using a BeagleBone Black + Salsa cape + (user cables and antennas)

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
  viewer program use the _same_ stage1.json and stage2.json files.

  @verbatim
  Usage: FrameLogger [-option(s)]
  where options include:
  -g                    - Enable Gnuplot of radar frames
  -s [setting file]     - Specify settings output filename
  -l [log file]         - Specify dataLog filename
  -n [num trials]       - Specify number of trials to run
  -d [delay]            - Specify processing delay in microseconds
  -r [runs]             - Specify number of runs to perform
  -f [framerate]        - Specify framerate
  -t [type]             - Specify type Ancho or Cayenne or Chipotle
  -c [copyPath]             - Directory on computer to transfer the files to
  @endverbatim

  Example user session:
  @verbatim
  # ./FrameLogger -l testLog1 -g
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

  The example session produces a binary file, testLog1, that can then be used
  by other tools which can play back and process the data.

  There is an indicated area (in the code) where the user could add custom DSP
  which could effect the frames. One example use might be to move the frame based
  on how the data is processed. In the current form, there is signal normalization
  and a simple delay added.

  This program is also intended to be used as a basis for _user_ customization.
  Often, collecting data is performed in the field; it can be difficult to
  produce realistic data in an office environment. Using the data log concept
  for field data collection and later post-processing playback can save time.

  In this demonstration, the only _user_ customization is that the signal output
  is normalized into the DAC range. The actual data logged is the radar counter
  values, but the signal is normalized internally so the Gnuplot display would
  display well (independent of settings changes).

  @note
  Using the Gnuplot during data collection can slow the framerate. Typically, it
  is easier to just collect data without plotting it.

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
#include <time.h>
#include <dirent.h>

// SalsaLib include
#include "anchoHelper.h"
#include "cayenneHelper.h"
#include "chipotleHelper.h"
#include "radarHelper.h"
#include "radarDSP.h"

// Novelda radar API include
#include "Radarlib3.h"

#define CLOCKID CLOCK_REALTIME

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
void LEDHelper(int radarType, SalsaLED led, int value);

// -----------------------------------------------------------------------------
// Variables
// -----------------------------------------------------------------------------

// Connection string used when connecting to the radar
static char *radarConnectionStr = "";

// Number of samplers we want
static int numberOfSamplers;
// Additional values?
// Additional values?

// Pointers to arrays for signal storage
static uint32_t *radarFrames;
static double *timedelta;
//static double *radarScaled;
// Additional signals?
// Additional signals?

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------

void Usage()
{
  printf("Usage: FrameLogger [-option(s)]\n");
  printf("where options include:\n");
  printf(" -%c %-18s - %-40s\n", 'g', "", "Enable Gnuplot of radar frames");
  printf(" -%c %-18s - %-40s\n", 's', "[setting file]", "Specify settings output filename");
  printf(" -%c %-18s - %-40s\n", 'l', "[log file]", "Specify dataLog filename");
  printf(" -%c %-18s - %-40s\n", 'n', "[num trials]", "Specify number of trials to run");
  printf(" -%c %-18s - %-40s\n", 'd', "[delay]", "Specify processing delay in microseconds");
  printf(" -%c %-18s - %-40s\n", 'r', "[runs]", "Specify number of runs to perform");
  printf(" %-c %-18s - %-40s\n", 'f', "[framerate]", "Specify framerate");
  printf(" %-c %-18s - %-40s\n", 't', "[type]", "Specify radar type, Ancho, Cayenne, or Chipotle");
  printf(" -%c %-18s - %-40s\n", 'c', "[copyPath]", "Directory on computer to transfer the files to");
}

void LEDHelper(int radarType, SalsaLED led, int value)
{
  switch (radarType)
  {
    case 2:
      anchoHelper_setLED(led, value);
      break;
    case 11:
      chipotleHelper_setLED(led,value);
      break;
    case 10:
      cayenneHelper_setLED(led, value);
      break;
  }
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

int main(int argc, char **argv)
{
  // Radar handle used as a reference to a structure with information about the
  // connected radar
  RadarHandle_t rh = NULL;

  // Return status from radar helper calls
  int status;

  // Loop counters
  //int i;
  int numTrials = 1;

  int numRuns = 1;
  int runs;
  int runNum;

  //timing
  int frameRate = 200;
  float fpsEst;

  // Getopt Flags
  bool showGnuPlot = false;
  bool saveSettingsFile = false;
  bool saveDataLogFile = false;

  // Used by the command-line parser
  int c;
  opterr = 0;

  // Delay to artificially reduce framerate by adding delay
  int processingDelay = 1;

  // Radar settings to LOG to the file
  int iterations;
  int pps;
  int dacMin;
  int dacMax;
  int dacStep;

  int pgSelect; //Ancho...
  float offsetDistance;
  float sampleDelayToReference;
  float temperature;

  int pulseGenFineTune; //Cayenne or Chipotle...
  int samplingRate;
  int clkDivider;

  // Radar values which effect distance estimation
  double samplesPerSecond;

  // Pointers to input/output files
  const char *inFile_stage1 = NULL;
  const char *inFile_stage2 = NULL;
  const char *settingsFile = NULL;
  const char *dataLogFile = NULL;
  const char *copyPath = NULL;

  // Pointer to datalog file
  FILE *dataLog;

  //type of radar
  int radarType = -1; //2 for X2, 10 for X1-Cayenne, 11 for X1-Chipotle

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  //
  // Process command-line arguments
  //
  while ((c = getopt(argc, argv, "gs:l:n:d:r:f:t:c:")) != -1) {
    switch (c) {
    /* Enable Gnuplot of radar data */
    case 'g':
      showGnuPlot = true;
      break;

    /* Save configuration JSON to file */
    case 's':
      saveSettingsFile = true;
      settingsFile = optarg;
      break;

    /* Save DataLog to binary file */
    case 'l':
      saveDataLogFile = true;
      dataLogFile = optarg;
      break;

    /* Quit after N iterations */
    case 'n':
      numTrials = atoi(optarg);
      // Some simple user validation
      if (numTrials < 1) {
        fprintf(stderr, "Please make #trials an integer > 0\n");
        exit(0);
      }
      break;

    /* Processing delay in microseconds */
    case 'd':
      processingDelay = atoi(optarg);
      if (processingDelay < 1) {
        fprintf(stderr, "Please make delay an integer\n");
        exit(0);
      }
      break;

    /* Quit after R repeats */
    case 'r':
      numRuns = atoi(optarg);
      // Some simple user validation
      if (numRuns < 1) {
        fprintf(stderr, "Please make #runs an integer > 0\n");
        exit(0);
      }
      break;

    /* framerate */
    case 'f':
      frameRate = atoi(optarg);
      // Some simple user validation
      if (frameRate < 1) {
        fprintf(stderr, "Please make frame rate an integer > 0\n");
        exit(0);
      }
      break;

    case 't':
      if (optarg[0] == 'a' || optarg[0] == 'A') {
        radarType = 2;
        radarConnectionStr = "BeagleBone!SPI device: 0!FE_Salsa!NVA6201";
        inFile_stage1 = "stage1Ancho.json";
        inFile_stage2 = "stage2Ancho.json";

      } else if (optarg[0] == 'c' || optarg[0] == 'C') {
        radarConnectionStr = "BeagleBone!SPI device: 0!FE_Salsa!NVA6100";

        if (optarg[1] == 'h' || optarg[1] == 'H') {
          radarType = 11;
          inFile_stage1 = "stage1Chipotle.json";
          inFile_stage2 = "stage2Chipotle.json";
        } else {
          radarType = 10;
          inFile_stage1 = "stage1Cayenne.json";
          inFile_stage2 = "stage2Cayenne.json";
        }
      }
      break;

    case 'c':
      copyPath = optarg;
      break;

    default:
      Usage();
      exit(0);
    }
  }

  if (radarType == -1) {
    printf("No radar type specified....Exiting the program....\n");
    exit(0);
  }


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  //
  // Initiate a radar handle
  //
  status = radarHelper_open(&rh, radarConnectionStr);
  if (status) return 1;

  //
  // Configure the radar using the Stage 1 configuration JSON file
  //
  status = radarHelper_configFromFile(rh, inFile_stage1, 1);
  if (status) return 1;

  //
  // Do radar timing measurements
  //
  status = radarHelper_doAction(rh, "MeasureAll");
  if (status) return 1;

  // Set the low frequency pulse generator
  if (radarType == 11) {
    setIntValueByName(rh, "PulseGen", 1);
  }

  //
  // Configure the radar using the Stage 2 configuration JSON file
  //
  status = radarHelper_configFromFile(rh, inFile_stage2, 2);
  if (status) return 1;

  //
  // Save radar settings to file (optional)
  //

  if (saveSettingsFile) {
    status = radarHelper_saveConfigToFile(rh, settingsFile);
    if (status) return 1;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  // Turn ON the Red LED
  LEDHelper(radarType, LED_Red, 1);

  // Turn OFF the other LEDs
  LEDHelper(radarType, LED_Blue, 0);
  LEDHelper(radarType, LED_Green0, 0);
  LEDHelper(radarType, LED_Green1, 0);

  // Determine the number of samplers in the radar frame
  numberOfSamplers = getIntValueByName(rh, "SamplersPerFrame");

  // Get the other radar settings to LOG
  iterations = getIntValueByName(rh, "Iterations");
  pps = getIntValueByName(rh, "PulsesPerStep");
  dacMin = getIntValueByName(rh, "DACMin");
  dacMax = getIntValueByName(rh, "DACMax");
  dacStep = getIntValueByName(rh, "DACStep");
  samplesPerSecond = getFloatValueByName(rh, "SamplesPerSecond");

  if (radarType == 2) {
    pgSelect = getIntValueByName(rh, "PGSelect");
    samplesPerSecond = getFloatValueByName(rh, "SamplesPerSecond");
    offsetDistance = getFloatValueByName(rh, "OffsetDistanceFromReference");
    sampleDelayToReference = getFloatValueByName(rh, "SampleDelayToReference");
  } else {
    pulseGenFineTune = getIntValueByName(rh, "PulseGenFineTune");
    samplingRate = getIntValueByName(rh, "SamplingRate");
    clkDivider = getIntValueByName(rh, "ClkDivider");
  }

  //
  // Allocate memory for signal storage
  //
  radarFrames = (uint32_t *)malloc(numTrials*numberOfSamplers * sizeof (uint32_t));
  timedelta = (double *) malloc(numTrials*sizeof(double));
  //radarScaled = (double *)malloc(numberOfSamplers * sizeof (double));

  //
  // Setup Gnuplot (if necessary)
  //
  FILE *gnuplotPipe;
  if (showGnuPlot) {
    // Open a pipe to gnuplot
    gnuplotPipe = popen("gnuplot -persistant", "w");

    // Set up Gnuplot
    fprintf(gnuplotPipe, "set term x11 \n");
    fprintf(gnuplotPipe, "set title \"FrameLogger Demo\" \n");
    fprintf(gnuplotPipe, "set xlabel \"sample#\" \n");
    fprintf(gnuplotPipe, "set xrange [0:%d] \n", numberOfSamplers - 1);
#ifdef SHOW_FULL_Y_AXIS_RANGE
    fprintf(gnuplotPipe, "set yrange [%d:%d] \n", GNUPLOT_SCALED_Y_MIN, GNUPLOT_SCALED_Y_MAX);
#else
    fprintf(gnuplotPipe, "set yrange [%d:%d] \n", GNUPLOT_PRACTICAL_Y_MIN, GNUPLOT_PRACTICAL_Y_MAX);
#endif
    fprintf(gnuplotPipe, "set ylabel \"Normalized DAC\" \n");
  }


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


  // Turn ON the Red LED
  LEDHelper(radarType, LED_Red, 1);

  //
  // Run radar loop N times
  //

  runs = numRuns;
  runNum = 0;

  while (runs > 0) {

    // Give user some feedback
    runNum++;
    fprintf(stderr, "\nStarting radar loop %d: (#trials = %d)\n", runNum, numTrials);

    //
    // Setup datalog (if necessary)
    //
    char nameBuffer[50];
    if (saveDataLogFile) {
      //remove capture data from previous runs
      system("exec rm -r ../data/*");

      sprintf(nameBuffer, "%s%d", dataLogFile, runNum);
      dataLog = fopen(nameBuffer, "wb");
      if (!dataLog) {
        fprintf(stderr, "Unable to open %s!\n", dataLogFile);
        return 1;
      }

      //
      // Begin dataLog with the number of samples in the signal and num trials
      //
      uint32_t magic = FRAME_LOGGER_MAGIC_NUM;
      fwrite(&magic, sizeof (uint32_t), 1, dataLog);
      fwrite(&iterations, sizeof (int), 1, dataLog);
      fwrite(&pps, sizeof (int), 1, dataLog);
      fwrite(&dacMin, sizeof (int), 1, dataLog);
      fwrite(&dacMax, sizeof (int), 1, dataLog);
      fwrite(&dacStep, sizeof (int), 1, dataLog);
      fwrite(&radarType, sizeof (int), 1, dataLog);
      if (radarType == 2) {
        fwrite(&samplesPerSecond, sizeof (float), 1, dataLog);
        fwrite(&pgSelect, sizeof (int), 1, dataLog);
        fwrite(&offsetDistance, sizeof (float), 1, dataLog);
        fwrite(&sampleDelayToReference, sizeof (float), 1, dataLog);
      } else {
        fwrite(&samplesPerSecond, sizeof (double), 1, dataLog);
        fwrite(&pulseGenFineTune, sizeof (int), 1, dataLog);
        fwrite(&samplingRate, sizeof (int), 1, dataLog);
        fwrite(&clkDivider, sizeof (int), 1, dataLog);
      }
      fwrite(&numberOfSamplers, sizeof (int), 1, dataLog);
      fwrite(&numTrials, sizeof (int), 1, dataLog);
      fwrite(&numRuns, sizeof (int), 1, dataLog);
      fwrite(&frameRate, sizeof (int), 1, dataLog);
    }

    struct timespec now, start, tstart = {0};
    double ms_wait;
    clock_gettime(CLOCKID, &start);
    for (int t = 0; t < numTrials; t++) {
      clock_gettime(CLOCKID, &tstart);
      //printf("%f\n",ms_diff(&tstart, &end));
      timedelta[t] = (double)(ms_diff(&tstart, &start)/1000.0);
      // Get a radar frame
      status = radarHelper_getFrameRaw(rh, radarFrames+t*numberOfSamplers,
				       numberOfSamplers);
      if (status) {
        fclose(dataLog);
        return 1;
      }
      // Read the current temperature
      //isAncho ? anchoHelper_readTemp(&temperature) : cayenneHelper_readTemp(&temperature);

      //Wait as needed to control frameRate
      /* clock_gettime(CLOCKID, &now); */
      /* ms_wait = (t+1) * 1000.0/frameRate - ms_diff(&now, &start); */

      /* struct timespec timer; */
      /* timer.tv_sec = 0; */
      /* timer.tv_nsec = (long)((ms_wait) * 1000000); */

      /* //if (timer.tv_nsec > 0) { */
      /* while (ms_wait > 0) {//nanosleep(&timer, &timer) == -1) */
      /* 	clock_gettime(CLOCKID, &now); */
      /* 	ms_wait = (t+1) * 1000.0/frameRate - ms_diff(&now, &start); */
      /* 	continue; */
      /* } */

      do {
	clock_gettime(CLOCKID, &now);
	ms_wait = (t+1) * 1000.0/frameRate - ms_diff(&now, &start);
      } while (ms_wait > 0);

    }

    //frames per second
    fpsEst = numTrials/(ms_diff(&now, &start)/1000.0);
    fprintf(stderr, "estimated fps: %f\n", fpsEst);

    if (saveDataLogFile) {
        //fwrite(&temperature, sizeof (float), 1, dataLog);
        fwrite(timedelta, sizeof(double), numTrials, dataLog);
        fwrite(radarFrames, sizeof (uint32_t), numberOfSamplers*numTrials, dataLog);
	      fwrite(&fpsEst, sizeof (float), 1, dataLog);

        // Close the dataLog if necessary
        fclose(dataLog); 

        //copy datalog to host computer
        if (copyPath != NULL) {
          char copyBuffer[150];
          sprintf(copyBuffer, "exec scp %s %s", nameBuffer, copyPath);
          printf("Copying data to host computer...\n");
          system(copyBuffer);
        }
    }

    // Decrement run counter
    runs--;
  }
  printf("\n");



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
  free (radarFrames);
  free (timedelta);
  //free (radarScaled);

  //kill the radar screen used to run this program
  system("exec pkill radar");

  return 0;
}
