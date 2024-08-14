#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fftw3.h>
#include <stdbool.h>
#include <complex.h>
#include <string.h>
#include "salsa.h"

// Function processes radar frames for various purposes
bool procRadarFrames(const char *localDataPath, const char *captureName, double tagHz);