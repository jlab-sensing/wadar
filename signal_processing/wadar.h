#ifndef WADAR_H
#define WADAH_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "proc.h"
#include "utils.h"

double wadar(bool displayAllFrames, char *localDataPath, char *airFramesName, char *trialName, double tagHz, int frameCount, int captureCount, double tagDepth);

#endif WADAR_H