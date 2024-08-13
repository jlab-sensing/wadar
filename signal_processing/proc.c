#include "proc.h"
#include "salsa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>

#define PI 3.14159265358979323846

//#define PROC_TEST

#ifdef PROC_TEST
int main()
{
    RadarData *radarData = salsaLoad("/home/ericdvet/jlab/wadar/signal_processing/testFile.frames");
    complex float *basebandSignal;
    double *rfSignal;

    rfSignal = (double *)malloc(512 * sizeof(double));
    basebandSignal = (complex float *)malloc(512 * sizeof(complex float));

    for (int i = 0; i < 512; i++)
    {
        rfSignal[i] = radarData->frameTot[i];
    }
    NoveldaDDC(rfSignal, basebandSignal);

    for (int i = 0; i < 512; i++)
    {
        printf("%f + %f\n", creal(basebandSignal[i]), cimag(basebandSignal[i]));
    }

    return 0;
}
#endif