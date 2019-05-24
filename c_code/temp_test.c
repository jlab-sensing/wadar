/**
  ANCHO cape -- test program for temperature sensor
*/
#include <stdio.h>

// SalsaLib include
#include "anchoHelper.h"

int main(void)
{
  float temperature;

  // Read the current temperature
  anchoHelper_readTemp(&temperature);

  // Report it
  printf("ANCHO temperature = %f C\n", temperature);

  return 0;
}
