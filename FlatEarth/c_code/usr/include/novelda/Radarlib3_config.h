/***********************************************************//**
* @file Radarlib3_config.h
* @brief Configures which part of radarlib3 to include and configure
*
* The different configuration options will more or less functionality for
* Radarlib3. This may be useful for reduced code size, reduced memory
* requirements or increased speed.
*/
/*
* Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
* Novelda AS grant to Customer a nonexclusive and nontransferable license
* to use this software code under terms regulated in a Software License Agreement
* or similar agreement between %Novelda AS and the Customer.
* There is NO WARRANTY, to the extent permitted by law.
****************************************************************/

#ifndef RADARLIB3_DEFAULT_CONFIG_H_
#define RADARLIB3_DEFAULT_CONFIG_H_


//
// Configurations of Radar ICs
//

/**
 * @def config_NVA6000
 * Include support for NVA6000 Radar IC
 */
#ifndef config_NVA6000
  #define config_NVA6000 1
#endif

/**
 * @def config_NVA6100
 * Include support for NVA6100 Radar IC
 */
#ifndef config_NVA6100
 #define config_NVA6100 1
#endif

/**
 * @def config_NVA6201
 * Include support for NVA6201 Radar IC
 */
#ifndef config_NVA6201
 #define config_NVA6201 1
#endif

/**
 * @def config_NVA6202
 * Include support for NVA6202 Radar IC
 */
#ifndef config_NVA6202
 #define config_NVA6202 1
#endif



//
// Configurations of Radar Modules
//


/**
 * @def config_NVA_R6x1
 * Include support for NVA-R6x1 Radar Modules
 */
#ifndef config_NVA_R6x1
  #define config_NVA_R6x1 1
#endif

/**
 * @def config_NVA_R661
 * Include support for NVA-R661 Radar Modules
 */
#ifndef config_NVA_R661
 #define config_NVA_R661 1
#endif

/**
 * @def config_NVA_R661
 * Include support for NVA-R661 Radar Modules
 */
#ifdef __arm__
#ifndef config_FE_Salsa
 #define config_FE_Salsa 1
#endif
#endif

/**
 * @def config_XTI_Sxxx
 * Include support for XTI-Sxxx Radar Modules
 */
#ifndef config_XTI_Sxxx
 #define config_XTI_Sxxx 1
#endif





//
// Configurations of Interface modules
//

/**
 * @def config_FTx232H
 * Include support for FTx232H Interface Module
 *
 * Supported on Windows
 */
#ifndef config_FTx232H
#if defined _WIN32 || defined __CYGWIN__ || defined __APPLE__
  #define config_FTx232H 1
#else
  #define config_FTx232H 0
#endif
#endif

/**
 * @def config_NVA_FTDI
 * Include support for NVA_FTDI Interface Module
 *
 * Supported on Linux
 */
#ifndef config_NVA_FTDI
#if defined _WIN32 || defined __CYGWIN__ || defined __APPLE__
  #define config_NVA_FTDI 0
#else
  #define config_NVA_FTDI 1
#endif
#endif

/**
 * @def config_RaspberryPi
 * Include support for RaspberryPi Interface Module
 *
 * Supported on Raspberry Pi
 */
/*
#ifdef __arm__
#ifndef config_RaspberryPi
 #define config_RaspberryPi 1
#endif
#endif // __arm__
*/


/**
 * @def config_BeagleBone
 * Include support for BeagleBone Interface Module
 *
 * Supported on BeagleBone
 */

#ifdef __arm__
#ifndef config_BeagleBone
 #define config_BeagleBone 1
#endif
#endif // __arm__
// #define config_BeagleBone 1

/**
 * @def config_ATMEL_SAM4E_EK
 * Include support for ATMEL_SAM4E_EK Interface Module
 *
 * Supported on Atmel ATSAM4E_EK evaluation kit
 */
#ifndef config_ATMEL_SAM4E_EK
 #define config_ATMEL_SAM4E_EK 0
#endif

/**
 * @def config_FESimulation
 * Include support for FESimulation Interface Module
 *
 */
#ifndef FE_Simulator_All
  //#define FE_Simulator_All 1
#endif

#ifdef FE_Simulator_All
  #ifndef config_FESimulation_X2
   #define config_FESimulation_X2 1
  #endif

  #ifndef config_FE_Simulator
   #define config_FE_Simulator 1
  #endif
#endif



//
// Configuration of features in Radarlib
//

/**
 * @def config_NVA_RADAR_FRAME_LOGGING
 * Functions to log radar frames to file or another device
 */
#ifndef config_NVA_RADAR_FRAME_LOGGING
  #define config_NVA_RADAR_FRAME_LOGGING 1
#endif

/**
 * @def config_NVA_VALUE_FLOAT_SIZE
 * Set the size (and precision) of the floating point type ValueFloat_t
 *
 * Standard size is 64 bit (double).
 * This configuration parameter could be adjusted to match the native
 * size/precision for the FPU used. This may increase calculation speed.
 *
 * Legal values:
 *  - 32 - single precision float (same as float)
 *  - 64 - double precision float (same as double)
 */
#ifndef config_NVA_VALUE_FLOAT_SIZE
  #define config_NVA_VALUE_FLOAT_SIZE 64
#endif

/**
 * @def config_VARIABLE_SIZE
 * Set size in bits for variables in the variable system
 *
 * This configuration parameter should match the maximum size of registers in the radar IC used or set to standard value 64.
 *
 * Legal values:
 *  - 32 - Legal value only for NVA6000 or NVA6100
 *  - 64 - Legal value for NVA6000, NVA6100
 */
#ifndef config_VARIABLE_SIZE
  #define config_VARIABLE_SIZE 64
#endif

/**
 * @def config_MAX_RADARS
 * Set the number of max connected radars
 *
 * Some variables like the variable array, must be unique for each connected
 * radar. To achieve this, the different software modules must work on a copy
 * of the original variable array for each connected radar.
 *
 * If only one radar can be connected at a time, there is no need
 * for working on a copy. This will save some memory.
 *
 *  - Set to 1, only one radar can be connected, but less memory will be allocated
 *  - Set to 0, unlimited radars can be connected
 *  - Set to > 1, max number of connected radars will be limited to this number
 *
 */
#ifndef config_MAX_RADARS
  #define config_MAX_RADARS 0
#endif


/**
 * @def config_STACK_TRACE
 * Include code for easy tracking of stack trace when an error occurs.
 */
#ifndef config_STACK_TRACE
  #define config_STACK_TRACE 1
#endif

/**
 * @def config_NVA_SPI_LOGGING
 * Functions to log SPI data to file or another device
 */
#ifndef config_NVA_SPI_LOGGING
  #define config_NVA_SPI_LOGGING 1
#endif
#endif // RADARLIB3_DEFAULT_CONFIG_H_
