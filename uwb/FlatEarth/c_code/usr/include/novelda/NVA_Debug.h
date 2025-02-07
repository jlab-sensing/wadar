/***********************************************************//**
* @file NVA_Debug.h
* @brief Defines macros for debug print
*
* @details
* Can be used to print debug output to the console with different levels of output.
* A global variable "NVA_DebugLevel" is used to set the level of debug output.
* The defined "DEBUG_xxx_PRINTF()" macros are wrappers around a printf statement and
* can be used as a normal "printf()" function.
*
* "DEBUG_xxx_PRINTF()" macros and the variable "NVA_DebugLevel" can be used
* when "NVA_Debug.h" is included in the source file and
* "NVA_Debug.c" (part of the Radarlib3_support library)
* are compiled and linked in.
*/
/*
* Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
* Novelda AS grant to Customer a non exclusive and non transferable license
* to use this software code under terms regulated in a Software License Agreement
* or similar agreement between Novelda AS and the Customer.
* There is NO WARRANTY, to the extent permitted by law.
****************************************************************/

#ifndef NVA_DEBUG_H_
#define NVA_DEBUG_H_

#include <stdio.h>

/**
 * Controls the amount of debug output from macros defined in NVA_Debug.h
 */
extern DLL_PUBLIC int NVA_DebugLevel;

#define DEBUG_OFF 0       ///< No debug output
#define DEBUG_ERROR 1     ///< Only error are output
#define DEBUG_WARNING 2   ///< Only error and warnings are output
#define DEBUG_INFO 3      ///< Info, warning and errors are output
#define DEBUG_ALL 10      ///< All debug output are output

/**
 * \def DEBUG_PRINTF
 * \brief Macro to print to console when NVA_DebugLevel == DEBUG_ALL
 *
 * \def DEBUG_ERROR_PRINTF
 * \brief Macro to print to console when NVA_DebugLevel >= DEBUG_ERROR
 *
 * \def DEBUG_WARNING_PRINTF
 * \brief Macro to print to console when NVA_DebugLevel >= DEBUG_WARNING
 *
 * \def DEBUG_INFO_PRINTF
 * \brief Macro to print to console when NVA_DebugLevel >= DEBUG_INFO
 */

#ifdef NVA_DEBUG
#define DEBUG_PRINTF(...) if (NVA_DebugLevel == DEBUG_ALL) printf(__VA_ARGS__)
#define DEBUG_ERROR_PRINTF(...) if (NVA_DebugLevel >= DEBUG_ERROR) printf(__VA_ARGS__)
#define DEBUG_WARNING_PRINTF(...) if (NVA_DebugLevel >= DEBUG_WARNING) printf(__VA_ARGS__)
#define DEBUG_INFO_PRINTF(...) if (NVA_DebugLevel >= DEBUG_INFO) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#define DEBUG_ERROR_PRINTF(...)
#define DEBUG_WARNING_PRINTF(...)
#define DEBUG_INFO_PRINTF(...)
#endif

#endif /* NVA_DEBUG_H_ */
