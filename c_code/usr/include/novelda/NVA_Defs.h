/***********************************************************//**
* \file NVA_Defs.h
* \brief Define various \#defines for different target OS, compilers etc.
*/
/*
* Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
* Novelda AS grant to Customer a nonexclusive and nontransferable license
* to use this software code under terms regulated in a Software License Agreement
* or similar agreement between Novelda AS and the Customer.
* There is NO WARRANTY, to the extent permitted by law.
****************************************************************/

#ifndef NVA_DEFS_H_
#define NVA_DEFS_H_

#include "Radarlib3_config.h"

/**
 * @def CALL_CONV
 * @brief Defines calling conventions for all functions available by the API.
 *
 * Normal values are __cdecl and __stdcall.
 *  __cdecl is the only variant supported in make files for building Windows Radarlib3.NET library
 *
 * @def DLL_PUBLIC
 * @brief Defines attribute to make a library entry visible when building shared library.
 *
 * BUILDING_DLL must be defined when building shared library,
 * BUILDING_DLL must not be defined when using header files to use a shared library.
 *
 * @def DLL_LOCAL
 * @brief Defines attribute to make a library entry not visible when building shared library.
 *
 * BUILDING_DLL must be defined when building shared library,
 * BUILDING_DLL must not be defined when using header files to use a shared library.
 *
 * @def ENVIRONMENT32
 * @brief Is defined when building in a 32 bit environment.
 *
 * Used for conditional compile.
 *
 * @def ENVIRONMENT64
 * @brief Is defined when building in a 64 bit environment
 *
 * Used for conditional compile.
 */
#if defined _WIN32 || defined __CYGWIN__ // _WIN32 is defined for both 32-bit and 64-bit Windows
  #ifdef BUILDING_DLL
    #ifdef __GNUC__
      #define DLL_PUBLIC __attribute__ ((dllexport))
    #else
      #define DLL_PUBLIC __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
    #endif
  #else
    #ifdef __GNUC__
      #define DLL_PUBLIC __attribute__ ((dllimport))
    #else
      #define DLL_PUBLIC __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
    #endif
  #endif
  #define DLL_LOCAL
  #ifndef CALL_CONV
    #if defined (_MSC_VER)
      #define CALL_CONV __cdecl
    #elif defined (__GNUC__)
      #define CALL_CONV __attribute__ ((cdecl))
    #else
      #define CALL_CONV __attribute__ ((cdecl))
    #endif
  #endif
#else
  #if defined __GNUC__ && __GNUC__ >= 4
    #define DLL_PUBLIC __attribute__ ((visibility ("default")))
    #define DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define DLL_PUBLIC
    #define DLL_LOCAL
  #endif
  #ifndef CALL_CONV
    #define CALL_CONV  // Use compiler/linker default
  #endif
#endif

#ifndef PROG_VERSION
  #define PROG_VERSION "0.0.0"  ///< Default value only used if not defined in Makefile
#endif
#ifndef PROG_NAME
  #define PROG_NAME "Unknown"  ///< Default value only used if not defined in Makefile
#endif

#define CONNECTION_STRING_DELIMITER1 '!'  ///< Used as delimiter between parts of a connection string
#define CONNECTION_STRING_DELIMITER2 '#'  ///< Used as delimiter between several connection strings

#define LIST_DELIMITER ','  ///< Used as delimiter between items in a list

// Check which environment we are running under
#ifdef DOXYGEN
  // define macros when doxygen is running to avoid warning when documenting these defines
  #define ENVIRONMENT64
  #define ENVIRONMENT32
#endif

#if defined (_WIN32) || defined (_WIN64)
  #if defined (_WIN64)
    #define ENVIRONMENT64
  #else
    #define ENVIRONMENT32
  #endif
#endif

// Check GCC
#if defined (__GNUC__)
  #if defined (__x86_64__) || defined (__ppc64__)
    #define ENVIRONMENT64
  #else
    #define ENVIRONMENT32
  #endif
#endif

#if defined (__GNUC__)
  #define INLINE __inline__
#elif defined (_MSC_VER)
  #define INLINE __inline
  #define inline __inline
#endif

/**
 * \def VALUE_INT_FORMAT
 * \brief Format string for a 64 bit integer - used in printf format
 *
 * \def VALUE_INT_hex_FORMAT
 * \brief Format string for a 64 bit hex integer - used in printf format
 *
 * \def VALUE_INT_HEX_FORMAT
 * \brief Format string for a 64 bit HEX integer - used in printf format
 *
 */
#if defined(_MSC_VER) || defined(__MINGW32__) || defined(__MINGW64__)
  #if defined (config_VARIABLE_SIZE) && config_VARIABLE_SIZE > 32
    #define VALUE_INT_FORMAT "I64u"
    #define VALUE_INT_hex_FORMAT "I64x"
    #define VALUE_INT_HEX_FORMAT "I64X"
  #else
    #define VALUE_INT_FORMAT "lu"
    #define VALUE_INT_hex_FORMAT "x"
    #define VALUE_INT_HEX_FORMAT "X"
  #endif
#else
  #if defined (config_VARIABLE_SIZE) && config_VARIABLE_SIZE > 32
    #define VALUE_INT_FORMAT "llu"
    #define VALUE_INT_hex_FORMAT "llx"
    #define VALUE_INT_HEX_FORMAT "llX"
  #else
    #define VALUE_INT_FORMAT "lu"
    #define VALUE_INT_hex_FORMAT "lx"
    #define VALUE_INT_HEX_FORMAT "lX"
  #endif
#endif

/**
 * \def UNUSED_ARGUMENT
 * \brief Macro to mark unused arguments so compiler warnings are avoided.
 */
#if defined (_MSC_VER)
  #define UNUSED_ARGUMENT(var)  UNUSED_ ## var
#elif defined (__GNUC__)
  #define UNUSED_ARGUMENT(var) UNUSED_ ## var __attribute__((__unused__))
#else
  #define UNUSED_ARGUMENT(var) (var)
#endif

/**
 * \def NULL
 * \brief Define NULL if not already defined
 */
#ifndef NULL
  #define NULL ((void *)0)
#endif

#endif /* NVA_DEFS_H_ */
