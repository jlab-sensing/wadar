/**
   @file ChipotleHelper/inc/timerHelper.h

   Routines for timing functions

   Copyright 2017 by FlatEarth, Inc

   @par Environment
   BeagleBone Black + ancho cape

   @par Compiler
   GNU GCC

   @author Raymond Weber
*/

#ifndef TIMER_HELPER_h
#define TIMER_HELPER_h

#if defined _WIN32 || defined __CYGWIN__
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
#else
#if __GNUC__ >= 4
#define DLL_PUBLIC __attribute__ ((visibility ("default")))
#define DLL_LOCAL __attribute__ ((visibility ("hidden")))
#else
#define DLL_PUBLIC
#define DLL_LOCAL
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif


/** Get the time on the timer
  This is useful for determining how long functions took to run
  (eg: get the time at start and end and compute the delta)
*/
DLL_PUBLIC double timerHelper_getTime_msec();


#ifdef __cplusplus
}
#endif
#endif
