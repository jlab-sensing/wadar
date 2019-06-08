/**
   @file cilantroHelper.h

   Definitions for Cilantro-cape specific functionality

   Copyright 2015 by FlatEarth, Inc

   @par Environment
   BeagleBone Black + Cilantro cape

   @par Compiler
   GNU GCC

   @author Justin Hadella
*/
#ifndef Cilantro_HELPER_h
#define Cilantro_HELPER_h

#ifdef __cplusplus
extern "C"
{
#endif


/**
  Initialize io and test the cilantro cape
  @returns status of the cap/init process
  @ingroup module_cilantro
*/
int cilantroHelper_init();

/**
  Select the enabled RF ouput channel
  @param id Select which output to use (0 or 1)
  @returns 0 if valid id, 2 otherwise
  @ingroup module_cilantro
*/
int cilantroHelper_select(int id);

/**
  Frees the handle to the iopins
  @ingroup module_cilantro
*/
int cilantroHelper_free();

/**
  Check the cape EEPROM address to see if cape is present at this address
  @param capeNumber Address to search at (0-3)
  @returns 0 if present, 1 otherwise
  @ingroup module_cilantro
*/
int cilantroHelper_isCilantro(int capeNumber);




/**
  @defgroup module_cilantro Cilantro
  Group of functions specific to using the Cilantro switching cape

  @ingroup modules
*/





/**
  @defgroup modules Hardware Modules
  @ingroup salsaLib
*/




#ifdef __cplusplus
}
#endif
#endif // Cilantro_HELPER_h

