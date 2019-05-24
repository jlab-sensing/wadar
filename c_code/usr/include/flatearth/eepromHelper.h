/**
   @file eepromHelper.h

   Helper functions to make reading and writing the BBB EEPROM easier
   @ingroup module_eepromHelper

   @copyright Copyright (c) 2017 Flat Earth Inc

   @author Raymond Weber
*/

#ifndef EEPROM_HELPER_h
#define EEPROM_HELPER_h

// General include
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------


/**
   Read a EEPROM memory on and populate the structure

   @param path Sysfs path to the eeprom to try to read

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_eepromhelper
*/
int eepromHelper_readCapeEEPROM(char *path);


/**
   Get the revision number for the cape

   @param revision Pre-allocated string buffer to get the revision name

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_eepromhelper
*/
int eepromHelper_getCapeRevision(char *revision);


/**
   Get the board name for the cape

   @param boardName Pre-allocated string buffer to get the capes name

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_eepromhelper
*/
int eepromHelper_getCapeBoardName(char *boardName);


/**
   Get the manufacturer name for the cape

   @param manufacturer Pre-allocated string buffer to get the capes name

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_eepromhelper
*/
int eepromHelper_getCapeManufacturer(char *manufacturer);


/**
   Get the part number name for the cape

   @param partNumber Pre-allocated string buffer to get the part number

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_eepromhelper
*/
int eepromHelper_getCapePartNumber(char *partNumber);


/**
   Get the serial number for the cape

   @param serialNumber Pre-allocated string buffer to get the serial number

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_eepromhelper
*/
int eepromHelper_getCapeSerialNumber(char *serialNumber);


/** Get the version for the cape

  @param version Pre-allocated string buffer to get the version number

  @pre readCapeEEPROM() must be run before this function will return valid data

  @return NVA_SUCCESS on success, else an error code
  @ingroup module_eepromhelper
*/
int eepromHelper_getCapeVersion(char *version);










/** Read all known spots that the EEPROM can exist on the BBB
  @note this is a prerequisite for all other bbb_eepromHelper function
  @return SUCCESS or error code
  @ingroup module_bbbeepromhelper
*/
int bbb_eeprom_readAll();

/** Return the board revision for a 0 indexed cape number on a BBB
  @pre bbb_eeprom_readAll() must have been run prior to this command
  @return SUCCESS or error code
  @ingroup module_bbbeepromhelper
*/
int bbb_eepromHelper_getCapeRevision(int capeNumber, char *revision);

/** Return the board name for a 0 indexed cape number on a BBB
  @pre bbb_eeprom_readAll() must have been run prior to this command
  @return SUCCESS or error code
  @ingroup module_bbbeepromhelper
*/
int bbb_eepromHelper_getCapeBoardName(int capeNumber,char *boardName);

/** Return the board manufacturer for a 0 indexed cape number on a BBB
  @pre bbb_eeprom_readAll() must have been run prior to this command
  @return SUCCESS or error code
  @ingroup module_bbbeepromhelper
*/
int bbb_eepromHelper_getCapeManufacturer(int capeNumber, char *manufacturer);

/** Return the part number for a 0 indexed cape number on a BBB
  @pre bbb_eeprom_readAll() must have been run prior to this command
  @return SUCCESS or error code
  @ingroup module_bbbeepromhelper
*/
int bbb_eepromHelper_getCapePartNumber(int capeNumber, char *partNumber);

/** Return the serial number for a 0 indexed cape number on a BBB
  @pre bbb_eeprom_readAll() must have been run prior to this command
  @return SUCCESS or error code
  @ingroup module_bbbeepromhelper
*/
int bbb_eepromHelper_getCapeSerialNumber(int capeNumber, char *serialNumber);

/** Return the version number for a 0 indexed cape number on a BBB
  @pre bbb_eeprom_readAll() must have been run prior to this command
  @return SUCCESS or error code
  @ingroup module_bbbeepromhelper
*/
int bbb_eepromHelper_getCapeVersion(int capeNumber, char *version);


/**
  @defgroup module_eepromhelper EEPROM Read Functions
  Group of functions specific to reading the EEPROM memory at sysfs paths

  @ingroup modules
*/


/**
  @defgroup module_bbbeepromhelper Beaglebone EEPROM Read Functions
  Group of functions specific to reading the EEPROM memory on a beaglebone cape

  @ingroup modules
*/


#ifdef __cplusplus
}
#endif
#endif

