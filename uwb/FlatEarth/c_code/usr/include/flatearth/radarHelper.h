/**
   @file radarHelper.h

   Helper functions to make using Radarlib3 easier

   In general, using these helper function allows the user to write shorter,
   more concise code. The typical flow when using Radarlib3 is to call an NVA
   function then check the return status, optionally displaying an error
   message to the user.

   The radar helper functions act as a simple wrapper, checking the return code
   and displaying the error message. Thus, most of the functions contained here
   are just a convenience.

   However, the radarHelper_configFromFile() is useful as it can be used to set
   a group of radar variables using a JSON file. In the JSON file, each setting
   is represented as an object { "key" : value }. The "key" represents the name
   of a Radarlib3 variable, and the value is a number. The API will read the #
   as a int or float depending on the variable. If the value is null (the word)
   then that variable is treated as a "read-only" value.

   The third parameter into radarHelper_configFromFile() represents the stage.
   In Radarlib3, there is a subset of variables, such as 'SamplingRate' and
   'ClkDivider' which effect other variables related to distance. In order to
   accurately set frame offset distance or estimate distance, these variables
   must be set in stages with a timing measurement in between.

   Here in an example of a stage 1 and stage 2 configuration file:

   stage1.json:
  @verbatim
  [
  {"DACMin" : 1000},
  {"DACMax" : 7000},
  {"DACStep" : 8},
  {"FrameStitch" : 1},
  {"Iterations" : 200},
  {"ClkDivider" : 1},
  {"PulsesPerStep" : 16},
  {"SamplingRate" : 1}
  ]
  @endverbatim

   stage2.json:
  @verbatim
  [
  {"OffsetDistanceFromReference" : 0.0},
  {"SampleDelayToReference" : 1.5e-9},
  {"SamplesPerSecond" : null},
  {"SamplersPerFrame" : null},
  {"SampleDelay" : null}
  ]
  @endverbatim

   Using this set of JSON stages, the code to set the configuration might look
   like:
  @code
  // Connection string used when connecting to the radar (BBB + Cayenne example)
  static char *radarConnectionStr = "BeagleBone!SPI device: 0!DEFAULT!NVA6100";

  ...

  RadarHandle_t radarHandle;
  radarHelper_open(&radarHandle, radarConnectionStr);

  ...

  // Load the Stage 1 configuration file
  radarHelper_configFromFile(radarHandle, "stage1.json", 1);

  // Execute the timing measurement
  radarHelper_doAction(radarHandle, "MeasureAll");

  // Load the Stage 2 configuration file
  radarHelper_configFromFile(radarHandle, "stage2.json", 2);

  ...
  @endcode
   The radarHelper_saveConfigToFile() function can be used to save the current
   Radarlib3 configuration to a file. The contents are determined by the values
   used in the call to radarHelper_configFromFile().

   @note
   The Radarlib3 variable 'SamplingRate' can be set to 2 on the NVA6100 (X1)
   radar chip. However, when this setting is used, performing a calibration
   (aka timing measurement) will fail since this particular setting doesn't
   support this action. Beware!

   @ingroup module_radarhelper

   @copyright Copyright (c) 2015 Flat Earth Inc

   @author Justin Hadella
*/

#ifndef RADAR_HELPER_h
#define RADAR_HELPER_h

// Novelda radar API include
#include "Radarlib3.h"

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
   Function indicates whether or not value parameter within valid range

   @param [in] *arg       The string which represents the variable to test
   @param [in] *value     The value to test
   @param [in]  minValue  The minimum value allowed (inclusive)
   @param [in]  maxValue  The maximum value allowed (inclusive)

   @return true if the value in within the given range, otherwise false
   @ingroup module_radarhelper
*/
bool IsValidIntParameter(const char *arg, Value_t *value, int minValue, int maxValue);

/**
   Function indicates whether or not value parameter within valid range

   @param [in] *arg       The string which represents the variable to test
   @param [in] *value     The value to test
   @param [in]  minValue  The minimum value allowed (inclusive)
   @param [in]  maxValue  The maximum value allowed (inclusive)

   @return true if the value in within the given range, otherwise false
   @ingroup module_radarhelper
*/
bool IsValidFloatParameter(const char *arg, Value_t *value, float minValue, float maxValue);

/**
   Function used to directly return a floating-point variable based on the name
   of the given variable

   @param [in]  handle  Handle the to the radar object
   @param [in] *name    String representing the variable name

   @return the given variable's value
   @ingroup module_radarhelper
*/
float getFloatValueByName(RadarHandle_t handle, const char *name);

/**
   Function used to directly return a integer variable based on the name of the
   given variable

   @param [in]  handle  Handle the to the radar object
   @param [in] *name    String representing the variable name

   @return the given variable's value
   @ingroup module_radarhelper
*/
int getIntValueByName(RadarHandle_t handle, const char *name);

/**
   Function used to set floating-point variable based on the name of the given
   variable

   @param [in]  handle  Handle the to the radar object
   @param [in] *name    String representing the variable name
   @param [in]  value   Value to set
   @return the given variable's value
   @ingroup module_radarhelper
*/
int setFloatValueByName(RadarHandle_t handle, const char *name, float value);

/**
   Function used to set an integer variable based on the name of the given variable

   @param [in]  handle  Handle the to the radar object
   @param [in] *name    String representing the variable name
   @param [in]  value   Value to set
   @return the given variable's value
   @ingroup module_radarhelper
*/
int setIntValueByName(RadarHandle_t handle, const char *name, int value);

// -----------------------------------------------------------------------------
// Radar Helper Functions
// -----------------------------------------------------------------------------

/**
   Function used to get the version of the radar helper library DLL

   @return version of the dll
   @ingroup module_radarhelper
*/
float radarHelper_DLLversion();

/**
   Helper function used to execute an action

   @param [in]  handle  Handle the to the radar object
   @param [in] *action    String representing the action

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_radarhelper
   @deprecated Version checks in the future should be based on the DPKG version
*/
int radarHelper_doAction(RadarHandle_t handle, char *action);

/**
   Helper function used to set a variable

   @param [in]  handle   Handle the to the radar object
   @param [in] *varName  String representing the variable
   @param [in]  value    The value to set the variable to

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_radarhelper
*/
int radarHelper_setVar(RadarHandle_t handle, char *varName, Value_t value);

/**
   Helper function used to get a variable

   @param [in]   handle   Handle the to the radar object
   @param [in]  *varName  String representing the variable
   @param [out] *value    The value read

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_radarhelper
*/
int radarHelper_getVar(RadarHandle_t handle, char *varName, Value_t *value);

/**
   Helper function used to open a connection to the radar

   @param [out] *handle      Handle the to the radar object
   @param [in]  *moduleName  String representing the radar

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_radarhelper
*/
int radarHelper_open(RadarHandle_t *handle, char *moduleName);

/**
   Helper function used to close connection to radar

   @param [out] *handle      Handle the to the radar object

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_radarhelper
*/
int radarHelper_close(RadarHandle_t *handle);

/**
   Helper function used to read in a user configuration JSON file and then set
   the radar configuration to match

   @param [in]  handle   Handle the to the radar object
   @param [in] *path     Path to JSON configuration file

   @return 0 on success, otherwise 1 on failure
*/
//int radarHelper_configFromFile(RadarHandle_t handle, char const *path);

/**
   Helper function used to read in a user configuration JSON file and then set
   the radar configuration to match

   @param [in]  handle   Handle the to the radar object
   @param [in] *path     Path to JSON configuration file
   @param [in] stage     Pre or post timing calibration register setup (1==pre calibration, 2==post calibration)

   @return 0 on success, otherwise 1 on failure
   @ingroup module_radarhelper
*/
int radarHelper_configFromFile(RadarHandle_t handle, char const *path, int stage);


/**
   Helper function used to save current user configuration to a JSON file

   @param [in]  handle   Handle the to the radar object
   @param [in] *path     Path to JSON configuration file

   @return 0 on success, otherwise 1 on failure
   @ingroup module_radarhelper
*/
int radarHelper_saveConfigToFile(RadarHandle_t handle, char *path);


/**
   Helper function used to get a radar frame consisting of the raw counters

   @param [in]   handle    Handle the to the radar object
   @param [out] *counters  Pointer to array which will store the counter values
   @param [in]   length    The number of counters in the array

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_radarhelper
*/
int radarHelper_getFrameRaw(RadarHandle_t handle, uint32_t *counters, int length);





/**
   Read a EEPROM memory on the beaglebone and populate the structure

   @param path Sysfs path to the eeprom to try to read

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_rh_eepromhelper
*/
int eepromHelper_readCapeEEPROM(char *path);


/**
   Get the name of the cape

   @param capeName Pre-allocated string buffer to get the capes name

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_rh_eepromhelper
*/
int eepromHelper_getCapeName(char *capeName);


/**
   Get the revision number for the cape

   @param revision Pre-allocated string buffer to get the revision name

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_rh_eepromhelper
*/
int eepromHelper_getCapeRevision(char *revision);


/**
   Get the board name for the cape

   @param boardName Pre-allocated string buffer to get the capes name

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_rh_eepromhelper
*/
int eepromHelper_getCapeBoardName(char *boardName);


/**
   Get the manufacturer name for the cape

   @param manufacturer Pre-allocated string buffer to get the capes name

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_rh_eepromhelper
*/
int eepromHelper_getCapeManufacturer(char *manufacturer);


/**
   Get the part number name for the cape

   @param partNumber Pre-allocated string buffer to get the part number

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_rh_eepromhelper
*/
int eepromHelper_getCapePartNumber(char *partNumber);


/**
   Get the serial number for the cape

   @param serialNumber Pre-allocated string buffer to get the serial number

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_rh_eepromhelper
*/
int eepromHelper_getCapeSerialNumber(char *serialNumber);


/**
   Get the version for the cape

   @param version Pre-allocated string buffer to get the version number

   @pre readCapeEEPROM() must be run before this function will return valid data

   @return NVA_SUCCESS on success, else an error code
   @ingroup module_rh_eepromhelper
*/
int eepromHelper_getCapeVersion(char *version);




/**
  @defgroup module_rh_eepromhelper RadarHelper Beaglebone EEPROM Read Functions
  Group of functions spacific to reading the EEPROM memory on a beaglebone cape

  @ingroup modules
*/


/**
  @defgroup module_radarhelper Radar Functions
  Group of functions specific to using the Novelda Radar Chips

  @ingroup modules
*/

#ifdef __cplusplus
}
#endif
#endif

