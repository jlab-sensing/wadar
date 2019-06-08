/**
   @file

   chipotleHelper.h

   Definitions for chipotle-cape specific functionality

   Copyright 2015 by FlatEarth, Inc

   @par Environment
   BeagleBone Black + chipotle cape

   @par Compiler
   GNU GCC

   @author Justin Hadella
*/
#ifndef chipotle_HELPER_h
#define chipotle_HELPER_h

#ifdef __cplusplus
extern "C"
{
#endif

// -----------------------------------------------------------------------------
// LED Enumeration
// -----------------------------------------------------------------------------

#ifndef LED_ENUM
#define LED_ENUM
/** Enumeration containing the module LED GPIO Addresses @ingroup module_chipotle */
typedef enum
{
  LED_Red    = 44, /*<< Red LED on the radar module board gpio1[12]   */ /* gpio1[12] == 1 * 32 + 12 == 44 */
  LED_Blue   = 26, /*<< Blue LED on the radar module board gpio0[26]  */ /* gpio0[26] == 0 * 32 + 26 == 26 */
  LED_Green0 = 46, /*<< Green LED on the radar module board gpio1[14] */ /* gpio1[14] == 1 * 32 + 14 == 46 */
  LED_Green1 = 65  /*<< Green LED on the radar module board gpio2[1]  */ /* gpio2[1]  == 2 * 32 +  1 == 65 */
} SalsaLED;
#endif

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

/**
  Helper function used to set a chipotle-cape LED

  @param [in] led    Enumeration representing the LED to set
  @param [in] value  Set to 0 to turn 'OFF' LED, 1 to turn 'ON' LED

  @return 0 on success, otherwise negative file-access return code
  @ingroup module_chipotle
*/
int chipotleHelper_setLED(SalsaLED led, int value);


/** Helper function to read the chipotle Cape Temperature sensor
  Example Usage:

  @verbatim
  float temperature;                    //Create a variable to hold the temperature
  chipotleHelper_readTemp(&temperature);          //Get the temperature
  printf("The temp sensor read: %f degC",temperature);  //Print the temperature
  @endverbatim

  @param temperature Pass by reference float of the temperature read on the cape
  @return Status code documented in error_codes.h
  @ingroup module_chipotle
*/
int chipotleHelper_readTemp(float *temperature);

/** Function to set the radar to use the External Clock Pin
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_chipotle
*/
int chipotleHelper_setExtMClk();

/** Function to set the radar to use the internal clock
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_chipotle
*/
int chipotleHelper_setIntMClk();

/** Function to activate the radars master clock line
  @note Not asserting this line will not affect register reads or writes
  but the radar frames will return as all 0's

  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_chipotle
*/
int chipotleHelper_enableMClk();

/** Function to deactivates the radars master clock line
  @note Asserting this line will not affect register reads or writes
  but the radar frames will return as all 0's

  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_chipotle
*/
int chipotleHelper_disableMClk();

/** Function to enable the CAN transceiver
  @note This will cause a noticeable increase in power consumption

  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_chipotle
*/
int chipotleHelper_enableCAN();

/** Function to disable the CAN transceiver
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_chipotle
*/
int chipotleHelper_disableCAN();

/** Function to enable the radar chip
  @note This will cause a noticeable increase in power consumption

  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_chipotle
*/
int chipotleHelper_enableRadar();

/** Function to disable the radar chip
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_chipotle
*/
int chipotleHelper_disableRadar();

/** Convert a A/D counter value to a raw voltage (Experimental!)
  @param counts A/D counter value
  @param pps Pulses per step used to collect the counter value
  @param iter Number of iterations use to collect the counter value
  @param dacStep DAC Step used to collect the counter value
  @return Double containing the voltage
  @ingroup module_chipotle
*/
double chipotleHelper_CounterToVolts(int counts, int pps, int iter, int dacStep);


/** Use a radar frame to get the temperature of the radar chip itself (Experimental!)
  This parameter uses the fact that the bandgap voltage of the receiver is viewable as the mean of
  the radar frame.  As such it should be able to be used to extract temperature at the Radar chip itself.

  @note THIS IS AN EXPERIMENTAL FUNCTION

  @param [out] temperature Measured temperature on the radar chip
  @param [in] radarFrame radar frame from the radar
  @param [in] pps Pulses per step used to collect the counter value
  @param [in] iter Number of iterations use to collect the counter value
  @param [in] dacStep DAC Step used to collect the counter value
  @return Status message
  @ingroup module_chipotle
*/
int chipotleHelper_readRadarTemp(float *temperature, int *radarFrame, int pps, int iter, int dacStep);

/** Get all the information from the EEPROM on the cape
  @bug This function segfaults and is not usable yet
  @ingroup module_chipotle
*/
int chipotleHelper_getCapeInfo(int capeNumber, char *moduleName, char *serialNumber, char *versionNumber);

/** Get a simple response for if the cape is a chipotle module based on its EEPROM
  @ingroup module_chipotle
*/
int chipotleHelper_ischipotle(int capeNumber);

/** Get the version number of the cape from the EEPROM
  @ingroup module_chipotle
*/
int chipotleHelper_capeVersion(int capeNumber, char *versionNumber);

/** Get the serial number of the cape from the EEPROM
  @ingroup module_chipotle
*/
int chipotleHelper_capeSerial(int capeNumber, char *serialNumber);

/** Get the name of the cape from the EEPROM
  @ingroup module_chipotle
*/
int chipotleHelper_capeName(int capeNumber, char *moduleName);


/**
  @defgroup module_chipotle Chipotle
  Group of functions specific to using the Chipotle switching cape

  @ingroup modules
*/


#ifdef __cplusplus
}
#endif
#endif // chipotle_HELPER_h

