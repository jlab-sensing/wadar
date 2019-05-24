/**
   @file

   AnchoHelper.h

   Definitions for Ancho-cape specific functionality

   @copyright 2015 by FlatEarth, Inc

   @par Environment
   BeagleBone Black + Ancho cape

   @par Compiler
   GNU GCC

   @author Justin Hadella & Raymond Weber
*/
#ifndef ANCHO_HELPER_h
#define ANCHO_HELPER_h

#ifdef __cplusplus
extern "C"
{
#endif

// -----------------------------------------------------------------------------
// LED Enumerationf
// -----------------------------------------------------------------------------

#ifndef LED_ENUM
#define LED_ENUM
/**
  SysFS pin numbers for the LED's on the Ancho cape
  @ingroup module_ancho
*/
typedef enum
{
  LED_Red    = 44, /* gpio1[12] == 1 * 32 + 12 == 44 */
  LED_Blue   = 26, /* gpio0[26] == 0 * 32 + 26 == 26 */
  LED_Green0 = 46, /* gpio1[14] == 1 * 32 + 14 == 46 */
  LED_Green1 = 65  /* gpio2[1]  == 2 * 32 +  1 == 65 */
} SalsaLED;
#endif

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

/**
  Helper function used to set a Ancho-cape LED

  @param [in] led    Enumeration representing the LED to set
  @param [in] value  Set to 0 to turn 'OFF' LED, 1 to turn 'ON' LED

  @return 0 on success, otherwise negative file-access return code
  @ingroup module_ancho
*/
int anchoHelper_setLED(SalsaLED led, int value);

/** Get a simple response for if the cape is a cayenne module based on its EEPROM
  @ingroup module_ancho
*/
int anchoHelper_isAncho(int capeNumber);

/** Helper function to read the Ancho Cape Temperature sensor
  @param temperature Float of the temperature read on the cape
  @return Status code documented in error_codes.h
  @ingroup module_ancho
*/
int anchoHelper_readTemp(float *temperature);

/** Function to set the radar to use the External Clock Pin
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_setExtMClk();

/** Function to set the radar to use the internal clock
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_setIntMClk();

/** Function to activate the radars master clock line
  @note Not asserting this line will not affect register reads or writes
  but the radar frames will return as all 0's
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_enableMClk();

/** Function to deactivates the radars master clock line
  @note Asserting this line will not affect register reads or writes
  but the radar frames will return as all 0's
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_disableMClk();

/** Function to enable the CAN transiever
  @note This will cause a noticible increase in power consumption
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_enableCAN();

/** Function to disable the CAN transiever
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_disableCAN();

/** Function to enable the radar chip
  @note This will cause a noticible increase in power consumption
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_enableRadar();

/** Function to disable the radar chip
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_disableRadar();

/** Function to disable the radar chip
  @param capeNumber Integer value from 1-4 as to which SPI select line to use.Wow
  @return SUCCESS or ERROR_WRITING_TO_PIN status codes
  @ingroup module_ancho
*/
int anchoHelper_selectCape(int capeNumber);

/** This reads the voltage that the radar chip is operating at
  @param voltage Pass by reference double to hold the set voltage
  @return SUCCESS status code
  @ingroup module_ancho
*/
int anchoHelper_readTxVoltage(double *voltage);

/** This allows the radar chip voltage to be modified
  @ingroup module_ancho
*/
int anchoHelper_writeTxVoltage(double voltage);

/** This allows the radar chip voltage to be modified, run once
  By default, the voltage is locked at its turn on setting
  @ingroup module_ancho
*/
int anchoHelper_enableVoltageControl();

/** Get the current 50-TP memory address
  @param address Pass by reference to hold the current address (last written) in memory that is being used
  @ingroup module_ancho
*/
int anchoHelper_voltageCurrOTPLocation(int *address);

/** Read a memory address in the one time programmable memory for the startup Tx voltage
  @param voltage Double pointer to hold the set voltage
  @param address Address to read in the memory (50 Addresses available)
  @ingroup module_ancho
*/
int anchoHelper_VoltageOTPRead(double *voltage, int address);

/** This function will lock the voltage and make it ignore future writeTxVoltage commands
  @ingroup module_ancho
*/
int anchoHelper_disableVoltageControl();


/**
  @defgroup module_ancho Ancho
  Group of functions specific to using the Ancho X2 Radar Cape
  @ingroup modules
*/



#ifdef __cplusplus
}
#endif
#endif // ANCHO_HELPER_h

