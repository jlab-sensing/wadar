/**
   @file Common/errorCodes.h

   Library containing salsaLib error condition codes

   @copyright Copyright 2015 by FlatEarth, Inc

   @par Environment
   Environment Independent

   @par Compiler
   Compiler Independent

   @author Raymond Weber
*/

#ifndef ERROR_CODES_h
#define ERROR_CODES_h


/** General Errors = 0-99
    Cluttermap = 100-199
    DSP = 200-299
    Doppler = 300-399
    Kalman = 400-499
    Localization = 500-599
    RadarSimple = 600-699
  CayenneHelper/AnchoHelper = 700-799
*/
typedef enum
{
  SUCCESS=0,                       /**< Function exited properly */
  GENERALFAILURE=1,                /**< Function failed, no details avalible */

  NEGATIVE_NOT_ALLOWED=2,          /**< Function failed, negative number forbidden */
  POSITIVE_NOT_ALLOWED=3,          /**< Function failed, positive number forbidden */
  NONPOSITIVE_NOT_ALLOWED=4,       /**< Function failed, values must be > 0 */
  MAXIMUM_VALUE_EXCEEDED=5,        /**< Function failed, values exceeds max allowed */
  VALUE_NOT_IN_RANGE=6,            /**< Function failed, param not in valid range */
  INVALID_ENUM=7,                  /**< Function failed, invalid enum param */
  INVALID_ACCESS=8,                /**< Function failed, invalid access */
  MALLOC_FAILED=9,                 /**< Attempt to malloc memory failed */

  NO_DEFAULT_LIBRARY=10,           /**< "There was no default library selected for this function during build"} */
  LIBRARY_NOT_BUILT_IN=11,         /**< "The library supporting this function wasn't compiled into the library"} */

  NOTIMPLIMENTED=99,               /**< Function is planned for a future release */
  LISTTOLONG=100,                  /**< Cluttermap is requested that exceeds the library maximum value */
  WRONGCLUTTERMAPROUTINE=101,      /**< Initializion is for a different cluttermap then was called */
  CLUTTERMAP_NOT_INITIALIZED=102,  /**< Cluttermap structure was created but not initialized before first use */
  WINDOWTYPENOTVALID=200,          /**< An unknown window type was requested */
  INVALID_RADAR_SETTINGS=201,      /**< Invalid PGEN, Radar Type, or Sampler */
  UNINITIALIZED_STRUCTURE=202,     /**< Unable to write to the structure as it was not initialized properly */
  FILENOTFOUND=601,                /**< JSON configuration file not found */
  JSONPARSEFAILED=602,             /**< JSON Parser failed to read file properly */
  INVALIDREGISTER=603,             /**< Invalid register to change */
  INVALIDREGISTERVALUE=604,        /**< Invalid register value requested */
  RADARNOTFOUND=605,               /**< Could not find the radar module */
  RADARERRORSETTINGREGISTER=606,
  LED_NOT_SET=700,                 /**< Could not set the requested LED state */
  SPI1_NOT_ENABLED=701,            /**< Could not do the SPI transaction as the SPI Port is not present in /dev */
  SPI2_NOT_ENABLED=702,            /**< Could not do the SPI transaction as the SPI Port is not present in /dev */
  SPI2_TEMP_TRANSFER_FAILED = 703, /**< SPI transaction failed, temp sensor did not respond */
  ERROR_WRITING_TO_PIN = 704,      /**< Could not adjust pin, not present in /sys/class/gpio or not in the correct mode */
  INVALID_CAPE_ID = 705,           /**< A request was made to switch to a invalid radar cape */
  INVALID_OTP_ACCESS_CODE = 706,   /**< Incorrect access code to program OTP memory */
  INVALID_MATRIX_SIZE = 901,       /**< The matrix size was invalid (likely zero or negative) */
  SINGULAR_MATRIX = 902,           /**< Warning: The resulting matrix is singular */
  UNKNOWN_ERROR = 999              /**< This is a EOL and should never occur */
} errorCodes;

#ifdef __cplusplus
extern "C"
{
#endif

//DLL_PUBLIC
/** This function gets an error message as a string to match an error code
  @param errorCode A numerical error code
  @return String with the error message
  @note This function currently resides in CayenneHelper so to use it, the program must be linked to that library
*/
char *SalsaLib_GetErrorDescription(int errorCode);

#ifdef __cplusplus
}
#endif
#endif


