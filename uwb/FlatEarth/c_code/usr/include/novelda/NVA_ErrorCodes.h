/***********************************************************//**
* \file NVA_ErrorCodes.h
* \brief Define all error codes used in Radarlib3
*/
/*
* Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
* Novelda AS grant to Customer a nonexclusive and nontransferable license
* to use this software code under terms regulated in a Software License Agreement
* or similar agreement between Novelda AS and the Customer.
* There is NO WARRANTY, to the extent permitted by law.
****************************************************************/

#ifndef NVA_ERRORCODES_H_
#define NVA_ERRORCODES_H_

#include "NVA_Defs.h"

enum
{
  NVA_SUCCESS = 0,                      ///< No error occurred.
  NVA_ERROR = 1,                        ///< An error occurred. Normally a more specific NVA_xxx error code will be returned.
  NVA_BUFFER_TOO_SMALL,                 ///< Supplied buffer is too small - length of buffer is normally sent to a function by a separate parameter.
  NVA_STACK_TRACE_NOT_INITIALIZED,      ///< When using the stack trace functionality, the memory used must be initialized. This is normally done by calling an xxxInit function.
  NVA_NOT_IMPLEMENTED,                  ///< Function is not implemented.
  NVA_NO_VALID_SETUP_FOUND,             ///< Will be returned when calling NVA_RestoreVariablesFromFlash when there is no valid setup in Flash to restore.
  NVA_FILE_ERROR_OPEN,                  ///< Could not open file
  NVA_MALLOC_ERROR,                     ///< Could not allocate memory
  NVA_FUNCTION_ID_ERROR,                ///< No such function ID defined for this platform
  NVA_IF_ERROR = 0x0100,                ///< An error occurred in an interface module function. Normally a more specific NVA_IF_xxx error code will be returned, but if no specific error code are defined this error code will be returned.
  NVA_IF_COULD_NOT_INITIALIZE_HANDLE,   ///< Interface handle could not be initialized.
  NVA_IF_COULD_NOT_INITIALIZE_DEVICE,   ///< Interface device could not be initialized.
  NVA_IF_COULD_NOT_LIST_DEVICES,        ///< Could not get information about interface devices.
  NVA_IF_COULD_NOT_OPEN_CONNECTION,     ///< Could not connect to specified interface device. Could happen if interface identification (serial number etc.) is wrong, or interface device is already connected.
  NVA_IF_COULD_NOT_SET_IO_PINS,         ///< Could not set GPIO pins on the interface device.
  NVA_IF_COULD_NOT_READ_IO_PINS,        ///< Could not read GPIO pins on the interface device.
  NVA_IF_COULD_NOT_READ_STATUS,         ///< Could not read status from the interface device.
  NVA_IF_COULD_NOT_READ_DATA,           ///< Could not read data from the interface device.
  NVA_IF_COULD_NOT_WRITE_DATA,          ///< Could not write data to the interface device.
  NVA_IF_COULD_NOT_DISCONNECT_DEVICE,   ///< An error occurred when trying to disconnect the interface device.
  NVA_IF_NO_DEVICES_FOUND,              ///< No interface devices could be found. Some interface devices needs a couple of seconds to initialize after the are physically connected to the computer.
  NVA_IF_UNKNOWN,                       ///< Interface name unknown.
  NVA_IF_NOT_INITIALIZED,               ///< Trying to access an uninitialized interface device.
  NVA_IF_TIMEOUT,                       ///< Timeout when communicating with the interface device.
  NVA_IF_LOG_FILE_NOT_OPEN,             ///< Log file not open
  NVA_IF_LOG_FILE_ERROR_OPEN,           ///< Could not open log file
  NVA_MODULE_ERROR = 0x0200,            ///< An error occurred in a module module function. Normally a more specific NVA_MODULE_xxx error code will be returned, but if no specific error code are defined this error code will be returned.
  NVA_MODULE_UNKNOWN,                   ///< Trying to communicate with a radar module with a name not known.
  NVA_MODULE_NOT_INITIALIZED,           ///< Module handle and module must be initialized before calling this function.
  NVA_FLASH_ERROR = 0x0280,             ///< An error occurred when trying to access the Flash device on the radar module.
  NVA_FLASH_PROGRAM_ERROR,              ///< An error occurred when programming the Flash device on the radar module.
  NVA_FLASH_ERASE_ERROR,                ///< An error occurred when reading from the Flash device on the radar module.
  NVA_CHIP_ERROR = 0x0300,              ///< An error occurred in a chip module function. Normally a more specific NVA_CHIP_xxx error code will be returned, but if no specific error code are defined this error code will be returned.
  NVA_CHIP_UNKNOWN,                     ///< Trying to communicate with a chip with a name not known.
  NVA_CHIP_NOT_INITIALIZED,             ///< Chip handle and chip module must be initialized before calling this function.
  NVA_CHIP_COULD_NOT_WRITE_REGISTER,    ///< When writing a chip register, the value read back from the register was not the same as the value written to the register.
  NVA_TMEAS_ERROR,                      ///< An error occurred when running a timing measurement function.
  NVA_TMEAS_COULD_NOT_FIND_EDGE,        ///< Could not find a correct edge of the MCLK when running a timing measurement function.
  NVA_TMEAS_COULD_NOT_FIND_LEVEL,       ///< Could not find a correct level of the MCLK when running a timing measurement function.
  NVA_RADAR_ERROR = 0x0400,             ///< An error occurred in an interface module function. Normally a more specific NVA_IF_xxx error code will be returned, but if no specific error code are defined this error code will be returned.
  NVA_RADAR_NOT_INITIALIZED,            ///< Radar handle and radar module must be initialized before calling this function.
  NVA_VARSYSTEM_ERROR = 0x0500,         ///< An error occurred in an variable system module function. Normally a more specific NVA_VARSYSTEM_xxx error code will be returned, but if no specific error code are defined this error code will be returned.
  NVA_VARSYSTEM_NOT_VISIBLE,            ///< Trying to access a variable not visible.
  NVA_VARSYSTEM_UNKNOWN_VAR,            ///< Trying to access an unknown variable - check variable name.
  NVA_VARSYSTEM_UNKNOWN_ACTION,         ///< Trying to execute an unknown action - check action name.
  NVA_VARSYSTEM_VAR_READONLY,           ///< Trying to set a read only variable.
  NVA_VARSYSTEM_ILLEGAL_VALUE,          ///< Trying to set a variable with an illegal value.
  NVA_PROCESSING_MODULE_ERROR = 0x600,  ///< An error occurred in a processing module function. Normally a more specific NVA_PROCESSING_MODULE_xxx error code will be returned, but if no specific error code are defined this error code will be returned.
  NVA_PROCESSING_MODULE_UNKNOWN,        ///< Processing module unknown.
  NVA_PROCESSING_FILTER_NOT_FOUND,      ///< Processing filter not found
  NVA_LAST_ERROR                        ///< Indicating last error code - can be used to check if an error code is valid.
};


#ifdef __cplusplus
extern "C" {
#endif

/**
 * This function is called by NVA_GetErrorMessage. Users of Radarlib3 should not call this function directly, but use NVA_GetErrorDescription instead.
 */
DLL_PUBLIC char * CALL_CONV GetErrorMessage(int errorNo);

#ifdef __cplusplus
}
#endif

#endif /* NVA_ERRORCODES_H_ */
