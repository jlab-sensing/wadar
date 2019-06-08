/***********************************************************//**
* @file Radarlib3.h
* @brief Defines all API functions for Radarlib3
*/
/*
* Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
* Novelda AS grant to Customer a nonexclusive and nontransferable license
* to use this software code under terms regulated in a Software License Agreement
* or similar agreement between %Novelda AS and the Customer.
* There is NO WARRANTY, to the extent permitted by law.
****************************************************************/

#ifndef RADARLIB3_H_
#define RADARLIB3_H_

#include "Radarlib3_config.h"
#include "NVA_StackTrace.h"
#include "NVA_Defs.h"
#include "NVA_Types.h"
#if (config_NVA_RADAR_FRAME_LOGGING == 1)
  #include <stdio.h>
#endif

#define MAX_INTERFACES 5                   ///< Size of array used to hold information about interfaces
#define MAX_CONNECTION_STRING_LENGTH 96    ///< Maximum length of a connection string
#define STRING_UNKNOWN "Unknown"           ///< String used when Radar Module or Radar IC is unknown

#if defined(config_NVA_SPI_LOGGING) && (config_NVA_SPI_LOGGING == 1)
#define INTERFACE_NO_LOGGING 0       ///< Used with NVA_InterfaceLogging - turn off logging
#define INTERFACE_OPEN_LOG_FILE 1    ///< Used with NVA_InterfaceLogging - open log file
#define INTERFACE_CLOSE_LOG_FILE 2   ///< Used with NVA_InterfaceLogging - close log file
#define INTERFACE_LOG_TO_FILE 3      ///< Used with NVA_InterfaceLogging - turn on logging
#endif

/**
 * Define ID for SPI channel available on each radar module
 * The IDs are used by the interface module in use to map a SPI ID to an actual SPI channel
 */
enum NVA_SPI_ID_
{
  SPI_CHANNEL_ID_RADAR = 0,
  SPI_CHANNEL_ID_FLASH
};

/**
 * Not implemented
 */
typedef enum {
  INTERFACE_SPI_POS_CLOCK_OUT = 0x0000, ///< Clock out SPI data on positive clock edge (default)
  INTERFACE_SPI_MSB_FIRST     = 0x0000, ///< Clock out SPI data as most significant bit first (default)
  INTERFACE_SPI_NEG_CLOCK_OUT = 0x0100, ///< Clock out SPI data on negative clock edge
  INTERFACE_SPI_LSB_FIRST     = 0x0800  ///< Clock out SPI data most significant bit first (not implemented yet)
} INTERFACE_SPI_Modes_t;

/**
 * Different modes Radar Frame logging can be set to
 */
enum _RADAR_FRAME_LOGGING_Mode{
  RADAR_FRAME_LOGGING_NONE =        0x0000, ///< No logging
  RADAR_FRAME_LOGGING_RAW =         0x0001, ///< Log raw frames
  RADAR_FRAME_LOGGING_NORMALIZED =  0x0002, ///< Log normalized frames
  RADAR_FRAME_LOGGING_CSV =         0x0010, ///< Write as CSV file
  RADAR_FRAME_LOGGING_BINARY =      0x0020  ///< Write as binary file
};

/**
 * ChipHandle_t is a pointer to a Chip_t_ struct with all information about a Novelda chip
 * Chip_t_ is defined in chip/chip.h
 */
typedef struct Chip_t_ * ChipHandle_t;

/**
 * ModuleHandle_t is a pointer to a Module_t_ struct with all information about a Novelda module
 * Module_t_ is defined in module/module.h
 */
typedef struct Module_t_ * ModuleHandle_t;

/**
 * InterfaceHandle_t is a pointer to an Interface_t_ struct with all information about a Novelda interface device
 * Interface_t_ is defined in interface/interface.h
 */
typedef struct Interface_t_ * InterfaceHandle_t;

/**
 * ProcessingHandle_t is a pointer to a Processing_t_ struct with all information about a Novelda processing module
 * Processing_t_ is defined in processing/processing.h
 */
typedef struct ProcessingModule_t_ * ProcessingModuleHandle_t;

/**
 * ConnectionInfo_t is a pointer to an ConnectionInfo_t_ struct with information about a connection
 * such as interface name, module name, chip name etc.
 */
typedef struct ConnectionInfo_t_ * ConnectionInfo_t;

/**
 * Radar_t holds all information about the current radar.
 */
typedef struct Radar_t_ Radar_t;

/**
 * RadarHandle_t is a pointer to an Radar_t struct with all information about a Novelda radar object
 */
typedef Radar_t * RadarHandle_t;

/**
 * Radar_t_ struct holds information and pointers to all necessary objects a radar object needs
 */
struct Radar_t_
{
    char connectionString[MAX_CONNECTION_STRING_LENGTH];         ///< Holds a string with information/identification of the connected device
    ConnectionInfo_t connectionInfo;    ///< Struct with names and info about connected device
    InterfaceHandle_t interfaceHandle;  ///< Pointer to the interface device that is connected
    int (CALL_CONV *interfaceRegisterCallbacks)(RadarHandle_t radarHandle); ///< Pointer to a function for registering call-back functions used on some platforms.
    ChipHandle_t chipHandle;            ///< Pointer to the chip that is connected
    ModuleHandle_t moduleHandle;        ///< Pointer to the module that is connected
    ErrorHandle_t errorHandle;          ///< Pointer to an ErrorHandle_t holding information about last error and stack trace
    int allInitialized;                 ///< Flag set to 1 when all sub modules are initialized. This flag is also linked into some of the submodules.
#if (config_NVA_RADAR_FRAME_LOGGING == 1)
    void * logFile;                     ///< Pointer to file handle of radar frame log file
    int logMode;                        ///< Log mode for radar frame logging
    int logPrecision;                   ///< Precision of values written to radar frame log
#endif
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NVA_InitHandle initializes the RadarHandle
 *
 * This function allocates memory to the Radar_t struct pointed to by the radarHandle.
 * Must be called before any other function that needs the radarHandle as parameter.
 * To free the memory allocated, a call to @ref NVA_FreeHandle must be done.
 *
 * @param[in,out] radarHandle Pointer to a RadarHandle_t variable.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 *
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_InitHandle(RadarHandle_t * radarHandle);

/**
 * @brief Free memory allocated by @ref NVA_InitHandle
 *
 * @param[in,out] radarHandle Pointer to a RadarHandle_t variable.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 *
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_FreeHandle(RadarHandle_t * radarHandle);

/**
 * @brief Get the version string of this library.
 *
 * @param[out] stringBuffer Pointer to a string buffer. Library version string is returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 *
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_Version(char * stringBuffer, int bufferLength);


/**
 * @brief Get the name string of this library.
 *
 * @param[out] stringBuffer Pointer to a string buffer. Library name string is returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 *
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_Name(char * stringBuffer, int bufferLength);

/**
 * @brief Get the error description and the stack traced of functions involved.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle).
 *                    If radarHandle is not initialized, it is possible to set this argument to NULL and pass an error code in the "errorCode" argument.
 * @param[out] stringBuffer Pointer to a string buffer. The error message and stack trace is returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @param[in] errorCode    If radarHandle is not initialized, it is possible to pass an error code to get the error message for this error code.
 *                     This argument is only used if radarHandle is NULL.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup error_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_GetErrorDescription(RadarHandle_t radarHandle, char * stringBuffer, int bufferLength, int errorCode);

/**
 * @brief Get a list of Radar Modules connected to the system.
 *
 * The string returned contains connection strings delimited with \ref CONNECTION_STRING_DELIMITER2.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] stringBuffer Pointer to string buffer. The list of connection string for connected modules are returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ListRadarModules(RadarHandle_t radarHandle, char * stringBuffer, int bufferLength);

/**
 * @brief List supported Radar ICs
 *
 * Delimiter string is defined by \ref LIST_DELIMITER.
 *
 * @param[out] stringBuffer Pointer to string buffer. The list of supported Radar ICs supported are returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ListSupportedChips(char * stringBuffer, int bufferLength);

/**
 * @brief List supported Radar Modules
 *
 * Delimiter string is defined by \ref LIST_DELIMITER.
 *
 * @param[out] stringBuffer Pointer to string buffer. The list of supported Radar Modules are returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ListSupportedModules(char * stringBuffer, int bufferLength);

/**
 * @brief List supported Interfaces (IO modules)
 *
 * Delimiter string is defined by \ref LIST_DELIMITER.
 *
 * @param stringBuffer Pointer to string buffer. The list of supported Interfaces are returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ListSupportedInterfaces(char * stringBuffer, int bufferLength);

/**
 * @brief Open a connection to a Radar Module and initialize the interface, radar module and radar chip.
 *
 * The connection string parameter consist of the following parts delimited by \ref CONNECTION_STRING_DELIMITER1\n
 *   \li Interface type name
 *   \li Interface identification
 *   \li Radar Module name
 *   \li Radar IC name
 *
 * A list of available Radar Modules connected to a system, can be obtained by calling \ref NVA_ListRadarModules.
 *
 * A typical connection string for connecting to a NVA-R641 Radar Module via a FTDI USB-SPI cable is:
 *
 *      "FTx232H!Serial No: (FTUP3KET)!NVA-R641!NVA6100"
 *
 * This function will also set all variables to their default values.
 *
 * <i><b>When running under a Windows Operating system, NVA_OpenRadar will alter the resolution of the system Timer
 * by calling NtSetTimerResolution in order to get better performance.</b></i>
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] connectionString A pointer to a char buffer with a connection string.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_OpenRadar(RadarHandle_t radarHandle, const char * connectionString);

/**
 * @brief Close the connection to a Radar Module.
 *
 * Will close a connection opened by \ref NVA_OpenRadar.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_CloseRadar(RadarHandle_t radarHandle);

/**
 * @brief Power off the Radar Module.
 *
 * Will power off the Radar Module. \ref NVA_PowerOnRadar must be used to power on and initialize the Radar Module again.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_PowerOffRadar(RadarHandle_t radarHandle);

/**
 * @brief Power on the Radar Module.
 *
 * Will power on a Radar Module that has been powered off by using \ref NVA_PowerOffRadar.\n
 * <b>All variables will be initialized to their default values when running this function!</b>
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup com_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_PowerOnRadar(RadarHandle_t radarHandle);

/**
 * @brief List all variables related to connected Radar IC, Radar Module, interface and internal Radarlib3 variables.
 *
 * Delimiter string is defined by \ref LIST_DELIMITER.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] stringBuffer Pointer to string buffer. The list of all variables are returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ListVariables(RadarHandle_t radarHandle, char * stringBuffer, int bufferLength);

/**
 * @brief List all actions related to connected Radar IC, Radar Module, interface and internal Radarlib3 variables.
 *
 * Delimiter string is defined by \ref LIST_DELIMITER.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] stringBuffer Pointer to string buffer. The list of all actions are returned in this buffer.
 * @param[in] bufferLength Length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ListActions(RadarHandle_t radarHandle, char * stringBuffer, int bufferLength);

/**
 * @brief Set a named variables value.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name
 * @param[in] value
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarSetValue_ByName(RadarHandle_t radarHandle, const char *name, Value_t value);

/**
 * @brief Set a named integer variables value.
 *
 * It is possible to reset a variable to its defualt value, by setting the integer value to 0x7fffffff.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable.
 * @param[in] value Value of variable.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarSetIntValue_ByName(RadarHandle_t radarHandle, const char *name, ValueInt_t value);

/**
 * @brief Set a named floating point variables value.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable.
 * @param[in] value Value of variable.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarSetFloatValue_ByName(RadarHandle_t radarHandle, const char *name, ValueFloat_t value);

/**
 * @brief Set the value of a named variable, parameter is a floating point regardless of variable type.
 *
 * \ref ValueFloat_t is normally defined as double.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[in] value Value of variable.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarSetValueAsFloat_ByName(RadarHandle_t radarHandle, const char * name, ValueFloat_t value);

/**
 * @brief Set a named variables value.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable.
 * @param[in] value Value of variable as a string.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarSetValue_ByString(RadarHandle_t radarHandle, const char * name, char * value);

/**
 * @brief Get the value of a named variable.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] value A pointer to the variable where the value of the variable is returned.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetValue_ByName(RadarHandle_t radarHandle, const char *name, Value_t * value);

/**
 * @brief Get the value of a named integer value.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] value A pointer to the variable where the value of the variable is returned.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetIntValue_ByName(RadarHandle_t radarHandle, const char * name, ValueInt_t * value);

/**
 * @brief Get the value of a named floating point variable.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] value A pointer to the variable where the value of the variable is returned.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetFloatValue_ByName(RadarHandle_t radarHandle, const char * name, ValueFloat_t * value);

/**
 * @brief Get the value of a named variable and return the value as a floating point regardless of variable type.
 *
 * \ref ValueFloat_t is normally defined as double.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] value A pointer to the variable where the value of the variable is returned.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetValueAsFloat_ByName(RadarHandle_t radarHandle, const char * name, ValueFloat_t * value);

/**
 * @brief Get the valid symbol values a named enum variable can be set to.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] items  An array of integer where the valid symbol values are returned.
 * @param[in,out] length The length of the items array.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetEnumsItems_ByName(RadarHandle_t radarHandle, const char * name, int * items, int * length);

/**
 * @brief Get the value of a named enum variable.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] value A pointer to the variable where the value of the variable is returned.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetEnumValue_ByName(RadarHandle_t radarHandle, const char * name, ValueInt_t * value);


/**
 * @brief Get the type of a named variable.
 *
 * This can be used to choose which method to use to set or get the variables value.
 * Use \ref NVA_VarTypeIsFloat, \ref NVA_VarTypeIsInt or \ref NVA_VarTypeIsEnum with the returned type as parameter
 * to decode the value of type.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] type  A pointer to an int where the type of the variable will be returned.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetType_ByName(RadarHandle_t radarHandle, const char *name, int * type);

/**
 * @brief Check if the variable type is a floating point type.
 *
 * @param[in] type The type value to check.
 * @return 1 if type is floating point, otherwise return 0.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarTypeIsFloat(int type);

/**
 * @brief Check if the variable type is an integer type.
 *
 * @param[in] type The type value to check.
 * @return 1 if type is integer, otherwise return 0.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarTypeIsInt(int type);

/**
 * @brief Check if the variable type is an enum type.
 *
 * @param[in] type The type value to check.
 * @return 1 if type is enum type, otherwise return 0.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarTypeIsEnum(int type);

/**
 * @brief Get various properties of an integer or a enum variable.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] minValue    The valid maximum value this variable can have.
 * @param[out] maxValue    The valid minimum value this variable can have.
 * @param[out] isWriteable 1 if variable can be modified, otherwise 0.
 * @param[out] isAuto      1 if variable is an auto value - this is normally the default value, but can also be a value dependant on another variable.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetIntProperties_ByName(RadarHandle_t radarHandle, const char * name, ValueInt_t * minValue, ValueInt_t * maxValue, int * isWriteable, int * isAuto);

/**
 * @brief Get various properties of a floating point variable.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[out] minValue    The valid maximum value this variable can have.
 * @param[out] maxValue    The valid minimum value this variable can have.
 * @param[out] isWriteable 1 if variable can be modified, otherwise 0.
 * @param[out] isAuto      1 if variable is an auto value - this is normally the default value, but can also be a value dependant on another variable.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetFloatProperties_ByName(RadarHandle_t radarHandle, const char * name, ValueFloat_t * minValue, ValueFloat_t * maxValue, int * isWriteable, int * isAuto);


/**
 * @brief Get the corresponding symbol string to the specified symbol value.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of variable
 * @param[in] symbol       The symbol value
 * @param[out] stringBuffer A pointer to a string buffer where the symbol string are returned.
 * @param[in] bufferLength The length of the stringBuffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarGetSymbolString(RadarHandle_t radarHandle, const char * name, Sym_t symbol, char * stringBuffer, int bufferLength);

/**
 * @brief Execute the named action.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] name  Pointer to a null terminated string buffer with name of the action to execute.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ExecuteAction_ByName(RadarHandle_t radarHandle, const char * name);

/**
 * @brief Get the content of SamplerBuffer on the Radar IC
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] counters A pointer to an array of counters where the sampled frame will be returned.
 * @param[in] length   Length of the counters array.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_GetSamplerBuffer(RadarHandle_t radarHandle, uint32_t * counters, int length);

/**
 * @brief Capture a radar frame and get the result. The result is the raw counter values.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] counters A pointer to an array of counters where the sampled frame will be returned.
 * @param[in] length   Length of the counters array.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup control_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_GetFrameRaw(RadarHandle_t radarHandle, uint32_t * counters, int length);

/**
 * @brief Capture a radar frame and get the result. The result is normalized between 0.0 and 100.0.
 *
 * This function call \ref NVA_GetFrameNormalizedDouble.
 *
 * \note When using this version of the function, the macro config_NVA_VALUE_FLOAT_SIZE in include/Radarlib3_config.h must be set to 32.
 *
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] counters A pointer to an array of counters (double) where the sampled frame will be returned.
 * @param[in] length   Length of the counters array.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup control_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_GetFrameNormalized(RadarHandle_t radarHandle, double * counters, int length);

/**
 * @brief Capture a radar frame and get the result. The result is normalized between 0.0 and 100.0.
 *
 * \note When using this version of the function, the macro config_NVA_VALUE_FLOAT_SIZE in include/Radarlib3_config.h must be set to 32.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] counters A pointer to an array of counters (double) where the sampled frame will be returned.
 * @param[in] length   Length of the counters array.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup control_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_GetFrameNormalizedDouble(RadarHandle_t radarHandle, double * counters, int length);

/**
 * @brief Capture a radar frame and get the result. The result is normalized between 0.0 and 100.0.
 *
 * \note When using this version of the function, the macro config_NVA_VALUE_FLOAT_SIZE in include/Radarlib3_config.h must be set to 64.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] counters A pointer to an array of counters (float) where the sampled frame will be returned.
 * @param[in] length   Length of the counters array.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup control_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_GetFrameNormalizedFloat(RadarHandle_t radarHandle, float * counters, int length);


/**
 * @brief Calculate the number of columns and rows of a CDF with the current settings of variables.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] cols  A pointer to an integer where the number of columns will be returned.
 * @param[out] rows  A pointer to an integer where the number of rows will be returned.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup control_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_CalculateCDF_Size(RadarHandle_t radarHandle, int * cols, int * rows);

/**
 * @brief Capture a CDF with the current settings of variables.
 *
 * Use \ref NVA_CalculateCDF_Size to get the size of the CDF. The counter array must be the size of columns * rows.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] counters An array of counters where the result of the CDF will be returned.
 * @param[in] length   Length of the counters array.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup control_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_GetCDF(RadarHandle_t radarHandle, uint32_t * counters, int length);


/**
 * @brief Read the information about the Radar Module connected.
 *
 * The information is stored in the OTP part of the Flash on the Radar Module.
 * The available length of the OTP register is 64 byte of user programmable area, a factory programmed unique is available in the next 64 byte.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] buffer      A pointer to a buffer where the information is returned.
 * @param[in] bufferLength The length of the buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ModuleReadBoardInfo(RadarHandle_t radarHandle, unsigned char * buffer, int bufferLength);

/**
 * @brief Program the OTP part of the Flash on the Radar Module connected.
 *
 * The information is stored in the OTP part of the Flash on the Radar Module.
 * The available length of the OTP register is 64 byte.
 *
 * @param[in] radarHandle   A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] buffer        A pointer to a buffer with the information to be programmed.
 * @param[in] bufferLength  The length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ModuleProgramBoardInfo(RadarHandle_t radarHandle, unsigned char * buffer, int bufferLength);

/**
 * @brief Read a memory area of the Flash on the Radar Module.
 *
 * @param[in] radarHandle  A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] address      Address to start reading from.
 * @param[out] buffer       A pointer to a buffer where the memory buffer read are returned.
 * @param[in] bufferLength The length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ModuleReadFlashMemory(RadarHandle_t radarHandle, int address, unsigned char * buffer, int bufferLength);

/**
 * @brief Program/write a memory area of the Flash on the Radar Module.
 *
 * Address must be aligned to a 4kB segment. The 4kB segment will be erased before writing the data specified.
 * It is not possible to write more than 4kB of data.
 *
 * @param[in] radarHandle  A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] address      Address to start reading from.
 * @param[in] buffer       A pointer to a buffer with the data to program to Flash.
 * @param[in] bufferLength The length of buffer.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_ModuleWriteFlashMemory(RadarHandle_t radarHandle, int address, unsigned char * buffer, int bufferLength);

/**
 * @brief Save variable values to the Flash module on a radar module.
 *
 * The setup can be restored from Flash by \ref NVA_VarsLoadFromFlash.
 *
 * A setup is not necessarily compatible between revisions of Radarlib3.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarsSaveToFlash(RadarHandle_t radarHandle);

/**
 * @brief Restore variable values from values already stored in the Flash module on a radar module.
 *
 * A valid setup must already be stored to Flash by \ref NVA_VarsSaveToFlash before this function can be called.
 *
 * A setup is not necessarily compatible between revisions of Radarlib3.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarsLoadFromFlash(RadarHandle_t radarHandle);

/**
 * @brief Save variable values to a file.
 *
 * The setup can be restored from a file by \ref NVA_VarsLoadFromFile.
 *
 * A setup is not necessarily compatible between revisions of Radarlib3.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] fileName Name of the file
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarsSaveToFile(RadarHandle_t radarHandle, char * fileName);

/**
 * @brief Restore variable values from values already stored in a file.
 *
 * A valid setup must already be stored to file by \ref NVA_VarsSaveToFile before this function can be called.
 *
 * A setup is not necessarily compatible between revisions of Radarlib3.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] fileName Name of the file
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarsLoadFromFile(RadarHandle_t radarHandle, char * fileName);

/**
 * @brief Reset all variables to their default values.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup var_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_VarsResetAllToDefault(RadarHandle_t radarHandle);


/**
 * @brief Get the value of a register on the Radar Chip
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] address Register address to read
 * @param[in] length Length of register in number of bytes
 * @param[out] value Pointer to a ValueInt_t variable where value of register can be returned
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_RegisterRead(RadarHandle_t radarHandle, int address, int length, ValueInt_t * value);
/**
 * @brief Set the value of a register on the Radar Chip
 *
 * \note Writing a new value to a register will not update the corresponding variable(s)!
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] address Register address to set
 * @param[in] length Length of register in number of bytes
 * @param[in] value Value to set register to
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_RegisterWrite(RadarHandle_t radarHandle, int address, int length, ValueInt_t value);

/**
 * @brief Set the value of an IO pin on the connected interface
 *
 * \note Not all interface modules has IO pins. Check pin numbers available for each interface.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] pinNo       Pin number to set - start from 0.
 * @param[in] value       Value of IO pin - 0 is low, 1 is high.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_IO_SetValue(RadarHandle_t radarHandle, int pinNo, int value);

/**
 * @brief Set the direction and initial value of an IO pin on the connected interface
 *
 * \note Not all interface modules has IO pins. Check pin numbers available for each interface.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] pinNo       Pin number to set - start from 0.
 * @param[in] direction   Direction of IO pin - 0 is input, 1 is output.
 * @param[in] value       Value of IO pin - 0 is low, 1 is high.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_IO_SetDirection(RadarHandle_t radarHandle, int pinNo, int direction, int value);

/**
 * @brief Get the value of an IO pin on the connected interface
 *
 * \note Not all interface modules has IO pins. Check pin numbers available for each interface.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] pinNo       Pin number to set - start from 0.
 * @param[out] value       Pointer to an int where value of IO pin is returned - 0 is low, 1 is high.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_IO_GetValue(RadarHandle_t radarHandle, int pinNo, int * value);

/**
 * @brief Generic function for send/receive data bytes over SPI port on the connected interface
 *
 * SlaveSelect/ChipSelect will only go low while data are transferred over SPI.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] chipSelect  Pin number acting as slave-select/chip select for the SPI device
 * @param[in] inBuffer    Pointer to a buffer with data to transmit
 * @param[in] inLength    Length of data to transmit (in bytes)
 * @param[out] outBuffer   Pointer to a buffer where received data are returned.
 * @param[in] outLength   Length of outBuffer to transmit (in bytes)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_SPI_SendReceive(RadarHandle_t radarHandle, int chipSelect, uint8_t * inBuffer, int inLength, uint8_t * outBuffer, int outLength);

/**
 * @brief Generic function for send/receive data bits over SPI port on the connected interface
 *
 * SlaveSelect/ChipSelect will only go low while data are transferred over SPI.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] chipSelect  Pin number acting as slave-select/chip select for the SPI device
 * @param[in] inBuffer    Pointer to a buffer with data to transmit
 * @param[in] inLength    Length of data to transmit (in bits)
 * @param[out] outBuffer  Pointer to a buffer where received data are returned.
 * @param[in] outLength   Length of outBuffer to transmit (in bits)
 * @param[in] mode        Mode according to \ref INTERFACE_SPI_Modes_t.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_SPI_SendReceiveBits(RadarHandle_t radarHandle, int chipSelect, uint8_t * inBuffer, int inLength, uint8_t * outBuffer, int outLength, int mode);


/**
 * @brief Get the character used as delimiter between the parts of a connection string
 *
 * @return delimiter character
 * @ingroup misc_funcs
 */
DLL_PUBLIC char CALL_CONV NVA_ConnectionStringDelimiter1(void);

/**
 * @brief Get the character used as delimiter between connection strings where more than one connection string are found.
 *
 * @return delimiter character
 * @ingroup misc_funcs
 */
DLL_PUBLIC char CALL_CONV NVA_ConnectionStringDelimiter2(void);

/**
 * @brief Get the character used as list delimiter in Radarlib3.
 *
 * Function returning list of variables names, action names etc. uses this delimiter between list items.
 *
 * @return delimiter character
 * @ingroup misc_funcs
 */
DLL_PUBLIC char CALL_CONV NVA_ListDelimiter(void);


#if defined(config_NVA_SPI_LOGGING) && (config_NVA_SPI_LOGGING == 1)
/**
 * @brief Control logging facility in interface module.
 *
 * This function will do different actions depending on value of mode argument.\n
 * Possible mode values:
 * \li INTERFACE_NO_LOGGING 0
 * \li INTERFACE_OPEN_LOG_FILE 1
 * \li INTERFACE_CLOSE_LOG_FILE 2
 * \li INTERFACE_LOG_TO_FILE 3
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] mode        Control mode for logging facility
 * @param[in] fileName    File name of log file (only necessary when opening a log file, else NULL)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_InterfaceLogging(RadarHandle_t radarHandle, int mode, char * fileName);

/**
 * @brief Insert a comment in interface log file.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] stringBuffer      A pointer to the comment string.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_InterfaceLogComment(RadarHandle_t radarHandle, char * stringBuffer);
#endif // #if defined(config_NVA_SPI_LOGGING) && (config_NVA_SPI_LOGGING == 1)

/**
 * @brief Get detailed results of timing measurements.
 *
 * Elements stored in the resultArray for NVA6000 and NVA6100:
 * - timingMeasurementValue: The value of timingMeasurementValue at the time the measurement was done
 * - timingMeasurementRawValue: The "raw" value of timingMeasurementValue at the time the measurement was done
 * - PRF: Pulse Repetition Frequency
 * - delayInCoarseMeasured: Flag indicating that all coarse tune elements are measured
 * - delayInMediumMeasured: Flag indicating that all medium tune elements are measured
 * - delayInCoarseMT[352]: Array with number of medium tune elements in each coarse tune element
 * - delayInCoarseFT[352]: Array with number of fine tune elements (inn addition to medium) in each coarse tune element.
 * - delayInMediumFT[64]: Array with number of fine tune elements in each medium tune element
 * - coarseEdge[60]: Array with the number of the coarse tune element where a clock edge is found
 * - mediumEdge[60]: Array with the number of the medium tune element where a clock edge is found
 * - fineEdge[60]: Array with the number of the fine tune element where a clock edge is found.
 * - lastEdge: Indicates how many clock edges that was found in the delay chain.
 * - samplerCoarse: Number of coarse tune elements in the sampler
 * - samplerMedium: Number of additional medium tune elements in the sampler
 * - samplerFine: Number of additional fine tune elements in the sampler
 * - delayCoarseTune: Average length of a coarse tune delay element (in seconds) - use delayLengthCoarseTune instead
 * - delayMediumTune: Average length of a medium tune delay element (in seconds) - use delayLengthMediumTune instead
 * - delayFineTune: Average length of a fine tune delay element (in seconds)
 * - samplesPerSecond: Samples per second (sampling rate)
 * - mediumInCoarseAverage: Average number of medium tune delay elements in one coarse tune delay element
 * - fineInMediumAverage: Average number of fine tune delay elements in one medium tune delay element
 * - delayLengthCoarseTune[352]: Array with the calculated length in time (seconds) for each coarse tune element
 * - delayLengthMediumTune[64]: Array with the calculated length in time (seconds) for each medium tune element
 *
 * Elements stored in the resultArray for NVA620x:
 * - timingMeasurementValue: The value of timingMeasurementValue at the time the measurement was done
 * - timingMeasurementRawValue: The "raw" value of timingMeasurementValue at the time the measurement was done
 * - PRF: Pulse Repetition Frequency
 * - delayInCoarseMeasured: Flag indicating that all coarse tune elements are measured
 * - delayInMediumMeasured: Flag indicating that all medium tune elements are measured
 * - delayInFineMeasured: Flag indicating that all medium tune elements are measured
 * - delayCoarseTune: Average length of a coarse tune delay element (in seconds) - use delayLengthCoarseTune instead
 * - delayMediumTune: Average length of a medium tune delay element (in seconds) - use delayLengthMediumTune instead
 * - delayFineTune: Average length of a fine tune delay element (in seconds)
 * - samplesPerSecond: Samples per second (sampling rate)
 * - mediumInCoarseAverage: Average number of medium tune delay elements in one coarse tune delay element
 * - fineInMediumAverage: Average number of fine tune delay elements in one medium tune delay element
 * - delayLengthCoarseTune[352]: Array with the calculated length in time (seconds) for each coarse tune element
 * - delayLengthMediumTune[64]: Array with the calculated length in time (seconds) for each medium tune element*
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[out] resultArray A pointer to an array of double.
 * @param[in] length      A pointer to an integer with length of resultArray.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_TimingMeasurementResults(RadarHandle_t radarHandle, double * resultArray, int length);

/**
 * @brief Save the current timing measurement data to the Flash module on the Radar Module board.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] index       An address index determining the address block to save the data to. Must be between 0 and 14.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_TimingMeasurementSaveDataToFlash(RadarHandle_t radarHandle, int index);

/**
 * @brief Load the timing measurement data from the Flash module on the Radar Module board.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] index       An address index determining the address block to load the data from. Must be between 0 and 14.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_TimingMeasurementLoadDataFromFlash(RadarHandle_t radarHandle, int index);

#if (config_NVA_RADAR_FRAME_LOGGING == 1)
/**
 * @brief Start logging of radar frames values to file.
 *
 * Logging mode can be the following modes:
 *  - CSV or binary - 0x0010 is CSV, 0x0020 is binary
 *  - Raw or normalized values - 0x0001 is raw, 0x0002 is normalized
 *
 * See also \ref _RADAR_FRAME_LOGGING_Mode
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @param[in] fileName    File name for log file.
 * @param[in] mode        Mode of logging.
 * @param[in] precision   number of decimals in CSV or number of bytes (4 or 8) in binary format.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_FrameLoggingStart(RadarHandle_t radarHandle, const char * fileName, int mode, int precision);

/**
 * @brief Stop logging of radar frames values to file.
 *
 * @param[in] radarHandle A pointer to an initialized RadarHandle_t (radarHandle must be initialized by calling \ref NVA_InitHandle)
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 * @ingroup misc_funcs
 */
DLL_PUBLIC int CALL_CONV NVA_FrameLoggingStop(RadarHandle_t radarHandle);
#endif






#ifdef __cplusplus
}
#endif



#endif /* RADARLIB3_H_ */
