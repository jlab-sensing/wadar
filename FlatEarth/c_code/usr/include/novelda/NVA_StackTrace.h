/***********************************************************//**
 * \file NVA_StackTrace.h
 * \brief Implements a kind of stack trace functionality.
 *
 * \details
 * NVA_StackTrace is a way to implement stack trace functionality in RadarLib3.
 * A call to the \ref DEFINE_ERROR macro is done where an error occurres and the error code is returned to the calling function.
 * The calling function must check the error code (status) from the called function and call the \ref ADD_STACK_TRACE macro to add stack trace information.
 * A stack trace includes file name, function name and line number where the call to \ref DEFINE_ERROR or \ref ADD_STACK_TRACE are done.
 * That means the line number will be offset a couple of lines from where the error occurred, but it will locate the code executed when the error occurred.
 * The \ref NVA_GetErrorDescription will include the stack trace information in the error message string.\n
 * Stack tracing in Radarlib3 is implemented by including an \ref ErrorHandle_t pointer to an \ref ErrorInfo_t_ struct in the \ref Radar_t struct.
 * All objects referenced by the Radar_t struct also implements an ErrorHandle_t pointer and these pointer points to the same ErrorInfo_t struct.
 *
 * Example of error message including a stack trace:
 * \code
 * ERROR 0x104: Could not open connection to interface device - FTx232H - Check if serial number is correct
 *   at: FTx232H_ConnectDevice in FTx232H.c:379
 *   from: interfaceConnect in interface.c:201
 *   from: NVA_OpenRadar in Radarlib3.c:307
 * \endcode
*/
/*
 * Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
 * Novelda AS grant to Customer a nonexclusive and nontransferable license
 * to use this software code under terms regulated in a Software License Agreement
 * or similar agreement between Novelda AS and the Customer.
 * There is NO WARRANTY, to the extent permitted by law.
 ****************************************************************/

#ifndef STACKTRACE_H_
#define STACKTRACE_H_

#include <string.h>

#include "NVA_Defs.h"
#include "NVA_ErrorCodes.h"
#include "NVA_String.h"

/**
 * \brief Struct to hold info about the current error
 */
typedef struct ErrorInfo_t_
{
    int lastError;                 ///< ID of last error
#if defined(config_STACK_TRACE) && (config_STACK_TRACE != 0)
    char additionalErrorInfo[256]; ///< Additional information about the error where it occurred
    char stackTrace[2048];         ///< Function name, file name and line number for each stack level
#endif // #if defined(config_STACK_TRACE) && (config_STACK_TRACE != 0)
} ErrorInfo_t;

typedef ErrorInfo_t * ErrorHandle_t; ///< Pointer to an ErrorInfo struct

/**
 * @brief Macro used to set stack-trace and error status.
 *
 * @param HANDLE pointer to an ErrorHandle_t
 * @param STATUS error code
 * @param ADDITIONAL_INFO A string with optional information about the error. The string may be empty ("").
 */
#if defined(config_STACK_TRACE) && (config_STACK_TRACE != 0)
#define DEFINE_ERROR(HANDLE, STATUS, ADDITIONAL_INFO) \
{ \
    (HANDLE)->stackTrace[0] = 0; \
    ADD_STACK_TRACE((HANDLE)); \
    (HANDLE)->lastError = (STATUS); \
    strcpy((HANDLE)->additionalErrorInfo, (ADDITIONAL_INFO)); \
}
#else
#define DEFINE_ERROR(HANDLE, STATUS, ADDITIONAL_INFO) \
{ \
    (HANDLE)->lastError = (STATUS); \
}
#endif // #if defined(config_STACK_TRACE) && (config_STACK_TRACE != 0)

/**
 * @brief Macro used to add stack-trace.
 *
 * A call to \ref DEFINE_ERROR must be done before using this macro.
 *
 * @param HANDLE pointer to an ErrorHandle_t
 */
#if defined(config_STACK_TRACE) && (config_STACK_TRACE != 0)
#define ADD_STACK_TRACE(HANDLE) \
{ \
    char localBuffer[10];         \
    if (strlen((HANDLE)->stackTrace) > 0) \
      strcat((HANDLE)->stackTrace, "\r\n   from: "); \
    else \
      strcat((HANDLE)->stackTrace, "   at: "); \
    strcat((HANDLE)->stackTrace, __FUNCTION__); \
    strcat((HANDLE)->stackTrace, " in "); \
    strcat((HANDLE)->stackTrace, __FILE__); \
    strcat((HANDLE)->stackTrace, ":"); \
    NVA_itoa(__LINE__, localBuffer, 10); \
    strcat((HANDLE)->stackTrace, localBuffer); \
}
#else
#define ADD_STACK_TRACE(HANDLE)
#endif // #if defined(config_STACK_TRACE) && (config_STACK_TRACE != 0)

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Initialize the ErrorHandle_t and allocates memory for stack trace and error code.
 *
 * @param toERROR_HANDLE Pointer to the ErrorHandle_t that should be initialized.
 * @param fromERROR_HANDLE Pointer to an existing ErrorHandle_t that should be linked to the toERROR_HANDLE.
 *        If this parameter is NULL, a new ErrorHandle_t will be allocated and initialized.
 */
DLL_PUBLIC void CALL_CONV NVA_StackTraceInit(ErrorHandle_t * toERROR_HANDLE,
    ErrorHandle_t * fromERROR_HANDLE);

/**
 * @brief Deallocates memory that has been allocated by \ref NVA_StackTraceInit.
 *
 * @param errorHandle Pointer to the ErrorHandle_t that should be deallocated
 */
DLL_PUBLIC void CALL_CONV NVA_StackTraceFree(ErrorHandle_t * errorHandle);

/**
 * @brief Get the error message indicated by the errorNo and the full stack trace stored in the errorHandle.
 *
 * @param errorHandle Pointer to the ErrorHandle_t.
 * @param buffer A char array where the error message and stack trace are returned.
 * @param length Length of the char buffer.
 * @param errorNo The error code to get the error message.
 */
DLL_PUBLIC int CALL_CONV NVA_ErrorDescription(ErrorHandle_t errorHandle,
    char* buffer, int length, int errorNo);

#ifdef __cplusplus
}
#endif

#endif /* STACKTRACE_H_ */
