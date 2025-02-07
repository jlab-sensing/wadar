/***********************************************************//**
* \file NVA_String.h
* \brief Utility functions for manipulating character strings not present in standard library.
*/
/*
* Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
* Novelda AS grant to Customer a nonexclusive and nontransferable license
* to use this software code under terms regulated in a Software License Agreement
* or similar agreement between Novelda AS and the Customer.
* There is NO WARRANTY, to the extent permitted by law.
****************************************************************/

#ifndef NVA_STRING_H_
#define NVA_STRING_H_

#include "NVA_Defs.h"
#include "NVA_Types.h"

#if defined (_MSC_VER)
  #include "msc_compat.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Concatenates a char to a char array (string)
 *
 * @param destination Destination string
 * @param source      Source character
 * @return A char pointer to the destination string.
 */
DLL_PUBLIC char * CALL_CONV NVA_chrcat(char * destination, const char source);

/**
 * @brief Concatenates and integer to a char array (string)
 *
 * @param destination Destination string
 * @param source      Source integer that will be converted to a string and concatenated
 * @param base        Number based used in conversion from int to string
 * @return A char pointer to the destination string.
 */
DLL_PUBLIC char * CALL_CONV NVA_intcat(char * destination, const int source, const int base);

/**
 * @brief Trim white spaces at the left of a string. The source is trimmed by copying to the destination.
 *
 * @param destination  Destination string - the trimmed string.
 * @param source       Source string - the string to left trim.
 * @return A char pointer to the destination string.
 */
DLL_PUBLIC char * CALL_CONV NVA_strltrim(char * destination, const char * source);

/**
 * @brief Split a delimited string into an array of strings with the individual parts.
 *
 * The parameter stringArray must point to char buffers with allocated memory space.
 *
 * @param stringList     The delimited string.
 * @param stringArray    An array with pointers to char strings where the individual parts are placed.
 * @param arrayLength    The length of the stringArray.
 * @param elementLength  The length of each element of stringArray.
 * @param delimiter      The delimiter character.
 * @return Status of the operation. NVA_SUCCESS if all is OK, @see NVA_ErrorCodes.h for more information about error codes.
 */
DLL_PUBLIC int CALL_CONV NVA_strsplit(const char * stringList, char * stringArray[], int arrayLength, int elementLength, char delimiter);

/**
 * \brief Convert an integer to a char string
 *
 * \param value  Integer value
 * \param result Char buffer where the result are returned
 * \param base   Number base used when converting integer to char string
 * \return Pointer to the buffer with result
 */
DLL_PUBLIC char * CALL_CONV NVA_itoa(int value, char* result, int base);


/**
 * \brief Convert a string buffer to an integer value (unsigned integer)
 * @param value Pointer to string buffer
 * @return integer value
 */
ValueInt_t NVA_atoi(const char * value);


#ifdef __cplusplus
}
#endif

#endif /* NVA_STRING_H_ */
