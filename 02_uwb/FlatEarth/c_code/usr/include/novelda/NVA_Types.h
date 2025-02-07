/*****************************************************************************/
/**
 * \file NVA_Types.h
 * \brief Definitions of variable types used and prototypes for functions used to convert between types.
*/
/*
 * Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
 * Novelda AS grant to Customer a nonexclusive and nontransferable license
 * to use this software code under terms regulated in a Software License Agreement
 * or similar agreement between Novelda AS and the Customer.
 * There is NO WARRANTY, to the extent permitted by law.
 ******************************************************************************/
#ifndef _NVA_TYPES_H
#define _NVA_TYPES_H

#include "NVA_Defs.h"

#if defined (_MSC_VER)
  #include "msc_stdint.h"
#elif defined __BORLANDC__
  typedef unsigned __int32 uint32_t;
  typedef unsigned __int64 uint64_t;
#else
  #include <stdint.h>
#endif


typedef unsigned char uint8; ///< 8 bit unsigned integer

typedef uint16_t Sym_t; ///< Symbol type used by variable system functions

/**
 * Macro for adding suffix to integer or float constant
 */
#define NVA_SUFFIX_APPEND(x, y) x ## y

/**
 * \typedef ValueInt_t
 * \brief Variable typed used to hold the integer part of a variable
 */
/**
 * \def VALUE_INT(value)
 * Add the proper suffix for a 32 bit or 64 bit integer constant
 */
#if defined (config_VARIABLE_SIZE) && config_VARIABLE_SIZE > 32
typedef uint64_t ValueInt_t;
#define VALUE_INT(value) NVA_SUFFIX_APPEND(value, ULL)
#else
typedef uint32_t ValueInt_t;
#define VALUE_INT(value) NVA_SUFFIX_APPEND(value, UL)
#endif

/**
 * Max register size in bytes
 */
#define MAX_REGISTER_SIZE_BYTES ((config_VARIABLE_SIZE + 7) / 8)

/**
 * \typedef ValueFloat_t
 * \brief Variable typed used to hold the floating point part of a variable
 */
/**
 * \def VALUE_FLOAT(value)
 * Add the proper suffix for a 32 bit or 64 bit floating point constant
 */
#if defined (config_NVA_VALUE_FLOAT_SIZE) && config_NVA_VALUE_FLOAT_SIZE > 32
typedef double ValueFloat_t;
#define VALUE_FLOAT(value) (value)
#else
typedef float ValueFloat_t;
#define VALUE_FLOAT(value) NVA_SUFFIX_APPEND(value, f)
#endif


/**
 * \brief Union to hold the variable/parameter value
 *
 * A flag must be used to indicate which variant is used, this flag is not part of this union.
 * Normally the flag is part of the struct holding information about the variable.
 * The flag can be found by using the function NVA_GetVarFlags in Radarlib2
 */
typedef union {
  ValueInt_t i;     ///< Integer variant of value
  ValueFloat_t f;   ///< Floating point variant of value
} Value_t;

/**
 *  Flag used in convertion functions to indicate endianess of individual bytes of a variable
 */
#define NVA_LITTLE_ENDIAN 1
/**
 *  Flag used in convertion functions to indicate endianess of individual bytes of a variable
 */
#define NVA_BIG_ENDIAN 0

#define INT32_MAXVALUE 2147483647  ///< Max value of a signed INT32 bit integer

/**
 * \brief Return integer variant of Value_t
 */
#define value_to_int(value) ((value.i))

/**
 * \brief Return floating point variant of Value_t
 */
#define value_to_float(value) ((value.f))

/** 
 * \brief Convert an integer value to a Value_t type
 * 
 * @param value 
 * 
 * @return Value_t record (union)
 */
static inline Value_t * int_to_value(ValueInt_t valueInt, Value_t * value)
{
  value->i = valueInt;
  return value;
}

/** 
 * \brief Convert an floating point value to a Value_t type
 * 
 * @param value 
 * 
 * @return Value_t record (union)
 */
static inline Value_t * float_to_value(ValueFloat_t valueFloat, Value_t * value)
{
  value->f = valueFloat;
  return value;
}


#ifdef _MSC_VER
#define llround(value) (long)floor((value) + (double)0.5)
#endif

/**
 * \brief Macro calculating the length of an array
 */
#define LENGTH_OF_ARRAY(array) (sizeof((array))/sizeof((array[0])))

#endif  // _NVA_TYPES_H
