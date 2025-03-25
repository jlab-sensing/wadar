/***********************************************************//**
* \file msc_stdint.h
* \brief typedef of types not defined in some versions of Microsoft Visual C
*/
/*
* Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
* Novelda AS grant to Customer a nonexclusive and nontransferable license
* to use this software code under terms regulated in a Software License Agreement
* or similar agreement between Novelda AS and the Customer.
* There is NO WARRANTY, to the extent permitted by law.
****************************************************************/

#ifndef _MSC_VER // [
#error "Use this header only with Microsoft Visual C++ compilers!"
#endif // _MSC_VER ]

#ifndef MSC_STDINT_H_
#define MSC_STDINT_H_

typedef signed   __int8   int8_t;    ///< signed 8 bit integer
typedef signed   __int16  int16_t;   ///< signed 16 bit integer
typedef signed   __int32  int32_t;   ///< signed 32 bit integer
typedef unsigned __int8   uint8_t;   ///< unsigned 8 bit integer
typedef unsigned __int16  uint16_t;  ///< unsigned 16 bit integer
typedef unsigned __int32  uint32_t;  ///< unsigned 32 bit integer
typedef signed   __int64  int64_t;   ///< signed 64 bit integer
typedef unsigned __int64  uint64_t;  ///< unsigned 64 bit integer

#endif // MSC_STDINT_H_
