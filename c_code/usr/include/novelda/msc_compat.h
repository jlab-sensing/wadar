/***********************************************************//**
* \file msc_compat.h
* \brief Definition/redefinition of functions not defined in some versions of Microsoft Visual C but are part of C99 standard
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

#ifndef MSC_COMPAT_H_
#define MSC_COMPAT_H_

//#include <windows.h>

/**
 * strncasecmp is not included in Visual C, use _strnicmp instead
 */
#define strncasecmp(string1, string2, length) _strnicmp((string1), (string2), (length))

/**
 * lround is not included in Visual C 2010
 */
#define lround(value) (long int)floor((value) + 0.5)
#endif // MSC_COMPAT_H_
