/**
  @file cluttermap.h

  Definitions for external functions in cluttermap library

  @copyright 2015 by FlatEarth, Inc

  @author Raymond J. Weber
  @author Justin Hadella
*/

#ifndef CLUTTERMAP_h
#define CLUTTERMAP_h

#ifdef __cplusplus
extern "C"
{
#endif

#include "static.h"
#include "adaptive.h"
#include "MTI.h"


/**Implimented Cluttermaps
*/
enum
{
  STATIC,          /**< Static Cluttermap */
  ADAPTIVE,        /**< Adatpive Cluttermap */
  MTI              /**< Moving Target Indicator (MTI) Cluttermap */
} cluttermapTypes_enum;



/**
  @defgroup cluttermaps Cluttermaps
  @ingroup salsaLib
*/



#ifdef __cplusplus
}
#endif
#endif
