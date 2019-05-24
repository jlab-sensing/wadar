/***********************************************************//**
* @file NVA_Bits.h
* @brief Defines of bit manipulation macros
*/
/*
* Copyright (C) 2009-2014 Novelda AS <software@novelda.no>\n
* Novelda AS grant to Customer a nonexclusive and nontransferable license
* to use this software code under terms regulated in a Software License Agreement
* or similar agreement between Novelda AS and the Customer.
* There is NO WARRANTY, to the extent permitted by law.
****************************************************************/

#ifndef NVA_BITS_H_
#define NVA_BITS_H_

// Ensure that bit manipulating macros works with 32 bit integers
//#if sizeof(unsigned int) == 4
//#define ONE_MASK (unsigned long)0x00000001
//#else
#define ONE_MASK 1 ///< Define a word with first bit set to "1" for use in bit manipulation macros
//#endif

// Bit manipulations when a bit number us used
// Example of use:
// Set bit 4: SETBIT(var1, 4);
// Clear bit 5: CLEARBIT(var2, 5);

/** Set a specified bit as a bit mask */
#define BIT(bit) (ONE_MASK << (bit))
/** Set specified bit in a word to "1" */
#define SETBIT(address,bit) ((address) |= BIT(bit))
/** Clear specified bit in a word to "0" */
#define CLEARBIT(address,bit) ((address) &= ~BIT(bit))
/** Set specified bit in a word to specified value */
#define SETBITVALUE(address,bit,value) ((address) = (((address) & ~(BIT(bit))) | (value << (bit))))
/** Flip/invert specified bit in a word */
#define FLIPBIT(address,bit) ((address) ^= BIT(bit))
/** Check if specified bit is set to "1" */
#define CHECKBIT(address,bit) ((address) & BIT(bit))
/** Get value of specified bit */
#define GETBITVALUE(address,bit) (((address) >> (bit)) & ONE_MASK)

// Bit manipulations when a bit mask us used
/** Set bits in a word */
#define SETBITMASK(x,y) (x |= (y))
/** Clear bits in a word */
#define CLEARBITMASK(x,y) (x &= (~y))
/** Flip bits in a word */
#define FLIPBITMASK(x,y) (x ^= (y))
/** Check if bits are set */
#define CHECKBITMASK(x,y) (x & (y))

#endif /* NVA_BITS_H_ */
