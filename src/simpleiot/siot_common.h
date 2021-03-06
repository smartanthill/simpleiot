/*******************************************************************************
Copyright (C) 2015 OLogN Technologies AG

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2 as
    published by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*******************************************************************************/


#if !defined __SIOT_COMMON_H__
#define __SIOT_COMMON_H__

// common includes
#include <hal_platform.h>

// compiler-specific: disabling certain warnings
#ifdef _MSC_VER
#pragma warning (disable: 4996) // "The POSIX name for this item is deprecated. Instead, use the ISO C++ conformant name <...>"
#else
#endif

// ZEPTO_VM LEVELS
#define ZEPTO_VM_ONE 1
#define ZEPTO_VM_TINY 2
#define ZEPTO_VM_SMALL 3
#define ZEPTO_VM_MEDIUM 4

#if !defined ZEPTO_VM_LEVEL
#define ZEPTO_VM_LEVEL ZEPTO_VM_ONE
#endif

#ifdef ZEPTO_VM_ONE
#define ZEPTO_VM_USE_SIMPLE_FRAME
#endif

#define SA_LITTLE_ENDIAN 0
#define SA_BIG_ENDIAN 1
#define SA_USED_ENDIANNES SA_LITTLE_ENDIAN
//#define SA_USED_ENDIANNES SA_BIG_ENDIAN

#ifndef bool
#define bool uint8_t
#endif
#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif

#ifndef INLINE
#define INLINE static inline
#endif
#ifndef NOINLINE
#define NOINLINE      __attribute__ ((noinline))
#endif
#ifndef FORCE_INLINE
#define FORCE_INLINE static inline __attribute__((always_inline))
#endif

INLINE void zepto_memset( void* dest, uint8_t val, uint16_t cnt )
{
	uint16_t i;
	for ( i=0; i<cnt; i++ )
		((uint8_t*)dest)[i] = val;
}

INLINE void zepto_memcpy( void* dest, const void* src, uint16_t cnt )
{
	uint16_t i;
	for ( i=0; i<cnt; i++ )
		((uint8_t*)dest)[i] = ((uint8_t*)src)[i];
}

INLINE int8_t zepto_memcmp( const void* s1, const void* s2, uint16_t cnt )
{
	uint16_t i;
	for ( i=0; i<cnt; i++ )
	{
		if ( ((uint8_t*)s1)[i] > ((uint8_t*)s2)[i] ) return 1;
		if ( ((uint8_t*)s1)[i] < ((uint8_t*)s2)[i] ) return (int8_t)(-1);
	}
	return 0;
}

INLINE void zepto_memmov( void* dest, const void* src, uint16_t cnt )
{
	uint16_t i;
	if ( dest <= src )
		for ( i=0; i<cnt; i++ )
			((uint8_t*)dest)[i] = ((uint8_t*)src)[i];
	else
		for ( i=cnt; i>0; i-- )
			((uint8_t*)dest)[i-1] = ((uint8_t*)src)[i-1];
}

#define ZEPTO_MEMSET zepto_memset
#define ZEPTO_MEMCPY zepto_memcpy
#define ZEPTO_MEMMOV zepto_memmov
#define ZEPTO_MEMCMP zepto_memcmp

#ifndef ZEPTO_PROG_CONSTANT_LOCATION
#define ZEPTO_PROG_CONSTANT_LOCATION
#endif

#ifndef ZEPTO_PROG_CONSTANT_READ_BYTE
#define ZEPTO_PROG_CONSTANT_READ_BYTE(x) (*(x))
#endif

#ifndef ZEPTO_PROG_CONSTANT_READ_PTR
#define ZEPTO_PROG_CONSTANT_READ_PTR(x) ((void*)(*(x)))
#endif

#ifndef ZEPTO_MEMCPY_FROM_PROGMEM
#define ZEPTO_MEMCPY_FROM_PROGMEM ZEPTO_MEMCPY
#endif

// Master/Slave distinguishing bit; USED_AS_MASTER is assumed to be a preprocessor definition if necessary


#ifdef USED_AS_MASTER
#define MASTER_SLAVE_BIT 1
#else // USED_AS_MASTER
#define MASTER_SLAVE_BIT 0
#endif

// debug helpers

#ifdef SA_DEBUG
#if !defined DEBUG_PRINTING
#define DEBUG_PRINTING
#endif
#endif

#ifdef DEBUG_PRINTING
#include <stdio.h>
#define printf FORBIDDEN_CALL_USE_MACROS_INSTEAD
#define ZEPTO_DEBUG_PRINTF_1( s ) fprintf( stdout, s )
#define ZEPTO_DEBUG_PRINTF_2( s, x1 ) fprintf( stdout, s, x1 )
#define ZEPTO_DEBUG_PRINTF_3( s, x1, x2 ) fprintf( stdout, s, x1, x2 )
#define ZEPTO_DEBUG_PRINTF_4( s, x1, x2, x3 ) fprintf( stdout, s, x1, x2, x3 )
#define ZEPTO_DEBUG_PRINTF_5( s, x1, x2, x3, x4 ) fprintf( stdout, s, x1, x2, x3, x4 )
#define ZEPTO_DEBUG_PRINTF_6( s, x1, x2, x3, x4, x5 ) fprintf( stdout, s, x1, x2, x3, x4, x5 )
#define ZEPTO_DEBUG_PRINTF_7( s, x1, x2, x3, x4, x5, x6 ) fprintf( stdout, s, x1, x2, x3, x4, x5, x6 )
#else // DEBUG_PRINTING
#include <stdio.h>
//#define printf FORBIDDEN_CALL_OF_PRINTF
#define fprintf FORBIDDEN_CALL_OF_FPRINTF
#define ZEPTO_DEBUG_PRINTF_1( s )
#define ZEPTO_DEBUG_PRINTF_2( s, x1 )
#define ZEPTO_DEBUG_PRINTF_3( s, x1, x2 )
#define ZEPTO_DEBUG_PRINTF_4( s, x1, x2, x3 )
#define ZEPTO_DEBUG_PRINTF_5( s, x1, x2, x3, x4 )
#define ZEPTO_DEBUG_PRINTF_6( s, x1, x2, x3, x4, x5 )
#define ZEPTO_DEBUG_PRINTF_7( s, x1, x2, x3, x4, x5, x6 )
#endif // DEBUG_PRINTING

#ifdef SA_DEBUG
#include <assert.h>
#define ZEPTO_DEBUG_ASSERT( x ) assert( x )
#define ZEPTO_RUNTIME_CHECK( x ) assert( x )
#else
#define assert FORBIDDEN_CALL_OF_ASSERT
#define ZEPTO_DEBUG_ASSERT( x )
#define ZEPTO_RUNTIME_CHECK( x )  //TODO: define
#endif

#endif // __SIOT_COMMON_H__
