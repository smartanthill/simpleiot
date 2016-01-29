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

#include "siot_stats_counters.h"

#ifdef ENABLE_STATS_COUNTERS

#if SIOT_STATS_CTR_16_MAX != 0
uint16_t siot_stats_counters_16[ SIOT_STATS_CTR_16_MAX ];
#endif

#if SIOT_STATS_CTR_32_MAX != 0
uint32_t siot_stats_counters_32[ SIOT_STATS_CTR_32_MAX ];
#endif

void siot_init_ctrs()
{
	ZEPTO_MEMSET( siot_stats_counters_16, 0, sizeof(siot_stats_counters_16) );
#if SIOT_STATS_CTR_32_MAX != 0
	ZEPTO_MEMSET( siot_stats_counters_32, 0, sizeof(siot_stats_counters_32) );
#endif
}

void siot_load_ctrs( MEMORY_HANDLE mem_h )
{
	ZEPTO_DEBUG_PRINTF_1( "siot_load_ctrs() called:\n" );
	uint16_t i, j;
	// global, 16-bit
	ZEPTO_DEBUG_PRINTF_2( "global, 16-bit (%d counters):\n", SIOT_STATS_CTR_16_MAX );
	for ( i=0; i<SIOT_STATS_CTR_16_MAX; i++ )
	{
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_16[i]) );
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_16[i]>>8) );
		ZEPTO_DEBUG_PRINTF_3( "%d: %d\n", i, siot_stats_counters_16[i] );
	}
	// global, 32-bit
#if SIOT_STATS_CTR_32_MAX != 0
	ZEPTO_DEBUG_PRINTF_2( "global, 32-bit (%d counters):\n", SIOT_STATS_CTR_32_MAX );
	for ( i=0; i<SIOT_STATS_CTR_32_MAX; i++ )
	{
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_32[i]) );
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_32[i]>>8) );
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_32[i]>>16) );
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_32[i]>>24) );
		ZEPTO_DEBUG_PRINTF_3( "%d: %d\n", i, siot_stats_counters_32[i] );
	}
#endif
	// bus-specific, 16 bit
	ZEPTO_DEBUG_PRINTF_2( "bus-specific, 16-bit (%d buses):\n", hal_get_bus_count() );
	for ( i=0; i<hal_get_bus_count(); i++ )
	{
		for ( j=0; j<SIOT_STATS_CTR_BUS_SPECIFIC_16_MAX; j++ )
		{
			zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_bus_specific_16[ i * SIOT_STATS_CTR_BUS_SPECIFIC_16_MAX + j ]) );
			zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_bus_specific_16[ i * SIOT_STATS_CTR_BUS_SPECIFIC_16_MAX + j ]>>8) );
			ZEPTO_DEBUG_PRINTF_4( "%d, %d: %d\n", i, j, siot_stats_counters_bus_specific_16[ i * SIOT_STATS_CTR_BUS_SPECIFIC_16_MAX + j ] );
		}
	}
}

#endif // ENABLE_STATS_COUNTERS