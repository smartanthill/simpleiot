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

uint16_t siot_stats_counters_16[ SIOT_STATS_CTR_16_MAX ];
uint32_t siot_stats_counters_32[ SIOT_STATS_CTR_32_MAX ];

void siot_init_ctrs()
{
	ZEPTO_MEMSET( siot_stats_counters_16, 0, sizeof(siot_stats_counters_16) );
	ZEPTO_MEMSET( siot_stats_counters_32, 0, sizeof(siot_stats_counters_32) );
}

void siot_load_ctrs( MEMORY_HANDLE mem_h )
{
	uint16_t i;
	for ( i=0; i<SIOT_STATS_CTR_16_MAX; i++ )
	{
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_16[i]) );
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_16[i]>>8) );
	}
	for ( i=0; i<SIOT_STATS_CTR_32_MAX; i++ )
	{
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_32[i]) );
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_32[i]>>8) );
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_32[i]>>16) );
		zepto_write_uint8( mem_h, (uint8_t)(siot_stats_counters_32[i]>>24) );
	}
}

