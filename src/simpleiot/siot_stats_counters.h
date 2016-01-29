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

#if ! defined __SIOT_STATS_COUNTERS_H__
#define __SIOT_STATS_COUNTERS_H__

#define ENABLE_STATS_COUNTERS //TODO: make it global!

#ifdef ENABLE_STATS_COUNTERS

#include "siot_common.h"
#include "../simpleiot_hal/siot_mem_mngmt.h"
#include <simpleiot/siot_bus_data.h>

// NOTE: counter names have no info about their byte size

// Global counters - 16
#define SIOT_STATS_CTR_RESEND_GDP_OVERCOUNT 0
#define SIOT_STATS_CTR_ROUTE_UPDATE_REQUEST 1
#define SIOT_STATS_CTR_ROUTE_UPDATE_REQUEST_1ST_RT_CHECKSUM_FAILED 2
#define SIOT_STATS_CTR_ROUTE_UPDATE_REQUEST_2ND_RT_CHECKSUM_FAILED 3
#define SIOT_STATS_CTR_SP_REPORTS_BROKEN 4

#define SIOT_STATS_CTR_16_MAX 5
extern uint16_t siot_stats_counters_16[ SIOT_STATS_CTR_16_MAX ];

// Global counters - 32
#define SIOT_STATS_CTR_32_MAX 0
#if SIOT_STATS_CTR_32_MAX != 0
extern uint32_t siot_stats_counters_32[ SIOT_STATS_CTR_32_MAX ];
#endif

// Operating with global counters
#define SIOUT_INCREMENT_CTR( ctr ) (siot_stats_counters_16[ctr])++;
//#define SIOUT_INCREMENT_CTR( ctr )

// Bus-specific counters - 16
#define SIOT_STATS_CTR_PACKET_RECEIVED 0
#define SIOT_STATS_CTR_PACKET_SENT 1
#define SIOT_STATS_CTR_PACKET_HEADER_CHECKSUM_FAILED 2
#define SIOT_STATS_CTR_PACKET_FULL_CHECKSUM_FAILED 3
#define SIOT_STATS_CTR_PACKET_NOT_FOR_THIS_DEVICE 4
#define SIOT_STATS_CTR_PACKET_TTL_MAX 5

#define SIOT_STATS_CTR_BUS_SPECIFIC_16_MAX 6
extern uint16_t* siot_stats_counters_bus_specific_16;

// Operating with bus-specific counters
#define SIOUT_INCREMENT_CTR_PER_BUS( bus, ctr ) {ZEPTO_DEBUG_ASSERT( bus < hal_get_bus_count() ); (siot_stats_counters_bus_specific_16[ (bus) * SIOT_STATS_CTR_BUS_SPECIFIC_16_MAX + (ctr) ])++;}
//#define SIOUT_INCREMENT_CTR_PER_BUS( bus, ctr )


// initializing and reporting
void siot_init_ctrs();
void siot_load_ctrs( MEMORY_HANDLE mem_h );

#else // ENABLE_STATS_COUNTERS

#define SIOT_STATS_CTR_BUS_SPECIFIC_16_MAX 0

#define SIOUT_INCREMENT_CTR( ctr )
#define SIOUT_INCREMENT_CTR_PER_BUS( bus, ctr )
#define siot_init_ctrs()
#define siot_load_ctrs( mem_h )

#endif // ENABLE_STATS_COUNTERS


// obsolete staff (to be totally removed soon)
#define INIT_COUNTER_SYSTEM
#define PRINT_COUNTERS()
#define INCREMENT_COUNTER( i, name )
#define INCREMENT_COUNTER_IF( i, name, cond )
#define UPDATE_MAX_COUNTER( i, name, val )
#define PRINT_COUNTERS()


#endif // __SIOT_STATS_COUNTERS_H__
