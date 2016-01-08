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

#include "siot_common.h"
#include "../simpleiot_hal/siot_mem_mngmt.h"

// NOTE: counter names have no info about their byte size
#define SIOT_STATS_CTR_RESEND_GDP_OVERCOUNT 0
#define SIOT_STATS_CTR_ROUTE_UPDATE_REQUEST 1
#define SIOT_STATS_CTR_ROUTE_UPDATE_REQUEST_1ST_RT_CHECKSUM_FAILED 2
#define SIOT_STATS_CTR_ROUTE_UPDATE_REQUEST_2ND_RT_CHECKSUM_FAILED 3
#define SIOT_STATS_CTR_ROUTE_UPDATE_REQUEST_PACKET_HEADER_CHECKSUM_FAILED 4
#define SIOT_STATS_CTR_ROUTE_UPDATE_REQUEST_PACKET_FULL_CHECKSUM_FAILED 5
#define SIOT_STATS_CTR_SP_REPORTS_BROKEN 6

#define SIOT_STATS_CTR_16_MAX 7
extern uint16_t siot_stats_counters_16[ SIOT_STATS_CTR_16_MAX ];

#define SIOT_STATS_CTR_32_MAX 1
extern uint32_t siot_stats_counters_32[ SIOT_STATS_CTR_32_MAX ];

#define SIOUT_INCREMENT_CTR( x ) (siot_stats_counters_16[x])++;

void siot_init_ctrs();
void siot_load_ctrs( MEMORY_HANDLE mem_h );


// obsolete staff (to be totally removed soon)
#define INIT_COUNTER_SYSTEM
#define PRINT_COUNTERS()
#define INCREMENT_COUNTER( i, name )
#define INCREMENT_COUNTER_IF( i, name, cond )
#define UPDATE_MAX_COUNTER( i, name, val )
#define PRINT_COUNTERS()


#endif // __SIOT_STATS_COUNTERS_H__
