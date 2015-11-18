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

#include "siot_m_protocol.h"

// TODO: code below represents h/w-dependent information;
//       it is placed here TEMPORARILY for the sake of progress of remaining development
//       and MUST be REVISED and REWORKED ASAP!!!
//       It appears that the list is constant during lifecycle (changes require recompilation);
//       Therefore, current "quick" solution seems to be not too far from actual

typedef struct _BUS_LIST_ITEM
{
	uint16_t bus_id;
	uint8_t bus_type;
} BUS_LIST_ITEM;

#ifdef USED_AS_MASTER
#define BUS_LIST_ITEM_COUNT 1
BUS_LIST_ITEM bus_list[ BUS_LIST_ITEM_COUNT ] = {0, 0};
#else
#ifdef USED_AS_RETRANSMITTER
#define BUS_LIST_ITEM_COUNT 2 // WHY? - see comment above - it's just for immediate testing purposes
BUS_LIST_ITEM bus_list[ BUS_LIST_ITEM_COUNT ] = {{0, 0}, {1,1}};
#else
#define BUS_LIST_ITEM_COUNT 1
BUS_LIST_ITEM bus_list[ BUS_LIST_ITEM_COUNT ] = {0, 0};
#endif
#endif

#define BUS_TYPE_UNDEFINED 0XFF

uint8_t get_bus_type_by_bus_id( uint16_t bus_id )
{
	uint8_t idx;
	for ( idx=0; idx<BUS_LIST_ITEM_COUNT; idx++ )
		if ( bus_list[idx].bus_id == bus_id )
			return bus_list[idx].bus_type;
	return BUS_TYPE_UNDEFINED;
}

// [end of TODO comment]


typedef struct _SIOT_MESH_LAST_HOP_DATA
{
	uint16_t last_hop_id;
	uint16_t last_hop_bus_id;
	uint8_t conn_quality;
} SIOT_MESH_LAST_HOP_DATA;

// service functions

uint16_t zepto_parser_calculate_checksum_of_part_of_response( MEMORY_HANDLE mem_h, uint16_t offset, uint16_t sz, uint16_t accum_val )
{
	uint16_t i;
	for ( i=offset; i<offset+sz; i++ )
	{
		accum_val += memory_object_read_response_byte( mem_h, i ); // do something simple for a while
		// TODO: actual implementation
	}
	return accum_val;
}

uint16_t zepto_parser_calculate_checksum_of_part_of_request( MEMORY_HANDLE mem_h, parser_obj* po_start, uint16_t sz, uint16_t accum_val )
{
	uint16_t i;
	parser_obj po;
	zepto_parser_init_by_parser( &po, po_start );
	for ( i=0; i<sz; i++ )
	{
		accum_val += zepto_parse_uint8( &po ); // do something simple for a while
		// TODO: actual implementation
	}
	return accum_val;
}

#ifdef USED_AS_MASTER

// NOTE: On a ROOT (MASTER) side any implementation is inevitably quite complicated as
// (1) ROOT has to operate with many devices, and
// (2) ROOT implements almost all "decision-making" logic.
// On the other hand, ROOT is expected to run on a reasonably powerfull platforms.
// This results in that implementation at ROOT may not necessarily satisfy limitations of that for DEVICEs.
// To avoid confusions we put all necessary processing in a separate file
// which should NOT be taken as a code sample for embadded programming.

// We will declare all necessary interface calls below.

#else // USED_AS_MASTER

#ifdef USED_AS_RETRANSMITTER

typedef struct _MESH_PENDING_RESENDS
{
	uint8_t type;
	MEMORY_HANDLE packet_h;
	uint8_t resend_cnt;
	sa_time_val next_resend_time;
	uint16_t checksum;
	uint16_t target_id; // TODO: for a terminating device it is always 0 (rooot), right? - subject for optimization
	uint16_t bus_id;
	uint16_t next_hop;
} MESH_PENDING_RESENDS;

typedef struct _MESH_PENDING_RESENDS_PACKET_INFO
{
	uint16_t checksum;
	uint16_t target_id;
	uint16_t bus_id;
	uint16_t next_hop;
} MESH_PENDING_RESENDS_PACKET_INFO;

#define SIOT_MESH_LINK_TABLE_SIZE_MAX 256
#define SIOT_MESH_ROUTE_TABLE_SIZE_MAX 256

uint8_t siot_mesh_link_table_size;
static SIOT_MESH_LINK siot_mesh_link_table[ SIOT_MESH_LINK_TABLE_SIZE_MAX ];
uint8_t siot_mesh_route_table_size;
static SIOT_MESH_ROUTE siot_mesh_route_table[ SIOT_MESH_ROUTE_TABLE_SIZE_MAX ];
static SIOT_MESH_RETRANSM_COMMON_DATA siot_mesh_retransm_common_data;

void siot_mesh_init_tables()  // TODO: this call reflects current development stage and may or may not survive in the future
{
	siot_mesh_route_table_size = 0;
	siot_mesh_link_table_size = 0;
}

void siot_mesh_init_at_life_start()
{
	siot_mesh_link_table_size = 0;
	siot_mesh_route_table_size = 0;
}

// TODO: for terminating device below processing is much easier!!! Reimplement!!!

void siot_mesh_clear_rout_table()
{
	siot_mesh_route_table_size = 0;
}

uint8_t siot_mesh_target_to_link_id( uint16_t target_id, uint16_t* link_id )
{
	uint16_t i;
	for ( i=0; i<siot_mesh_route_table_size; i++ )
		if ( siot_mesh_route_table[i].TARGET_ID == target_id )
		{
			*link_id = siot_mesh_route_table[i].LINK_ID;
			return SIOT_MESH_RET_OK;
		}
	return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_get_link( uint16_t link_id, SIOT_MESH_LINK* link )
{
	uint16_t i;
	for ( i=0; i<siot_mesh_link_table_size; i++ )
	{
		if ( siot_mesh_link_table[i].LINK_ID < link_id )
			continue;
		if ( siot_mesh_link_table[i].LINK_ID == link_id ) // found
		{
			ZEPTO_MEMCPY( link, siot_mesh_link_table + i, sizeof(SIOT_MESH_LINK) );
			return SIOT_MESH_RET_OK;
		}
		break;
	}
	return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;
}

uint8_t siot_mesh_get_bus_id_for_target( uint16_t target_id, uint16_t* bus_id )
{
	uint16_t link_id;
	uint8_t ret_code = siot_mesh_target_to_link_id( target_id, &link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		SIOT_MESH_LINK link;
		ret_code = siot_mesh_get_link( link_id, &link );
		if ( ret_code == SIOT_MESH_RET_OK )
			*bus_id = link.BUS_ID;
	}
	return ret_code;
}

uint8_t siot_mesh_get_bus_id_to_root( uint16_t* bus_id )
{
	return siot_mesh_get_bus_id_for_target( 0, bus_id );
}

uint8_t siot_mesh_get_link_to_root( SIOT_MESH_LINK* link )
{
	uint16_t i;
	uint16_t link_id;
	uint8_t ret_code = siot_mesh_target_to_link_id( 0, &link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		for ( i=0; i<siot_mesh_link_table_size; i++ )
		{
			if ( siot_mesh_link_table[i].LINK_ID < link_id )
				continue;
			if ( siot_mesh_link_table[i].LINK_ID == link_id ) // found
			{
				ZEPTO_MEMCPY( link, siot_mesh_link_table + i, sizeof(SIOT_MESH_LINK) );
				return SIOT_MESH_RET_OK;
			}
			break;
		}
	}
	return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;
}

uint8_t siot_mesh_add_link( SIOT_MESH_LINK* link )
{
	uint16_t i;
	uint16_t link_id = link->LINK_ID;
	if ( siot_mesh_link_table_size && siot_mesh_link_table[0].LINK_ID > link_id )
	{
		if ( siot_mesh_link_table_size == SIOT_MESH_LINK_TABLE_SIZE_MAX )
			return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;
		ZEPTO_MEMMOV( siot_mesh_link_table + 1, siot_mesh_link_table, sizeof(SIOT_MESH_LINK) * siot_mesh_link_table_size );
		ZEPTO_MEMCPY( siot_mesh_link_table, link, sizeof(SIOT_MESH_LINK) );
		siot_mesh_link_table_size++;
		return SIOT_MESH_RET_OK;
	}
	for ( i=0; i<siot_mesh_link_table_size; i++ )
	{
		if ( siot_mesh_link_table[i].LINK_ID < link_id )
			continue;
		if ( siot_mesh_link_table[i].LINK_ID == link_id ) // found; update
		{
			ZEPTO_MEMCPY( siot_mesh_link_table + i, link, sizeof(SIOT_MESH_LINK) );
			return SIOT_MESH_RET_OK;
		}
		else // greater, must be inserted after the current one
		{
			if ( siot_mesh_link_table_size == SIOT_MESH_LINK_TABLE_SIZE_MAX )
				return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;
			ZEPTO_MEMMOV( siot_mesh_link_table + i + 1, siot_mesh_link_table + i, sizeof(SIOT_MESH_LINK) * ( siot_mesh_link_table_size - i ) );
			ZEPTO_MEMCPY( siot_mesh_link_table + i, link, sizeof(SIOT_MESH_LINK) );
			siot_mesh_link_table_size++;
			return SIOT_MESH_RET_OK;
		}
	}
	// as we are here, the entry does not exist at all, and we have to add it rather than modify existing
	if ( siot_mesh_link_table_size == SIOT_MESH_LINK_TABLE_SIZE_MAX )
		return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;

	ZEPTO_MEMCPY( siot_mesh_link_table + siot_mesh_link_table_size, link, sizeof(SIOT_MESH_LINK) );
	siot_mesh_link_table_size++;
	return SIOT_MESH_RET_OK;
}

uint8_t siot_mesh_remove_link( uint16_t link_id )
{
	uint16_t i;
	for ( i=0; i<siot_mesh_link_table_size; i++ )
		if ( siot_mesh_link_table[i].LINK_ID == link_id )
		{
			ZEPTO_MEMMOV( siot_mesh_link_table + i, siot_mesh_link_table + i + 1, sizeof(SIOT_MESH_ROUTE) * ( siot_mesh_link_table_size - i - 1 ) );
			siot_mesh_link_table_size--;
			return SIOT_MESH_RET_OK;
		}
	return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_delete_route( uint16_t target_id )
{
	uint16_t i;
	for ( i=0; i<siot_mesh_route_table_size; i++ )
		if ( siot_mesh_route_table[i].TARGET_ID == target_id )
		{
			ZEPTO_MEMMOV( siot_mesh_route_table + i, siot_mesh_route_table + i + 1, sizeof(SIOT_MESH_ROUTE) * ( siot_mesh_route_table_size - i - 1 ) );
			siot_mesh_route_table_size--;
			return SIOT_MESH_RET_OK;
		}
	return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_add_route( uint16_t target_id, uint16_t link_id )
{
	uint16_t i;
	if ( siot_mesh_route_table_size && siot_mesh_route_table[0].TARGET_ID > target_id )
	{
		if ( siot_mesh_route_table_size == SIOT_MESH_ROUTE_TABLE_SIZE_MAX )
			return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;
		ZEPTO_MEMMOV( siot_mesh_route_table + 1, siot_mesh_route_table, sizeof(SIOT_MESH_ROUTE) * siot_mesh_route_table_size );
		siot_mesh_route_table[0].TARGET_ID = target_id;
		siot_mesh_route_table[0].LINK_ID = link_id;
		siot_mesh_route_table_size++;
		return SIOT_MESH_RET_OK;
	}
	for ( i=0; i<siot_mesh_route_table_size; i++ )
	{
		if ( siot_mesh_route_table[i].TARGET_ID < target_id )
			continue;
		if ( siot_mesh_route_table[i].TARGET_ID == target_id ) // found
		{
			siot_mesh_route_table[i].LINK_ID = link_id;
			return SIOT_MESH_RET_OK;
		}
		else
		{
			if ( siot_mesh_route_table_size == SIOT_MESH_ROUTE_TABLE_SIZE_MAX )
				return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;
			ZEPTO_MEMMOV( siot_mesh_route_table + i + 1, siot_mesh_route_table + i, sizeof(SIOT_MESH_ROUTE) * ( siot_mesh_route_table_size - i ) );
			siot_mesh_route_table[i].TARGET_ID = target_id;
			siot_mesh_route_table[i].LINK_ID = link_id;
			siot_mesh_route_table_size++;
			return SIOT_MESH_RET_OK;
		}
		break;
	}
	// as we are here, the entry does not exist at all, and we have to add it rather than modify existing
	if ( siot_mesh_route_table_size == SIOT_MESH_ROUTE_TABLE_SIZE_MAX )
		return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;

	siot_mesh_route_table[siot_mesh_route_table_size].TARGET_ID = target_id;
	siot_mesh_route_table[siot_mesh_route_table_size].LINK_ID = link_id;
	siot_mesh_route_table_size++;
	return SIOT_MESH_RET_OK;
}

uint16_t siot_mesh_calculate_route_table_checksum()
{
	return 0;
}

#define MESH_PENDING_RESEND_TYPE_SELF_PACKET 0
//#define MESH_PENDING_RESEND_TYPE_RETRANSMITTED_PACKET 1

#define SIOT_MESH_RET_RESEND_TASK_NONE_EXISTS 6
#define SIOT_MESH_RET_RESEND_TASK_NOT_NOW 7
#define SIOT_MESH_RET_RESEND_TASK_INTERM 8
#define SIOT_MESH_RET_RESEND_TASK_FINAL 9
#define SIOT_MESH_RET_RESEND_TASK_FROM_SANTA 10
#define SIOT_MESH_RET_RESEND_TASK_UNICAST 11
#define SIOT_MESH_RET_RESEND_TASK_UNICAST_POSTFINAL 12

#define MESH_PENDING_RESEND_FROM_SANTA 0
#define MESH_PENDING_RESEND_TYPE_SELF_REPEATED 1
#define MESH_PENDING_RESEND_TYPE_TRANSIT_UNICAST 2

void siot_mesh_add_resend_task( MEMORY_HANDLE packet/*, uint8_t type*/, const sa_time_val* currt, uint16_t checksum, uint16_t target_id, uint16_t bus_id, uint16_t next_hop, sa_time_val* time_to_next_event )
{
	// 1. prepare resend task
	MESH_PENDING_RESENDS resend;
	resend.type = MESH_PENDING_RESEND_TYPE_SELF_REPEATED;
	resend.bus_id = bus_id;
	resend.target_id = target_id;
	resend.next_hop = next_hop;
	resend.packet_h = acquire_memory_handle();
	resend.checksum = checksum;
	resend.resend_cnt = SIOT_MESH_SUBJECT_FOR_MESH_RESEND;
	sa_time_val diff_tval;
	TIME_MILLISECONDS16_TO_TIMEVAL( MESH_RESEND_PERIOD_MS, diff_tval );
	sa_hal_time_val_copy_from( &(resend.next_resend_time), currt );
	SA_TIME_INCREMENT_BY_TICKS( resend.next_resend_time, diff_tval );
	if ( resend.packet_h == MEMORY_HANDLE_INVALID )
	{
		ZEPTO_DEBUG_ASSERT( 0 == "memory deficiency (getting invalid handle) is not considered yet" ); // TODO: consider and implement
	}
#if 0
	resend.type = type;
	switch ( type )
	{
		case MESH_PENDING_RESEND_TYPE_SELF_PACKET:
		{
			zepto_copy_request_to_response_of_another_handle( packet, resend.packet_h );
			break;
		}
		case MESH_PENDING_RESEND_TYPE_RETRANSMITTED_PACKET:
		{
			zepto_copy_response_to_response_of_another_handle( packet, resend.packet_h );
			break;
		}
	}
#else // 0
	zepto_copy_request_to_response_of_another_handle( packet, resend.packet_h );
#endif
	zepto_response_to_request( resend.packet_h );

	// 2. add resend task
	zepto_memman_append_locally_generated_data( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );

	// 3. calculate time to the nearest event
	uint16_t offset;
	sa_time_val remaining;
//	SA_TIME_SET_INFINITE_TIME( *time_to_next_event );
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		uint8_t in_future = sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), &remaining );
		sa_hal_time_val_copy_from_if_src_less( time_to_next_event, &remaining );
		if ( !in_future )
			break;
	}
}

void siot_mesh_at_root_add_send_from_santa_task( MEMORY_HANDLE packet, const sa_time_val* currt, sa_time_val* time_to_next_event, uint16_t bus_id, uint16_t next_hop )
{
	// 1. add resend task
	MESH_PENDING_RESENDS resend;
	resend.type = MESH_PENDING_RESEND_FROM_SANTA;
	resend.packet_h = packet;
	resend.bus_id = bus_id;
	resend.target_id = SIOT_MESH_TARGET_UNDEFINED;
	resend.next_hop = next_hop;
	resend.resend_cnt = 1;
	sa_hal_time_val_copy_from( &(resend.next_resend_time), currt );
	zepto_memman_append_locally_generated_data( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
	SA_TIME_SET_ZERO_TIME( *time_to_next_event );
}

void siot_mesh_at_root_add_resend_unicast_task( MEMORY_HANDLE packet, const sa_time_val* currt, sa_time_val* time_to_next_event, uint16_t target_id, uint16_t bus_id, uint16_t next_hop )
{
	// 1. add resend task
	MESH_PENDING_RESENDS resend;
	resend.type = MESH_PENDING_RESEND_TYPE_TRANSIT_UNICAST;
	resend.packet_h = packet;
	resend.bus_id = bus_id;
	resend.target_id = target_id;
	resend.next_hop = next_hop;
	resend.resend_cnt = SIOT_MESH_SUBJECT_FOR_MESH_RESEND_UNICAST_IN_TRANSIT;
	sa_time_val diff_tval;
	TIME_MILLISECONDS16_TO_TIMEVAL( MESH_RESEND_PERIOD_MS, diff_tval );
	sa_hal_time_val_copy_from( &(resend.next_resend_time), currt );
	SA_TIME_INCREMENT_BY_TICKS( resend.next_resend_time, diff_tval );
	if ( resend.packet_h == MEMORY_HANDLE_INVALID )
	{
		ZEPTO_DEBUG_ASSERT( 0 == "memory deficiency (getting invalid handle) is not considered yet" ); // TODO: consider and implement
	}
	zepto_copy_response_to_response_of_another_handle( packet, resend.packet_h );
	zepto_memman_append_locally_generated_data( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );

	// 3. calculate time to the nearest event
	uint16_t offset;
	sa_time_val remaining;
//	SA_TIME_SET_INFINITE_TIME( *time_to_next_event );
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		uint8_t in_future = sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), &remaining );
		sa_hal_time_val_copy_from_if_src_less( time_to_next_event, &remaining );
		if ( !in_future )
			break;
	}
}

uint8_t siot_mesh_get_resend_task( MEMORY_HANDLE packet, const sa_time_val* currt, sa_time_val* time_to_next_event, MESH_PENDING_RESENDS_PACKET_INFO* info )
{
	uint8_t it = 0, it_oldest = (uint8_t)(-1);
	uint16_t offset;
	MESH_PENDING_RESENDS resend;
	sa_time_val oldest_time_point;
	sa_hal_time_val_copy_from( &oldest_time_point, currt );

	if ( zepto_memman_get_currently_allocated_size_for_locally_generated_data( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER ) == 0 )
		return SIOT_MESH_RET_RESEND_TASK_NONE_EXISTS;

	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		if ( it_oldest == (uint8_t)(-1) )
		{
			if ( sa_hal_time_val_is_less_eq( &(resend.next_resend_time), &oldest_time_point ) ) // used slot is older than current time
			{
				sa_hal_time_val_copy_from( &oldest_time_point, &(resend.next_resend_time) );
				it_oldest = it;
			}
		}
		else
		{
			if ( sa_hal_time_val_is_less( &(resend.next_resend_time), &oldest_time_point ) ) // used slot is yet older
			{
				sa_hal_time_val_copy_from( &oldest_time_point, &(resend.next_resend_time) );
				it_oldest = it;
			}
		}
		it++;
	}

	if ( it_oldest == (uint8_t)(-1) ) // none good for present time; just calculate time to wake up us next time
	{
		for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
		{
			if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
				break;
			bool ret = sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
			ZEPTO_DEBUG_ASSERT( ret ); // non-zero time indeed remains
		}
		return SIOT_MESH_RET_RESEND_TASK_NOT_NOW;
	}

	// there is a packet to resend
	zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, it_oldest * sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );

	info->bus_id = resend.bus_id;
	info->target_id = resend.target_id;
	info->next_hop = resend.next_hop;
	info->checksum = resend.checksum;

	switch ( resend.type )
	{
		case MESH_PENDING_RESEND_TYPE_SELF_REPEATED:
		{
			bool final_in_seq = resend.resend_cnt == 1;
			ZEPTO_DEBUG_ASSERT( resend.resend_cnt > 0 );
			zepto_parser_free_memory( packet );
			zepto_copy_request_to_response_of_another_handle( resend.packet_h, packet );
		//	zepto_response_to_request( packet );

			if ( final_in_seq ) // erase resend task
			{
				release_memory_handle( resend.packet_h );
				for ( offset=it_oldest * sizeof( MESH_PENDING_RESENDS ); ; offset += sizeof( MESH_PENDING_RESENDS ) )
				{
					if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset + sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
						break;
					zepto_memman_write_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
				}
				zepto_memman_trim_locally_generated_data_at_right( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, sizeof( MESH_PENDING_RESENDS ) );
			}
			else // update task
			{
				(resend.resend_cnt) --;
				sa_time_val diff_tval;
				TIME_MILLISECONDS16_TO_TIMEVAL( MESH_RESEND_PERIOD_MS, diff_tval );
				sa_hal_time_val_copy_from( &(resend.next_resend_time), currt );
				SA_TIME_INCREMENT_BY_TICKS( resend.next_resend_time, diff_tval );
				// update data in memory
				zepto_memman_write_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, it_oldest * sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
			}

			// now we calculate remaining time for only actually remaining tasks
			ZEPTO_DEBUG_PRINTF_1( "\nRequest tasks @siot_mesh_get_resend_task():\n" );
			for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
			{
				if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
					break;
				bool ret = sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
		//		ZEPTO_DEBUG_ASSERT( ret ); // non-zero time indeed remains
				ZEPTO_DEBUG_PRINTF_6( "%04x%04x: (RESEND) target: %d, packet_h:%d, resend_cnt: %d\n", resend.next_resend_time.high_t, resend.next_resend_time.low_t, resend.target_id, resend.packet_h, resend.resend_cnt );
			}

			return final_in_seq ? SIOT_MESH_RET_RESEND_TASK_FINAL : SIOT_MESH_RET_RESEND_TASK_INTERM;
		}
		case MESH_PENDING_RESEND_FROM_SANTA:
		{
/*if ( resend.resend_cnt != 1 )
{
	resend.resend_cnt = resend.resend_cnt;
}*/
			ZEPTO_DEBUG_ASSERT( resend.resend_cnt == 1 );
			zepto_parser_free_memory( packet );
			zepto_copy_response_to_response_of_another_handle( resend.packet_h, packet );
			release_memory_handle( resend.packet_h );
			for ( offset=it_oldest * sizeof( MESH_PENDING_RESENDS ); ; offset += sizeof( MESH_PENDING_RESENDS ) )
			{
				if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset + sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
					break;
				zepto_memman_write_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
			}
			zepto_memman_trim_locally_generated_data_at_right( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, sizeof( MESH_PENDING_RESENDS ) );

			// now we calculate remaining time for only actually remaining tasks
			ZEPTO_DEBUG_PRINTF_1( "\nRequest tasks @siot_mesh_get_resend_task():\n" );
			for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
			{
				if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
					break;
				bool ret = sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
		//		ZEPTO_DEBUG_ASSERT( ret ); // non-zero time indeed remains
				ZEPTO_DEBUG_PRINTF_6( "%04x%04x: (FROM-SANTA) target: %d, packet_h:%d, resend_cnt: %d\n", resend.next_resend_time.high_t, resend.next_resend_time.low_t, resend.target_id, resend.packet_h, resend.resend_cnt );
			}

			return SIOT_MESH_RET_RESEND_TASK_FROM_SANTA;
		}
		case MESH_PENDING_RESEND_TYPE_TRANSIT_UNICAST:
		{
			ZEPTO_DEBUG_ASSERT( resend.resend_cnt > 0 );
			zepto_parser_free_memory( packet );
		//	zepto_response_to_request( packet );
			uint8_t ret_val;

			if ( resend.resend_cnt == 0 ) // erase resend task
			{
				ZEPTO_DEBUG_ASSERT( resend.packet_h == MEMORY_HANDLE_INVALID );
				for ( offset=it_oldest * sizeof( MESH_PENDING_RESENDS ); ; offset += sizeof( MESH_PENDING_RESENDS ) )
				{
					if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset + sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
						break;
					zepto_memman_write_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
				}
				zepto_memman_trim_locally_generated_data_at_right( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, sizeof( MESH_PENDING_RESENDS ) );
				ret_val = SIOT_MESH_RET_RESEND_TASK_UNICAST_POSTFINAL;
			}
			else
			{
				zepto_copy_response_to_response_of_another_handle( resend.packet_h, packet );
				if ( resend.resend_cnt == 1 )// update task
				{
					release_memory_handle( resend.packet_h );
					resend.packet_h = MEMORY_HANDLE_INVALID;
				}
				(resend.resend_cnt) --;
				sa_time_val diff_tval;
				TIME_MILLISECONDS16_TO_TIMEVAL( MESH_RESEND_PERIOD_MS, diff_tval );
				sa_hal_time_val_copy_from( &(resend.next_resend_time), currt );
				SA_TIME_INCREMENT_BY_TICKS( resend.next_resend_time, diff_tval );
				// update data in memory
				zepto_memman_write_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, it_oldest * sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
				ret_val = SIOT_MESH_RET_RESEND_TASK_UNICAST;
			}

			// now we calculate remaining time for only actually remaining tasks
			ZEPTO_DEBUG_PRINTF_1( "\nRequest tasks @siot_mesh_get_resend_task():\n" );
			for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
			{
				if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
					break;
				bool ret = sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
		//		ZEPTO_DEBUG_ASSERT( ret ); // non-zero time indeed remains
				ZEPTO_DEBUG_PRINTF_6( "%04x%04x: (RESEND) target: %d, packet_h:%d, resend_cnt: %d\n", resend.next_resend_time.high_t, resend.next_resend_time.low_t, resend.target_id, resend.packet_h, resend.resend_cnt );
			}

			return ret_val;
		}
		default:
		{
			// we should not be here, anyway
			ZEPTO_DEBUG_ASSERT( 0 == "Unexpected resend task type" );
			return SIOT_MESH_RET_RESEND_TASK_NOT_NOW;
		}
	}
}

void siot_mesh_remove_resend_task_by_hash( uint16_t checksum, const sa_time_val* currt, sa_time_val* time_to_next_event )
{
	uint16_t offset, gap;
	MESH_PENDING_RESENDS resend;
	sa_time_val oldest_time_point;
	sa_hal_time_val_copy_from( &oldest_time_point, currt );

	if ( zepto_memman_get_currently_allocated_size_for_locally_generated_data( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER ) == 0 )
		return;

	gap = 0;
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		if ( resend.checksum == checksum )
		{
			if ( resend.packet_h != MEMORY_HANDLE_INVALID )
				release_memory_handle( resend.packet_h );
			gap += sizeof( MESH_PENDING_RESENDS );
		}
		else
			if ( gap )	zepto_memman_write_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset - gap, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
	}
	if ( gap )	zepto_memman_trim_locally_generated_data_at_right( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, gap );

	// now we calculate remaining time for only actually remaining tasks
	ZEPTO_DEBUG_PRINTF_1( "\nRequest tasks @siot_mesh_remove_resend_task_by_hash():\n" );
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
		ZEPTO_DEBUG_PRINTF_6( "%04x%04x: target: %d, packet_h:%d, resend_cnt: %d\n", resend.next_resend_time.high_t, resend.next_resend_time.low_t, resend.bus_id, resend.packet_h, resend.resend_cnt );
	}
}

void siot_mesh_remove_resend_task_by_device_id( uint16_t target_id, const sa_time_val* currt, sa_time_val* time_to_next_event )
{
	uint16_t offset, gap;
	MESH_PENDING_RESENDS resend;
	sa_time_val oldest_time_point;
	sa_hal_time_val_copy_from( &oldest_time_point, currt );

	if ( zepto_memman_get_currently_allocated_size_for_locally_generated_data( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER ) == 0 )
		return;

	gap = 0;
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		if ( resend.type != MESH_PENDING_RESEND_TYPE_SELF_REPEATED )
			continue;
		if ( resend.target_id == target_id )
		{
			if ( resend.packet_h != MEMORY_HANDLE_INVALID )
				release_memory_handle( resend.packet_h );
			gap += sizeof( MESH_PENDING_RESENDS );
		}
		else
			if ( gap )	zepto_memman_write_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset - gap, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
	}
	if ( gap )	zepto_memman_trim_locally_generated_data_at_right( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, gap );

	// now we calculate remaining time for only actually remaining tasks
	ZEPTO_DEBUG_PRINTF_1( "\nRequest tasks @siot_mesh_remove_resend_task_by_device_id():\n" );
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
		ZEPTO_DEBUG_PRINTF_6( "%04x%04x: target: %d, packet_h:%d, resend_cnt: %d\n", resend.next_resend_time.high_t, resend.next_resend_time.low_t, resend.bus_id, resend.packet_h, resend.resend_cnt );
	}
}
#else // USED_AS_RETRANSMITTER

typedef struct _MESH_PENDING_RESENDS
{
	MEMORY_HANDLE packet_h;
	uint8_t resend_cnt;
	sa_time_val next_resend_time;
	uint16_t checksum;
//	uint16_t target_id; // TODO: for a terminating device it is always 0 (rooot), right? - subject for optimization
} MESH_PENDING_RESENDS;

//#define SIOT_MESH_LINK_TABLE_SIZE_MAX 1
//#define SIOT_MESH_ROUTE_TABLE_SIZE_MAX 1
typedef struct _SIOT_MESH_LINK_TO_ROOT_AT_TERMINAL_DEVICE
{
	uint8_t valid;
	uint16_t NEXT_HOP; // note: this link will be common for all targets that are reachable through a device with NEXT_HOP device ID
	uint16_t BUS_ID; // type is inspired in section "Communicating Routing Table Information over SACCP" by "BUS-ID is an Encoded-Unsigned-Int<max=2> field"
	uint8_t INTRA_BUS_ID; // INTRA-BUS-ID=NULL means that the entry is for an incoming link. Incoming link entries are relatiely rare, and are used to specify LINK-DELAYs.
	uint8_t NEXT_HOP_ACKS; // NEXT-HOP-ACKS is a flag which is set if the nearest hop (over (BUS-ID,INTRA-BUS-ID)) is known to be able not only to receive packets, but to send ACKs back
	uint16_t LINK_DELAY_UNIT; // type is inspired: same section as above
	uint16_t LINK_DELAY; // type is inspired: same section as above
	uint16_t LINK_DELAY_ERROR; // type is inspired: same section as above
} SIOT_MESH_LINK_TO_ROOT_AT_TERMINAL_DEVICE;

SIOT_MESH_LINK_TO_ROOT_AT_TERMINAL_DEVICE link_to_root;

void siot_mesh_init_tables()  // TODO: this call reflects current development stage and may or may not survive in the future
{
	link_to_root.valid = 0;
}

void siot_mesh_init_at_life_start()
{
	link_to_root.valid = 0;
}

void siot_mesh_clear_rout_table()
{
	link_to_root.valid = 0;
}

uint8_t siot_mesh_target_to_link_id( uint16_t target_id, uint16_t* link_id )
{
	if (link_to_root.valid )
	{
			*link_id = 0; // formal value
			return SIOT_MESH_RET_OK;
	}
	else
		return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_get_link( uint16_t link_id, SIOT_MESH_LINK* link )
{
	ZEPTO_DEBUG_ASSERT( link_id == 0 );
	if (link_to_root.valid )
	{
		link->LINK_ID = 0;
		link->NEXT_HOP = link_to_root.NEXT_HOP;
		link->BUS_ID = link_to_root.BUS_ID;
		link->INTRA_BUS_ID = link_to_root.INTRA_BUS_ID;
		link->NEXT_HOP_ACKS = link_to_root.NEXT_HOP_ACKS;
		link->LINK_DELAY_UNIT = link_to_root.LINK_DELAY_UNIT;
		link->LINK_DELAY = link_to_root.LINK_DELAY;
		link->LINK_DELAY_ERROR = link_to_root.LINK_DELAY_ERROR;
		return SIOT_MESH_RET_OK;
	}
	else
		return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;
}

uint8_t siot_mesh_add_link( SIOT_MESH_LINK* link )
{
	ZEPTO_DEBUG_ASSERT( link->LINK_ID == 0 );
	link_to_root.valid = 1;

	link_to_root.NEXT_HOP = link->NEXT_HOP;
	link_to_root.BUS_ID = link->BUS_ID;
	link_to_root.INTRA_BUS_ID = link->INTRA_BUS_ID;
	link_to_root.NEXT_HOP_ACKS = link->NEXT_HOP_ACKS;
	link_to_root.LINK_DELAY_UNIT = link->LINK_DELAY_UNIT;
	link_to_root.LINK_DELAY = link->LINK_DELAY;
	link_to_root.LINK_DELAY_ERROR = link->LINK_DELAY_ERROR;
	return SIOT_MESH_RET_OK;
}

uint8_t siot_mesh_remove_link( uint16_t link_id )
{
	if ( link_id == 0 )
	{
		link_to_root.valid = 0;
		return SIOT_MESH_RET_OK;
	}
	else
		return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_delete_route( uint16_t target_id )
{
	if ( target_id == 0 )
	{
		link_to_root.valid = 0;
		return SIOT_MESH_RET_OK;
	}
	else
		return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_add_route( uint16_t target_id, uint16_t link_id )
{
	ZEPTO_DEBUG_ASSERT( target_id == 0 );
	ZEPTO_DEBUG_ASSERT( link_id == 0 );
	return SIOT_MESH_RET_OK;
}

uint16_t siot_mesh_calculate_route_table_checksum()
{
	return 0;
}

#define PENDING_RESENDS_MAX 2 // WARNING: if changed, have more handles (at least, potentially)!
MESH_PENDING_RESENDS pending_resends[PENDING_RESENDS_MAX] = { {MEMORY_HANDLE_MESH_1, 0}, { MEMORY_HANDLE_MESH_2, 0} };

#endif // USED_AS_RETRANSMITTER


#define SIOT_MESH_LAST_HOP_DATA_MAX 4

static SIOT_MESH_LAST_HOP_DATA _last_hops_0[ SIOT_MESH_LAST_HOP_DATA_MAX ];
static SIOT_MESH_LAST_HOP_DATA _last_hops_1[ SIOT_MESH_LAST_HOP_DATA_MAX ];

typedef struct _SIOT_MESH_LAST_REQUEST
{
	uint16_t rq_id;
//	MEMORY_HANDLE hop_list;
	SIOT_MESH_LAST_HOP_DATA* hop_list;
	uint8_t hop_list_sz;
	bool ineffect;
} SIOT_MESH_LAST_REQUEST;

static SIOT_MESH_LAST_REQUEST last_requests[2] = 
{
	{0xFFFF, _last_hops_0, 0, false},
	{0xFFFF, _last_hops_1, 0, false},
};

//static uint16_t last_rq_id;

#define SIOT_MESH_QUALITY_OF_FIRST_IS_BETTER( a, b ) (a) < (b)

void siot_mesh_add_last_hop_data( uint8_t slot_id, uint16_t last_hop_id, uint16_t last_hop_bus_id, uint8_t conn_q )
{
	ZEPTO_DEBUG_ASSERT( slot_id < 2 );
	SIOT_MESH_LAST_HOP_DATA* last_hops = last_requests[slot_id].hop_list;
	// we keep this array sorted in order of packets (with the same request id
	if ( last_requests[slot_id].hop_list_sz < SIOT_MESH_LAST_HOP_DATA_MAX )
	{
		last_hops[last_requests[slot_id].hop_list_sz].last_hop_id = last_hop_id;
		last_hops[last_requests[slot_id].hop_list_sz].last_hop_bus_id = last_hop_bus_id;
		last_hops[last_requests[slot_id].hop_list_sz].conn_quality = conn_q;
		(last_requests[slot_id].hop_list_sz)++;
	}
	else
	{
		ZEPTO_DEBUG_ASSERT( last_requests[slot_id].hop_list_sz == SIOT_MESH_LAST_HOP_DATA_MAX );
		uint8_t i;
		uint8_t worst = 0;
		for ( i=1; i<SIOT_MESH_LAST_HOP_DATA_MAX; i++)
			if ( ! SIOT_MESH_QUALITY_OF_FIRST_IS_BETTER( last_hops[ i ].conn_quality, last_hops[ worst ].conn_quality ) )
				worst = i;
		if ( SIOT_MESH_QUALITY_OF_FIRST_IS_BETTER( conn_q, last_hops[ worst ].conn_quality ) )
		{
			ZEPTO_MEMMOV( last_hops + worst, last_hops + worst + 1, ( SIOT_MESH_LAST_HOP_DATA_MAX - worst - 1 ) * sizeof( SIOT_MESH_LAST_HOP_DATA ) );
			last_hops[ SIOT_MESH_LAST_HOP_DATA_MAX - 1 ].last_hop_id = last_hop_id;
			last_hops[ SIOT_MESH_LAST_HOP_DATA_MAX - 1 ].last_hop_bus_id = last_hop_bus_id;
			last_hops[ SIOT_MESH_LAST_HOP_DATA_MAX - 1 ].conn_quality = conn_q;
		}
	}
}

void siot_mesh_init_last_hop_data_storage( uint8_t slot_id, uint16_t request_id, uint16_t last_hop_id, uint16_t last_hop_bus_id, uint8_t conn_q )
{
	ZEPTO_DEBUG_ASSERT( slot_id < 2 );
	last_requests[slot_id].hop_list_sz = 0;
	last_requests[slot_id].ineffect = true;
	last_requests[slot_id].rq_id = request_id;
	siot_mesh_add_last_hop_data( slot_id, last_hop_id, last_hop_bus_id, conn_q );
}

bool siot_mesh_clean_last_hop_data_storage_if_single_element( uint8_t slot_id )
{
	ZEPTO_DEBUG_ASSERT( slot_id < 2 );
	ZEPTO_DEBUG_ASSERT( last_requests[slot_id].hop_list_sz != 0 );
	if ( last_requests[slot_id].hop_list_sz == 1 )
	{
		last_requests[slot_id].hop_list_sz = 0;
		last_requests[slot_id].ineffect = false;
		return true;
	}
	return false;
}

void siot_mesh_write_last_hop_data_as_opt_headers( uint8_t slot_id, MEMORY_HANDLE mem_h, bool no_more_headers, uint16_t* request_id )
{
	ZEPTO_DEBUG_ASSERT( slot_id < 2 );
	SIOT_MESH_LAST_HOP_DATA* last_hops = last_requests[slot_id].hop_list;
	uint8_t i;
	ZEPTO_DEBUG_ASSERT( last_requests[slot_id].hop_list_sz <= SIOT_MESH_LAST_HOP_DATA_MAX );
	*request_id = last_requests[slot_id].rq_id;
	ZEPTO_DEBUG_ASSERT( last_requests[slot_id].hop_list_sz > 0 ); // is it at all possible to have here 0 at time of calling? 
	for ( i=0; i<last_requests[slot_id].hop_list_sz-1; i++)
	{
		// TODO: use bit-field processing where applicable
		uint16_t header = 1 | ( SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP << 1 ) | ( last_hops[ i ].last_hop_id << 4 );
		zepto_parser_encode_and_append_uint16( mem_h, header );
		zepto_parser_encode_and_append_uint16( mem_h, last_hops[ i ].last_hop_bus_id );
		zepto_parser_encode_and_append_uint8( mem_h, last_hops[ i ].conn_quality );
	}
	ZEPTO_DEBUG_ASSERT( i == last_requests[slot_id].hop_list_sz - 1 ); 
	uint16_t header = ( no_more_headers ? 0: 1 ) | ( SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP << 1 ) | ( last_hops[ i ].last_hop_id << 4 );
	zepto_parser_encode_and_append_uint16( mem_h, header );
	zepto_parser_encode_and_append_uint16( mem_h, last_hops[ i ].last_hop_bus_id );
	zepto_parser_encode_and_append_uint8( mem_h, last_hops[ i ].conn_quality );
}

#endif // USED_AS_MASTER


////////////////////////////  ROUTE and LINK table processing   //////////////////////////////////

#ifdef USED_AS_MASTER

void siot_mesh_form_ack_packet( MEMORY_HANDLE mem_h, uint16_t target_id, uint16_t num_err, uint16_t ack_checksum ) // TODO: check whether it is the same as for DEVICE, and move to common part, if so
{
	// Samp-Ack-Nack-Packet: | SAMP-ACK-NACK-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | Target-Address | NUMBER-OF-ERRORS | ACK-CHESKSUM | HEADER-CHECKSUM | OPTIONAL-DELAY-UNIT | OPTIONAL-DELAY-PASSED | OPTIONAL-DELAY-LEFT |
	// SAMP-ACK-NACK-AND-TTL: encoded uint16, bit[0]=1, bits[1..3] = SAMP_ACK_NACK_PACKET, bit [4] = EXTRA-HEADERS-PRESENT, and bits [5..] being TTL
	bool add_opt_delays = false;
	uint16_t header;
	// TODO: if extra headers are to be added, edit lines below
	if ( add_opt_delays )
	{
		header = 1 | ( SIOT_MESH_ACK_NACK_PACKET << 1 ) | ( 1 << 4 ); // '1', packet type, 1 (at least one extra header: hop list item), rezerved (zeros)
		zepto_parser_encode_and_append_uint16( mem_h, header );
		ZEPTO_DEBUG_ASSERT( 0 == "Adding optional headers in SIOT_MESH_ACK_NACK_PACKET is not implemented yet" );
	}
	else
	{
		header = 1 | ( SIOT_MESH_ACK_NACK_PACKET << 1 ) | ( 0 << 4 ); // '1', packet type, 0 (no extra headers), rezerved (zeros)
		zepto_parser_encode_and_append_uint16( mem_h, header );
	}

	// LAST-HOP
	zepto_parser_encode_and_append_uint16( mem_h, 0 );

	// Target-Address
	header = 0 | ( target_id << 1 ); // NODE-ID, no more data
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// NUMBER-OF-ERRORS
	zepto_parser_encode_and_append_uint16( mem_h, num_err );

	// ACK-CHESKSUM
	zepto_parser_encode_and_append_uint16( mem_h, ack_checksum );

	// HEADER-CHECKSUM
	uint16_t rsp_sz = memory_object_get_response_size( mem_h );
	uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	if ( add_opt_delays )
	{
		ZEPTO_DEBUG_ASSERT( 0 == "Adding optional delays in SIOT_MESH_ACK_NACK_PACKET is not implemented yet" );
		// OPTIONAL-DELAY-UNIT
					
		// OPTIONAL-DELAY-PASSED
					
		// OPTIONAL-DELAY-LEFT
	}
}
#if 0
void siot_mesh_form_packet_from_santa( MEMORY_HANDLE mem_h, uint16_t target_id, uint16_t bus_id_used )
{
	// ASSUMPTIONS OF THE CURRENT IMPLEMENTATION
	// 1. As seen from this function parameters in the current implementation we assume only one device at a time to be found
	// 2. We also assume that the Root has a single bus

	// Santa Packet structure: | SAMP-FROM-SANTA-DATA-PACKET-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | REQUEST-ID | OPTIONAL-DELAY-UNIT | MULTIPLE-RETRANSMITTING-ADDRESSES | BROADCAST-BUS-TYPE-LIST | Target-Address | OPTIONAL-TARGET-REPLY-DELAY | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
	// TODO: here and then use bit-field processing instead

	parser_obj po, po1;

	// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, OPTIONAL-EXTRA-HEADERS
	uint16_t header = 1 | ( SIOT_MESH_FROM_SANTA_DATA_PACKET << 1 ) | ( 4 << 5 ); // '1', packet type, 0 (no extra headers), TTL = 4
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// LAST-HOP
	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // ROOT

	// LAST-HOP-BUS-ID
	zepto_parser_encode_and_append_uint16( mem_h, bus_id_used );

	// REQUEST-ID
	static uint16_t rq_id = 0; // REQUEST-ID
	rq_id++; // TODO: make true global; TODO: think whether this id is globally or per-device unique
	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // REQUEST-ID

	// OPTIONAL-DELAY-UNIT is present only if EXPLICIT-TIME-SCHEDULING flag is present; currently we did not added it

	// MULTIPLE-RETRANSMITTING-ADDRESSES 
	uint16_t retransmitter_count = write_retransmitter_list_for_from_santa_packet( mem_h );

	// BROADCAST-BUS-TYPE-LIST
	write_bus_types_for_device_for_from_santa_packet( mem_h, target_id );

	// Target-Address
	header = 0 | ( target_id << 1 ); // NODE-ID, no more data
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// OPTIONAL-TARGET-REPLY-DELAY

	// OPTIONAL-PAYLOAD-SIZE

	// HEADER-CHECKSUM
	uint16_t rsp_sz = memory_object_get_response_size( mem_h );
	uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	// PAYLOAD
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	zepto_parse_skip_block( &po1, zepto_parsing_remaining_bytes( &po ) );
	zepto_append_part_of_request_to_response( mem_h, &po, &po1 );

	// FULL-CHECKSUM
	checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );
}
#endif // 0

void siot_mesh_form_unicast_packet( uint16_t target_id, MEMORY_HANDLE mem_h, uint16_t link_id, uint16_t* bus_id, bool request_ack, uint16_t* packet_checksum )
{
	//  | SAMP-UNICAST-DATA-PACKET-FLAGS-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | LAST-HOP | Non-Root-Address | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

	parser_obj po, po1;
	uint16_t header;
	SIOT_MESH_LINK link;
	uint8_t ret_code = siot_mesh_get_link( link_id, &link );
	ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK ); // we do not expect that being here we have an invalid link_id
	ZEPTO_DEBUG_ASSERT( link.LINK_ID == link_id );
	*bus_id = link.BUS_ID;
	// !!! TODO: it might happen that this packet is not a reply to 'from Santa'; check, which data should be added this case

	// header: bit[0]: 0, bit[1]: ACKNOWLEDGED-DELIVERY flag, bit[2]: 0, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL
	uint16_t ttl = 4; // TODO: source??
	// TODO: set other fields as necessary
	header = 0 | ( request_ack ? 2 : 0 ) | ( 1 << 4 ) | ( ttl << 5 );
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// OPTIONAL-EXTRA-HEADERS
	// (none so far)

	// NEXT-HOP
	zepto_parser_encode_and_append_uint16( mem_h, link.NEXT_HOP );

	// LAST-HOP
	zepto_parser_encode_and_append_uint16( mem_h, 0 );

	// Non-Root-Address
	// we implement quick coding assuming no extra data follow
	// TODO: full implementation with VIA fields, etc
	zepto_parser_encode_and_append_uint16( mem_h, target_id << 1 );

	// OPTIONAL-PAYLOAD-SIZE

	// HEADER-CHECKSUM
	uint16_t rsp_sz = memory_object_get_response_size( mem_h );
	uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	// PAYLOAD
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	zepto_parse_skip_block( &po1, zepto_parsing_remaining_bytes( &po ) );
	zepto_append_part_of_request_to_response( mem_h, &po, &po1 );

	// FULL-CHECKSUM
	checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	*packet_checksum = checksum;
}

uint8_t handler_siot_mesh_send_packet( sa_time_val* currt, waiting_for* wf, uint16_t target_id, MEMORY_HANDLE mem_h, uint8_t resend_cnt, uint16_t* bus_id )
{
	uint16_t link_id;
	uint8_t ret_code = siot_mesh_at_root_target_to_link_id( target_id, &link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		if ( resend_cnt < SIOT_MESH_SUBJECT_FOR_ACK )
		{
			uint16_t checksum;
			siot_mesh_form_unicast_packet( target_id, mem_h, link_id, bus_id, false, &checksum );
			ZEPTO_DEBUG_PRINTF_1( "         ############  handler_siot_mesh_send_packet(): known route  ###########\n" );
			return SIOT_MESH_RET_OK;
		}

		uint16_t checksum;
		siot_mesh_form_unicast_packet( target_id, mem_h, link_id, bus_id, true, &checksum );
		sa_time_val wait_tval;
		siot_mesh_at_root_add_resend_task( mem_h, currt, checksum, target_id, *bus_id, &wait_tval );
		sa_hal_time_val_copy_from_if_src_less( &(wf->wait_time), &wait_tval );

		return SIOT_MESH_RET_PASS_TO_SEND;
	}
	else
	{
		ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );

		// packet "from Santa" will be sent... let's form a packet

		// TODO: determine which physical links we will use; we will have to iterate over all of them
		// NOTE: currently we assume that we have a single link with bus_id = 0
//		zepto_response_to_request( mem_h );
		siot_mesh_form_packets_from_santa_and_add_to_task_list( currt, wf, mem_h, target_id );
		zepto_parser_free_memory( mem_h );

		return SIOT_MESH_RET_OK;
	}
}

uint8_t siot_mesh_process_received_tosanta_or_forwardtosanta_packet( MEMORY_HANDLE mem_h, uint16_t src_id, uint16_t bus_id_at_src, uint16_t first_receiver_id, uint8_t conn_quality_at_first_receiver, bool payload_present, uint16_t request_id )
{
	// here we already know that the packet is good enough
	parser_obj po, po1, po2;
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	uint16_t total_packet_sz = zepto_parsing_remaining_bytes( &po );

	uint16_t header = zepto_parse_encoded_uint16( &po );
	// TODO: here and then use bit-field processing instead
	ZEPTO_DEBUG_ASSERT ( header & 1 ); // must already be checked
	uint16_t packet_type = ( header >> 1 ) & 0x7;
	ZEPTO_DEBUG_ASSERT ( packet_type == SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET || packet_type == SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET ); // must already be checked

	// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, presence of OPTIONAL-EXTRA-HEADERS
	bool extra_headers_present = ( header >> 4 ) & 0x1;
	uint16_t TTL = header >> 5;
	// now we're done with the header; proceeding to optional headers...
	while ( extra_headers_present )
	{
		header = zepto_parse_encoded_uint16( &po );
		extra_headers_present = header & 0x1;
		uint8_t generic_flags = (header >> 1) & 0x3; // bits[1,2]
		switch ( generic_flags )
		{
			case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
			{
				uint16_t last_hop_id = header >> 4;
				uint16_t last_hop_bus_id = zepto_parse_encoded_uint16( &po ); 
				uint8_t conn_q = zepto_parse_encoded_uint8( &po );
				siot_mesh_at_root_add_last_hop_in_data( src_id, last_hop_id, last_hop_bus_id, conn_q ); 
				break;
			}
			case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
			case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
			case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
			{
				ZEPTO_DEBUG_ASSERT( NULL == "Error: other optional headers are not implemented\n" );
				break;
			}
			default:
			{
				ZEPTO_DEBUG_ASSERT( NULL == "Error: unexpected type of optional header\n" );
				break;
			}
		}
	}

	if ( packet_type == SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET )
	{
		// NEXT-HOP
		uint16_t next_hop = zepto_parse_encoded_uint16( &po );
#ifdef SA_DEBUG
		ZEPTO_DEBUG_ASSERT( next_hop == 0 );
#endif // SA_DEBUG
	}

	// SOURCE-ID
#ifdef SA_DEBUG
	ZEPTO_DEBUG_ASSERT( src_id == zepto_parse_encoded_uint16( &po ) ); // must coincide
#else // SA_DEBUG
	zepto_parse_encoded_uint16( &po ); // just skip
#endif // SA_DEBUG

	// BUS-ID-AT-SOURCE
#ifdef SA_DEBUG
	ZEPTO_DEBUG_ASSERT( bus_id_at_src == zepto_parse_encoded_uint16( &po ) ); // must coincide
#else // SA_DEBUG
	zepto_parse_encoded_uint16( &po ); // just skip
#endif // SA_DEBUG

	// REQUEST-ID
#ifdef SA_DEBUG
	ZEPTO_DEBUG_ASSERT( request_id == zepto_parse_encoded_uint16( &po ) ); // must coincide
#else // SA_DEBUG
	zepto_parse_encoded_uint16( &po ); // just skip
#endif // SA_DEBUG


	siot_mesh_at_root_add_last_hop_out_data( src_id, bus_id_at_src, first_receiver_id, conn_quality_at_first_receiver );

	// OPTIONAL-PAYLOAD-SIZE

	// HEADER-CHECKSUM
#ifdef SA_DEBUG
	uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
	uint16_t checksum = zepto_parse_uint8( &po );
	checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
	ZEPTO_DEBUG_ASSERT( actual_checksum == checksum );
#else // SA_DEBUG
	// just skip - everything must already be checked since we're here
	zepto_parse_uint8( &po );
	zepto_parse_uint8( &po );
#endif // SA_DEBUG

#ifdef SA_DEBUG
	uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
	ZEPTO_DEBUG_ASSERT( remaining_size >= 2 ); // has already been checked by a caller
	if ( payload_present )
	{
		parser_obj rq_start_po;
		uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
		ZEPTO_DEBUG_ASSERT( remaining_size > 2 );
		zepto_parser_init_by_parser( &po2, &po );
		zepto_parser_init_by_parser( &rq_start_po, &po );
		zepto_parse_skip_block( &po, remaining_size - 2 );
		zepto_append_part_of_request_to_response( mem_h, &po2, &po );
		actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &rq_start_po, remaining_size - 2, actual_checksum );
		checksum = zepto_parse_uint8( &po );
		checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
		ZEPTO_DEBUG_ASSERT( actual_checksum == checksum );
		return SIOT_MESH_RET_PASS_TO_PROCESS;
	}
	else
	{
		uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
		ZEPTO_DEBUG_ASSERT( remaining_size == 2 );
		actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po, 0, actual_checksum );
		checksum = zepto_parse_uint8( &po );
		checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
		ZEPTO_DEBUG_ASSERT( actual_checksum == checksum );
		return SIOT_MESH_RET_OK;
	}
#else // SA_DEBUG
	if ( payload_present )
	{
		uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
		zepto_parser_init_by_parser( &po1, &po );
		zepto_parse_skip_block( &po, remaining_size - 2 );
		zepto_convert_part_of_request_to_response( mem_h, &po2, &po );
		return SIOT_MESH_RET_PASS_TO_PROCESS;
	}
	else
		return SIOT_MESH_RET_OK;
#endif // SA_DEBUG
}

uint8_t handler_siot_mesh_prepare_route_update( MEMORY_HANDLE mem_h, uint16_t* recipient )
{
	uint8_t ret_code;
	uint16_t flags = 0;

	// TEMPORARY CODE: add ccp staff
	zepto_write_uint8( mem_h, 0x5 ); // first, control
	zepto_write_uint8( mem_h, 0x5 ); // SACCP_PHY_AND_ROUTING_DATA

	zepto_parser_encode_and_append_uint16( mem_h, flags );

	// here we should add initial checksum
	zepto_write_uint8( mem_h, 0 );
	zepto_write_uint8( mem_h, 0 );

	ret_code = siot_mesh_at_root_load_update_to_packet( mem_h, recipient );
	if ( ret_code != SIOT_MESH_RET_OK )
		return ret_code;

	// here we should add resulting checksum
	zepto_write_uint8( mem_h, 0 );
	zepto_write_uint8( mem_h, 0 );

	return SIOT_MESH_RET_OK;
}

void handler_siot_mesh_process_route_update_response( uint16_t source_dev_id, MEMORY_HANDLE mem_h )
{
	parser_obj po;
	zepto_parser_init( &po, mem_h );
	uint8_t main_byte = zepto_parse_uint8( &po );
	if ( main_byte == 0 )
	{
		// we remove the latest update to this device from collection of updates
		siot_mesh_at_root_update_done( source_dev_id );
	}
	else
	{
		ZEPTO_DEBUG_ASSERT( 0 == "Route table update error processing is not yet implemented" );
	}
}

uint8_t handler_siot_mesh_timer( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t* device_id, uint16_t* bus_id )
{
	// WARNING: this development is just in the middle ... don't be surprized ...
	// NOTE: there are a number of things that can be done by timer; on this development stage we assume that they happen somehow in the order as implemented
	// TODO: actual implementation

	ZEPTO_DEBUG_ASSERT( memory_object_get_response_size( mem_h ) == 0 );
	*device_id = SIOT_MESH_TARGET_UNDEFINED;
	*bus_id = SIOT_MESH_BUS_UNDEFINED;

	uint8_t ret_code = siot_mesh_at_root_get_resend_task( mem_h, currt, device_id, bus_id, &(wf->wait_time) );
	switch (ret_code )
	{
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_NONE_EXISTS:
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_NOT_NOW:
			break;
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_INTERM:
		{
			ZEPTO_DEBUG_ASSERT( *device_id != SIOT_MESH_TARGET_UNDEFINED );
			uint16_t link_id;
			zepto_response_to_request( mem_h );
			uint16_t checksum;
			bool route_known = siot_mesh_at_root_target_to_link_id( *device_id, &link_id ) == SIOT_MESH_RET_OK;
			if ( route_known )
			{
				siot_mesh_form_unicast_packet( *device_id, mem_h, link_id, bus_id, true, &checksum );
				return SIOT_MESH_RET_PASS_TO_SEND;
			}
			else
			{
				siot_mesh_at_root_remove_resend_task_by_device_id( *device_id, currt, &(wf->wait_time) );
				// TODO: determine which physical links we will use; we will have to iterate over all of them
				// NOTE: currently we assume that we have a single link with bus_id = 0
				siot_mesh_form_packets_from_santa_and_add_to_task_list( currt, wf, mem_h, *device_id );
				return SIOT_MESH_RET_OK;
			}
			break;
		}
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_FINAL:
		{
			ZEPTO_DEBUG_ASSERT( *device_id != SIOT_MESH_TARGET_UNDEFINED );
			zepto_response_to_request( mem_h );
			siot_mesh_at_root_remove_link_to_target_no_ack_from_immediate_hop( *device_id );
			siot_mesh_form_packets_from_santa_and_add_to_task_list( currt, wf, mem_h, *device_id );
			return SIOT_MESH_RET_OK;
			break;
		}
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_FROM_SANTA:
		{
			ZEPTO_DEBUG_ASSERT( *bus_id != SIOT_MESH_BUS_UNDEFINED );
			return SIOT_MESH_RET_PASS_TO_SEND;
			break;
		}
	}
	zepto_parser_free_memory( mem_h );

	static bool route_table_created = false;
	static bool hop_data_added = false;
	uint16_t target_id;
//	if ( !hop_data_added )
	{
		uint16_t bus_id_at_target;
		uint16_t id_from;
		uint16_t bus_id_at_prev;
		uint16_t id_next;
		uint8_t ret_code = siot_mesh_at_root_find_best_route( &target_id, &bus_id_at_target, &id_from, &bus_id_at_prev, &id_next );
		if ( ret_code == SIOT_MESH_AT_ROOT_RET_OK )
		{
			siot_mesh_at_root_remove_last_hop_data( target_id );
			ret_code = siot_mesh_at_root_add_updates_for_device_when_route_is_added( target_id, bus_id_at_target, id_from, bus_id_at_prev, id_next /*more data may be required*/ );
			hop_data_added = true;
		}
	}
//	if ( !route_table_created )
	{
		ret_code = handler_siot_mesh_prepare_route_update( mem_h, &target_id );
		if ( ret_code == SIOT_MESH_RET_OK )
		{
			route_table_created = true;
			*device_id = target_id;
			return SIOT_MESH_RET_PASS_TO_CCP;
		}
		else
			zepto_parser_free_response( mem_h );
	}
		
	return SIOT_MESH_RET_OK;
}

uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint16_t* src_id, uint8_t conn_quality, uint8_t error_cnt )
{
	// When a packet just comes, it can be anything starting from total garbage through incomplete packet good for sending NAK only to a packet that is good for further processing.
	// Complete processing may require a number of steps that are hard to revert, if performed, and a packet is not good enough for processing.
	// Since that we first try to quickly figure out what kind of "joy" we have actually received, and then to perform full respective processing.


	parser_obj po, po1, po2;
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	uint16_t total_packet_sz = zepto_parsing_remaining_bytes( &po );

#ifdef SA_DEBUG
	{
		uint16_t i;
		parser_obj podbg;
		zepto_parser_init( &podbg, mem_h );
		ZEPTO_DEBUG_PRINTF_2( "handler_siot_mesh_receive_packet(): packet received: sz = %d, packet = ", total_packet_sz );
		for ( i=0; i<total_packet_sz; i++)
			ZEPTO_DEBUG_PRINTF_2( "%02x ", zepto_parse_uint8( &podbg ) );
		ZEPTO_DEBUG_PRINTF_1( "\n" );
	}
#endif // SA_DEBUG

	uint16_t header = zepto_parse_encoded_uint16( &po );
	// TODO: here and then use bit-field processing instead
	if ( header & 1 ) // predefined packet types
	{
		uint8_t packet_type = ( header >> 1 ) & 0x7;
		switch ( packet_type )
		{
			case SIOT_MESH_ACK_NACK_PACKET:
			{
				// Samp-Ack-Nack-Packet: | SAMP-ACK-NACK-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | Target-Address | NUMBER-OF-ERRORS | ACK-CHESKSUM | HEADER-CHECKSUM | OPTIONAL-DELAY-UNIT | OPTIONAL-DELAY-PASSED | OPTIONAL-DELAY-LEFT |
				// SAMP-ACK-NACK-AND-TTL: encoded uint16, bit[0]=1, bits[1..3] = SAMP_ACK_NACK_PACKET, bit [4] = EXTRA-HEADERS-PRESENT, and bits [5..] being TTL

				// WARNING: we should not do any irreversible changes until checksums are virified; 
				//          thus, until that point all data should only be locally collected, and applied only after checksum verification (or do two passes)
				bool extra_headers_present = ( header >> 4 ) & 0x1;
				uint16_t TTL = header >> 5;

				bool delay_flag_present = false;

				// now we're done with the header; proceeding to optional headers...
				while ( extra_headers_present )
				{
					ZEPTO_DEBUG_ASSERT( 0 == "optional headers in \'ACK_NACK\' packet are not yet implemented" );
				}

				// LAST-HOP
				uint16_t last_hop_id = zepto_parse_encoded_uint16( &po );
				ZEPTO_DEBUG_ASSERT( last_hop_id != 0 ); // any but ROOT

				// Target-Address
				header = zepto_parse_encoded_uint16( &po );
				ZEPTO_DEBUG_ASSERT( (header & 1) == 0 ); // we have not yet implemented extra data
				uint16_t target_id = header >> 1;
				if ( target_id != 0 )
				{
					ZEPTO_DEBUG_PRINTF_2( "Packet for device %d received (from Santa); ignored\n", target_id );
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				}

				// NUMBER-OF-ERRORS
				uint16_t num_of_errors = zepto_parse_encoded_uint16( &po );

				// ACK-CHESKSUM
				uint16_t ack_checksum = zepto_parse_uint8( &po );
				ack_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;

				// HEADER-CHECKSUM
				uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
				uint16_t checksum = zepto_parse_uint8( &po );
				checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
				zepto_parser_init_by_parser( &po1, &po );
				if ( actual_checksum != checksum ) // we have not received even a header
				{
					// TODO: cleanup, if necessary
					return SIOT_MESH_RET_GARBAGE_RECEIVED;
				}

				if ( delay_flag_present )
				{
					// OPTIONAL-DELAY-UNIT
					
					// OPTIONAL-DELAY-PASSED
					
					// OPTIONAL-DELAY-LEFT
				}

				siot_mesh_at_root_remove_resend_task_by_hash( checksum, currt, &(wf->wait_time) );

				return SIOT_MESH_RET_OK;
			}
			case SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET:
			case SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET:
			{
				// NOTE: processing this packet leads to updating of certain MESH data structures.
				//       since packet can be broken in a number of ways, we have two options:
				//       (1) apply changes immeditely when a respective part of the packet is parsed, and be ready to roll them back, if packet is broken; or
				//       (2) check packet integrity and only then apply changes
				//       In current implementation we have selected a second approach.

				// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, presence of OPTIONAL-EXTRA-HEADERS
				bool extra_headers_present = ( header >> 4 ) & 0x1;
				// uint16_t TTL = header >> 5;
				// now we're done with the header; proceeding to optional headers...
				while ( extra_headers_present )
				{
					header = zepto_parse_encoded_uint16( &po );
					extra_headers_present = header & 0x1;
					uint8_t generic_flags = (header >> 1) & 0x3; // bits[1,2]
					switch ( generic_flags )
					{
						case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
						{
							zepto_parse_encoded_uint16( &po ); // last_hop_bus_id; just skip
							zepto_parse_encoded_uint8( &po ); // connection quality; just skip
							break;
						}
						case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
						case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
						case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: other optional headers are not implemented\n" );
							break;
						}
						default:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: unexpected type of optional header\n" );
							break;
						}
					}
				}

				if ( packet_type == SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET )
				{
					// NEXT-HOP
					uint16_t next_hop = zepto_parse_encoded_uint16( &po );
					if ( next_hop != 0 )
					{
						ZEPTO_DEBUG_PRINTF_2( "Packet for device %d received (unicast); ignored\n", next_hop );
						return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
					}
				}

				// (FORWARDED-)SOURCE-ID
				*src_id = zepto_parse_encoded_uint16( &po );

				// (FORWARDED-)BUS-ID-AT-SOURCE
				uint16_t bus_id_at_src = zepto_parse_encoded_uint16( &po );

				// REQUEST-ID
				uint16_t request_id = zepto_parse_encoded_uint16( &po );

				// OPTIONAL-PAYLOAD-SIZE

				// HEADER-CHECKSUM
				uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
				uint16_t checksum = zepto_parse_uint8( &po );
				checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
				zepto_parser_init_by_parser( &po1, &po );

				if ( actual_checksum != checksum ) // we have not received even a header -- total garbage received
					return SIOT_MESH_RET_GARBAGE_RECEIVED;

				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
				if ( remaining_size >= 2 )
				{
					zepto_parser_init_by_parser( &po2, &po );
					zepto_parse_skip_block( &po, remaining_size - 2 );
					actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po2, remaining_size - 2, actual_checksum );
					checksum = zepto_parse_uint8( &po );
					checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
					if ( actual_checksum != checksum )
					{
						// TODO: we have only a partially received packet; prepare NACK
						ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NACK is not yet implemented\n" );
						return SIOT_MESH_RET_PASS_TO_SEND;
					}
					else
					{
						// Note: don't be surprized by implementation of call below: we cannot add data until all checks are done; here we check, and there we add
						return siot_mesh_process_received_tosanta_or_forwardtosanta_packet( mem_h, *src_id, bus_id_at_src, 0, conn_quality, remaining_size > 2, request_id );
					}
				}
				else // packet is broken; subject for NAK
				{
					// TODO: we have only a partially received packet; prepare NACK
					ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NACK is not yet implemented\n" );
					return SIOT_MESH_RET_PASS_TO_SEND;
				}

				ZEPTO_DEBUG_ASSERT( NULL == "Error: we should not reach this point\n" );
				break;
			}
			case SIOT_MESH_ROUTING_ERROR_PACKET:
			{
				// Samp-Ack-Nack-Packet: | SAMP-ACK-NACK-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | Target-Address | NUMBER-OF-ERRORS | ACK-CHESKSUM | HEADER-CHECKSUM | OPTIONAL-DELAY-UNIT | OPTIONAL-DELAY-PASSED | OPTIONAL-DELAY-LEFT |
				// SAMP-ACK-NACK-AND-TTL: encoded uint16, bit[0]=1, bits[1..3] = SAMP_ACK_NACK_PACKET, bit [4] = EXTRA-HEADERS-PRESENT, and bits [5..] being TTL

				// WARNING: we should not do any irreversible changes until checksums are virified; 
				//          thus, until that point all data should only be locally collected, and applied only after checksum verification (or do two passes)
				bool extra_headers_present = ( header >> 4 ) & 0x1;
				uint16_t TTL = header >> 5;

				// now we're done with the header; proceeding to optional headers...
				while ( extra_headers_present )
				{
					ZEPTO_DEBUG_ASSERT( 0 == "optional headers in \'ACK_NACK\' packet are not yet implemented" );
				}

				// ERROR-CODE
				uint8_t error_code = zepto_parse_uint8( &po );

				// HEADER-CHECKSUM
				uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
				uint16_t checksum = zepto_parse_uint8( &po );
				checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
				zepto_parser_init_by_parser( &po1, &po );

				if ( actual_checksum != checksum ) // we have not received even a header -- total garbage received
					return SIOT_MESH_RET_GARBAGE_RECEIVED;

				// packet integrity
				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
				bool packet_is_integral = remaining_size >= 2;
				if ( packet_is_integral )
				{
					zepto_parser_init_by_parser( &po1, &po );
					zepto_parser_init_by_parser( &po2, &po );
					zepto_parse_skip_block( &po1, remaining_size - 2 );
					actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po2, remaining_size - 2, actual_checksum );
					checksum = zepto_parse_uint8( &po1 );
					checksum |= ((uint16_t)zepto_parse_uint8( &po1 )) << 8;
					packet_is_integral = actual_checksum == checksum;
				}
				if ( packet_is_integral )
				{
					// NOTE/TODO: here we rely only on checksum-based integrity check; we should be ready to failure of any of zepto_parse_encoded_uint16() calls below
					// TODO: what should we do with incorrectly constructed packets?
					uint16_t reporting_device_id = zepto_parse_encoded_uint16( &po );
					uint16_t failed_hop_id = zepto_parse_encoded_uint16( &po );
					uint16_t failed_target = zepto_parse_encoded_uint16( &po );
					uint16_t original_packet_checksum = zepto_parse_encoded_uint16( &po );

					// TODO: think about potential cases, say, an error is reported based on the old state of the routing table while a new one is already in effect (may require more data to be added to the packet)
					siot_mesh_at_root_remove_link_to_target_route_error_reported( reporting_device_id, failed_hop_id );
					//+++++TODO: are there any other items to be scheduled/removed?
					return SIOT_MESH_RET_OK;
				}
				else // packet is broken; subject for NAK
				{
					// TODO: we have only a partially received packet; prepare NACK
					ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NACK is not yet implemented\n" );
					return SIOT_MESH_RET_PASS_TO_SEND;
				}
			}
			case SIOT_MESH_FROM_SANTA_DATA_PACKET:
			{
				return SIOT_MESH_RET_GARBAGE_RECEIVED; // TODO: do preliminary parsing to ensure that this packet has not been intended for the root (that is, there is no insane device)
			}
			default:
			{
				ZEPTO_DEBUG_PRINTF_2( "Error: packet type %d received; not expected or not implemented\n", (header >> 1) & 0x7 );
				ZEPTO_DEBUG_ASSERT( NULL == "Error: processing of mesh packets of this type is not implemented\n" );
				break;
			}
		}
	}
	else
	{
		//  | SAMP-UNICAST-DATA-PACKET-FLAGS-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | LAST-HOP | Non-Root-Address | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

		// header: bit[0]: 0, bit[1]: ACKNOWLEDGED-DELIVERY flag, bit[2]: 0, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL
		bool ack_requested = ( header >> 1 ) & 0x1;
		ZEPTO_DEBUG_ASSERT( 0 == (( header >> 2 ) & 0x1 ) ); // reserved
		bool extra_headers_present = ( header >> 3 ) & 0x1;
		bool is_from_root = ( header >> 4 ) & 0x1;

		if ( is_from_root )
		{
			ZEPTO_DEBUG_PRINTF_1( "Packet directed from ROOT received; ignored\n" );
			return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
		}
		// uint16_t TTL = header >> 5;
		// now we're done with the header; proceeding to optional headers...
		while ( extra_headers_present )
		{
			header = zepto_parse_encoded_uint16( &po );
			extra_headers_present = header & 0x1;
			uint8_t generic_flags = (header >> 1) & 0x3; // bits[1,2]
			switch ( generic_flags )
			{
				case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
				case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
				case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
				case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
				{
					ZEPTO_DEBUG_ASSERT( NULL == "Error: other optional headers are not implemented\n" );
					break;
				}
			}
		}

		// NEXT-HOP
		uint16_t next_hop = zepto_parse_encoded_uint16( &po );
		if ( next_hop != 0 )
		{
			ZEPTO_DEBUG_PRINTF_2( "Packet for device %d received (unicast); ignored\n", next_hop );
			return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
		}

		// LAST-HOP
		uint16_t last_hop = zepto_parse_encoded_uint16( &po );

		// Non-Root-Address
		// we implement quick coding assuming no extra data follow
		// TODO: full implementation with VIA fields, etc
		*src_id = zepto_parse_encoded_uint16( &po );
		ZEPTO_DEBUG_ASSERT( (*src_id & 1) == 0 ); // TODO: provide full implementation
		*src_id >>= 1;

		// OPTIONAL-PAYLOAD-SIZE

		// HEADER-CHECKSUM
		uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
		uint16_t header_checksum = zepto_parse_uint8( &po );
		header_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
		zepto_parser_init_by_parser( &po1, &po );

		if ( actual_checksum != header_checksum ) // we have not received even a header -- total garbage received
			return SIOT_MESH_RET_GARBAGE_RECEIVED;

		uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
		uint16_t packet_reported_checksum;
		if ( remaining_size >= 2 )
		{
			zepto_parser_init_by_parser( &po2, &po );
			zepto_parse_skip_block( &po, remaining_size - 2 );
			actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po2, remaining_size - 2, actual_checksum );
			packet_reported_checksum = zepto_parse_uint8( &po );
			packet_reported_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
			if ( actual_checksum != packet_reported_checksum )
			{
				// TODO: we have only a partially received packet; prepare NACK
				ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NACK is not yet implemented\n" );
				return SIOT_MESH_RET_PASS_TO_SEND;
			}
			else
			{
				// Note: don't be surprized by implementation of call below: we cannot add data until all checks are done; here we check, and there we add
				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po2 );
				zepto_parser_init_by_parser( &po1, &po2 );
				zepto_parse_skip_block( &po1, remaining_size - 2 );
				zepto_convert_part_of_request_to_response( mem_h, &po2, &po1 );

				if ( ack_requested )
				{
					zepto_parser_free_memory( mem_ack_h );
					siot_mesh_form_ack_packet( mem_ack_h, last_hop, error_cnt, packet_reported_checksum );
					return SIOT_MESH_RET_SEND_ACK_AND_PASS_TO_PROCESS;
				}
				return SIOT_MESH_RET_PASS_TO_PROCESS;
			}
		}
		else // packet is broken; subject for NAK
		{
			// TODO: we have only a partially received packet; prepare NACK
			ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NACK is not yet implemented\n" );
			return SIOT_MESH_RET_PASS_TO_SEND;
		}

		ZEPTO_DEBUG_ASSERT( NULL == "Error: we should not reach this point\n" );

		return SIOT_MESH_RET_PASS_TO_PROCESS;
	}
	return SIOT_MESH_RET_OK;
}

#else // USED_AS_MASTER

void siot_mesh_form_ack_packet( MEMORY_HANDLE mem_h, uint16_t target_id, uint16_t num_err, uint16_t ack_checksum )
{
	// Samp-Ack-Nack-Packet: | SAMP-ACK-NACK-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | Target-Address | NUMBER-OF-ERRORS | ACK-CHESKSUM | HEADER-CHECKSUM | OPTIONAL-DELAY-UNIT | OPTIONAL-DELAY-PASSED | OPTIONAL-DELAY-LEFT | FULL-CHECKSUM |
	// SAMP-ACK-NACK-AND-TTL: encoded uint16, bit[0]=1, bits[1..3] = SAMP_ACK_NACK_PACKET, bit [4] = EXTRA-HEADERS-PRESENT, and bits [5..] being TTL
	bool add_opt_delays = false;
	uint16_t header;
	// TODO: if extra headers are to be added, edit lines below
	if ( add_opt_delays )
	{
		header = 1 | ( SIOT_MESH_ACK_NACK_PACKET << 1 ) | ( 1 << 4 ); // '1', packet type, 1 (at least one extra header: hop list item), rezerved (zeros)
		zepto_parser_encode_and_append_uint16( mem_h, header );
		ZEPTO_DEBUG_ASSERT( 0 == "Adding optional headers in SIOT_MESH_ACK_NACK_PACKET is not implemented yet" );
	}
	else
	{
		header = 1 | ( SIOT_MESH_ACK_NACK_PACKET << 1 ) | ( 0 << 4 ); // '1', packet type, 0 (no extra headers), rezerved (zeros)
		zepto_parser_encode_and_append_uint16( mem_h, header );
	}

	// LAST-HOP
	zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID );

	// Target-Address
	header = 0 | ( target_id << 1 ); // NODE-ID, no more data
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// NUMBER-OF-ERRORS
	zepto_parser_encode_and_append_uint16( mem_h, num_err );

	// ACK-CHESKSUM
	zepto_parser_encode_and_append_uint16( mem_h, ack_checksum );

	// HEADER-CHECKSUM
	uint16_t rsp_sz = memory_object_get_response_size( mem_h );
	uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	if ( add_opt_delays )
	{
		ZEPTO_DEBUG_ASSERT( 0 == "Adding optional delays in SIOT_MESH_ACK_NACK_PACKET is not implemented yet" );
		// OPTIONAL-DELAY-UNIT
					
		// OPTIONAL-DELAY-PASSED
					
		// OPTIONAL-DELAY-LEFT
	}

	// FULL-CHECKSUM
	checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );
}


#ifdef USED_AS_RETRANSMITTER
void siot_mesh_rebuild_unicast_packet_for_forwarding( MEMORY_HANDLE mem_h, uint16_t target_id, uint16_t* packet_checksum, bool being_resent )
{
	// NOTE: here we assume that the input packet is integral
	//  | SAMP-UNICAST-DATA-PACKET-FLAGS-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | LAST-HOP | Non-Root-Address | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

	parser_obj po, po1;
	uint16_t header;

	// header: bit[0]: 0, bit[1]: ACKNOWLEDGED-DELIVERY flag, bit[2]: 0, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL

	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	uint16_t total_packet_sz = zepto_parsing_remaining_bytes( &po );

	// HEADER (we update only the value of TTL)
	header = zepto_parse_encoded_uint16( &po );
	bool extra_headers_present = ( header >> 3 ) & 0x1;
	ZEPTO_DEBUG_ASSERT( ( header & 1 ) == 0 ); // for this type of the packet
	ZEPTO_DEBUG_ASSERT( ( header & 4 ) == 0 ); // reserved
	uint16_t ttl = header >> 5;
	ZEPTO_DEBUG_ASSERT( ttl > 0 ); // otherwise it should be dropped before calling this function
	if ( !being_resent )
		ttl--;
	header = (header & 0x1f) | (ttl << 5); // that is, first bits and decremented TTL
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// OPTIONAL-EXTRA-HEADERS
	if ( extra_headers_present )
	{
		ZEPTO_DEBUG_ASSERT( 0 == "Not yet implemented" );
	}

	// NEXT-HOP
	uint16_t link_id;
	uint8_t ret_code = siot_mesh_target_to_link_id( target_id, &link_id );
	SIOT_MESH_LINK link;
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		uint8_t ret_code = siot_mesh_get_link( link_id, &link );
		ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK ); // we do not expect that being here we have an invalid link_id
		ZEPTO_DEBUG_ASSERT( link.LINK_ID == link_id );
	}
	else
	{
		ZEPTO_DEBUG_ASSERT( 0 == "link does not exist; processing of VIA fields is not yet implemented" );
	}

	uint16_t next_hop = zepto_parse_encoded_uint16( &po ); // just skip
	zepto_parser_encode_and_append_uint16( mem_h, link.NEXT_HOP );

	// LAST-HOP
	uint16_t last_hop = zepto_parse_encoded_uint16( &po ); // just skip
	zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID );

	// Non-Root-Address
	// we implement quick coding assuming no extra data follow
	// TODO: full implementation with VIA fields, etc
	uint16_t target_id_in_packet = zepto_parse_encoded_uint16( &po ); // just skip
#ifdef SA_DEBUG
	target_id_in_packet >>= 1;
	ZEPTO_DEBUG_ASSERT( (target_id_in_packet & 1) == 0 ); // TODO: provide full implementation
	ZEPTO_DEBUG_ASSERT( target_id_in_packet == target_id ); // TODO: provide full implementation
#endif
	zepto_parser_encode_and_append_uint16( mem_h, target_id << 1 );

	// OPTIONAL-PAYLOAD-SIZE

	// HEADER-CHECKSUM
	uint16_t header_checksum = zepto_parse_uint8( &po ); // just skip
	header_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8; // just skip

	uint16_t rsp_sz = memory_object_get_response_size( mem_h );
	uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	// PAYLOAD
	zepto_parser_init_by_parser( &po1, &po );
	uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
	ZEPTO_DEBUG_ASSERT( remaining_size >= 2 );

	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	zepto_parse_skip_block( &po1, remaining_size - 2 );
	zepto_append_part_of_request_to_response( mem_h, &po, &po1 );

	// FULL-CHECKSUM
	checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	*packet_checksum = checksum;
}
#endif // USED_AS_RETRANSMITTER

void siot_mesh_form_packet_to_santa( MEMORY_HANDLE mem_h, uint8_t mesh_val )
{
	// Santa Packet structure: | SAMP-TO-SANTA-DATA-OR-ERROR-PACKET-NO-TTL | OPTIONAL-EXTRA-HEADERS| SOURCE-ID | REQUEST-ID | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
	// TODO: here and then use bit-field processing instead

	parser_obj po, po1;
	uint16_t header;

	// !!! TODO: it might happen that this packet is not a reply to 'from Santa'; check, which data should be added this case

	uint16_t request_id = 0;
	if ( mesh_val < 2 )
	{
		// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, OPTIONAL-EXTRA-HEADERS
		header = 1 | ( SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET << 1 ) | ( 1 << 4 ); // '1', packet type, 1 (at least one extra header: hop list item), rezerved (zeros)
		zepto_parser_encode_and_append_uint16( mem_h, header );
		ZEPTO_DEBUG_ASSERT( mesh_val < 2 );
		siot_mesh_write_last_hop_data_as_opt_headers( mesh_val, mem_h, true, &request_id );
	}
	else
	{
		header = 1 | ( SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET << 1 ) | ( 0 << 4 ); // '1', packet type, 0 (no extra headers), rezerved (zeros)
		zepto_parser_encode_and_append_uint16( mem_h, header );
	}

	// SOURCE-ID
	zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID );

	// BUS-ID-AT-SOURCE
	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // TODO: actual value must be used

	// REQUEST-ID
	zepto_parser_encode_and_append_uint16( mem_h, request_id );

	// OPTIONAL-PAYLOAD-SIZE

	// HEADER-CHECKSUM
	uint16_t rsp_sz = memory_object_get_response_size( mem_h );
	uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	// PAYLOAD
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	zepto_parse_skip_block( &po1, zepto_parsing_remaining_bytes( &po ) );
	zepto_append_part_of_request_to_response( mem_h, &po, &po1 );

	// FULL-CHECKSUM
	checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );
}

void siot_mesh_form_routing_error_packet( MEMORY_HANDLE mem_h, uint16_t failed_next_hop, uint16_t failed_target, uint16_t original_packet_checksum, uint16_t* packet_checksum )
{
	// Hmp-Routing-Error-Packet: | HMP-ROUTING-ERROR-PACKET-AND-TTL | OPTIONAL-EXTRA-HEADERS | ERROR-CODE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

	// HEADER
	// HMP-ROUTING-ERROR-PACKET-AND-TTL; encoded uint16 with bit[0]=1, bits[1..3]: HMP_ROUTING_ERROR_PACKET, bit [4]: EXTRA-HEADERS-PRESENT, and bits [5..]: TTL
	uint16_t header;
	uint16_t ttl = 4; // TODO: source?
	uint16_t extra_h_present = 0; // TODO: source?
	header = 1 | ( SIOT_MESH_ROUTING_ERROR_PACKET << 1 ) | ( extra_h_present << 4 ) | ( ttl << 5 );
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// OPTIONAL-EXTRA-HEADERS
	if ( extra_h_present )
	{
		// TODO: implement
		ZEPTO_DEBUG_ASSERT( 0 == "Adding optional headers is not implemented" );
	}

	// ERROR-CODE
	uint8_t error_code = 0; // TODO: source?
	zepto_write_uint8( mem_h, error_code );

	// HEADER-CHECKSUM
	uint16_t rsp_sz = memory_object_get_response_size( mem_h );
	uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	// PAYLOAD
	// | ERROR-REPORTING-DEVICE-ID | NEXT_HOP_AT_ORIGIN | TARGET-DEVICE-ID | ORIGINAL-PACKET-CHECKSUM |
	zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID );
	if ( failed_next_hop == SIOT_MESH_NEXT_HOP_UNDEFINED )
		zepto_parser_encode_and_append_uint16( mem_h, 0 );
	else
		zepto_parser_encode_and_append_uint16( mem_h, failed_next_hop + 1 );
	zepto_parser_encode_and_append_uint16( mem_h, failed_target );
	zepto_parser_encode_and_append_uint16( mem_h, original_packet_checksum );

	// FULL-CHECKSUM
	checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	*packet_checksum = checksum;
}

void siot_mesh_form_unicast_packet( MEMORY_HANDLE mem_h, uint16_t link_id, uint16_t* bus_id, uint16_t target_id, bool request_ack, uint16_t* packet_checksum )
{
	//  | SAMP-UNICAST-DATA-PACKET-FLAGS-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | LAST-HOP | Non-Root-Address | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

	parser_obj po, po1;
	uint16_t header;
	SIOT_MESH_LINK link;
	uint8_t ret_code = siot_mesh_get_link( link_id, &link );

	ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK ); // we do not expect that being here we have an invalid link_id
	ZEPTO_DEBUG_ASSERT( link.LINK_ID == link_id );
	*bus_id = link.BUS_ID;
	// !!! TODO: it might happen that this packet is not a reply to 'from Santa'; check, which data should be added this case

	// header: bit[0]: 0, bit[1]: ACKNOWLEDGED-DELIVERY flag, bit[2]: 0, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL
	uint16_t ttl = 4; // TODO: source??
	// TODO: set other fields as necessary
	header = 0 | ( request_ack ? 2 : 0 ) | ( ttl << 5 );
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// OPTIONAL-EXTRA-HEADERS
	// (none so far)

	// NEXT-HOP
	zepto_parser_encode_and_append_uint16( mem_h, link.NEXT_HOP );

	// LAST-HOP
	zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID );

	// Non-Root-Address
	// we implement quick coding assuming no extra data follow
	// TODO: full implementation with VIA fields, etc
	zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID << 1 );

	// OPTIONAL-PAYLOAD-SIZE

	// HEADER-CHECKSUM
	uint16_t rsp_sz = memory_object_get_response_size( mem_h );
	uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	// PAYLOAD
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	zepto_parse_skip_block( &po1, zepto_parsing_remaining_bytes( &po ) );
	zepto_append_part_of_request_to_response( mem_h, &po, &po1 );

	// FULL-CHECKSUM
	checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

	*packet_checksum = checksum;
}

#ifdef USED_AS_RETRANSMITTER

typedef struct _SIOT_MESH_FROM_SANTA_PRETASK
{
	MEMORY_HANDLE mem_h;
	uint16_t bus_id;
	uint16_t next_hop;
} SIOT_MESH_FROM_SANTA_PRETASK;

typedef struct _SIOT_MESH_FROM_SANTA_PRETASK_HOLDER
{
	MEMORY_HANDLE holder_h;
	uint16_t obj_cnt;
} SIOT_MESH_FROM_SANTA_PRETASK_HOLDER;

void siot_mesh_from_santa_pretask_holder_init( SIOT_MESH_FROM_SANTA_PRETASK_HOLDER* holder )
{
	holder->holder_h = acquire_memory_handle();
	holder->obj_cnt = 0;
	ZEPTO_DEBUG_ASSERT( holder->holder_h != MEMORY_HANDLE_INVALID );
}

void siot_mesh_from_santa_pretask_holder_add_pretask( SIOT_MESH_FROM_SANTA_PRETASK_HOLDER* holder, SIOT_MESH_FROM_SANTA_PRETASK* pretask )
{
	zepto_memman_append_locally_generated_data( holder->holder_h, sizeof(SIOT_MESH_FROM_SANTA_PRETASK), (uint8_t*)pretask );
	(holder->obj_cnt)++;
}

uint16_t siot_mesh_from_santa_pretask_holder_get_count( const SIOT_MESH_FROM_SANTA_PRETASK_HOLDER* holder )
{
#ifdef SA_DEBUG
	ZEPTO_DEBUG_ASSERT( holder->holder_h != MEMORY_HANDLE_INVALID );
	uint16_t data_sz = zepto_memman_get_currently_allocated_size_for_locally_generated_data( holder->holder_h );
	ZEPTO_DEBUG_ASSERT( ( (uint8_t*)( ((SIOT_MESH_FROM_SANTA_PRETASK*)0) + holder->obj_cnt ) ) - ( (uint8_t*)( (SIOT_MESH_FROM_SANTA_PRETASK*)0) ) ==  data_sz );
#endif // SA_DEBUG
	return holder->obj_cnt;
}

bool siot_mesh_from_santa_pretask_holder_get_pretask( const SIOT_MESH_FROM_SANTA_PRETASK_HOLDER* holder, uint16_t idx, SIOT_MESH_FROM_SANTA_PRETASK* pretask )
{
	if ( idx >= holder->obj_cnt )
		return false;
	zepto_memman_read_locally_generated_data_by_offset( holder->holder_h, idx * sizeof(SIOT_MESH_FROM_SANTA_PRETASK), sizeof(SIOT_MESH_FROM_SANTA_PRETASK), (uint8_t*)pretask );
	return true;
}

bool siot_mesh_from_santa_pretask_holder_update_pretask( SIOT_MESH_FROM_SANTA_PRETASK_HOLDER* holder, uint8_t idx, SIOT_MESH_FROM_SANTA_PRETASK* pretask )
{
	if ( idx >= holder->obj_cnt )
		return false;
	zepto_memman_write_locally_generated_data_by_offset( holder->holder_h, idx * sizeof(SIOT_MESH_FROM_SANTA_PRETASK), sizeof(SIOT_MESH_FROM_SANTA_PRETASK), (uint8_t*)pretask );
	return true;
}

void siot_mesh_from_santa_pretask_holder_release( SIOT_MESH_FROM_SANTA_PRETASK_HOLDER* holder )
{
	uint8_t idx;
	SIOT_MESH_FROM_SANTA_PRETASK pretask;
	for (idx=0; idx<siot_mesh_from_santa_pretask_holder_get_count(holder); idx++)
	{
		siot_mesh_from_santa_pretask_holder_get_pretask( holder, idx, &pretask );
/*		if ( pretask.mem_h != MEMORY_HANDLE_INVALID )
			release_memory_handle( pretask.mem_h );*/
	}
	release_memory_handle( holder->holder_h );
	holder->obj_cnt = 0;
}

#endif // USED_AS_RETRANSMITTER

#ifdef USED_AS_RETRANSMITTER
uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint8_t* mesh_val, uint8_t signal_level, uint8_t error_cnt, uint16_t* bus_id, uint16_t* ack_bus_id )
#else // USED_AS_RETRANSMITTER
uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint8_t* mesh_val, uint8_t signal_level, uint8_t error_cnt, uint16_t* ack_bus_id )
#endif // USED_AS_RETRANSMITTER
{
	parser_obj po, po1, po2;
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	uint16_t total_packet_sz = zepto_parsing_remaining_bytes( &po );

#ifdef SA_DEBUG
	{
		uint16_t i;
		parser_obj podbg;
		zepto_parser_init( &podbg, mem_h );
		ZEPTO_DEBUG_PRINTF_2( "handler_siot_mesh_receive_packet(): packet received: sz = %d, packet = ", total_packet_sz );
		for ( i=0; i<total_packet_sz; i++)
			ZEPTO_DEBUG_PRINTF_2( "%02x ", zepto_parse_uint8( &podbg ) );
		ZEPTO_DEBUG_PRINTF_1( "\n" );
	}
#endif // SA_DEBUG

	*mesh_val = 0xFF; // valid values make sense only in certain cases and will be loaded then

	uint16_t header = zepto_parse_encoded_uint16( &po );
	// TODO: here and then use bit-field processing instead
	if ( header & 1 ) // predefined packet types
	{
		uint8_t packet_type = ( header >> 1 ) & 0x7;
		switch ( packet_type )
		{
			case SIOT_MESH_ACK_NACK_PACKET:
			{
				// Samp-Ack-Nack-Packet: | SAMP-ACK-NACK-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | Target-Address | NUMBER-OF-ERRORS | ACK-CHESKSUM | HEADER-CHECKSUM | OPTIONAL-DELAY-UNIT | OPTIONAL-DELAY-PASSED | OPTIONAL-DELAY-LEFT |
				// SAMP-ACK-NACK-AND-TTL: encoded uint16, bit[0]=1, bits[1..3] = SAMP_ACK_NACK_PACKET, bit [4] = EXTRA-HEADERS-PRESENT, and bits [5..] being TTL

				// WARNING: we should not do any irreversible changes until checksums are virified; 
				//          thus, until that point all data should only be locally collected, and applied only after checksum verification (or do two passes)
				bool extra_headers_present = ( header >> 4 ) & 0x1;
				uint16_t TTL = header >> 5;

				bool delay_flag_present = false;

				// now we're done with the header; proceeding to optional headers...
				while ( extra_headers_present )
				{
					ZEPTO_DEBUG_ASSERT( 0 == "optional headers in \'ACK_NACK\' packet are not yet implemented" );
				}

				// LAST-HOP
				uint16_t last_hop_id = zepto_parse_encoded_uint16( &po );

				// Target-Address
				header = zepto_parse_encoded_uint16( &po );
				ZEPTO_DEBUG_ASSERT( (header & 1) == 0 ); // we have not yet implemented extra data
				uint16_t target_id = header >> 1;
				if ( target_id != DEVICE_SELF_ID )
				{
					ZEPTO_DEBUG_PRINTF_3( "Packet for device %d received (ACK-NACK); ignored (self id: %d)\n", target_id, DEVICE_SELF_ID );
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				}

				// NUMBER-OF-ERRORS
				uint16_t num_of_errors = zepto_parse_encoded_uint16( &po );

				// ACK-CHESKSUM
				uint16_t ack_checksum = zepto_parse_uint8( &po );
				ack_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;

				ZEPTO_DEBUG_PRINTF_4( "SIOT_MESH_ACK_NACK_PACKET: LAST-HOP: %d, target_id: %d, ack_checksum: 0x%04x\n", last_hop_id, target_id, ack_checksum );

				// HEADER-CHECKSUM
				uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
				uint16_t checksum = zepto_parse_uint8( &po );
				checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
				zepto_parser_init_by_parser( &po1, &po );
				if ( actual_checksum != checksum ) // we have not received even a header
				{
					// TODO: cleanup, if necessary
					return SIOT_MESH_RET_GARBAGE_RECEIVED;
				}

				if ( delay_flag_present )
				{
					// OPTIONAL-DELAY-UNIT
					
					// OPTIONAL-DELAY-PASSED
					
					// OPTIONAL-DELAY-LEFT
				}

				// TODO: check full-packet checksum

				// remove resend tasks (if any)
#ifdef USED_AS_RETRANSMITTER
				siot_mesh_remove_resend_task_by_hash( checksum, currt, &(wf->wait_time) );
#else // USED_AS_RETRANSMITTER
				uint8_t i;
				for ( i=0; i<PENDING_RESENDS_MAX; i++ )
					if ( pending_resends[i].resend_cnt && pending_resends[i].checksum == ack_checksum ) // used slot with received checksum
					{
						zepto_parser_free_memory( pending_resends[i].packet_h );
						pending_resends[i].resend_cnt = 0;
					}

				// calc wait time (if tasks remain)
				for ( i=0; i<PENDING_RESENDS_MAX; i++ )
					if ( pending_resends[i].resend_cnt ) // used slot
						sa_hal_time_val_get_remaining_time( currt, &(pending_resends[i].next_resend_time), &(wf->wait_time) );
#endif // USED_AS_RETRANSMITTER

				return SIOT_MESH_RET_OK;
			}
#ifdef USED_AS_RETRANSMITTER
			case SIOT_MESH_FROM_SANTA_DATA_PACKET:
			{
				// Being optimistic regarding packet integrity we declare a number of parser objects denoting 
				// starting (and sometimes ending0 positions of certain parts of the packet to avoid double parsing
				parser_obj po_target_start, po_target_end;
				parser_obj po_bustype_start, po_bustype_end;
				parser_obj po_payload_start, po_payload_end;
				parser_obj po_headers_start, po_headers_end; // header and optional headers
				parser_obj po_retransmitter_start;

				// WARNING: we should not do any irreversible changes until checksums are virified; 
				//          thus, until that point all data should only be locally collected, and applied only after checksum verification (or do two passes)
				bool more_packets_follow = false; // MORE-PACKETS-FOLLOW
				bool target_collect_last_hops = false; // TARGET-COLLECT-LAST-HOPS
				bool explicit_time_scheduling = false; // EXPLICIT-TIME-SCHEDULING
				bool is_probe = false; // IS-PROBE

				// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, presence of OPTIONAL-EXTRA-HEADERS
				uint8_t extra_headers_present = ( header >> 4 ) & 0x1;
				uint16_t TTL = header >> 5;
				// now we're done with the header; proceeding to optional headers...
				uint8_t ehp = extra_headers_present;
				zepto_parser_init_by_parser( &po_headers_start, &po );
				while ( ehp )
				{
					ZEPTO_DEBUG_ASSERT( 0 == "optional headers in \'FROM-SANTA\' packet are not yet implemented" );
					header = zepto_parse_encoded_uint16( &po );
					ehp = header & 0x1;
					uint8_t generic_flags = (header >> 1) & 0x3; // bits[1,2]
					switch ( generic_flags )
					{
						case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
						{
							more_packets_follow = header >> 3;
							target_collect_last_hops = header >> 4;
							explicit_time_scheduling = header >> 5;
							is_probe = header >> 7;
							break;
						}
						case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
						case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
						case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: not implemented\n" );
							break;
						}
					}
				}

				ZEPTO_DEBUG_ASSERT( !more_packets_follow ); // not yet implemented
				ZEPTO_DEBUG_ASSERT( !explicit_time_scheduling ); // not yet implemented
				ZEPTO_DEBUG_ASSERT( !is_probe ); // not yet implemented

				zepto_parser_init_by_parser( &po_headers_end, &po );


				// LAST-HOP
				uint16_t last_hop_id = zepto_parse_encoded_uint16( &po );
//				ZEPTO_DEBUG_ASSERT( last_hop_id == 0 ); // from ROOT as long as we have not implemented and do not expect other options

				// LAST-HOP-BUS-ID
				uint16_t last_hop_bus_id = zepto_parse_encoded_uint16( &po );
//				ZEPTO_DEBUG_ASSERT( last_hop_bus_id == 0 ); // from ROOT as long as we have not implemented and do not expect other options

				// REQUEST-ID
				uint16_t request_id = zepto_parse_encoded_uint16( &po );

				ZEPTO_DEBUG_PRINTF_3( "FROM-SANTA: last_hop_id = %d, request_id = %d, retransmitters: ", last_hop_id, request_id );

				// OPTIONAL-DELAY-UNIT is present only if EXPLICIT-TIME-SCHEDULING flag is present; currently we did not added it

				// MULTIPLE-RETRANSMITTING-ADDRESSES 
				// At this point we should quickly go through and to check only whether we're in the list (if we're a retransmitter), or just to skip this part, if we're a terminating device
				zepto_parser_init_by_parser( &po_retransmitter_start, &po );

				bool among_retransmitters = false;
				uint16_t retransmitter_cnt = 0;
				header = zepto_parse_encoded_uint16( &po );
				while ( header != 0 )
				{
					uint16_t retr_id = header >> 1;
					if ( retr_id == 0 )
					{
						ZEPTO_DEBUG_ASSERT( 0 == "Case retransmitter-header = 0 has not been considered yet" );
						return 0;
					}
					retr_id --;

					ZEPTO_DEBUG_PRINTF_2( " %d,", retr_id );
					if ( retr_id == DEVICE_SELF_ID )
						among_retransmitters = true;
					else
						retransmitter_cnt ++;
					header = zepto_parse_encoded_uint16( &po );
				}

				// BROADCAST-BUS-TYPE-LIST
				// we will use it in two ways: by adding/copying to outgoing packets (if any), and practically, if we're in the list of retransmitters
				zepto_parser_init_by_parser( &po_bustype_start, &po );

				ZEPTO_DEBUG_PRINTF_1( "; bus-types:" );
				uint8_t bus_type = zepto_parse_uint8( &po );
				while ( bus_type != 0 ) // 0 is a terminator
				{
					ZEPTO_DEBUG_PRINTF_2( " %d,", bus_type - 1 );
					bus_type = zepto_parse_uint8( &po );
				}

				zepto_parser_init_by_parser( &po_bustype_end, &po );

				// Multiple-Target-Addresses
				ZEPTO_DEBUG_PRINTF_1( "; targets:" );
				bool among_targets = false;
				zepto_parser_init_by_parser( &po_target_start, &po );

				header = zepto_parse_encoded_uint16( &po );
				while ( header != 0 ) // terminator of the list, "EXTRA_DATA_FOLLOWS=0 and NODE-ID=0"
				{
					uint16_t target_id = header >> 1;
					ZEPTO_DEBUG_PRINTF_2( " %d,", target_id );
					if ( target_id == DEVICE_SELF_ID )
						among_targets = true;
					header = zepto_parse_encoded_uint16( &po );
				}
				ZEPTO_DEBUG_PRINTF_1( ";\n" );

				zepto_parser_init_by_parser( &po_target_end, &po );

				// OPTIONAL-TARGET-REPLY-DELAY
				uint16_t optional_target_reply_delay;
				if ( explicit_time_scheduling )
					optional_target_reply_delay = zepto_parse_encoded_uint16( &po );

				// OPTIONAL-PAYLOAD-SIZE
				uint16_t optional_payload_size;
				if ( more_packets_follow )
					optional_payload_size = zepto_parse_encoded_uint16( &po );


				// HEADER-CHECKSUM
				uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
				uint16_t header_reported_checksum = zepto_parse_uint8( &po );
				header_reported_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
				zepto_parser_init_by_parser( &po1, &po );


				if ( actual_checksum != header_reported_checksum ) // we have not received even a header
				{
					// TODO: cleanup, if necessary
					return SIOT_MESH_RET_GARBAGE_RECEIVED;
				}

				// payload and FULL-CHECKSUM
				zepto_parser_init_by_parser( &po_payload_start, &po );

				bool second_checksum_ok;
				uint16_t packet_reported_checksum;
				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
				if ( remaining_size >= 2 )
				{
					parser_obj rq_start_po;
					zepto_parser_init_by_parser( &po2, &po );
					zepto_parser_init_by_parser( &rq_start_po, &po );
					zepto_parse_skip_block( &po, remaining_size - 2 );
					zepto_parser_init_by_parser( &po_payload_end, &po ); // payload end
					zepto_append_part_of_request_to_response( mem_h, &po2, &po );
					actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &rq_start_po, remaining_size - 2, actual_checksum );
					packet_reported_checksum = zepto_parse_uint8( &po );
					packet_reported_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
					second_checksum_ok = actual_checksum == packet_reported_checksum;
				}
				else
				{
					second_checksum_ok = false;
				}

				if ( !second_checksum_ok )
				{
					// TODO: we have only a partially received packet; prepare NAK
					ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NAK is not yet implemented\n" );
					return SIOT_MESH_RET_PASS_TO_SEND;
				}

				// Now we can assume that we have received a quite good packet, and we will process it further

				// NOTE: we can appear here for two reasons: 
				//       we're a terminating device, and the packet is "for us", and
				//		 we're a retransmitter, and we should further analyze the packet

				if ( among_targets )
				{
					// now we can add this packet to the list of last hop's data
					// TODO: check TARGET-COLLECT-LAST-HOPS flag!!!
					ZEPTO_DEBUG_ASSERT( ( signal_level >> 4 ) == 0 ); // 4 bits
					ZEPTO_DEBUG_ASSERT( ( error_cnt >> 3 ) == 0 ); // 3 bits; TODO: what we do in case of more errors?

					// is it a packet of the same request id?
					bool one_of_recent_requests = false;
					if ( last_requests[0].ineffect && request_id == last_requests[0].rq_id )
					{
						one_of_recent_requests = true;
						siot_mesh_add_last_hop_data( 0, last_hop_id, last_hop_bus_id, signal_level | ( error_cnt << 4 ) );
						*mesh_val = 0;
					}
					else if ( last_requests[1].ineffect && request_id == last_requests[1].rq_id )
					{
						one_of_recent_requests = true;
						siot_mesh_add_last_hop_data( 1, last_hop_id, last_hop_bus_id, signal_level | ( error_cnt << 4 ) );
						*mesh_val = 1;
					}
					else
					{
						ZEPTO_DEBUG_ASSERT( last_requests[0].ineffect == false || last_requests[1].ineffect == false );
						*mesh_val = 0;
						if ( last_requests[0].ineffect )
							*mesh_val = 1;
						siot_mesh_init_last_hop_data_storage( *mesh_val, request_id, last_hop_id, last_hop_bus_id, signal_level | ( error_cnt << 4 ) );
					}

					return SIOT_MESH_RET_PASS_TO_PROCESS;
				}

				if ( TTL == 0 )
				{
					return SIOT_MESH_RET_OK;
				}
				TTL--;


				// init pretask holder (we will need it even if the list of retransmitters is empty as we, potentially may need to send something via our buses0
				SIOT_MESH_FROM_SANTA_PRETASK_HOLDER pretask_holder;
				siot_mesh_from_santa_pretask_holder_init( &pretask_holder );
				SIOT_MESH_FROM_SANTA_PRETASK pretask;


				if ( retransmitter_cnt )
				{
					parser_obj current_retr_po;
					MEMORY_HANDLE retransmitter_list_h = acquire_memory_handle();
					uint16_t retr_link_id;

					zepto_parser_init_by_parser( &current_retr_po, &po_retransmitter_start );
					uint16_t retr_header = zepto_parse_encoded_uint16( &current_retr_po );
					retransmitter_cnt = 0;
					while ( retr_header != 0 )
					{
						uint16_t retr_id = header >> 1;
						if ( (retr_header >> 1) != DEVICE_SELF_ID )
							if( siot_mesh_target_to_link_id( retr_header >> 1, &retr_link_id ) == SIOT_MESH_RET_OK )
							{
								zepto_parser_encode_and_append_uint16( retransmitter_list_h, header );
								retransmitter_cnt++;
							}
						retr_header = zepto_parse_encoded_uint16( &current_retr_po );
					}
					zepto_parser_encode_and_append_uint16( retransmitter_list_h, header ); // terminator

					if ( retransmitter_cnt )
					{
						parser_obj po_block_start, po_block_end;
						SIOT_MESH_LINK link;
						uint8_t idx;

						retr_header = zepto_parse_encoded_uint16( &current_retr_po );
						while ( retr_header != 0 )
						{
							MEMORY_HANDLE output_h = acquire_memory_handle();

							uint16_t retr_id = retr_header >> 1;
							ZEPTO_DEBUG_ASSERT( retr_id != DEVICE_SELF_ID );
							uint8_t ret_code = siot_mesh_target_to_link_id( retr_id, &retr_link_id );
							ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK );
							ret_code = siot_mesh_get_link( retr_link_id, &link );
							ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK );

							// THIS IS ONE OF RETRANSMITTERS WE HAVE A ROUTE TO; PREPARE A PACKET

							// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL 
							uint16_t header = 1 | ( SIOT_MESH_FROM_SANTA_DATA_PACKET << 1 ) | ( extra_headers_present << 4) | ( TTL << 5 ); // '1', packet type, 0 (no extra headers), TTL = 4
							zepto_parser_encode_and_append_uint16( output_h, header );

							// OPTIONAL-EXTRA-HEADERS (copying)
							if ( extra_headers_present )
							{
								zepto_parser_init_by_parser( &po_block_start, &po_headers_start );
								zepto_parser_init_by_parser( &po_block_end, &po_headers_end );
								zepto_append_part_of_request_to_response( output_h, &po_block_start, &po_block_end );
							}

							// LAST-HOP
							zepto_parser_encode_and_append_uint16( output_h, DEVICE_SELF_ID );

							// LAST-HOP-BUS-ID
							zepto_parser_encode_and_append_uint16( output_h, link.BUS_ID );

							// REQUEST-ID (copying)
							zepto_parser_encode_and_append_uint16( output_h, request_id );

							// OPTIONAL-DELAY-UNIT is present only if EXPLICIT-TIME-SCHEDULING flag is present; currently we did not added it

							// MULTIPLE-RETRANSMITTING-ADDRESSES (inserting prepared)
							zepto_copy_response_to_response_of_another_handle( retransmitter_list_h, output_h );

							// BROADCAST-BUS-TYPE-LIST (copying)
							zepto_parser_init_by_parser( &po_block_start, &po_bustype_start );
							zepto_parser_init_by_parser( &po_block_end, &po_bustype_end );
							zepto_append_part_of_request_to_response( output_h, &po_block_start, &po_block_end );

							//  Multiple-Target-Addresses (copying)
							zepto_parser_init_by_parser( &po_block_start, &po_target_start );
							zepto_parser_init_by_parser( &po_block_end, &po_target_end );
							zepto_append_part_of_request_to_response( output_h, &po_block_start, &po_block_end );

							// OPTIONAL-TARGET-REPLY-DELAY
							if ( explicit_time_scheduling )
								zepto_parser_encode_and_append_uint16( output_h, optional_target_reply_delay );

							// OPTIONAL-PAYLOAD-SIZE
							if ( more_packets_follow )
								zepto_parser_encode_and_append_uint16( output_h, optional_payload_size );

							// HEADER-CHECKSUM
							uint16_t rsp_sz = memory_object_get_response_size( output_h );
							uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( output_h, 0, rsp_sz, 0 );
							zepto_write_uint8( output_h, (uint8_t)checksum );
							zepto_write_uint8( output_h, (uint8_t)(checksum>>8) );

							// PAYLOAD (copying)
							zepto_parser_init_by_parser( &po_block_start, &po_payload_start );
							zepto_parser_init_by_parser( &po_block_end, &po_payload_end );
							zepto_append_part_of_request_to_response( output_h, &po_block_start, &po_block_end );

							// FULL-CHECKSUM
							checksum = zepto_parser_calculate_checksum_of_part_of_response( output_h, rsp_sz + 2, memory_object_get_response_size( output_h ) - (rsp_sz + 2), checksum );
							zepto_write_uint8( output_h, (uint8_t)checksum );
							zepto_write_uint8( output_h, (uint8_t)(checksum>>8) );

							// now we have a ready packet to be sent via some bus; add it to the list of pre-tasks

							bool found = false;
							for ( idx=0; idx<siot_mesh_from_santa_pretask_holder_get_count( &pretask_holder ); idx++ )
							{
								siot_mesh_from_santa_pretask_holder_get_pretask( &pretask_holder, idx, &pretask );
								if ( pretask.bus_id == link.BUS_ID )
								{
									found = true;
									break;
								}
							}

							if ( found )
							{
								zepto_parser_free_memory( output_h );
							}
							else
							{
								pretask.mem_h = output_h;
								pretask.bus_id = link.BUS_ID;
								pretask.next_hop = link.NEXT_HOP;
								siot_mesh_from_santa_pretask_holder_add_pretask( &pretask_holder, &pretask );
							}

							header = zepto_parse_encoded_uint16( &current_retr_po );
						}
					}

					release_memory_handle( retransmitter_list_h );
				}

				if ( among_retransmitters ) // we need to send a packet via remaining buses of types from the list
				{
					// other buses
					uint16_t idx;
					parser_obj po_block_start, po_block_end;
					zepto_parser_init_by_parser( &po_block_start, &po_bustype_start );
					uint8_t bus_type = zepto_parse_uint8( &po_bustype_start );
					bool found;
					while ( bus_type != 0 ) // 0 is a terminator
					{
						bus_type --;
						found = false;
						for ( idx=0; idx<BUS_LIST_ITEM_COUNT; idx++ )
							if ( bus_list[ idx ].bus_type == bus_type )
							{
								found = true;
								break;
							}
						if ( !found ) continue; // we do not have a bus of such type

						// now we have to prepare a packet for each bus of this type which is not yet used
						for ( idx=0; idx<BUS_LIST_ITEM_COUNT; idx++ )
						{
							if ( bus_list[ idx ].bus_type == bus_type )
							{
								found = false;
								for ( idx=0; idx<siot_mesh_from_santa_pretask_holder_get_count( &pretask_holder ); idx++ )
								{
									siot_mesh_from_santa_pretask_holder_get_pretask( &pretask_holder, idx, &pretask );
									if ( pretask.bus_id == bus_list[ idx ].bus_id )
									{
										found = true;
										break;
									}
								}
								if ( found ) continue;

								// this bus is of a right type and was not used yet
								MEMORY_HANDLE output_h = acquire_memory_handle();

								// THIS IS ONE OF RETRANSMITTERS WE HAVE A ROUTE TO; PREPARE A PACKET

								// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL 
								uint16_t header = 1 | ( SIOT_MESH_FROM_SANTA_DATA_PACKET << 1 ) | ( extra_headers_present << 4) | ( TTL << 5 ); // '1', packet type, 0 (no extra headers), TTL = 4
								zepto_parser_encode_and_append_uint16( output_h, header );

								// OPTIONAL-EXTRA-HEADERS (copying)
								if ( extra_headers_present )
								{
									zepto_parser_init_by_parser( &po_block_start, &po_headers_start );
									zepto_parser_init_by_parser( &po_block_end, &po_headers_end );
									zepto_append_part_of_request_to_response( output_h, &po_block_start, &po_block_end );
								}

								// LAST-HOP
								zepto_parser_encode_and_append_uint16( output_h, DEVICE_SELF_ID );

								// LAST-HOP-BUS-ID
								zepto_parser_encode_and_append_uint16( output_h, bus_list[ idx ].bus_id );

								// REQUEST-ID (copying)
								zepto_parser_encode_and_append_uint16( output_h, request_id );

								// OPTIONAL-DELAY-UNIT is present only if EXPLICIT-TIME-SCHEDULING flag is present; currently we did not added it

								// MULTIPLE-RETRANSMITTING-ADDRESSES (none at this point)
								zepto_parser_encode_and_append_uint16( output_h, 0 ); // terminator of the list, "EXTRA_DATA_FOLLOWS=0 and NODE-ID=0"

								// BROADCAST-BUS-TYPE-LIST (copying)
								zepto_parser_init_by_parser( &po_block_start, &po_bustype_start );
								zepto_parser_init_by_parser( &po_block_end, &po_bustype_end );
								zepto_append_part_of_request_to_response_of_another_handle( mem_h, &po_block_start, &po_block_end, output_h );

								//  Multiple-Target-Addresses (copying)
								zepto_parser_init_by_parser( &po_block_start, &po_target_start );
								zepto_parser_init_by_parser( &po_block_end, &po_target_end );
								zepto_append_part_of_request_to_response_of_another_handle( mem_h, &po_block_start, &po_block_end, output_h );

								// OPTIONAL-TARGET-REPLY-DELAY
								if ( explicit_time_scheduling )
									zepto_parser_encode_and_append_uint16( output_h, optional_target_reply_delay );

								// OPTIONAL-PAYLOAD-SIZE
								if ( more_packets_follow )
									zepto_parser_encode_and_append_uint16( output_h, optional_payload_size );

								// HEADER-CHECKSUM
								uint16_t rsp_sz = memory_object_get_response_size( output_h );
								uint16_t checksum = zepto_parser_calculate_checksum_of_part_of_response( output_h, 0, rsp_sz, 0 );
								zepto_write_uint8( output_h, (uint8_t)checksum );
								zepto_write_uint8( output_h, (uint8_t)(checksum>>8) );

								// PAYLOAD (copying)
								zepto_parser_init_by_parser( &po_block_start, &po_payload_start );
								zepto_parser_init_by_parser( &po_block_end, &po_payload_end );
								zepto_append_part_of_request_to_response_of_another_handle( mem_h, &po_block_start, &po_block_end, output_h );

								// FULL-CHECKSUM
								checksum = zepto_parser_calculate_checksum_of_part_of_response( output_h, rsp_sz + 2, memory_object_get_response_size( output_h ) - (rsp_sz + 2), checksum );
								zepto_write_uint8( output_h, (uint8_t)checksum );
								zepto_write_uint8( output_h, (uint8_t)(checksum>>8) );

								// now we have a ready packet to be sent via some bus; add it to the list of pre-tasks

								pretask.mem_h = output_h;
								pretask.bus_id = bus_list[ idx ].bus_id;
								pretask.next_hop = SIOT_MESH_NEXT_HOP_UNDEFINED;
								siot_mesh_from_santa_pretask_holder_add_pretask( &pretask_holder, &pretask );
							}
						}

						bus_type = zepto_parse_uint8( &po_bustype_start );
					}
				}


				// now we are to form and add tasks for sending and relase pretask holder
				if ( siot_mesh_from_santa_pretask_holder_get_count( &pretask_holder ) )
				{
					uint8_t idx;
					for ( idx=0; idx<siot_mesh_from_santa_pretask_holder_get_count( &pretask_holder ); idx++ )
					{
						siot_mesh_from_santa_pretask_holder_get_pretask( &pretask_holder, idx, &pretask );
						siot_mesh_at_root_add_send_from_santa_task( pretask.mem_h, currt, &(wf->wait_time), pretask.bus_id, pretask.next_hop );
						pretask.mem_h = MEMORY_HANDLE_INVALID;
					}
					siot_mesh_from_santa_pretask_holder_release( &pretask_holder );
				}

				break;
			}
#else // USED_AS_RETRANSMITTER
			case SIOT_MESH_FROM_SANTA_DATA_PACKET:
			{
				// WARNING: we should not do any irreversible changes until checksums are virified; 
				//          thus, until that point all data should only be locally collected, and applied only after checksum verification (or do two passes)
				bool more_packets_follow = false; // MORE-PACKETS-FOLLOW
				bool target_collect_last_hops = false; // TARGET-COLLECT-LAST-HOPS
				bool explicit_time_scheduling = false; // EXPLICIT-TIME-SCHEDULING
				bool is_probe = false; // IS-PROBE

				// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, presence of OPTIONAL-EXTRA-HEADERS
				bool extra_headers_present = ( header >> 4 ) & 0x1;
				uint16_t TTL = header >> 5;
				// now we're done with the header; proceeding to optional headers...
				while ( extra_headers_present )
				{
					ZEPTO_DEBUG_ASSERT( 0 == "optional headers in \'FROM-SANTA\' packet are not yet implemented" );
					header = zepto_parse_encoded_uint16( &po );
					extra_headers_present = header & 0x1;
					uint8_t generic_flags = (header >> 1) & 0x3; // bits[1,2]
					switch ( generic_flags )
					{
						case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
						{
							more_packets_follow = header >> 3;
							target_collect_last_hops = header >> 4;
							explicit_time_scheduling = header >> 5;
							is_probe = header >> 7;
							break;
						}
						case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
						case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
						case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: not implemented\n" );
							break;
						}
					}
				}

				ZEPTO_DEBUG_ASSERT( !more_packets_follow ); // not yet implemented
				ZEPTO_DEBUG_ASSERT( !explicit_time_scheduling ); // not yet implemented
				ZEPTO_DEBUG_ASSERT( !is_probe ); // not yet implemented

				// LAST-HOP
				uint16_t last_hop_id = zepto_parse_encoded_uint16( &po );
//				ZEPTO_DEBUG_ASSERT( last_hop_id == 0 ); // from ROOT as long as we have not implemented and do not expect other options

				// LAST-HOP-BUS-ID
				uint16_t last_hop_bus_id = zepto_parse_encoded_uint16( &po );
//				ZEPTO_DEBUG_ASSERT( last_hop_bus_id == 0 ); // from ROOT as long as we have not implemented and do not expect other options

				// REQUEST-ID
				uint16_t request_id = zepto_parse_encoded_uint16( &po );

				ZEPTO_DEBUG_PRINTF_3( "FROM-SANTA: last_hop_id = %d, request_id = %d, bus-types: ", last_hop_id, request_id );

				// OPTIONAL-DELAY-UNIT is present only if EXPLICIT-TIME-SCHEDULING flag is present; currently we did not added it

				// MULTIPLE-RETRANSMITTING-ADDRESSES 
				// At this point we should quickly go through and to check only whether we're in the list (if we're a retransmitter), or just to skip this part, if we're a terminating device
				header = zepto_parse_encoded_uint16( &po );
				while ( header != 0 )
				{
					header = zepto_parse_encoded_uint16( &po );
				}
				// TODO: at this place a full list of intended retransmitters must be retrieved

				// BROADCAST-BUS-TYPE-LIST
				// TODO: what should we add here?
				uint8_t bus_type = zepto_parse_uint8( &po );
				while ( bus_type != 0 ) // 0 is a terminator
				{
					ZEPTO_DEBUG_PRINTF_2( " %d,", bus_type - 1 );
					bus_type = zepto_parse_uint8( &po );
				}

				// Target-Addresses
/*				header = zepto_parse_encoded_uint16( &po );
				ZEPTO_DEBUG_ASSERT( (header & 1) == 0 ); // we have not yet implemented extra data
				uint16_t target_id = header >> 1;
				if ( target_id != DEVICE_SELF_ID )
				{
					ZEPTO_DEBUG_PRINTF_3( "Packet for device %d received (from Santa); ignored (self id: %d)\n", target_id, DEVICE_SELF_ID );
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				}*/
				ZEPTO_DEBUG_PRINTF_1( "; targets:" );
				bool among_targets = false;
				header = zepto_parse_encoded_uint16( &po );
				while ( header != 0 ) // terminator of the list, "EXTRA_DATA_FOLLOWS=0 and NODE-ID=0"
				{
					uint16_t target_id = header >> 1;
					ZEPTO_DEBUG_PRINTF_2( " %d,", target_id );
					if ( target_id == DEVICE_SELF_ID )
						among_targets = true;
					header = zepto_parse_encoded_uint16( &po );
				}
				ZEPTO_DEBUG_PRINTF_1( ";\n" );
				if ( !among_targets )
				{
					ZEPTO_DEBUG_PRINTF_2( "Packet not for device received (from Santa); ignored (self id: %d)\n", DEVICE_SELF_ID );
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				}

				// OPTIONAL-TARGET-REPLY-DELAY

				// OPTIONAL-PAYLOAD-SIZE

				ZEPTO_DEBUG_PRINTF_2( "SIOT_MESH_FROM_SANTA_DATA_PACKET: LAST-HOP: %d\n", last_hop_id );

				// HEADER-CHECKSUM
				uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
				uint16_t header_reported_checksum = zepto_parse_uint8( &po );
				header_reported_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
				zepto_parser_init_by_parser( &po1, &po );


				if ( actual_checksum != header_reported_checksum ) // we have not received even a header
				{
					// TODO: cleanup, if necessary
					return SIOT_MESH_RET_GARBAGE_RECEIVED;
				}

				bool second_checksum_ok;
				uint16_t packet_reported_checksum;
				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
				if ( remaining_size >= 2 )
				{
					parser_obj rq_start_po;
					zepto_parser_init_by_parser( &po2, &po );
					zepto_parser_init_by_parser( &rq_start_po, &po );
					zepto_parse_skip_block( &po, remaining_size - 2 );
					zepto_append_part_of_request_to_response( mem_h, &po2, &po );
					actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &rq_start_po, remaining_size - 2, actual_checksum );
					packet_reported_checksum = zepto_parse_uint8( &po );
					packet_reported_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
					second_checksum_ok = actual_checksum == packet_reported_checksum;
				}
				else
				{
					second_checksum_ok = false;
				}

				if ( !second_checksum_ok )
				{
					// TODO: we have only a partially received packet; prepare NAK
					ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NAK is not yet implemented\n" );
					return SIOT_MESH_RET_PASS_TO_SEND;
				}

				// Now we can assume that we have received a quite good packet, and we will process it further

				// NOTE: we can appear here for two reasons: 
				//       we're a terminating device, and the packet is "for us", and
				//		 we're a retransmitter, and we should further analyze the packet

				ZEPTO_DEBUG_ASSERT( among_targets );
				{
					// now we can add this packet to the list of last hop's data
					// TODO: check TARGET-COLLECT-LAST-HOPS flag!!!
					ZEPTO_DEBUG_ASSERT( ( signal_level >> 4 ) == 0 ); // 4 bits
					ZEPTO_DEBUG_ASSERT( ( error_cnt >> 3 ) == 0 ); // 3 bits; TODO: what we do in case of more errors?

					// is it a packet of the same request id?
					bool one_of_recent_requests = false;
					if ( last_requests[0].ineffect && request_id == last_requests[0].rq_id )
					{
						one_of_recent_requests = true;
						siot_mesh_add_last_hop_data( 0, last_hop_id, last_hop_bus_id, signal_level | ( error_cnt << 4 ) );
						*mesh_val = 0;
					}
					else if ( last_requests[1].ineffect && request_id == last_requests[1].rq_id )
					{
						one_of_recent_requests = true;
						siot_mesh_add_last_hop_data( 1, last_hop_id, last_hop_bus_id, signal_level | ( error_cnt << 4 ) );
						*mesh_val = 1;
					}
					else
					{
						ZEPTO_DEBUG_ASSERT( last_requests[0].ineffect == false || last_requests[1].ineffect == false );
						*mesh_val = 0;
						if ( last_requests[0].ineffect )
							*mesh_val = 1;
						siot_mesh_init_last_hop_data_storage( *mesh_val, request_id, last_hop_id, last_hop_bus_id, signal_level | ( error_cnt << 4 ) );
					}

					return SIOT_MESH_RET_PASS_TO_PROCESS;
				}
				break;
			}
#endif // USED_AS_RETRANSMITTER
#ifdef USED_AS_RETRANSMITTER
			case SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET:
			case SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET:
			{
				// Hmp-To-Santa-Data-Or-Error-Packet: | HMP-TO-SANTA-DATA-OR-ERROR-PACKET-NO-TTL | OPTIONAL-EXTRA-HEADERS | SOURCE-ID | BUS-ID-AT-SOURCE | REQUEST-ID | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
				// to be converted to
				// Hmp-Forward-To-Santa-Data-Or-Error-Packet: | HMP-FORWARD-TO-SANTA-DATA-OR-ERROR-PACKET-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | FORWARDED-SOURCE-ID | FORWARDED-BUS-ID-AT-SOURCE | REQUEST-ID | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
				// or
				// Hmp-Forward-To-Santa-Data-Or-Error-Packet -> Hmp-Forward-To-Santa-Data-Or-Error-Packet
				// We start from an optimistic assumption that the packet is integral and start writing a Forward-To-Santa packet

				zepto_parser_free_response( mem_h );

				// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, presence of OPTIONAL-EXTRA-HEADERS
				uint8_t extra_headers_present = ( header >> 4 ) & 0x1;
				uint16_t TTL = header >> 5;
				if ( TTL == 0 )
				{
					ZEPTO_DEBUG_PRINTF_1( "Packet with TTL = 0 received; dropped\n" );
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				}
				uint16_t output_header = 1 | ( SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET << 1 ) | ( extra_headers_present << 4 ) | ( (TTL-1) << 5 );
				zepto_parser_encode_and_append_uint16( mem_h, output_header );

				// now we're done with the header; proceeding to optional headers...
				parser_obj po_eh_start;
				zepto_parser_init_by_parser( &po_eh_start, &po );
				while ( extra_headers_present )
				{
					header = zepto_parse_encoded_uint16( &po );
					extra_headers_present = header & 0x1;
					uint8_t generic_flags = (header >> 1) & 0x3; // bits[1,2]
					switch ( generic_flags )
					{
						case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
						{
							zepto_parse_encoded_uint16( &po ); // last_hop_bus_id; just skip
							zepto_parse_encoded_uint8( &po ); // connection quality; just skip
							break;
						}
						case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
						case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
						case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: other optional headers are not implemented\n" );
							break;
						}
						default:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: unexpected type of optional header\n" );
							break;
						}
					}
				}
				zepto_append_part_of_request_to_response( mem_h, &po_eh_start, &po );

				// NEXT-HOP (reading)
				uint16_t next_hop;
				if ( packet_type == SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET )
				{
					next_hop = zepto_parse_encoded_uint16( &po );
					if ( next_hop != DEVICE_SELF_ID )
					{
						ZEPTO_DEBUG_PRINTF_2( "Packet for device %d received (Forward-To-Santa); ignored\n", next_hop );
						return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
					}
				}
				else
				{
					ZEPTO_DEBUG_ASSERT( packet_type == SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET );
				}

				// NEXT-HOP (writing)
				uint16_t link_id;
				uint8_t ret_code = siot_mesh_target_to_link_id( 0, &link_id ); // root is a target
				if ( ret_code != SIOT_MESH_RET_OK )
				{
					ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );
					ZEPTO_DEBUG_ASSERT( 0 == "Receiving To-Santa by a retransmitter that has no route to root is not implemented. Do it ASAP!" );
					return 0;
				}
				SIOT_MESH_LINK link;
				ret_code = siot_mesh_get_link( link_id, &link );
				ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK ); // we do not expect that being here we have an invalid link_id
				ZEPTO_DEBUG_ASSERT( link.LINK_ID == link_id );
				next_hop = link.NEXT_HOP;
				*bus_id = link.BUS_ID;
				zepto_parser_encode_and_append_uint16( mem_h, next_hop );

				// (FORWARDED-)SOURCE-ID
				uint16_t src_id = zepto_parse_encoded_uint16( &po );
				zepto_parser_encode_and_append_uint16( mem_h, src_id );

				// (FORWARDED-)BUS-ID-AT-SOURCE
				uint16_t bus_id_at_src = zepto_parse_encoded_uint16( &po );
				zepto_parser_encode_and_append_uint16( mem_h, bus_id_at_src );

				// REQUEST-ID
				uint16_t request_id = zepto_parse_encoded_uint16( &po );
				zepto_parser_encode_and_append_uint16( mem_h, request_id );

				// OPTIONAL-PAYLOAD-SIZE

				// HEADER-CHECKSUM (checking)
				uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
				uint16_t checksum = zepto_parse_uint8( &po );
				checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
				zepto_parser_init_by_parser( &po1, &po );

				if ( actual_checksum != checksum ) // we have not received even a header -- total garbage received
				{
					ZEPTO_DEBUG_PRINTF_1( "Totally broken packet received; dropped\n" );
					zepto_parser_free_response( mem_h );
					return SIOT_MESH_RET_GARBAGE_RECEIVED;
				}

				parser_obj po_payload_start, po_payload_end;
				zepto_parser_init_by_parser( &po_payload_start, &po );
				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
				bool packet_ok = remaining_size >= 2;
				if ( packet_ok )
				{
					zepto_parser_init_by_parser( &po2, &po );
					zepto_parse_skip_block( &po, remaining_size - 2 );
					zepto_parser_init_by_parser( &po_payload_end, &po ); // payload end
					actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po2, remaining_size - 2, actual_checksum );
					checksum = zepto_parse_uint8( &po );
					checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
					packet_ok = actual_checksum == checksum;
				}

				if ( !packet_ok ) // packet is broken; subject for NACK
				{
					// TODO: we have only a partially received packet; prepare NACK
					ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NACK is not yet implemented\n" );
					zepto_parser_free_response( mem_h );
					return SIOT_MESH_RET_PASS_TO_SEND;
				}

				// HEADER-CHECKSUM (writing)
				uint16_t rsp_sz = memory_object_get_response_size( mem_h );
				checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
				zepto_write_uint8( mem_h, (uint8_t)checksum );
				zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

				// PAYLOAD (writing)
				zepto_append_part_of_request_to_response( mem_h, &po_payload_start, &po_payload_end );

				// FULL-CHECKSUM (writing)
				checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
				zepto_write_uint8( mem_h, (uint8_t)checksum );
				zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

				return SIOT_MESH_RET_PASS_TO_SEND;
				break;
			}
#else // USED_AS_RETRANSMITTER
			case SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET:
			case SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET:
			{
				ZEPTO_DEBUG_PRINTF_2( "Packet type %d received; not expected; ignored\n", (header >> 1) & 0x7 );
				return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
			}
#endif // USED_AS_RETRANSMITTER
#ifdef USED_AS_RETRANSMITTER
			case SIOT_MESH_ROUTING_ERROR_PACKET: // we should just forward it to the root
			{
				// Hmp-Routing-Error-Packet: | HMP-ROUTING-ERROR-PACKET-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST_HOP | ERROR-CODE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
				// we start from an optimistic assumption that the packet is integral
				zepto_parser_free_response( mem_h );

				// HMP-ROUTING-ERROR-PACKET-AND-TTL, presence of OPTIONAL-EXTRA-HEADERS
				uint8_t extra_headers_present = ( header >> 4 ) & 0x1;
				uint16_t TTL = header >> 5;
				if ( TTL == 0 )
				{
					ZEPTO_DEBUG_PRINTF_1( "Packet with TTL = 0 received; dropped\n" );
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				}
				uint16_t output_header = 1 | ( SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET << 1 ) | ( extra_headers_present << 4 ) | ( (TTL-1) << 5 );
				zepto_parser_encode_and_append_uint16( mem_h, output_header );

				// now we're done with the header; proceeding to optional headers...
				parser_obj po_eh_start;
				zepto_parser_init_by_parser( &po_eh_start, &po );
				while ( extra_headers_present )
				{
					header = zepto_parse_encoded_uint16( &po );
					extra_headers_present = header & 0x1;
					uint8_t generic_flags = (header >> 1) & 0x3; // bits[1,2]
					switch ( generic_flags )
					{
						case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
						{
							zepto_parse_encoded_uint16( &po ); // last_hop_bus_id; just skip
							zepto_parse_encoded_uint8( &po ); // connection quality; just skip
							break;
						}
						case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
						case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
						case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: other optional headers are not implemented\n" );
							break;
						}
						default:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: unexpected type of optional header\n" );
							break;
						}
					}
				}
				zepto_append_part_of_request_to_response( mem_h, &po_eh_start, &po );

				// ERROR-CODE (copying)
				uint16_t last_hop = zepto_parse_encoded_uint16( &po );
				zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID );

				// ERROR-CODE (copying)
				uint16_t error_code = zepto_parse_encoded_uint16( &po );
				zepto_parser_encode_and_append_uint16( mem_h, error_code );

				// HEADER-CHECKSUM (checking)
				uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
				uint16_t checksum = zepto_parse_uint8( &po );
				checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
				zepto_parser_init_by_parser( &po1, &po );

				if ( actual_checksum != checksum ) // we have not received even a header -- total garbage received
				{
					ZEPTO_DEBUG_PRINTF_1( "Totally broken packet received; dropped\n" );
					zepto_parser_free_response( mem_h );
					return SIOT_MESH_RET_GARBAGE_RECEIVED;
				}

				parser_obj po_payload_start, po_payload_end;
				zepto_parser_init_by_parser( &po_payload_start, &po );
				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
				bool packet_ok = remaining_size >= 2;
				uint16_t reported_checksum;
				if ( packet_ok )
				{
					zepto_parser_init_by_parser( &po2, &po );
					zepto_parse_skip_block( &po, remaining_size - 2 );
					zepto_parser_init_by_parser( &po_payload_end, &po ); // payload end
					actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po2, remaining_size - 2, actual_checksum );
					reported_checksum = zepto_parse_uint8( &po );
					reported_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
					packet_ok = actual_checksum == reported_checksum;
				}

				if ( !packet_ok ) // packet is broken; subject for NACK
				{
					// TODO: we have only a partially received packet; prepare NACK
					ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NACK is not yet implemented\n" );
					zepto_parser_free_response( mem_h );
					return SIOT_MESH_RET_PASS_TO_SEND;
				}

				// HEADER-CHECKSUM (writing)
				uint16_t rsp_sz = memory_object_get_response_size( mem_h );
				checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, 0, rsp_sz, 0 );
				zepto_write_uint8( mem_h, (uint8_t)checksum );
				zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

				// PAYLOAD (writing)
				zepto_append_part_of_request_to_response( mem_h, &po_payload_start, &po_payload_end );

				// FULL-CHECKSUM (writing)
				checksum = zepto_parser_calculate_checksum_of_part_of_response( mem_h, rsp_sz + 2, memory_object_get_response_size( mem_h ) - (rsp_sz + 2), checksum );
				zepto_write_uint8( mem_h, (uint8_t)checksum );
				zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );

				uint8_t ret_code = siot_mesh_get_bus_id_to_root( bus_id );
				if ( ret_code == SIOT_MESH_RET_OK )
				{
					zepto_parser_free_memory( mem_ack_h );
					ret_code = siot_mesh_get_bus_id_for_target( last_hop, ack_bus_id );
					if ( ret_code == SIOT_MESH_RET_OK )
					{
						siot_mesh_form_ack_packet( mem_ack_h, last_hop, error_cnt, reported_checksum );
						return SIOT_MESH_RET_SEND_ACK_AND_PASS_TO_SEND;
					}
					else
						return SIOT_MESH_RET_PASS_TO_SEND;
				}
				else
				{
					zepto_parser_free_memory( mem_h );
					siot_mesh_form_packet_to_santa( mem_h, 0xFF );
					// TODO: anything else?
					return SIOT_MESH_RET_PASS_TO_SEND;
				}

				break;
			}
#else // USED_AS_RETRANSMITTER
			case SIOT_MESH_ROUTING_ERROR_PACKET:
			{
				ZEPTO_DEBUG_PRINTF_2( "Packet type %d received; not expected; ignored\n", (header >> 1) & 0x7 );
				return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
			}
#endif // USED_AS_RETRANSMITTER
			default:
			{
				ZEPTO_DEBUG_PRINTF_2( "Packet type %d received; not implemented\n", (header >> 1) & 0x7 );
//				ZEPTO_DEBUG_ASSERT( NULL == "Error: processing of mesh packets of this type is not implemented\n" );
				return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				break;
			}
		}
	}
	else
	{
		//  | SAMP-UNICAST-DATA-PACKET-FLAGS-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | LAST-HOP | Non-Root-Address | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

		// header: bit[0]: 0, bit[1]: ACKNOWLEDGED-DELIVERY flag, bit[2]: 0, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL
		bool ack_requested = ( header >> 1 ) & 0x1;
		ZEPTO_DEBUG_ASSERT( 0 == (( header >> 2 ) & 0x1 ) ); // reserved
		bool extra_headers_present = ( header >> 3 ) & 0x1;
		bool is_from_root = ( header >> 4 ) & 0x1;


#if !defined USED_AS_RETRANSMITTER
		if ( !is_from_root )
		{
			ZEPTO_DEBUG_PRINTF_1( "Packet directed to ROOT received; ignored\n" );
			return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
		}
#endif // USED_AS_RETRANSMITTER

		uint16_t TTL = header >> 5;
		// now we're done with the header; proceeding to optional headers...
		while ( extra_headers_present )
		{
			header = zepto_parse_encoded_uint16( &po );
			extra_headers_present = header & 0x1;
			uint8_t generic_flags = (header >> 1) & 0x3; // bits[1,2]
			switch ( generic_flags )
			{
				case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
				case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
				case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
				case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
				{
					ZEPTO_DEBUG_ASSERT( NULL == "Error: other optional headers are not implemented\n" );
					break;
				}
			}
		}

		// NEXT-HOP
		uint16_t next_hop = zepto_parse_encoded_uint16( &po ); // TODO: here, in a correct packet we expect to see 0; what to do in case of errors?

		// LAST-HOP
		uint16_t last_hop = zepto_parse_encoded_uint16( &po );

		// Non-Root-Address
		// we implement quick coding assuming no extra data follow
		// TODO: full implementation with VIA fields, etc
		uint16_t target_id = zepto_parse_encoded_uint16( &po );
		ZEPTO_DEBUG_ASSERT( (target_id & 1) == 0 ); // TODO: provide full implementation
		target_id >>= 1;

#ifdef USED_AS_RETRANSMITTER
#else // USED_AS_RETRANSMITTER
		if ( target_id != DEVICE_SELF_ID )
		{
			// TODO: it is much more complicated in case of retransmitter
			ZEPTO_DEBUG_PRINTF_3( "Packet for device %d received (unicast); ignored (self id: %d)\n", target_id, DEVICE_SELF_ID );
			return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
		}
#endif // USED_AS_RETRANSMITTER

		// OPTIONAL-PAYLOAD-SIZE

		// HEADER-CHECKSUM
		uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
		uint16_t header_checksum = zepto_parse_uint8( &po );
		header_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
		zepto_parser_init_by_parser( &po1, &po );

		if ( actual_checksum != header_checksum ) // we have not received even a header -- total garbage received
			return SIOT_MESH_RET_GARBAGE_RECEIVED;

		// now we have a packet that is at least, partially, received; further processing depends on the level of its integrity

		uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
		bool packet_is_integral = remaining_size >= 2;
		uint16_t packet_reported_checksum;

		if ( packet_is_integral )
		{
			zepto_parser_init_by_parser( &po2, &po );
			zepto_parse_skip_block( &po, remaining_size - 2 );
			actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po2, remaining_size - 2, actual_checksum );
			packet_reported_checksum = zepto_parse_uint8( &po );
			packet_reported_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
			packet_is_integral = actual_checksum == packet_reported_checksum;
		}

		if ( !packet_is_integral )
		{
			// TODO: we have only a partially received packet; prepare NACK
			ZEPTO_DEBUG_ASSERT( NULL == "Error: sending NACK is not yet implemented\n" );
			return SIOT_MESH_RET_PASS_TO_SEND;
		}
		else
		{
#ifdef USED_AS_RETRANSMITTER
			// questions to be asked: "is this packet for us?"; "is this packet through us?" If none is "YES", ignore the packet
			if ( target_id == DEVICE_SELF_ID ) // for us
			{
				if ( !is_from_root ) // well, from us (looks like this packet is heavily misdirected) TODO: should we perform any action?
				{
					ZEPTO_DEBUG_PRINTF_1( "Packet directed from US to ROOT received; ignored\n" );
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				}

				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po2 );
				zepto_parser_init_by_parser( &po1, &po2 );
				zepto_parse_skip_block( &po1, remaining_size - 2 );
				zepto_convert_part_of_request_to_response( mem_h, &po2, &po1 );

				if ( ack_requested )
				{
					zepto_parser_free_memory( mem_ack_h );
					uint8_t ret_code = siot_mesh_get_bus_id_for_target( last_hop, ack_bus_id );
					if ( ret_code == SIOT_MESH_RET_OK )
					{
						siot_mesh_form_ack_packet( mem_ack_h, last_hop, error_cnt, packet_reported_checksum );
						return SIOT_MESH_RET_SEND_ACK_AND_PASS_TO_PROCESS;
					}
					else
					{
						// TODO: should we send a siot_mesh_form_routing_error_packet() ?
						return SIOT_MESH_RET_PASS_TO_PROCESS;
					}
				}
				else
					return SIOT_MESH_RET_PASS_TO_PROCESS;
			}
			if ( next_hop == DEVICE_SELF_ID ) // through us; so let's forward it (it is somehow similar to sending our own packets; just kindle re-assemble the packet received updating certain fields0
			{
				if ( ack_requested )
				{
					uint16_t num_err = 0; // TODO: ACTUAL SOURCE???
					zepto_parser_free_memory( mem_ack_h );
					siot_mesh_form_ack_packet( mem_ack_h, last_hop, num_err, packet_reported_checksum );
				}
				uint16_t ttl = header >> 5;
				uint16_t packet_checksum;
				if ( ttl == 0  ) // cannot forward it further; an error must be reported
				{
					ZEPTO_DEBUG_ASSERT( 0 == "reporting errors in case TTL == 0 is not yet implemented" );
					return SIOT_MESH_RET_OK;
				}

				siot_mesh_rebuild_unicast_packet_for_forwarding( mem_h, target_id, &packet_checksum, false );

				uint16_t link_id;
				uint8_t ret_code = siot_mesh_target_to_link_id( target_id, &link_id );
				if ( ret_code == SIOT_MESH_RET_OK )
				{
					SIOT_MESH_LINK link;
					uint8_t ret_code = siot_mesh_get_link( link_id, &link );
					ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK ); // we do not expect that being here we have an invalid link_id
					ZEPTO_DEBUG_ASSERT( link.LINK_ID == link_id );
					*bus_id = link.BUS_ID;
					siot_mesh_at_root_add_resend_unicast_task( mem_h, currt, &(wf->wait_time), target_id, link.BUS_ID, link.NEXT_HOP );

					if ( ack_requested ) // we need to add it to the list of resend tasks
					{
						zepto_parser_free_memory( mem_ack_h );
						uint8_t ret_code = siot_mesh_get_bus_id_for_target( last_hop, ack_bus_id );
						if ( ret_code == SIOT_MESH_RET_OK )
						{
							siot_mesh_form_ack_packet( mem_ack_h, last_hop, error_cnt, packet_reported_checksum );
							return SIOT_MESH_RET_SEND_ACK_AND_PASS_TO_SEND;
						}
						else
						{
							// TODO: should we send a siot_mesh_form_routing_error_packet() ?
							return SIOT_MESH_RET_PASS_TO_SEND;
						}
					}
					else
						return SIOT_MESH_RET_PASS_TO_SEND;
				}
				else
				{
					// depending on the direction of the packet we send either To-Santa or Routing-Error packet
					zepto_parser_free_memory( mem_h );
					if ( is_from_root )
					{
						uint16_t packet_checksum;
						siot_mesh_form_routing_error_packet( mem_h, SIOT_MESH_NEXT_HOP_UNDEFINED, target_id, packet_reported_checksum, &packet_checksum );
					}
					else
					{
						siot_mesh_form_packet_to_santa( mem_h, 0xFF );
					}
					return SIOT_MESH_RET_PASS_TO_SEND;
				}
			}
			else // not for us, not through us
			{
				return SIOT_MESH_RET_OK;
			}
#else // USED_AS_RETRANSMITTER
			// Note: don't be surprized by implementation of call below: we cannot add data until all checks are done; here we check, and there we add
			uint16_t remaining_size = zepto_parsing_remaining_bytes( &po2 );
			zepto_parser_init_by_parser( &po1, &po2 );
			zepto_parse_skip_block( &po1, remaining_size - 2 );
			zepto_convert_part_of_request_to_response( mem_h, &po2, &po1 );

			if ( ack_requested )
			{
				uint16_t num_err = 0; // TODO: ACTUAL SOURCE???
				zepto_parser_free_memory( mem_ack_h );
				siot_mesh_form_ack_packet( mem_ack_h, last_hop, num_err, packet_reported_checksum );
				return SIOT_MESH_RET_SEND_ACK_AND_PASS_TO_PROCESS;
			}

			return SIOT_MESH_RET_PASS_TO_PROCESS;
#endif // USED_AS_RETRANSMITTER
		}

		ZEPTO_DEBUG_ASSERT( NULL == "Error: we should not reach this point\n" );

		return SIOT_MESH_RET_OK;
	}
	return SIOT_MESH_RET_OK;
}

#ifdef USED_AS_RETRANSMITTER
bool siot_mesh_is_toward_root( MEMORY_HANDLE mem_h )
{
	parser_obj po;
	zepto_parser_init( &po, mem_h );
	uint16_t header = zepto_parse_encoded_uint16( &po );
	// TODO: here and then use bit-field processing instead
	if ( header & 1 ) // predefined packet types
	{
		uint8_t packet_type = ( header >> 1 ) & 0x7;
		switch ( packet_type )
		{
			case SIOT_MESH_FROM_SANTA_DATA_PACKET: 
				return false;
			case SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET:
			case SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET:
			case SIOT_MESH_ROUTING_ERROR_PACKET:
				return true;
			default:
				ZEPTO_DEBUG_ASSERT( 0 == "this call is not supposed to be done for this type of a packet" );
				return true;
		}
	}
	else
	{
		bool is_from_root = ( header >> 4 ) & 0x1;
		return 1 - is_from_root;
	}
}
#endif // USED_AS_RETRANSMITTER


uint8_t handler_siot_mesh_timer( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t* bus_id )
{
	uint16_t link_id;
#ifdef USED_AS_RETRANSMITTER
	ZEPTO_DEBUG_ASSERT( memory_object_get_response_size( mem_h ) == 0 );

	MESH_PENDING_RESENDS_PACKET_INFO info;
	uint8_t ret_code = siot_mesh_get_resend_task( mem_h, currt, &(wf->wait_time), &info );
	switch (ret_code )
	{
		case SIOT_MESH_RET_RESEND_TASK_NONE_EXISTS:
		case SIOT_MESH_RET_RESEND_TASK_NOT_NOW:
			break;
		case SIOT_MESH_RET_RESEND_TASK_INTERM:
		{
			zepto_response_to_request( mem_h );
			uint16_t checksum;
			bool route_known = siot_mesh_target_to_link_id( info.target_id, &link_id ) == SIOT_MESH_RET_OK;
			if ( route_known )
			{
				siot_mesh_form_unicast_packet( mem_h, link_id, bus_id, info.target_id, true, &checksum );
			}
			else
			{
				siot_mesh_remove_resend_task_by_device_id( info.target_id, currt, &(wf->wait_time) );
				siot_mesh_form_packet_to_santa( mem_h, 0xFF );
			}
			return SIOT_MESH_RET_PASS_TO_SEND;
			break;
		}
		case SIOT_MESH_RET_RESEND_TASK_FINAL:
		{
			// TODO: if the direction is toward the root, we send To-Santa; otherwise we send an error packet
			zepto_parser_free_memory( mem_h );
			siot_mesh_form_packet_to_santa( mem_h, 0xFF );
			return SIOT_MESH_RET_PASS_TO_SEND;
			break;
		}
#ifdef USED_AS_RETRANSMITTER
		case SIOT_MESH_RET_RESEND_TASK_FROM_SANTA:
		{
			*bus_id = info.bus_id;
			return SIOT_MESH_RET_PASS_TO_SEND;
			break;
		}
		case SIOT_MESH_RET_RESEND_TASK_UNICAST:
		{
			*bus_id = info.bus_id;
			return SIOT_MESH_RET_PASS_TO_SEND;
			break;
		}
		case SIOT_MESH_RET_RESEND_TASK_UNICAST_POSTFINAL:
		{
			// TODO: if the direction is toward the root, we send To-Santa; otherwise we send an error packet
			zepto_parser_free_memory( mem_h );
			if ( info.target_id == 0 )
				siot_mesh_form_packet_to_santa( mem_h, 0xFF );
			else
			{
				uint16_t packet_checksum;
				siot_mesh_form_routing_error_packet( mem_h, info.next_hop, info.target_id, info.checksum, &packet_checksum );
				// TODO: add to resend list
				SIOT_MESH_LINK link;
				uint8_t ret_code1 = siot_mesh_get_link_to_root( &link );
				if ( ret_code1 == SIOT_MESH_RET_OK ) // we have a route to root
				{
					*bus_id = link.BUS_ID;
					siot_mesh_at_root_add_resend_unicast_task( mem_h, currt, &(wf->wait_time), 0, link.BUS_ID, link.NEXT_HOP );
				}
				else
				{

					/// TODO: should we send a To-Santa packet in this case?
					// siot_mesh_form_packet_to_santa( mem_h, 0xFF );
				}
			}
			return SIOT_MESH_RET_PASS_TO_SEND;
			break;
		}
#endif
		default:
		{
			ZEPTO_DEBUG_ASSERT( 0 == "Unexpected ret code" );
			break;
		}
	}
	return SIOT_MESH_RET_OK;

#else // USED_AS_RETRANSMITTER
//	ZEPTO_DEBUG_PRINTF_1( "         +++++++++++++++++++############  handler_siot_mesh_timer() called  ###########++++++++++++++++++++++\n" );
	// TODO: processing on a terminating device is much easier, reimplement!!!
	uint8_t i;
	bool time_still_remains;
	bool packet_ready = false;
	ZEPTO_DEBUG_ASSERT( memory_object_get_request_size( mem_h ) == 0 );
	ZEPTO_DEBUG_ASSERT( memory_object_get_response_size( mem_h ) == 0 );
	// we first check, among outstanding resend tasks, the oldest to be performed; then we calculate a time of a nearest resend (it might be now (0) or in some future)
	sa_time_val oldest_time_point;
	SA_TIME_SET_INFINITE_TIME( oldest_time_point );
	uint8_t oldest_index = -1;
	for ( i=0; i<PENDING_RESENDS_MAX; i++ )
		if ( pending_resends[i].resend_cnt && sa_hal_time_val_is_less( &(pending_resends[i].next_resend_time), &oldest_time_point ) ) // used slot is yet older
		{
			sa_hal_time_val_copy_from( &oldest_time_point, &(pending_resends[i].next_resend_time) );
			oldest_index = i;
		}
	if ( oldest_index != (uint8_t)(-1) ) // at least, one potential candidate
	{
//		// find "second old"; this will give use how long to wait then
//		SA_TIME_SET_INFINITE_TIME( oldest_time_point );
			time_still_remains = sa_hal_time_val_get_remaining_time( currt, &(pending_resends[oldest_index].next_resend_time), &(wf->wait_time) );
			if ( !time_still_remains ) // time to process 
			{
				zepto_copy_request_to_response_of_another_handle( pending_resends[oldest_index].packet_h, mem_h ); // create our own copy
				zepto_response_to_request( mem_h );
				bool is_last = pending_resends[oldest_index].resend_cnt > 1;
//				bool route_known = siot_mesh_target_to_link_id( pending_resends[oldest_index].target_id, link_id ) == SIOT_MESH_RET_OK;
				bool route_known = siot_mesh_target_to_link_id( 0, &link_id ) == SIOT_MESH_RET_OK;
				if ( (!is_last) && route_known )
				{
					uint16_t checksum;
//					siot_mesh_form_unicast_packet( mem_h, *link_id, pending_resends[oldest_index].target_id, true, &checksum );
					siot_mesh_form_unicast_packet( mem_h, link_id, bus_id, 0, true, &checksum );
					ZEPTO_DEBUG_ASSERT( checksum == pending_resends[oldest_index].checksum );
					sa_time_val diff_tval;
					TIME_MILLISECONDS16_TO_TIMEVAL( MESH_RESEND_PERIOD_MS, diff_tval );
//					sa_hal_time_val_copy_from_if_src_less( &(wf->wait_time), &diff_tval );
					sa_hal_time_val_copy_from( &(pending_resends[oldest_index].next_resend_time), currt );
					SA_TIME_INCREMENT_BY_TICKS( pending_resends[oldest_index].next_resend_time, diff_tval );
					(pending_resends[oldest_index].resend_cnt)--;
					ZEPTO_DEBUG_PRINTF_3( "         ############  handler_siot_mesh_timer(): packet is abuot to be sent as unicast (resent cnt = %d, shecksum = %04x)  ###########\n", pending_resends[oldest_index].resend_cnt, checksum );
				}
				else
				{
					if ( is_last )
					{
						if ( siot_mesh_target_to_link_id( 0, &link_id ) == SIOT_MESH_RET_OK )
							siot_mesh_remove_link( link_id );
						siot_mesh_delete_route( 0 );
					}
//					siot_mesh_form_packet_to_santa( mem_h, 0xFF, pending_resends[oldest_index].target_id );
					siot_mesh_form_packet_to_santa( mem_h, 0xFF );
					pending_resends[oldest_index].resend_cnt = 0;
					ZEPTO_DEBUG_PRINTF_3( "         ############  handler_siot_mesh_timer(): packet is abuot to be sent as \'to santa\' (resent cnt = %d, shecksum = %04x)  ###########\n", pending_resends[oldest_index].resend_cnt, pending_resends[oldest_index].checksum );
				}
				if ( pending_resends[oldest_index].resend_cnt == 0 )
					zepto_parser_free_memory( pending_resends[oldest_index].packet_h );

				packet_ready = true;
			}
	}
	// now time period set to wf is at most as large as time until next resend (or not updated at all); let's check whether one of our resends should start earlier...

	for ( i=0; i<PENDING_RESENDS_MAX; i++ )
		if ( pending_resends[i].resend_cnt ) // used slot
			sa_hal_time_val_get_remaining_time( currt, &(pending_resends[i].next_resend_time), &(wf->wait_time) );
//	ZEPTO_DEBUG_PRINTF_3( "         ############  handler_siot_mesh_timer(): time to next event: %d, %d  ###########\n", wf->wait_time.low_t, wf->wait_time.high_t );

	return packet_ready ? SIOT_MESH_RET_PASS_TO_SEND : SIOT_MESH_RET_OK;
#endif
}

uint8_t handler_siot_mesh_send_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint8_t mesh_val, uint8_t resend_cnt, uint16_t target_id, uint16_t* bus_id )
{
	// TODO: what should we do, if the root in the route table, but incoming packet was "from Santa"?
	uint16_t link_id;
	uint8_t ret_code = siot_mesh_target_to_link_id( target_id, &link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		if ( resend_cnt < SIOT_MESH_SUBJECT_FOR_ACK )
		{
			uint16_t checksum;
			siot_mesh_form_unicast_packet( mem_h, link_id, bus_id, target_id, false, &checksum );
			ZEPTO_DEBUG_PRINTF_2( "         ############  handler_siot_mesh_send_packet(): known route (resend count: %d)  ###########\n", resend_cnt );
		}
		else
		{
#ifdef USED_AS_RETRANSMITTER
			uint16_t checksum;
			siot_mesh_form_unicast_packet( mem_h, link_id, bus_id, target_id, true, &checksum );
//			siot_mesh_add_resend_task( mem_h, MESH_PENDING_RESEND_TYPE_SELF_PACKET, currt, checksum, target_id, &(wf->wait_time) );

			SIOT_MESH_LINK link;
			uint8_t ret_code = siot_mesh_get_link( link_id, &link );
			ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK ); // we do not expect that being here we have an invalid link_id
			ZEPTO_DEBUG_ASSERT( link.LINK_ID == link_id );
			siot_mesh_add_resend_task( mem_h/*, MESH_PENDING_RESEND_TYPE_SELF_PACKET*/, currt, checksum, target_id, link.BUS_ID, link.NEXT_HOP, &(wf->wait_time) );
#else // USED_AS_RETRANSMITTER
			uint16_t checksum;
			siot_mesh_form_unicast_packet( mem_h, link_id, bus_id, target_id, true, &checksum );
			ZEPTO_DEBUG_PRINTF_3( "         ############  handler_siot_mesh_send_packet(): yet known route, requesting ACK (resend count: %d, shecksum = %04x)  ###########\n", resend_cnt, checksum );

			ZEPTO_DEBUG_ASSERT( PENDING_RESENDS_MAX == 2 );
			ZEPTO_DEBUG_ASSERT( pending_resends[0].resend_cnt == 0 || pending_resends[1].resend_cnt == 0 );
			uint8_t free_slot = pending_resends[0].resend_cnt == 0 ? 0 : 1;
			ZEPTO_DEBUG_PRINTF_2( "         ############  handler_siot_mesh_send_packet(): free slot = %d  ###########\n", free_slot );
			ZEPTO_DEBUG_ASSERT( memory_object_get_request_size( pending_resends[free_slot].packet_h ) == 0 );
			ZEPTO_DEBUG_ASSERT( memory_object_get_response_size( pending_resends[free_slot].packet_h ) == 0 );
			zepto_copy_request_to_response_of_another_handle( mem_h, pending_resends[free_slot].packet_h ); // create our own copy
			zepto_response_to_request( pending_resends[free_slot].packet_h );
			pending_resends[free_slot].checksum = checksum;
			pending_resends[free_slot].resend_cnt = SIOT_MESH_SUBJECT_FOR_MESH_RESEND;
			sa_time_val diff_tval;
			TIME_MILLISECONDS16_TO_TIMEVAL( MESH_RESEND_PERIOD_MS, diff_tval );
	//		sa_hal_time_val_copy_from_if_src_less( &(wf->wait_time), &diff_tval );
			sa_hal_time_val_copy_from( &(pending_resends[free_slot].next_resend_time), currt );
			SA_TIME_INCREMENT_BY_TICKS( pending_resends[free_slot].next_resend_time, diff_tval );
			ZEPTO_DEBUG_ASSERT( target_id == 0 ); // terminating sends to root only
	//		pending_resends[free_slot].target_id = (uint8_t)target_id;

			uint8_t i;
			for ( i=0; i<PENDING_RESENDS_MAX; i++ )
				if ( pending_resends[i].resend_cnt ) // used slot
					sa_hal_time_val_get_remaining_time( currt, &(pending_resends[i].next_resend_time), &(wf->wait_time) );
			ZEPTO_DEBUG_PRINTF_3( "         ############  handler_siot_mesh_send_packet(): time to next event: %d, %d  ###########\n", wf->wait_time.low_t, wf->wait_time.high_t );

#endif // USED_AS_RETRANSMITTER
		}
		return SIOT_MESH_RET_OK;
	}
	else
	{
		ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );

		// TODO: what link to use????
		siot_mesh_form_packet_to_santa( mem_h, mesh_val );
	}



	return SIOT_MESH_RET_OK;
}

uint8_t handler_siot_mesh_packet_rejected_broken( /*MEMORY_HANDLE mem_h, */uint8_t* mesh_val )
{
	if ( *mesh_val < 2 )
		if ( siot_mesh_clean_last_hop_data_storage_if_single_element( *mesh_val ) )
			*mesh_val = 0xFF;
	return SIOT_MESH_RET_OK;
}

void handler_siot_process_route_update_request( parser_obj* po, MEMORY_HANDLE reply )
{
	uint8_t ret_code;
	uint16_t flags = zepto_parse_encoded_uint16( po );
	// TODO: use bit field processing where applicable
	bool discard_rt = flags & 1;
	// TODO: address other flags: bit[1] being UPDATE-MAX-TTL flag, bit[2] being UPDATE-FORWARD-TO-SANTA-DELAY flag, bit[3] being UPDATE-MAX-NODE-RANDOM-DELAY flag, and bits[4..] reserved (MUST be zeros)
	//       by now we assume that none are set until implemented
	ZEPTO_DEBUG_ASSERT( (flags >> 1) == 0 ); // until implemented

	// TODO: we also assume that there is no optional header
	// TODO: check spec regarding optional headers

	// OPTIONAL-ORIGINAL-RT-CHECKSUM and discarding table
	if ( discard_rt ) // bit[0] being DISCARD-RT-FIRST (indicating that before processing MODIFICATIONS-LIST, the whole Routing Table must be discarded)
		siot_mesh_clear_rout_table();
	else
	{
		uint16_t checksum_before_calc = siot_mesh_calculate_route_table_checksum();
		uint16_t checksum_before_read = zepto_parse_uint8( po );
		checksum_before_read |= ((uint16_t)zepto_parse_uint8( po )) << 8;
		if ( checksum_before_read != checksum_before_calc )
		{
			ZEPTO_DEBUG_PRINTF_3( "Bad route table checksum: claimed 0x%04x, calculated: 0x%04x\n", checksum_before_read, checksum_before_calc );
			ZEPTO_DEBUG_ASSERT( 0 == "Reporting Bad route table checksum is not yet implemented" );
			return;
		}
	}

	// | OPTIONAL-MAX-TTL | OPTIONAL-FORWARD-TO-SANTA-DELAY-UNIT | OPTIONAL-FORWARD-TO-SANTA-DELAY | OPTIONAL-MAX-FORWARD-TO-SANTA-DELAY | OPTIONAL-MAX-NODE-RANDOM-DELAY-UNIT | OPTIONAL-MAX-NODE-RANDOM-DELAY | 

	// MODIFICATIONS-LIST
	uint16_t entry_header;
	do
	{
		entry_header = zepto_parse_encoded_uint16( po );
		uint8_t type = (entry_header>>1) & 3; // bits[1..2] equal to a 2-bit constant...
		switch ( type )
		{
			case ADD_OR_MODIFY_LINK_ENTRY:
			{
				SIOT_MESH_LINK link;
				// | ADD-OR-MODIFY-LINK-ENTRY-AND-LINK-ID | BUS-ID | NEXT-HOP | NEXT-HOP-ACKS-AND-INTRA-BUS-ID-PLUS-1 | OPTIONAL-LINK-DELAY-UNIT | OPTIONAL-LINK-DELAY | OPTIONAL-LINK-DELAY-ERROR |
				bool link_delay_present = ( entry_header >> 3 ) & 1;// bit[3] being LINK-DELAY-PRESENT flag
				link.LINK_ID = entry_header >> 4; // bits[4..] equal to LINK-ID
				link.BUS_ID = zepto_parse_encoded_uint16( po );
				link.NEXT_HOP = zepto_parse_encoded_uint16( po );
				// NEXT-HOP-ACKS-AND-INTRA-BUS-ID: Encoded-Unsigned-Int<max=4> bitfield substrate 
				uint32_t ibid_pl_1 = zepto_parse_encoded_uint32( po );
				link.NEXT_HOP_ACKS = ibid_pl_1 & 1; // bit[0] being a NEXT-HOP-ACKS flag for the Routing Table Entry
#if 0 // not yet implemented at all
				link.INTRA_BUS_ID = ibid_pl_1 >> 1; // bits[1..] representing INTRA-BUS-ID-PLUS-1
				if ( link.INTRA_BUS_ID == 0 ) // means that INTRA-BUS-ID==NULL, and therefore that the link entry is an incoming link entry
				{
					ZEPTO_DEBUG_ASSERT( 0 == "INTRA-BUS-ID==NULL is not yet implemented" );
					return;
				}
				else
					(link.INTRA_BUS_ID)--; // TODO: distinguishing NULL and 0
#endif // 0
				if ( link_delay_present )
				{
					link.LINK_DELAY_UNIT = zepto_parse_encoded_uint16( po );
					link.LINK_DELAY = zepto_parse_encoded_uint16( po );
					link.LINK_DELAY_ERROR = zepto_parse_encoded_uint16( po );
				}
				else
				{
					// TODO: how to mark that such values were not specified???
				}
				ret_code = siot_mesh_add_link( &link );
				if ( ret_code != SIOT_MESH_RET_OK )
				{
					ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_OUT_OF_RANGE );
					ZEPTO_DEBUG_ASSERT( 0 == "Link list Out-of-range error reporting is not yet implemented" );
				}
				break;
			}
			case DELETE_LINK_ENTRY:
			{
				uint16_t link_id = entry_header >> 3; // bits[3..] equal to LINK-ID
				ret_code = siot_mesh_remove_link( link_id );
				if ( ret_code != SIOT_MESH_RET_OK )
				{
					ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );
					ZEPTO_DEBUG_ASSERT( 0 == "Link to be removed is not found; error reporting is not yet implemented" ); // NOTE: it looks like, if we have already checked a checksum, this is not (statistically) possible
				}
				break;
			}
			case ADD_OR_MODIFY_ROUTE_ENTRY:
			{
				// | ADD-OR-MODIFY-ROUTE-ENTRY-AND-LINK-ID | TARGET-ID |
				uint16_t link_id = entry_header >> 3; // bits[3..] equal to LINK-ID
				uint16_t target_id = zepto_parse_encoded_uint16( po );
				ret_code = siot_mesh_add_route( target_id, link_id );
				if ( ret_code != SIOT_MESH_RET_OK )
				{
					ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_OUT_OF_RANGE );
					ZEPTO_DEBUG_ASSERT( 0 == "Route list Out-of-range error reporting is not yet implemented" );
				}
				break;
			}
			case DELETE_ROUTE_ENTRY:
			{
				uint16_t target_id = entry_header >> 3; // bits[3..] equal to TARGET-ID
				break;
			}
		}
	}
	while ( entry_header & 1 ); // bit[0] marks the end of MODIFICATIONS-LIST

	// RESULTING-RT-CHECKSUM
	uint16_t checksum_before_calc = siot_mesh_calculate_route_table_checksum();
	uint16_t checksum_before_read = zepto_parse_uint8( po );
	checksum_before_read |= ((uint16_t)zepto_parse_uint8( po )) << 8;
	if ( checksum_before_read != checksum_before_calc )
	{
		ZEPTO_DEBUG_ASSERT( 0 == "Reporting Bad route table checksum is not yet implemented" );
		return;
	}

	// if we're here, everything is fine, and we report no error
	zepto_parser_encode_and_append_uint8( reply, 0 ); // no error
}

#endif // USED_AS_MASTER