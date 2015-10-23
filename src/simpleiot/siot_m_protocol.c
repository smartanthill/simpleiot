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

//DECLARE_DEVICE_ID

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

typedef struct _MESH_PENDING_RESENDS
{
	MEMORY_HANDLE packet_h;
	uint8_t resend_cnt;
	sa_time_val next_resend_time;
	uint16_t checksum;
	uint16_t target_id; // TODO: for a terminating device it is always 0 (rooot), right? - subject for optimization
} MESH_PENDING_RESENDS;

#ifdef USED_AS_RETRANSMITTER

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

uint8_t siot_mesh_add_link( SIOT_MESH_LINK* link )
{
	uint16_t i;
	uint16_t link_id = link->LINK_ID;
	for ( i=0; i<siot_mesh_link_table_size; i++ )
	{
		if ( siot_mesh_link_table[i].LINK_ID < link_id )
			continue;
		if ( siot_mesh_link_table[i].LINK_ID == link_id ) // found
		{
			ZEPTO_MEMCPY( siot_mesh_link_table + i, link, sizeof(SIOT_MESH_LINK) );
			return SIOT_MESH_RET_OK;
		}
		break;
	}
	// as we are here, the entry does not exist at all, and we have to add it rather than modify existing
	if ( siot_mesh_link_table_size == SIOT_MESH_LINK_TABLE_SIZE_MAX )
		return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;

	ZEPTO_MEMMOV( siot_mesh_link_table + i + 1, siot_mesh_link_table + i, sizeof(SIOT_MESH_LINK) * ( siot_mesh_link_table_size - i ) );
	ZEPTO_MEMCPY( siot_mesh_link_table + i, link, sizeof(SIOT_MESH_LINK) );
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
	for ( i=0; i<siot_mesh_route_table_size; i++ )
	{
		if ( siot_mesh_route_table[i].TARGET_ID < target_id )
			continue;
		if ( siot_mesh_route_table[i].TARGET_ID == target_id ) // found
		{
			siot_mesh_route_table[i].LINK_ID = link_id;
			return SIOT_MESH_RET_OK;
		}
		break;
	}
	// as we are here, the entry does not exist at all, and we have to add it rather than modify existing
	if ( siot_mesh_route_table_size == SIOT_MESH_ROUTE_TABLE_SIZE_MAX )
		return SIOT_MESH_RET_ERROR_OUT_OF_RANGE;

	ZEPTO_MEMMOV( siot_mesh_route_table + i + 1, siot_mesh_route_table + i, sizeof(SIOT_MESH_ROUTE) * ( siot_mesh_route_table_size - i ) );
	siot_mesh_route_table[i].TARGET_ID = target_id;
	siot_mesh_route_table[i].LINK_ID = link_id;
	siot_mesh_route_table_size++;
	return SIOT_MESH_RET_OK;
}

uint16_t siot_mesh_calculate_route_table_checksum()
{
	return 0;
}

MEMORY_HANDLE resend_task_holder_mh = MEMORY_HANDLE_INVALID;

#define SIOT_MESH_RET_RESEND_TASK_NONE_EXISTS 6
#define SIOT_MESH_RET_RESEND_TASK_NOT_NOW 7
#define SIOT_MESH_RET_RESEND_TASK_INTERM 8
#define SIOT_MESH_RET_RESEND_TASK_FINAL 9
#define PENDING_RESENDS_MAX 2 // WARNING: if changed, have more handles (at least, potentially)!
MESH_PENDING_RESENDS pending_resends[PENDING_RESENDS_MAX] = { {MEMORY_HANDLE_MESH_1, 0}, { MEMORY_HANDLE_MESH_2, 0} };

void siot_mesh_add_resend_task( MEMORY_HANDLE packet, const sa_time_val* currt, uint16_t checksum, uint16_t target_id, sa_time_val* time_to_next_event )
{
	// 1. prepare resend task
	MESH_PENDING_RESENDS resend;
	resend.target_id = target_id;
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
	zepto_copy_request_to_response_of_another_handle( packet, resend.packet_h );
	zepto_response_to_request( resend.packet_h );

	// 2. add resend task
/*	parser_obj po_start, po_end;
	zepto_parser_init( &po_start, resend_task_holder_mh );
	zepto_parser_init( &po_end, resend_task_holder_mh );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po_start ) );
	zepto_convert_part_of_request_to_response( resend_task_holder_mh, &po_start, &po_end );
	zepto_write_block( resend_task_holder_mh, &resend, sizeof( MESH_PENDING_RESENDS ) );
	zepto_response_to_request( resend_task_holder_mh );*/
	zepto_memman_append_locally_generated_data( resend_task_holder_mh, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );

	// 3. calculate time to the nearest event
//	zepto_parser_init( &po_start, resend_task_holder_mh );
	uint16_t offset;
	sa_time_val remaining;
	SA_TIME_SET_INFINITE_TIME( *time_to_next_event );
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
//	while ( zepto_parsing_remaining_bytes( &po_start ) )
	{
//		zepto_parse_read_block( &po_start, &resend, sizeof( MESH_PENDING_RESENDS ) );
		if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		uint8_t in_future = sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), &remaining );
		sa_hal_time_val_copy_from_if_src_less( time_to_next_event, &remaining );
		if ( !in_future )
			break;
	}
}

uint8_t siot_mesh_get_resend_task( MEMORY_HANDLE packet, const sa_time_val* currt, uint16_t* target_id, sa_time_val* time_to_next_event )
{
	uint8_t it = 0, it_oldest = (uint8_t)(-1);
	uint16_t offset;
	MESH_PENDING_RESENDS resend;
	sa_time_val oldest_time_point;
	sa_hal_time_val_copy_from( &oldest_time_point, currt );

//	parser_obj po, po_end;
/*	zepto_parser_init( &po, resend_task_holder_mh );
	if ( zepto_parsing_remaining_bytes( &po ) == 0 )
		return SIOT_MESH_RET_RESEND_TASK_NONE_EXISTS;*/
	if ( zepto_memman_get_currently_allocated_size_for_locally_generated_data( resend_task_holder_mh ) )
		return SIOT_MESH_RET_RESEND_TASK_NONE_EXISTS;

/*	while ( zepto_parsing_remaining_bytes( &po ) )
	{
		zepto_parse_read_block( &po, &resend, sizeof( MESH_PENDING_RESENDS ) );*/
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
//	while ( zepto_parsing_remaining_bytes( &po_start ) )
	{
//		zepto_parse_read_block( &po_start, &resend, sizeof( MESH_PENDING_RESENDS ) );
		if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		if ( it_oldest == (uint8_t)(-1) )
		{
			if ( sa_hal_time_val_is_less_eq( &(resend.next_resend_time), &oldest_time_point ) ) // used slot is yet older
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

/*	zepto_parser_init( &po, resend_task_holder_mh );
	while ( zepto_parsing_remaining_bytes( &po ) )
	{
		zepto_parse_read_block( &po, &resend, sizeof( MESH_PENDING_RESENDS ) );*/
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
//	while ( zepto_parsing_remaining_bytes( &po_start ) )
	{
//		zepto_parse_read_block( &po_start, &resend, sizeof( MESH_PENDING_RESENDS ) );
		if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		ZEPTO_DEBUG_ASSERT( resend.resend_cnt > 0 );
		if ( it_oldest == (uint8_t)(-1) )
		{
			if ( sa_hal_time_val_is_less_eq( &(resend.next_resend_time), &oldest_time_point ) ) // used slot is yet older
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
/*		zepto_parser_init( &po, resend_task_holder_mh );
		while ( zepto_parsing_remaining_bytes( &po ) )
		{
			zepto_parse_read_block( &po, &resend, sizeof( MESH_PENDING_RESENDS ) );*/
		for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	//	while ( zepto_parsing_remaining_bytes( &po_start ) )
		{
	//		zepto_parse_read_block( &po_start, &resend, sizeof( MESH_PENDING_RESENDS ) );
			if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
				break;
			sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
		}
		return SIOT_MESH_RET_RESEND_TASK_NOT_NOW;
	}

	// there is a packet to resend
/*	zepto_parser_init( &po, resend_task_holder_mh );
	zepto_parser_init( &po_end, resend_task_holder_mh );
	zepto_parse_skip_block( &po_end, it_oldest * sizeof( MESH_PENDING_RESENDS ) );
	zepto_parse_read_block( &po, &resend, sizeof( MESH_PENDING_RESENDS ) );*/
	zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, it_oldest * sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );

	bool final_in_seq = resend.resend_cnt == 1;
	ZEPTO_DEBUG_ASSERT( resend.resend_cnt > 0 );
	*target_id = resend.target_id;
	zepto_parser_free_memory( packet );
	zepto_copy_request_to_response_of_another_handle( resend.packet_h, packet );
	zepto_response_to_request( packet );

	if ( final_in_seq ) // erase resend task
	{
		release_memory_handle( resend.packet_h );
/*		zepto_parser_init( &po, resend_task_holder_mh );
		zepto_parser_init( &po_end, resend_task_holder_mh );
		zepto_parse_skip_block( &po_end, it_oldest * sizeof( MESH_PENDING_RESENDS ) );
		zepto_append_part_of_request_to_response( resend_task_holder_mh, &po, &po_end );
		zepto_parse_skip_block( &po, sizeof( MESH_PENDING_RESENDS ) );
		zepto_parse_skip_block( &po_end, sizeof( MESH_PENDING_RESENDS ) );
		zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po ) );
		zepto_append_part_of_request_to_response( resend_task_holder_mh, &po, &po_end );
		zepto_response_to_request( resend_task_holder_mh );*/
		for ( offset=it_oldest * sizeof( MESH_PENDING_RESENDS ); ; offset += sizeof( MESH_PENDING_RESENDS ) )
		{
			if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset + sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
				break;
			zepto_memman_write_locally_generated_data_by_offset( resend_task_holder_mh, offset, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
		}
		zepto_memman_trim_locally_generated_data_at_right( resend_task_holder_mh, sizeof( MESH_PENDING_RESENDS ) );
	}
	else // update task
	{
		(resend.resend_cnt) --;
		sa_time_val diff_tval;
		TIME_MILLISECONDS16_TO_TIMEVAL( MESH_RESEND_PERIOD_MS, diff_tval );
		sa_hal_time_val_copy_from( &(resend.next_resend_time), currt );
		SA_TIME_INCREMENT_BY_TICKS( resend.next_resend_time, diff_tval );
		// update data in memory
/*		zepto_parser_init( &po, resend_task_holder_mh );
		zepto_parser_init( &po_end, resend_task_holder_mh );
		zepto_parse_skip_block( &po_end, it_oldest * sizeof( MESH_PENDING_RESENDS ) );
		zepto_append_part_of_request_to_response( resend_task_holder_mh, &po, &po_end );
		zepto_write_block( resend_task_holder_mh, &resend, sizeof( MESH_PENDING_RESENDS ) );
		zepto_parse_skip_block( &po, sizeof( MESH_PENDING_RESENDS ) );
		zepto_parse_skip_block( &po_end, sizeof( MESH_PENDING_RESENDS ) );
		zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po ) );
		zepto_append_part_of_request_to_response( resend_task_holder_mh, &po, &po_end );
		zepto_response_to_request( resend_task_holder_mh );*/
		zepto_memman_write_locally_generated_data_by_offset( resend_task_holder_mh, it_oldest * sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
	}

	// now we calculate remaining time for only actually remaining tasks
/*	zepto_parser_init( &po, resend_task_holder_mh );
	while ( zepto_parsing_remaining_bytes( &po ) )
	{
		zepto_parse_read_block( &po, &resend, sizeof( MESH_PENDING_RESENDS ) );
		sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
	}*/
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset + sizeof( MESH_PENDING_RESENDS ), sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
	}

	return final_in_seq ? SIOT_MESH_RET_RESEND_TASK_FINAL : SIOT_MESH_RET_RESEND_TASK_INTERM;
}

void siot_mesh_remove_resend_task_by_hash( uint16_t checksum, const sa_time_val* currt, sa_time_val* time_to_next_event )
{
	uint16_t offset, gap;
	MESH_PENDING_RESENDS resend;
//	bool one_found = false;
	sa_time_val oldest_time_point;
	sa_hal_time_val_copy_from( &oldest_time_point, currt );

/*	parser_obj po, po_end;
	zepto_parser_init( &po, resend_task_holder_mh );
	if ( zepto_parsing_remaining_bytes( &po ) == 0 )
		return;*/
	if ( zepto_memman_get_currently_allocated_size_for_locally_generated_data( resend_task_holder_mh ) )
		return;

/*	while ( zepto_parsing_remaining_bytes( &po ) )
	{
		zepto_parse_read_block( &po, &resend, sizeof( MESH_PENDING_RESENDS ) );*/
	gap = 0;
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset + gap, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		if ( resend.checksum == checksum )
			gap += sizeof( MESH_PENDING_RESENDS );
		else
			if ( gap )	zepto_memman_write_locally_generated_data_by_offset( resend_task_holder_mh, offset + gap, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
		{
		}
	}
	if ( gap )	zepto_memman_trim_locally_generated_data_at_right( resend_task_holder_mh, gap );

	// now we calculate remaining time for only actually remaining tasks
/*	for ( it = pending_resends.begin(); it != pending_resends.end(); ++it )
		sa_hal_time_val_get_remaining_time( currt, &(it->next_resend_time), time_to_next_event );*/
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset + gap, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
	}
}

void siot_mesh_remove_resend_task_by_device_id( uint16_t target_id, const sa_time_val* currt, sa_time_val* time_to_next_event )
{
/*	PENDING_RESENDS_ITERATOR it = pending_resends.begin(), it_erase;
	sa_time_val oldest_time_point;
	sa_hal_time_val_copy_from( &oldest_time_point, currt );

	if ( pending_resends.size() == 0 )
		return;

	bool one_found = false;

	while ( it != pending_resends.end() )
	{
		if ( it->target_id == target_id )
		{
			it_erase = it;
			++it;
			pending_resends.erase( it_erase);
		}
		else
			++it;
	}

	// now we calculate remaining time for only actually remaining tasks
	for ( it = pending_resends.begin(); it != pending_resends.end(); ++it )
		sa_hal_time_val_get_remaining_time( currt, &(it->next_resend_time), time_to_next_event );*/

	uint16_t offset, gap;
	MESH_PENDING_RESENDS resend;
	sa_time_val oldest_time_point;
	sa_hal_time_val_copy_from( &oldest_time_point, currt );

	if ( zepto_memman_get_currently_allocated_size_for_locally_generated_data( resend_task_holder_mh ) )
		return;

	gap = 0;
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset + gap, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		if ( resend.target_id == target_id )
			gap += sizeof( MESH_PENDING_RESENDS );
		else
			if ( gap )	zepto_memman_write_locally_generated_data_by_offset( resend_task_holder_mh, offset + gap, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) );
	}
	if ( gap )	zepto_memman_trim_locally_generated_data_at_right( resend_task_holder_mh, gap );

	// now we calculate remaining time for only actually remaining tasks
	for ( offset=0; ; offset+=sizeof( MESH_PENDING_RESENDS ) )
	{
		if ( ! zepto_memman_read_locally_generated_data_by_offset( resend_task_holder_mh, offset + gap, sizeof( MESH_PENDING_RESENDS ), (uint8_t*)(&resend) ) )
			break;
		sa_hal_time_val_get_remaining_time( currt, &(resend.next_resend_time), time_to_next_event );
	}
}
#else // USED_AS_RETRANSMITTER

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

void siot_mesh_form_packet_from_santa( MEMORY_HANDLE mem_h, uint16_t target_id, uint16_t bus_id_used )
{
	// Santa Packet structure: | SAMP-FROM-SANTA-DATA-PACKET-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | REQUEST-ID | OPTIONAL-DELAY-UNIT | MULTIPLE-RETRANSMITTING-ADDRESSES | BROADCAST-BUS-TYPE-LIST | Target-Address | OPTIONAL-TARGET-REPLY-DELAY | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
	// TODO: here and then use bit-field processing instead

	parser_obj po, po1;

	// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, OPTIONAL-EXTRA-HEADERS
	uint16_t header = 1 | ( SIOT_MESH_FROM_SANTA_DATA_PACKET << 1 ) | ( 4 << 5 ); // '1', packet type, 0 (no extra headers), TTL = 4
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// LAST-HOP
	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // TODO: is it a self-ID? Then for Client it is 0, and some unique velue for other devices. SOURCE???

	// LAST-HOP-BUS-ID
	zepto_parser_encode_and_append_uint16( mem_h, bus_id_used ); // TODO: is it a self-ID? Then for Client it is 0, and some unique velue for other devices. SOURCE???

	// REQUEST-ID
	static uint16_t rq_id = 0; // REQUEST-ID
	rq_id++; // TODO: make true global; TODO: think whether this id is globally or per-device unique
	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // REQUEST-ID

	// OPTIONAL-DELAY-UNIT is present only if EXPLICIT-TIME-SCHEDULING flag is present; currently we did not added it

	// MULTIPLE-RETRANSMITTING-ADDRESSES 
	// just adding terminator...
	header = 1 | 0; // TODO: Check that this interpretation is correct !!!!!!!!!!!!!
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// BROADCAST-BUS-TYPE-LIST
	// TODO: what should we add here?
	zepto_write_uint8( mem_h, 0 ); // list terminator

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

void siot_mesh_form_unicast_packet( uint16_t target_id, MEMORY_HANDLE mem_h, uint16_t link_id, bool request_ack, uint16_t* packet_checksum )
{
	//  | SAMP-UNICAST-DATA-PACKET-FLAGS-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | LAST-HOP | Non-Root-Address | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

	parser_obj po, po1;
	uint16_t header;
	SIOT_MESH_LINK link;
	uint8_t ret_code = siot_mesh_get_link( target_id, link_id, &link );
	ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK ); // we do not expect that being here we have an invalid link_id
	ZEPTO_DEBUG_ASSERT( link.LINK_ID == link_id );
	// !!! TODO: it might happen that this packet is not a reply to 'from Santa'; check, which data should be added this case

	// header: bit[0]: 0, bit[1]: GUARANTEED-DELIVERY flag, bit[2]: BACKWARD-GUARANTEED-DELIVERY, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL
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
	zepto_parser_encode_and_append_uint16( mem_h, target_id );

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

uint8_t handler_siot_mesh_send_packet( sa_time_val* currt, waiting_for* wf, uint16_t target_id, MEMORY_HANDLE mem_h, uint8_t resend_cnt, uint16_t* link_id )
{
	uint8_t ret_code = siot_mesh_at_root_target_to_link_id( target_id, link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		if ( resend_cnt < SIOT_MESH_SUBJECT_FOR_ACK )
		{
			uint16_t checksum;
			siot_mesh_form_unicast_packet( target_id, mem_h, *link_id, false, &checksum );
			ZEPTO_DEBUG_PRINTF_1( "         ############  handler_siot_mesh_send_packet(): known route  ###########\n" );
			return SIOT_MESH_RET_OK;
		}

		uint16_t checksum;
		siot_mesh_form_unicast_packet( target_id, mem_h, *link_id, false, &checksum );
		sa_time_val wait_tval;
		siot_mesh_at_root_add_resend_task( mem_h, currt, checksum, target_id, &wait_tval );
		sa_hal_time_val_copy_from_if_src_less( &(wf->wait_time), &wait_tval );

		return SIOT_MESH_RET_OK;
	}
	else
	{
		ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );

		// packet "from Santa" will be sent... let's form a packet

		// TODO: determine which physical links we will use; we will have to iterate over all of them
		// NOTE: currently we assume that we have a single link with bus_id = 0
		uint16_t bus_id_to_use = 0;
		siot_mesh_form_packet_from_santa( mem_h, target_id, bus_id_to_use );
	}


	return SIOT_MESH_RET_OK;
}

uint8_t siot_mesh_process_received_tosanta_packet( MEMORY_HANDLE mem_h, uint16_t src_id, uint16_t bus_id_at_src, uint16_t first_receiver_id, uint8_t conn_quality_at_first_receiver, bool payload_present, uint16_t request_id )
{
	// here we already know that the packet is good enough
	parser_obj po, po1, po2;
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	uint16_t total_packet_sz = zepto_parsing_remaining_bytes( &po );

	uint16_t header = zepto_parse_encoded_uint16( &po );
	// TODO: here and then use bit-field processing instead
	ZEPTO_DEBUG_ASSERT ( header & 1 ); // must already be checked
	ZEPTO_DEBUG_ASSERT ( ( ( header >> 1 ) & 0x7 ) == SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET ); // must already be checked

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
		return SIOT_MESH_RET_OK;
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
	}
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

uint8_t handler_siot_mesh_timer( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t* device_id, uint16_t* link_id )
{
	// WARNING: this development is just in the middle ... don't be surprized ...
	// NOTE: there are a number of things that can be done by timer; on this development stage we assume that they happen somehow in the order as implemented
	// TODO: actual implementation

	ZEPTO_DEBUG_ASSERT( memory_object_get_response_size( mem_h ) == 0 );

	uint8_t ret_code = siot_mesh_at_root_get_resend_task( mem_h, currt, device_id, &(wf->wait_time) );
	zepto_response_to_request( mem_h );
	switch (ret_code )
	{
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_NONE_EXISTS:
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_NOT_NOW:
			break;
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_INTERM:
		{
			uint16_t checksum;
			bool route_known = siot_mesh_at_root_target_to_link_id( *device_id, link_id ) == SIOT_MESH_RET_OK;
			if ( route_known )
			{
				siot_mesh_form_unicast_packet( *device_id, mem_h, *link_id, true, &checksum );
			}
			else
			{
				siot_mesh_at_root_remove_resend_task_by_device_id( *device_id, currt, &(wf->wait_time) );
				uint16_t bus_id_to_use = 0;
				siot_mesh_form_packet_from_santa( mem_h, *device_id, bus_id_to_use );
			}
			return SIOT_MESH_RET_PASS_TO_SEND;
			break;
		}
		case SIOT_MESH_AT_ROOT_RET_RESEND_TASK_FINAL:
		{
			siot_mesh_at_root_remove_link_to_target( *device_id );
			uint16_t bus_id_to_use = 0;
			siot_mesh_form_packet_from_santa( mem_h, *device_id, bus_id_to_use );
			return SIOT_MESH_RET_PASS_TO_SEND;
			break;
		}
	}

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
			ret_code = siot_mesh_at_root_add_updates_for_device( target_id, bus_id_at_target, id_from, bus_id_at_prev, id_next /*more data may be required*/ );
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
				ZEPTO_DEBUG_ASSERT( last_hop_id == 0 ); // from ROOT as long as we have not implemented and do not expect other options

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

				// SOURCE-ID
				*src_id = zepto_parse_encoded_uint16( &po );

				// BUS-ID-AT-SOURCE
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
						siot_mesh_process_received_tosanta_packet( mem_h, *src_id, bus_id_at_src, 0, conn_quality, remaining_size > 2, request_id );
						// TODO: what are and what to do with ret values?
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
				break;
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

		// header: bit[0]: 0, bit[1]: GUARANTEED-DELIVERY flag, bit[2]: BACKWARD-GUARANTEED-DELIVERY, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL
		bool ack_requested = ( header >> 1 ) & 0x1;
		ZEPTO_DEBUG_ASSERT( 0 == (( header >> 2 ) & 0x1 ) ); // reserved
		bool extra_headers_present = ( header >> 3 ) & 0x1;
		bool direction_flag = ( header >> 4 ) & 0x1;

		if ( direction_flag )
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
		uint16_t next_hop = zepto_parse_encoded_uint16( &po ); // TODO: here, in a correct packet we expect to see 0; what to do in case of errors?

		// LAST-HOP
		uint16_t last_hop = zepto_parse_encoded_uint16( &po );

		// Non-Root-Address
		*src_id = zepto_parse_encoded_uint16( &po );

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
}

uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint8_t* mesh_val, uint8_t signal_level, uint8_t error_cnt )
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
				ZEPTO_DEBUG_ASSERT( last_hop_id == 0 ); // from ROOT as long as we have not implemented and do not expect other options

				// Target-Address
				header = zepto_parse_encoded_uint16( &po );
				ZEPTO_DEBUG_ASSERT( (header & 1) == 0 ); // we have not yet implemented extra data
				uint16_t target_id = header >> 1;
				if ( target_id != DEVICE_SELF_ID )
				{
					ZEPTO_DEBUG_PRINTF_3( "Packet for device %d received (from Santa); ignored (self id: %d)\n", target_id, DEVICE_SELF_ID );
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

				// remove resend tasks (if any)
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

				return SIOT_MESH_RET_OK;
			}
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
				ZEPTO_DEBUG_ASSERT( last_hop_id == 0 ); // from ROOT as long as we have not implemented and do not expect other options

				// LAST-HOP-BUS-ID
				uint16_t last_hop_bus_id = zepto_parse_encoded_uint16( &po );
				ZEPTO_DEBUG_ASSERT( last_hop_bus_id == 0 ); // from ROOT as long as we have not implemented and do not expect other options

				// REQUEST-ID
				uint16_t request_id = zepto_parse_encoded_uint16( &po );

				// OPTIONAL-DELAY-UNIT is present only if EXPLICIT-TIME-SCHEDULING flag is present; currently we did not added it

				// MULTIPLE-RETRANSMITTING-ADDRESSES 
				header = zepto_parse_encoded_uint16( &po );
				ZEPTO_DEBUG_ASSERT( header == 1 ); // just terminator is expected in present implementation

				// BROADCAST-BUS-TYPE-LIST
				// TODO: what should we add here?
				uint8_t bus_type = zepto_parse_uint8( &po );
				while ( bus_type != 0 ) // 0 is a terminator
				{
					ZEPTO_DEBUG_ASSERT( 0 == "we have not implemented yet anything with non-empty list" );
					// do something
					bus_type = zepto_parse_uint8( &po );
				}

				// Target-Address
				header = zepto_parse_encoded_uint16( &po );
				ZEPTO_DEBUG_ASSERT( (header & 1) == 0 ); // we have not yet implemented extra data
				uint16_t target_id = header >> 1;
				if ( target_id != DEVICE_SELF_ID )
				{
					ZEPTO_DEBUG_PRINTF_3( "Packet for device %d received (from Santa); ignored (self id: %d)\n", target_id, DEVICE_SELF_ID );
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				}

				// OPTIONAL-TARGET-REPLY-DELAY

				// OPTIONAL-PAYLOAD-SIZE

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
				break;
			}
			default:
			{
				ZEPTO_DEBUG_PRINTF_2( "Packet type %d received; not expected or not implemented\n", (header >> 1) & 0x7 );
//				ZEPTO_DEBUG_ASSERT( NULL == "Error: processing of mesh packets of this type is not implemented\n" );
				return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
				break;
			}
		}
	}
	else
	{
/*		// for intermediate stages of development we treat all packets with first bit 0 as "ordinary packets" the rest of which is a payload to be passed to upper levels.
		// we assume that in such "underdeveloped" packets the whole header is 0
		ZEPTO_DEBUG_ASSERT( header == 0 ); // as long as packet processing will be properly developed with all details
		parser_obj po1;
		zepto_parser_init_by_parser( &po1, &po );
		zepto_parse_skip_block( &po1, zepto_parsing_remaining_bytes( &po ) );
		zepto_convert_part_of_request_to_response( mem_h, &po, &po1 );

		return SIOT_MESH_RET_PASS_TO_PROCESS;*/
		//  | SAMP-UNICAST-DATA-PACKET-FLAGS-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | LAST-HOP | Non-Root-Address | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

		// header: bit[0]: 0, bit[1]: GUARANTEED-DELIVERY flag, bit[2]: BACKWARD-GUARANTEED-DELIVERY, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL
		bool ack_requested = ( header >> 1 ) & 0x1;
		ZEPTO_DEBUG_ASSERT( 0 == (( header >> 2 ) & 0x1 ) ); // reserved
		bool extra_headers_present = ( header >> 3 ) & 0x1;
		bool direction_flag = ( header >> 4 ) & 0x1;

		if ( !direction_flag )
		{
			ZEPTO_DEBUG_PRINTF_1( "Packet directed to ROOT received; ignored\n" );
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
		uint16_t next_hop = zepto_parse_encoded_uint16( &po ); // TODO: here, in a correct packet we expect to see 0; what to do in case of errors?

		// LAST-HOP
		uint16_t last_hop = zepto_parse_encoded_uint16( &po );

		// Non-Root-Address
		uint16_t src_id = zepto_parse_encoded_uint16( &po );

		if ( src_id != DEVICE_SELF_ID )
		{
			// TODO: it is much more complicated in case of retransmitter
			ZEPTO_DEBUG_PRINTF_3( "Packet for device %d received (unicast); ignored (self id: %d)\n", src_id, DEVICE_SELF_ID );
			return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;
		}

		// OPTIONAL-PAYLOAD-SIZE

		// HEADER-CHECKSUM
		uint16_t actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &po1, total_packet_sz - zepto_parsing_remaining_bytes( &po ), 0 );
		uint16_t header_checksum = zepto_parse_uint8( &po );
		header_checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
		zepto_parser_init_by_parser( &po1, &po );

		if ( actual_checksum != header_checksum ) // we have not received even a header -- total garbage received
			return SIOT_MESH_RET_GARBAGE_RECEIVED;

		uint16_t packet_reported_checksum;

		uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
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
					uint16_t num_err = 0; // TODO: ACTUAL SOURCE???
					zepto_parser_free_memory( mem_ack_h );
					siot_mesh_form_ack_packet( mem_ack_h, last_hop, num_err, packet_reported_checksum );
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

void siot_mesh_form_packet_to_santa( MEMORY_HANDLE mem_h, uint8_t mesh_val, uint16_t target_id )
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

void siot_mesh_form_unicast_packet( MEMORY_HANDLE mem_h, uint16_t link_id, uint16_t target_id, bool request_ack, uint16_t* packet_checksum )
{
	//  | SAMP-UNICAST-DATA-PACKET-FLAGS-AND-TTL | OPTIONAL-EXTRA-HEADERS | NEXT-HOP | LAST-HOP | Non-Root-Address | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |

/*	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // header
	parser_obj po_start, po_end;
	zepto_parser_init( &po_start, mem_h );
	zepto_parser_init( &po_end, mem_h );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po_start ) );
	zepto_append_part_of_request_to_response( mem_h, &po_start, &po_end );*/
	parser_obj po, po1;
	uint16_t header;
	SIOT_MESH_LINK link;
	uint8_t ret_code = siot_mesh_get_link( link_id, &link );
if ( ret_code != SIOT_MESH_RET_OK )
{
	ret_code = ret_code;
}
	ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_OK ); // we do not expect that being here we have an invalid link_id
	ZEPTO_DEBUG_ASSERT( link.LINK_ID == link_id );
	// !!! TODO: it might happen that this packet is not a reply to 'from Santa'; check, which data should be added this case

	// header: bit[0]: 0, bit[1]: GUARANTEED-DELIVERY flag, bit[2]: BACKWARD-GUARANTEED-DELIVERY, bit[3]: EXTRA-HEADERS-PRESENT, bit[4]: DIRECTION-FLAG (is from the Root), bits[5..]: TTL
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
	zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID );

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

uint8_t handler_siot_mesh_timer( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t* link_id )
{
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
				bool route_known = siot_mesh_target_to_link_id( pending_resends[oldest_index].target_id, link_id ) == SIOT_MESH_RET_OK;
				if ( (!is_last) && route_known )
				{
					uint16_t checksum;
					siot_mesh_form_unicast_packet( mem_h, *link_id, pending_resends[oldest_index].target_id, true, &checksum );
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
						if ( siot_mesh_target_to_link_id( 0, link_id ) == SIOT_MESH_RET_OK )
							siot_mesh_remove_link( *link_id );
						siot_mesh_delete_route( 0 );
					}
					siot_mesh_form_packet_to_santa( mem_h, 0xFF, pending_resends[oldest_index].target_id );
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
}

uint8_t handler_siot_mesh_send_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint8_t mesh_val, uint8_t resend_cnt, uint16_t target_id, uint16_t* link_id )
{
	// TODO: what should we do, if the root in the route table, but incoming packet was "from Santa"?
	uint8_t ret_code = siot_mesh_target_to_link_id( target_id, link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		if ( resend_cnt < SIOT_MESH_SUBJECT_FOR_ACK )
		{
			uint16_t checksum;
			siot_mesh_form_unicast_packet( mem_h, *link_id, target_id, false, &checksum );
			ZEPTO_DEBUG_PRINTF_2( "         ############  handler_siot_mesh_send_packet(): known route (resend count: %d)  ###########\n", resend_cnt );
			return SIOT_MESH_RET_OK;
		}

#ifdef USED_AS_RETRANSMITTER
#if 0
		// we're to start our own resend sequence
		uint8_t i;
		uint8_t free_slot = 0xFF;
		for ( i=0; i<PENDING_RESENDS_MAX; i++ )
			if ( pending_resends[i].resend_cnt == 0 ) // an empty slot
			{
				free_slot = i;
				break;
			}
		ZEPTO_DEBUG_ASSERT( free_slot != 0xFF );	// TODO: if a higher level protocol (say, GDP) sends a new request before our resend sequence is over, we may have no empty slots
													//       practically this can be resolved in two ways: (1) by timing that guarantees that our secuence is over before that, and
													//                                                     (2) by supplying some kind of source ID (say, APP or CTR), and addressing somehow a case with the same source ID as already processed;
													//                                                     (3) anything else?
		bool none_used = true;
		for ( i=0; i<PENDING_RESENDS_MAX; i++ )
			if ( pending_resends[i].resend_cnt )
				none_used = false;
		uint8_t id;
		if ( none_used )
			id = 0;
		else
		{
		}
		for ( i=0; i<PENDING_RESENDS_MAX; i++ )
			if ( pending_resends[i].resend_cnt == 0 ) // an empty slot
			{
				slot_found = true;
				ZEPTO_DEBUG_ASSERT( memory_object_get_request_size( pending_resends[i].packet_h ) == 0 );
				ZEPTO_DEBUG_ASSERT( memory_object_get_response_size( pending_resends[i].packet_h ) == 0 );
				zepto_copy_request_to_response_of_another_handle( mem_h, pending_resends[i].packet_h ); // create our own copy
				pending_resends[i].resend_cnt = SIOT_MESH_SUBJECT_FOR_MESH_RESEND;
				sa_time_val diff_tval;
				TIME_MILLISECONDS16_TO_TIMEVAL( MESH_RESEND_PERIOD_MS, diff_tval );
				sa_hal_time_val_copy_from_if_src_less( &(wf->wait_time), &diff_tval );
				sa_hal_time_val_copy_from( &(pending_resends[i].next_resend_time), currt );
				SA_TIME_INCREMENT_BY_TICKS( pending_resends[i].next_resend_time, diff_tval );

				siot_mesh_form_unicast_packet( mem_h, *link_id, target_id, true );
				ZEPTO_DEBUG_PRINTF_1( "         ############  handler_siot_mesh_send_packet(): yet known route, requesting ACK  ###########\n" );
				return SIOT_MESH_RET_OK;
			}
#endif
#else // USED_AS_RETRANSMITTER
		uint16_t checksum;
		siot_mesh_form_unicast_packet( mem_h, *link_id, target_id, true, &checksum );
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
		pending_resends[free_slot].target_id = (uint8_t)target_id;

		uint8_t i;
		for ( i=0; i<PENDING_RESENDS_MAX; i++ )
			if ( pending_resends[i].resend_cnt ) // used slot
				sa_hal_time_val_get_remaining_time( currt, &(pending_resends[i].next_resend_time), &(wf->wait_time) );
		ZEPTO_DEBUG_PRINTF_3( "         ############  handler_siot_mesh_send_packet(): time to next event: %d, %d  ###########\n", wf->wait_time.low_t, wf->wait_time.high_t );

		return SIOT_MESH_RET_OK;
#endif // USED_AS_RETRANSMITTER

		// NOTE: subsequent retransmits initiated by MESH will be done by timer; as well as falling to sending 'to_santa' packet in the ultimate case
/*		else if ( resend_cnt < SIOT_MESH_SUBJECT_FOR_RECORD_DROP )
		{
			siot_mesh_form_unicast_packet( mem_h, *link_id, target_id );
			ZEPTO_DEBUG_PRINTF_1( "         ############  handler_siot_mesh_send_packet(): known route; ACK would be requested  ###########\n" );
		}
		else
		{
			ZEPTO_DEBUG_PRINTF_1( "         ############  handler_siot_mesh_send_packet(): route appears to be lost  ###########\n" );
			siot_mesh_form_packet_to_santa( mem_h, mesh_val, target_id );
		}*/
	}
	else
	{
		ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );

		// TODO: what link to use????
		siot_mesh_form_packet_to_santa( mem_h, mesh_val, target_id );
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
				// | ADD-OR-MODIFY-LINK-ENTRY-AND-LINK-ID | BUS-ID | NEXT-HOP-ACKS-AND-INTRA-BUS-ID-PLUS-1 | OPTIONAL-LINK-DELAY-UNIT | OPTIONAL-LINK-DELAY | OPTIONAL-LINK-DELAY-ERROR |
				bool link_delay_present = ( entry_header >> 3 ) & 1;// bit[3] being LINK-DELAY-PRESENT flag
				link.LINK_ID = entry_header >> 4; // bits[4..] equal to LINK-ID
				link.BUS_ID = zepto_parse_encoded_uint16( po );
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