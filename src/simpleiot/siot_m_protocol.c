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
	uint8_t conn_quality;
} SIOT_MESH_LAST_HOP_DATA;

#ifdef USED_AS_MASTER

// NOTE: On a ROOT (MASTER) side any implementation is inevitably quite complicated as
// (1) ROOT has to operate with many devices, and
// (2) ROOT implements almost all "decision-making" logic.
// On the other hand, ROOT is expected to run on a reasonably powerfull platforms.
// This results in that implementation at ROOT may not necessarily satisfy limitations of that for DEVICEs.
// To avoid confusions we put all necessary processing in a separate file
// which should NOT be taken as a code sample for embadded programming.

// We will declare all necessary interface calls below.
#define SIOT_MESH_LAST_HOP_DATA_MAX 4

uint16_t zepto_parser_calculate_checksum_of_part_of_response( MEMORY_HANDLE mem_h, uint16_t offset, uint16_t sz, uint16_t accum_val )
{
	return 0;
}
extern uint8_t siot_mesh_at_root_target_to_link_id( uint16_t target_id, uint16_t* link_id );
uint16_t zepto_parser_calculate_checksum_of_part_of_request( MEMORY_HANDLE mem_h, parser_obj* po_start, uint16_t sz, uint16_t accum_val )
{
	return 0;
}

#else

#define SIOT_MESH_LINK_TABLE_SIZE_MAX 256
#define SIOT_MESH_ROUTE_TABLE_SIZE_MAX 256

uint8_t siot_mesh_link_table_size;
static SIOT_MESH_LINK siot_mesh_link_table[ SIOT_MESH_LINK_TABLE_SIZE_MAX ];
uint8_t siot_mesh_route_table_size;
static SIOT_MESH_ROUTE siot_mesh_route_table[ SIOT_MESH_ROUTE_TABLE_SIZE_MAX ];
static SIOT_MESH_RETRANSM_COMMON_DATA siot_mesh_retransm_common_data;

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

void siot_mesh_add_last_hop_data( uint8_t slot_id, uint16_t last_hop_id, uint8_t conn_q )
{
	ZEPTO_DEBUG_ASSERT( slot_id < 2 );
	SIOT_MESH_LAST_HOP_DATA* last_hops = last_requests[slot_id].hop_list;
	// we keep this array sorted in order of packets (with the same request id
	if ( last_requests[slot_id].hop_list_sz < SIOT_MESH_LAST_HOP_DATA_MAX )
	{
		last_hops[last_requests[slot_id].hop_list_sz].last_hop_id = last_hop_id;
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
			last_hops[ SIOT_MESH_LAST_HOP_DATA_MAX - 1 ].conn_quality = conn_q;
		}
	}
}

void siot_mesh_init_last_hop_data_storage( uint8_t slot_id, uint16_t request_id, uint16_t last_hop_id, uint8_t conn_q )
{
	ZEPTO_DEBUG_ASSERT( slot_id < 2 );
	last_requests[slot_id].hop_list_sz = 0;
	last_requests[slot_id].ineffect = true;
	last_requests[slot_id].rq_id = request_id;
	siot_mesh_add_last_hop_data( slot_id, last_hop_id, conn_q );
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
		zepto_parser_encode_and_append_uint8( mem_h, last_hops[ i ].conn_quality );
	}
	ZEPTO_DEBUG_ASSERT( i == last_requests[slot_id].hop_list_sz - 1 ); 
	uint16_t header = ( no_more_headers ? 0: 1 ) | ( SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP << 1 ) | ( last_hops[ i ].last_hop_id << 4 );
	zepto_parser_encode_and_append_uint16( mem_h, header );
	zepto_parser_encode_and_append_uint8( mem_h, last_hops[ i ].conn_quality );
}

void siot_mesh_init_at_life_start()
{
	siot_mesh_link_table_size = 0;
	siot_mesh_route_table_size = 0;
}

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

#endif // !defined USED_AS_MASTER


////////////////////////////  ROUTE and LINK table processing   //////////////////////////////////

#ifdef USED_AS_MASTER

#define SIOT_MESH_IS_QUALITY_OF_INCOMING_CONNECTION_ADMISSIBLE( x ) ( (x) < 0x7F )
#define SIOT_MESH_IS_QUALITY_OF_OUTGOING_CONNECTION_ADMISSIBLE( x ) ( (x) < 0x7F )
#define SIOT_MESH_IS_QUALITY_OF_FIRST_INOUT_CONNECTION_BETTER( in1, out1, in2, out2 ) ( (in1<in2)||((in1==in2)&&(out1<out2)) ) /*TODO: this is a quick solution; think about better ones*/

bool siot_mesh_find_best_route( const SIOT_MESH_LAST_HOP_DATA* hops_in, uint16_t hops_in_sz, const SIOT_MESH_LAST_HOP_DATA* hops_out, uint16_t hops_out_sz, uint16_t* id_from, uint16_t* id_to )
{
	uint16_t i, j;
	uint16_t last_in, last_out;
	uint16_t match_cnt = 0;
	if ( hops_in_sz == 0 || hops_out_sz == 0 )
		return false;

	// 1. find matches (within c transmissions of admissible_quality
	for ( i=0; i<hops_in_sz; i++ )
		if ( SIOT_MESH_IS_QUALITY_OF_INCOMING_CONNECTION_ADMISSIBLE( (hops_in + i)->conn_quality ) )
			for ( j=0; j<hops_out_sz; j++ )
				if ( ( hops_in[i].last_hop_id == hops_out[i].last_hop_id ) && SIOT_MESH_IS_QUALITY_OF_OUTGOING_CONNECTION_ADMISSIBLE( (hops_out + i)->conn_quality ) )
					if ( match_cnt == 0 || SIOT_MESH_IS_QUALITY_OF_FIRST_INOUT_CONNECTION_BETTER( hops_in[i].conn_quality, hops_out[j].conn_quality, hops_in[last_in].conn_quality, hops_out[last_out].conn_quality ) )
					{
						last_in = i;
						last_out = j;
						*id_from = hops_in[i].last_hop_id;
						*id_to = hops_out[i].last_hop_id;
						match_cnt++;
					}
	if ( match_cnt != 0 )
		return true;

	return false;
}

void siot_mesh_form_packet_from_santa( MEMORY_HANDLE mem_h, uint8_t target_id )
{
	// Santa Packet structure: | SAMP-FROM-SANTA-DATA-PACKET-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | REQUEST-ID | OPTIONAL-DELAY-UNIT | MULTIPLE-RETRANSMITTING-ADDRESSES | BROADCAST-BUS-TYPE-LIST | Target-Address | OPTIONAL-TARGET-REPLY-DELAY | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
	// TODO: here and then use bit-field processing instead

	parser_obj po, po1;

	// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, OPTIONAL-EXTRA-HEADERS
	uint16_t header = 1 | ( SIOT_MESH_FROM_SANTA_DATA_PACKET << 1 ) | ( 4 << 5 ); // '1', packet type, 0 (no extra headers), TTL = 4
	zepto_parser_encode_and_append_uint16( mem_h, header );

	// LAST-HOP
	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // TODO: is it a self-ID? Then for Client it is 0, and some unique velue for other devices. SOURCE???

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

uint8_t handler_siot_mesh_send_packet( MEMORY_HANDLE mem_h, uint8_t target_id )
{
	uint16_t link_id;
	uint8_t ret_code = siot_mesh_at_root_target_to_link_id( target_id, &link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		// prepare a message for sending according to link_id received
		ZEPTO_DEBUG_ASSERT( NULL == "Error: not implemented\n" );
		return SIOT_MESH_RET_OK;
	}
	ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );

	// packet "from Santa" will be sent... let's form a packet
	siot_mesh_form_packet_from_santa( mem_h, target_id );


	return SIOT_MESH_RET_OK;
}

uint8_t siot_mesh_process_received_tosanta_packet( MEMORY_HANDLE mem_h, uint16_t src_id, bool payload_present, uint16_t request_id )
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

	SIOT_MESH_LAST_HOP_DATA last_hops[ SIOT_MESH_LAST_HOP_DATA_MAX ];
	uint8_t last_rq_hops_cnt = 0;

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
				last_hops[ last_rq_hops_cnt ].last_hop_id = header >> 4;
				last_hops[ last_rq_hops_cnt ].conn_quality = zepto_parse_encoded_uint8( &po );
				last_rq_hops_cnt++;
				ZEPTO_DEBUG_ASSERT( last_rq_hops_cnt <= SIOT_MESH_LAST_HOP_DATA_MAX ); // TODO: here at ROOT we may allow longer lists (with a bit different implementation)
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

	// REQUEST-ID
#ifdef SA_DEBUG
	ZEPTO_DEBUG_ASSERT( request_id == zepto_parse_encoded_uint16( &po ) ); // must coincide
#else // SA_DEBUG
	zepto_parse_encoded_uint16( &po ); // just skip
#endif // SA_DEBUG

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

uint8_t handler_siot_mesh_receive_packet( MEMORY_HANDLE mem_h )
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
			case SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET:
			{
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
							zepto_parse_encoded_uint8( &po ); // just skip
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
				uint16_t src_id = zepto_parse_encoded_uint16( &po );

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
						// TODO: we're done
						siot_mesh_process_received_tosanta_packet( mem_h, src_id, remaining_size > 2, request_id );
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
		ZEPTO_DEBUG_PRINTF_3( "Error: unexpected packet type %d,%d received\n", header& 1, (header >> 1) & 0xF );
		ZEPTO_DEBUG_ASSERT( NULL == "Error: processing of mesh packets of ANY type is not implemented\n" );
		return SIOT_MESH_RET_ERROR_ANY; // just as long as not implemented to make compiler happy
	}
	return SIOT_MESH_RET_OK;
}

#else // USED_AS_MASTER

uint8_t handler_siot_mesh_receive_packet( MEMORY_HANDLE mem_h, uint8_t* mesh_val, uint8_t signal_level, uint8_t error_cnt )
{
	parser_obj po, po1, po2;
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	uint16_t total_packet_sz = zepto_parsing_remaining_bytes( &po );

	*mesh_val = 0xFF; // valid values make sense only in certain cases and will be loaded then

	uint16_t header = zepto_parse_encoded_uint16( &po );
	// TODO: here and then use bit-field processing instead
	if ( header & 1 ) // predefined packet types
	{
		uint8_t packet_type = ( header >> 1 ) & 0x7;
		switch ( packet_type )
		{
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
				// TODO: compare with self-id
				if ( target_id != DEVICE_SELF_ID )
					return SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED;

				// OPTIONAL-TARGET-REPLY-DELAY

				// OPTIONAL-PAYLOAD-SIZE

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

				bool second_checksum_ok;
				uint16_t remaining_size = zepto_parsing_remaining_bytes( &po );
				if ( remaining_size >= 2 )
				{
					parser_obj rq_start_po;
					zepto_parser_init_by_parser( &po2, &po );
					zepto_parser_init_by_parser( &rq_start_po, &po );
					zepto_parse_skip_block( &po, remaining_size - 2 );
					zepto_append_part_of_request_to_response( mem_h, &po2, &po );
					actual_checksum = zepto_parser_calculate_checksum_of_part_of_request( mem_h, &rq_start_po, remaining_size - 2, actual_checksum );
					checksum = zepto_parse_uint8( &po );
					checksum |= ((uint16_t)zepto_parse_uint8( &po )) << 8;
					second_checksum_ok = actual_checksum == checksum;
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
					siot_mesh_add_last_hop_data( 0, last_hop_id, signal_level | ( error_cnt << 4 ) );
					*mesh_val = 0;
				}
				else if ( last_requests[1].ineffect && request_id == last_requests[1].rq_id )
				{
					one_of_recent_requests = true;
					siot_mesh_add_last_hop_data( 1, last_hop_id, signal_level | ( error_cnt << 4 ) );
					*mesh_val = 1;
				}
				else
				{
					ZEPTO_DEBUG_ASSERT( last_requests[0].ineffect == false || last_requests[1].ineffect == false );
					*mesh_val = 0;
					if ( last_requests[0].ineffect )
						*mesh_val = 1;
					siot_mesh_init_last_hop_data_storage( *mesh_val, request_id, last_hop_id, signal_level | ( error_cnt << 4 ) );
				}

				return SIOT_MESH_RET_PASS_TO_PROCESS;
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
		ZEPTO_DEBUG_PRINTF_3( "Error: unexpected packet type %d,%d received\n", header& 1, (header >> 1) & 0xF );
		ZEPTO_DEBUG_ASSERT( NULL == "Error: processing of mesh packets of ANY type is not implemented\n" );
		return SIOT_MESH_RET_ERROR_ANY; // just as long as not implemented to make compiler happy
	}
	return SIOT_MESH_RET_OK;
}

void siot_mesh_form_packet_to_santa( MEMORY_HANDLE mem_h, uint8_t* mesh_val, uint8_t target_id )
{
	// Santa Packet structure: | SAMP-TO-SANTA-DATA-OR-ERROR-PACKET-NO-TTL | OPTIONAL-EXTRA-HEADERS| SOURCE-ID | REQUEST-ID | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
	// TODO: here and then use bit-field processing instead

	parser_obj po, po1;
	uint16_t header;

	// !!! TODO: it might happen that this packet is not a reply to 'from Santa'; check, which data should be added this case

	uint16_t request_id = 0;
	if ( *mesh_val < 2 )
	{
		// SAMP-FROM-SANTA-DATA-PACKET-AND-TTL, OPTIONAL-EXTRA-HEADERS
		header = 1 | ( SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET << 1 ) | ( 1 << 4 ); // '1', packet type, 1 (at least one extra header: hop list item), rezerved (zeros)
		zepto_parser_encode_and_append_uint16( mem_h, header );
		ZEPTO_DEBUG_ASSERT( *mesh_val < 2 );
		siot_mesh_write_last_hop_data_as_opt_headers( *mesh_val, mem_h, true, &request_id );
	}
	else
	{
		header = 1 | ( SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET << 1 ) | ( 0 << 4 ); // '1', packet type, 0 (noe extra headers), rezerved (zeros)
		zepto_parser_encode_and_append_uint16( mem_h, header );
	}

	// SOURCE-ID
	zepto_parser_encode_and_append_uint16( mem_h, DEVICE_SELF_ID );

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

uint8_t handler_siot_mesh_send_packet( MEMORY_HANDLE mem_h, uint8_t* mesh_val, uint8_t target_id )
{
/*	parser_obj po_start, po_end;
	zepto_parser_init( &po_start, mem_h );
	zepto_parser_init( &po_end, mem_h );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po_start ) );
	zepto_convert_part_of_request_to_response( mem_h, &po_start, &po_end );
	return SIOT_MESH_RET_OK;*/

	// let's start implementing our response :)
	uint16_t link_id;
	uint8_t ret_code = siot_mesh_target_to_link_id( target_id, &link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		// prepare a message for sending according to link_id received
		// TODO: what should we do, if the root in the route table, but incoming packet was "from Santa"?
		ZEPTO_DEBUG_ASSERT( NULL == "Error: not implemented\n" );
		return SIOT_MESH_RET_OK;
	}
	ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );

	// packet "from Santa" will be sent... let's form a packet
	siot_mesh_form_packet_to_santa( mem_h, mesh_val, target_id );


	return SIOT_MESH_RET_OK;
}

uint8_t handler_siot_mesh_packet_rejected_broken( /*MEMORY_HANDLE mem_h, */uint8_t* mesh_val )
{
	ZEPTO_DEBUG_ASSERT( *mesh_val < 2 );
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
				link.INTRA_BUS_ID = ibid_pl_1 >> 1; // bits[1..] representing INTRA-BUS-ID-PLUS-1
				if ( link.INTRA_BUS_ID == 0 ) // means that INTRA-BUS-ID==NULL, and therefore that the link entry is an incoming link entry
				{
					ZEPTO_DEBUG_ASSERT( 0 == "INTRA-BUS-ID==NULL is not yet implemented" );
					return;
				}
				else
					(link.INTRA_BUS_ID)--; // TODO: distinguishing NULL and 0
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