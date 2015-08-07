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
#include "../simpleiot_hal/siot_mem_mngmt.h"


#define SIOT_MESH_LINK_TABLE_SIZE_MAX 256
#define SIOT_MESH_ROUTE_TABLE_SIZE_MAX 256

uint8_t siot_mesh_link_table_size;
static SIOT_MESH_LINK siot_mesh_link_table[ SIOT_MESH_LINK_TABLE_SIZE_MAX ];
uint8_t siot_mesh_route_table_size;
static SIOT_MESH_ROUTE siot_mesh_route_table[ SIOT_MESH_ROUTE_TABLE_SIZE_MAX ];
static SIOT_MESH_RETRANSM_COMMON_DATA siot_mesh_retransm_common_data;


void siot_mesh_init_at_life_start()
{
	siot_mesh_link_table_size = 0;
	siot_mesh_route_table_size = 0;
}

uint8_t siot_mesh_get_link_id( uint8_t target_id, uint8_t* link_id )
{
	uint8_t i;
	for ( i=0; i<siot_mesh_route_table_size; i++ )
		if ( siot_mesh_route_table[i].TARGET_ID == target_id )
		{
			*link_id = siot_mesh_route_table[i].LINK_ID;
			return SIOT_MESH_RET_OK;
		}
	return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_delete_link_id( uint8_t target_id )
{
	uint8_t i;
	for ( i=0; i<siot_mesh_route_table_size; i++ )
		if ( siot_mesh_route_table[i].TARGET_ID == target_id )
		{
			ZEPTO_MEMMOV( siot_mesh_route_table + i, siot_mesh_route_table + i + 1, sizeof(SIOT_MESH_ROUTE) * ( siot_mesh_route_table_size - i - 1 ) );
			siot_mesh_route_table_size--;
			return SIOT_MESH_RET_OK;
		}
	return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_add_link_id( uint8_t target_id, uint8_t link_id )
{
	uint8_t i;
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

	return SIOT_MESH_RET_ERROR_NOT_FOUND;
}

uint8_t siot_mesh_receive_packet( MEMORY_HANDLE mem_h )
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
			{
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
						case SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS:
						case SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN:
						case SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK:
						case SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP:
						{
							ZEPTO_DEBUG_ASSERT( NULL == "Error: not implemented\n" );
							break;
						}
					}
				}
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

uint8_t siot_mesh_send_packet( MEMORY_HANDLE mem_h, uint8_t target_id )
{
	parser_obj po, po1;
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );
	zepto_parse_skip_block( &po1, zepto_parsing_remaining_bytes( &po ) );
	zepto_convert_part_of_request_to_response( mem_h, &po, &po1 );
	uint8_t link_id;
	uint8_t ret_code = siot_mesh_get_link_id( target_id, &link_id );
	if ( ret_code == SIOT_MESH_RET_OK )
	{
		// prepare a message for sending according to link_id received
		ZEPTO_DEBUG_ASSERT( NULL == "Error: not implemented\n" );
		return SIOT_MESH_RET_OK;
	}
	ZEPTO_DEBUG_ASSERT( ret_code == SIOT_MESH_RET_ERROR_NOT_FOUND );

	// packet "from Santa" will be sent... let's form a packet
	// Santa Packet structure: | SAMP-FROM-SANTA-DATA-PACKET-AND-TTL | OPTIONAL-EXTRA-HEADERS | LAST-HOP | REQUEST-ID | OPTIONAL-DELAY-UNIT | MULTIPLE-RETRANSMITTING-ADDRESSES | BROADCAST-BUS-TYPE-LIST | Target-Address | OPTIONAL-TARGET-REPLY-DELAY | OPTIONAL-PAYLOAD-SIZE | HEADER-CHECKSUM | PAYLOAD | FULL-CHECKSUM |
	// TODO: here and then use bit-field processing instead
	uint16_t header = 1 | ( SIOT_MESH_FROM_SANTA_DATA_PACKET << 1 ) | ( 4 << 5 ); // '1', packet type, 0 (no extra headers), TTL = 4
	zepto_parser_encode_and_append_uint16( mem_h, header );
	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // LAST-HOP
	// WARNING!!! REQUEST-ID is a 4-byte value; when actually implemented, switch to 4-byte immediately
//	uint32_t rq_id = 0; // REQUEST-ID
//	zepto_parser_encode_and_append_uint( mem_h, &rq_id, 4 );
	zepto_parser_encode_and_append_uint16( mem_h, 0 ); // REQUEST-ID
	// OPTIONAL-DELAY-UNIT is present only if EXPLICIT-TIME-SCHEDULING flag is present; currently we did not added it

	// MULTIPLE-RETRANSMITTING-ADDRESSES 

	return SIOT_MESH_RET_OK;
}