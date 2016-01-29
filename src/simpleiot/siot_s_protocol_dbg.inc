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


/*******************************************************************************
  CONTENT OF THIS FILE IS INTENDED FOR DEBUG PURPOSES ONLY
  and implements a highly simplified siot/sp.
  
  In short, packet processing is reduced to either not checking packet integrity
  at all (VERY_DEBUG_SIOT_SP_NONE), or to checking only packet checksum
  (VERY_DEBUG_SIOT_SP_CHECKSUM); in all cases packet authenticity is not checked.
  Still, formally, implementation complies with its contract with "external world",
  at least, in part of manipulating with packet id (nonce).
  
  Main purpose is to quickly process a packet causing minimal influence on the
  sysetm (in particular, no eeprom access is required, no heavy cryptography is
  used, etc)
*******************************************************************************/

#if !defined VERY_DEBUG
#error This file is intended for DEBUG PURPOSES ONLY and cannot be used when VERY_DEBUG is not defined
#else // VERY_DEBUG
#if (!defined VERY_DEBUG_SIOT_SP_CHECKSUM) && (!defined VERY_DEBUG_SIOT_SP_NONE)
#error This file is intended for DEBUG PURPOSES ONLY and can be used when one of VERY_DEBUG_SIOT_SP_XXX is defined
#else // VERY_DEBUG_SIOT_SP_XXX


#include "siot_s_protocol.h"
#include "../simpleiot_hal/siot_mem_mngmt.h"

static 	SASP_DATA sasp_data;

void sasp_init_eeprom_data_at_lifestart( /*SASP_DATA* sasp_data*/ )
{
	sa_uint48_set_zero( sasp_data.nonce_lw );
	sa_uint48_set_zero( sasp_data.nonce_ls );
	sa_uint48_increment( sasp_data.nonce_ls );
}

void sasp_restore_from_backup( /*SASP_DATA* sasp_data*/ )
{
	sa_uint48_set_zero( sasp_data.nonce_lw );
	sa_uint48_set_zero( sasp_data.nonce_ls );
	sa_uint48_increment( sasp_data.nonce_ls );
}

void handler_sasp_save_state( /*SASP_DATA* sasp_data*/ )
{
}

void SASP_increment_nonce_last_sent( /*SASP_DATA* sasp_data*/ )
{
	sa_uint48_increment( sasp_data.nonce_ls );
}

uint8_t handler_sasp_get_packet_id( sa_uint48_t buffOut/*, SASP_DATA* sasp_data*/ )
{
	SASP_increment_nonce_last_sent( /*sasp_data*/ );
	sa_uint48_init_by( buffOut, sasp_data.nonce_ls );
	return SASP_RET_NONCE;
}

void SASP_update_nonce_low_watermark( /*SASP_DATA* sasp_data, */const sasp_nonce_type new_val )
{
	sa_uint48_init_by( sasp_data.nonce_lw, new_val ); // update Nonce Low Watermark
}


#ifdef VERY_DEBUG_SIOT_SP_NONE ////////////////// Integrity is not checked; packet IDs are checked

INLINE
void SASP_EncryptAndAddAuthenticationData( REQUEST_REPLY_HANDLE mem_h, const uint8_t* key, const sa_uint48_t packet_id )
{
	// in this DEBUG-only version we just prepend packet_id to the packet

	parser_obj po_start, po_end;
	zepto_parser_init( &po_start, mem_h );
	zepto_parser_init( &po_end, mem_h );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po_start ) );
	zepto_convert_part_of_request_to_response( mem_h, &po_start, &po_end );
	zepto_parser_encode_and_prepend_uint( mem_h, packet_id, SASP_NONCE_SIZE );
}

void SASP_IntraPacketAuthenticateAndDecrypt( const uint8_t* key, REQUEST_REPLY_HANDLE mem_h, const sa_uint48_t last_received_pid, sa_uint48_t received_pid )
{
	// in this DEBUG-only version we just read received_pid (no further checks)

	parser_obj po, po_end;
	zepto_parser_init( &po, mem_h );
	zepto_parser_decode_encoded_uint_as_sa_uint48( &po, received_pid );
	zepto_parser_init_by_parser( &po_end, &po );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po ) );
	zepto_convert_part_of_request_to_response( mem_h, &po, &po_end );
}


bool SASP_is_for_sasp( REQUEST_REPLY_HANDLE mem_h )
{
	parser_obj po;
	zepto_response_to_request( mem_h );
	zepto_parser_init( &po, mem_h );
	uint8_t first_byte = zepto_parse_uint8( &po );
	bool for_sasp = first_byte & 0x80; // TODO: use bit field processing instead

	// NOTE: we do not need to clear 'is-for-sasp' flag since, if set, it will be processed by sasp handler itself (no confusion on upper levels)

	zepto_parser_init( &po, mem_h );
	parser_obj po1;
	zepto_parser_init_by_parser( &po1, &po );
	zepto_parse_skip_block( &po1, zepto_parsing_remaining_bytes( &po ) );
	zepto_convert_part_of_request_to_response( mem_h, &po, &po1 );

	return for_sasp;
}

uint8_t handler_sasp_send( const uint8_t* key, const sa_uint48_t packet_id, MEMORY_HANDLE mem_h/*, SASP_DATA* sasp_data*/ )
{
	ZEPTO_DEBUG_ASSERT( sa_uint48_compare( packet_id, sasp_data.nonce_ls ) >= 0 );
	SASP_EncryptAndAddAuthenticationData( mem_h, key, packet_id );
	return SASP_RET_TO_LOWER_REGULAR;
}

uint8_t handler_sasp_receive( const uint8_t* key, uint8_t* pid, MEMORY_HANDLE mem_h/*, SASP_DATA* sasp_data*/ )
{
	INCREMENT_COUNTER( 11, "handler_sasp_receive()" );
	// 1. Perform intra-packet authentication
	parser_obj po;
	zepto_parser_init( &po, mem_h );
	uint16_t packet_size = zepto_parsing_remaining_bytes( &po );
	if ( packet_size == 0 )
	{
		return SASP_RET_IGNORE_PACKET_BROKEN;
	}

	SASP_IntraPacketAuthenticateAndDecrypt( key, mem_h, sasp_data.nonce_lw, pid );
	ZEPTO_DEBUG_PRINTF_7( "handler_sasp_receive(): PID: %02x %02x %02x %02x %02x %02x\n", pid[0], pid[1], pid[2], pid[3], pid[4], pid[5] );

	bool for_sasp = SASP_is_for_sasp(  mem_h );

	// 2. Is it an Error Old Nonce Message
	if ( for_sasp )
	{
		zepto_response_to_request( mem_h ); // since now it's we who are the recipient of this packet
		INCREMENT_COUNTER( 13, "handler_sasp_receive(), intended for SASP" );

		parser_obj po1;
		zepto_parser_init( &po1, mem_h );
		zepto_parse_skip_block( &po1, 2 );
		sa_uint48_t new_nls;
		zepto_parser_decode_encoded_uint_as_sa_uint48( &po1, new_nls );

		uint8_t* nls = sasp_data.nonce_ls;
#ifdef SA_DEBUG
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): packet for SASP received...\n" );
		ZEPTO_DEBUG_PRINTF_7( "   current  NLS: %02x %02x %02x %02x %02x %02x\n", nls[0], nls[1], nls[2], nls[3], nls[4], nls[5] );
		ZEPTO_DEBUG_PRINTF_7( "   proposed NLS: %02x %02x %02x %02x %02x %02x\n", new_nls[0], new_nls[1], new_nls[2], new_nls[3], new_nls[4], new_nls[5] );
#endif

		// Recommended value of NLS starts from the second byte of the message; we should update NLS, if the recommended value is greater
		int8_t nonceCmp = sa_uint48_compare( sasp_data.nonce_ls, new_nls ); // skipping First Byte and reserved byte
		if ( nonceCmp < 0 )
		{
			INCREMENT_COUNTER( 14, "handler_sasp_receive(), nonce las updated" );
			ZEPTO_MEMCPY( nls, new_nls, SASP_NONCE_SIZE );
			SASP_increment_nonce_last_sent( /*sasp_data*/ );
			// TODO: shuold we do anything else?
			return SASP_RET_TO_HIGHER_LAST_SEND_FAILED;
		}
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(), for SASP, error old nonce, not applied\n" );
		return SASP_RET_IGNORE_PACKET_NONCE_LS_NOT_APPLIED;
	}

	// 3. Compare nonces...
	int8_t nonceCmp = sa_uint48_compare( pid, sasp_data.nonce_lw );
	if ( nonceCmp == 0 )
	{
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): PACKET WITH THE SAME NONCE RECEIVED\n" );
		return SASP_RET_IGNORE_PACKET_LAST_REPEATED; // TODO: more detailed code might be necessary
	}
	if ( nonceCmp < 0 ) // error message must be prepared
	{
#ifdef SA_DEBUG
		INCREMENT_COUNTER( 15, "handler_sasp_receive(), error old nonce" );
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): old nonce; packet for SASP is being prepared...\n" );
		ZEPTO_DEBUG_PRINTF_7( "   nonce received: %02x %02x %02x %02x %02x %02x\n", pid[0], pid[1], pid[2], pid[3], pid[4], pid[5] );
		uint8_t* nlw = sasp_data.nonce_lw;
		ZEPTO_DEBUG_PRINTF_7( "   current    NLW: %02x %02x %02x %02x %02x %02x\n", nlw[0], nlw[1], nlw[2], nlw[3], nlw[4], nlw[5] );
#endif

		zepto_response_to_request( mem_h ); // we have to create a new response (with error old nonce)
		// we are to create a brand new output
		zepto_write_uint8( mem_h, 0x80 ); // First Byte; TODO: use bit field processing instead (should we?)
		zepto_write_uint8( mem_h, 0 ); // Reserved Byte
		zepto_parser_encode_and_append_uint( mem_h, sasp_data.nonce_lw, SASP_NONCE_SIZE );
		zepto_response_to_request( mem_h );

		sa_uint48_t ne;
		handler_sasp_get_packet_id( ne );
		SASP_EncryptAndAddAuthenticationData( mem_h, key, ne );
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): ------------------- ERROR OLD NONCE WILL BE SENT ----------------------\n" );
		return SASP_RET_TO_LOWER_ERROR;
	}

	// 4. Finally, we have a brand new packet
	SASP_update_nonce_low_watermark( /*sasp_data, */pid );
	INCREMENT_COUNTER( 16, "handler_sasp_receive(), passed up" );
	return SASP_RET_TO_HIGHER_NEW;
}



#elif defined VERY_DEBUG_SIOT_SP_CHECKSUM ////////////// integrity is checked using light checksum (just for self-confidence)


static void update_fletcher_checksum_16( uint8_t bt, uint16_t* state )
{
	// quick and dirty solution
	// TODO: implement
	uint8_t tmp = (uint8_t)(*state);
	uint8_t tmp1 = (*state) >> 8;
	tmp += bt;
	if ( tmp < bt )
		tmp += 1;
	if ( tmp == 0xFF )
		tmp = 0;
	tmp1 += tmp;
	if ( tmp1 < tmp )
		tmp1 += 1;
	if ( tmp1 == 0xFF )
		tmp1 = 0;
	*state = tmp1;
	*state <<= 8;
	*state += tmp;
}

INLINE
void SASP_EncryptAndAddAuthenticationData( REQUEST_REPLY_HANDLE mem_h, const uint8_t* key, const sa_uint48_t packet_id )
{
	// in this DEBUG-only version we just prepend packet_id and checksum to the packet
	// so that data in packet is as follows: | packet_id | payload | checksum (2 bytes covering the rest of the packet) |

	parser_obj po_start, po_end;
	zepto_parser_init( &po_start, mem_h );
	zepto_parser_init( &po_end, mem_h );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po_start ) );
	zepto_parser_encode_and_append_uint( mem_h, packet_id, SASP_NONCE_SIZE );
	zepto_append_part_of_request_to_response( mem_h, &po_start, &po_end );

	uint16_t sz = memory_object_get_response_size( mem_h );

	uint16_t checksum = 0;
	uint16_t i;
	for ( i=0; i<sz; i++)
		update_fletcher_checksum_16( memory_object_read_response_byte( mem_h, i ), &checksum );

	zepto_write_uint8( mem_h, (uint8_t)checksum );
	zepto_write_uint8( mem_h, (uint8_t)(checksum>>8) );
}

bool SASP_IntraPacketAuthenticateAndDecrypt( const uint8_t* key, REQUEST_REPLY_HANDLE mem_h, const sa_uint48_t last_received_pid, sa_uint48_t received_pid )
{
	// in this DEBUG-only version we test packet checksum
	// we expect a packet as follows: | packet_id | payload | checksum (2 bytes covering the rest of the packet) |

	parser_obj po_start, po_end;

	zepto_parser_init( &po_start, mem_h );
	uint16_t sz = zepto_parsing_remaining_bytes( &po_start );
	if ( sz <= 2 )
		return false;
	sz -= 2;

	uint16_t checksum_calculated = 0;
	uint16_t i;
	for ( i=0; i<sz; i++)
		update_fletcher_checksum_16( zepto_parse_uint8( &po_start ), &checksum_calculated );

	uint16_t checksum_read = zepto_parse_uint8( &po_start );
	checksum_read |= ((uint16_t)zepto_parse_uint8( &po_start )) << 8;

	bool ret = checksum_calculated == checksum_read;
	if ( !ret ) return false;

	zepto_parser_init( &po_start, mem_h );
	zepto_parser_decode_uint( &po_start, received_pid, SASP_NONCE_SIZE );
	zepto_parser_init_by_parser( &po_end, &po_start );
	sz = zepto_parsing_remaining_bytes( &po_start );
	if ( sz <= 2 ) return false;
	zepto_parse_skip_block( &po_end, sz - 2 );
	zepto_convert_part_of_request_to_response( mem_h, &po_start, &po_end );

	return true;
}

bool SASP_is_for_sasp( REQUEST_REPLY_HANDLE mem_h )
{
	parser_obj po;
	zepto_response_to_request( mem_h );
	zepto_parser_init( &po, mem_h );
	uint8_t first_byte = zepto_parse_uint8( &po );
	bool for_sasp = first_byte & 0x80; // TODO: use bit field processing instead

	// NOTE: we do not need to clear 'is-for-sasp' flag since, if set, it will be processed by sasp handler itself (no confusion on upper levels)

	zepto_parser_init( &po, mem_h );
	parser_obj po1;
	zepto_parser_init_by_parser( &po1, &po );
	zepto_parse_skip_block( &po1, zepto_parsing_remaining_bytes( &po ) );
	zepto_convert_part_of_request_to_response( mem_h, &po, &po1 );

	return for_sasp;
}

#ifdef SA_DEBUG
void DEBUG_SASP_EncryptAndAddAuthenticationDataChecked( MEMORY_HANDLE mem_h, const uint8_t* key, const sa_uint48_t nonce )
{
	parser_obj po, po1;
	zepto_parser_init( &po, mem_h );
	uint16_t inisz = zepto_parsing_remaining_bytes( &po );

	ZEPTO_DEBUG_PRINTF_7( "handler_sasp_send(): nonce used: %02x %02x %02x %02x %02x %02x\n", nonce[0], nonce[1], nonce[2], nonce[3], nonce[4], nonce[5] );
	ZEPTO_DEBUG_PRINTF_2( "handler_sasp_send(): size: %d\n", inisz );

	uint16_t encr_sz, decr_sz;
	uint8_t inimsg[1024]; zepto_parse_read_block( &po, inimsg, inisz );
	uint8_t checkedMsg[1024];
	uint8_t dbg_nonce[6];

	// do required job
	SASP_EncryptAndAddAuthenticationData( mem_h, key, nonce );

	// CHECK RESULTS

	// init parser object
	zepto_parser_init( &po, mem_h );
	zepto_parser_init( &po1, mem_h );


	// TODO: if this debug code remains in use, lines below must be replaced by getting a new handle
	uint8_t x_buff1_x2[1024];

	// copy output to input of a new handle and restore
	zepto_response_to_request( mem_h );
	encr_sz = zepto_parsing_remaining_bytes( &po1 );
	decr_sz = encr_sz;
	zepto_parse_read_block( &po1, x_buff1_x2, encr_sz );
	zepto_convert_part_of_request_to_response( mem_h, &po, &po1 ); // restore the picture
	zepto_response_to_request( MEMORY_HANDLE_DBG_TMP );
	zepto_write_block( MEMORY_HANDLE_DBG_TMP, x_buff1_x2, encr_sz );
	zepto_response_to_request( MEMORY_HANDLE_DBG_TMP );

	bool ipaad = SASP_IntraPacketAuthenticateAndDecrypt( key, MEMORY_HANDLE_DBG_TMP, sasp_data.nonce_lw, dbg_nonce );
	ZEPTO_DEBUG_ASSERT( ipaad );
	zepto_response_to_request( MEMORY_HANDLE_DBG_TMP );
	zepto_parser_init( &po, MEMORY_HANDLE_DBG_TMP );
	decr_sz = zepto_parsing_remaining_bytes( &po );
	zepto_parse_read_block( &po, checkedMsg, decr_sz );

	ZEPTO_DEBUG_PRINTF_7( "handler_sasp_send():     nonce: %02x %02x %02x %02x %02x %02x\n", nonce[0], nonce[1], nonce[2], nonce[3], nonce[4], nonce[5] );
	ZEPTO_DEBUG_PRINTF_7( "handler_sasp_send(): dbg_nonce: %02x %02x %02x %02x %02x %02x\n", dbg_nonce[0], dbg_nonce[1], dbg_nonce[2], dbg_nonce[3], dbg_nonce[4], dbg_nonce[5] );
	ZEPTO_DEBUG_ASSERT( ipaad );
	ZEPTO_DEBUG_ASSERT( decr_sz == inisz );
	ZEPTO_DEBUG_ASSERT( ZEPTO_MEMCMP( nonce, dbg_nonce, 6 ) == 0 );
	uint8_t k;
	for ( k=0; k<decr_sz; k++ )
		ZEPTO_DEBUG_ASSERT( inimsg[k] == checkedMsg[k] );
}
#endif

uint8_t handler_sasp_send( const uint8_t* key, const sa_uint48_t packet_id, MEMORY_HANDLE mem_h/*, SASP_DATA* sasp_data*/ )
{
	ZEPTO_DEBUG_ASSERT( sa_uint48_compare( packet_id, sasp_data.nonce_ls ) >= 0 );

#ifdef SA_DEBUG
//	DEBUG_SASP_EncryptAndAddAuthenticationDataChecked( mem_h, key, packet_id );
	SASP_EncryptAndAddAuthenticationData( mem_h, key, packet_id );
#else
	SASP_EncryptAndAddAuthenticationData( mem_h, key, packet_id );
#endif

	INCREMENT_COUNTER( 10, "handler_sasp_send()" );
	return SASP_RET_TO_LOWER_REGULAR;
}

uint8_t handler_sasp_receive( const uint8_t* key, uint8_t* pid, MEMORY_HANDLE mem_h/*, SASP_DATA* sasp_data*/ )
{
	INCREMENT_COUNTER( 11, "handler_sasp_receive()" );
	// 1. Perform intra-packet authentication
	parser_obj po;
	zepto_parser_init( &po, mem_h );
	uint16_t packet_size = zepto_parsing_remaining_bytes( &po );
	if ( packet_size == 0 )
	{
		return SASP_RET_IGNORE_PACKET_BROKEN;
	}

	bool ret_code = SASP_IntraPacketAuthenticateAndDecrypt( key, mem_h, sasp_data.nonce_lw, pid );
	ZEPTO_DEBUG_PRINTF_7( "handler_sasp_receive(): PID: %02x %02x %02x %02x %02x %02x\n", pid[0], pid[1], pid[2], pid[3], pid[4], pid[5] );

	if ( !ret_code )
	{
		INCREMENT_COUNTER( 12, "handler_sasp_receive(), garbage in" );
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): BAD PACKET RECEIVED\n" );
		return SASP_RET_IGNORE_PACKET_BROKEN; // TODO: more detailed code might be necessary
	}

	bool for_sasp = SASP_is_for_sasp(  mem_h );

	// 2. Is it an Error Old Nonce Message
	if ( for_sasp )
	{
		zepto_response_to_request( mem_h ); // since now it's we who are the recipient of this packet
		INCREMENT_COUNTER( 13, "handler_sasp_receive(), intended for SASP" );

		parser_obj po1;
		zepto_parser_init( &po1, mem_h );
		zepto_parse_skip_block( &po1, 2 );
		sa_uint48_t new_nls;
		zepto_parser_decode_encoded_uint_as_sa_uint48( &po1, new_nls );

		uint8_t* nls = sasp_data.nonce_ls;
#ifdef SA_DEBUG
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): packet for SASP received...\n" );
		ZEPTO_DEBUG_PRINTF_7( "   current  NLS: %02x %02x %02x %02x %02x %02x\n", nls[0], nls[1], nls[2], nls[3], nls[4], nls[5] );
		ZEPTO_DEBUG_PRINTF_7( "   proposed NLS: %02x %02x %02x %02x %02x %02x\n", new_nls[0], new_nls[1], new_nls[2], new_nls[3], new_nls[4], new_nls[5] );
#endif

		// Recommended value of NLS starts from the second byte of the message; we should update NLS, if the recommended value is greater
		int8_t nonceCmp = sa_uint48_compare( sasp_data.nonce_ls, new_nls ); // skipping First Byte and reserved byte
		if ( nonceCmp < 0 )
		{
			INCREMENT_COUNTER( 14, "handler_sasp_receive(), nonce las updated" );
			ZEPTO_MEMCPY( nls, new_nls, SASP_NONCE_SIZE );
//			sa_uint48_increment( sasp_data.nonce_ls );
			SASP_increment_nonce_last_sent( /*sasp_data*/ );
			// TODO: shuold we do anything else?
			return SASP_RET_TO_HIGHER_LAST_SEND_FAILED;
		}
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(), for SASP, error old nonce, not applied\n" );
		return SASP_RET_IGNORE_PACKET_NONCE_LS_NOT_APPLIED;
	}

	// 3. Compare nonces...
	int8_t nonceCmp = sa_uint48_compare( pid, sasp_data.nonce_lw );
	if ( nonceCmp == 0 )
	{
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): PACKET WITH THE SAME NONCE RECEIVED\n" );
		return SASP_RET_IGNORE_PACKET_LAST_REPEATED; // TODO: more detailed code might be necessary
	}
	else if ( nonceCmp < 0 ) // error message must be prepared
	{
#ifdef SA_DEBUG
		INCREMENT_COUNTER( 15, "handler_sasp_receive(), error old nonce" );
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): old nonce; packet for SASP is being prepared...\n" );
		ZEPTO_DEBUG_PRINTF_7( "   nonce received: %02x %02x %02x %02x %02x %02x\n", pid[0], pid[1], pid[2], pid[3], pid[4], pid[5] );
		uint8_t* nlw = sasp_data.nonce_lw;
		ZEPTO_DEBUG_PRINTF_7( "   current    NLW: %02x %02x %02x %02x %02x %02x\n", nlw[0], nlw[1], nlw[2], nlw[3], nlw[4], nlw[5] );
#endif

		zepto_response_to_request( mem_h ); // we have to create a new response (with error old nonce)
		// we are to create a brand new output
		zepto_write_uint8( mem_h, 0x80 ); // First Byte; TODO: use bit field processing instead (should we?)
		zepto_write_uint8( mem_h, 0 ); // Reserved Byte
		zepto_parser_encode_and_append_uint( mem_h, sasp_data.nonce_lw, SASP_NONCE_SIZE );
		zepto_response_to_request( mem_h );

//		uint8_t ne[ SASP_HEADER_SIZE ];
//		ZEPTO_MEMCPY( ne, pid, SASP_NONCE_SIZE );
		sa_uint48_t ne;
		handler_sasp_get_packet_id( ne );
		SASP_EncryptAndAddAuthenticationData( mem_h, key, ne );
		ZEPTO_DEBUG_PRINTF_1( "handler_sasp_receive(): ------------------- ERROR OLD NONCE WILL BE SENT ----------------------\n" );
//		ZEPTO_DEBUG_ASSERT( memory_object_get_response_size( mem_h ) <= 39 );
		return SASP_RET_TO_LOWER_ERROR;
	}

	// 4. Finally, we have a brand new packet
//	sa_uint48_init_by( sasp_data.nonce_lw, pid ); // update Nonce Low Watermark
	SASP_update_nonce_low_watermark( /*sasp_data, */pid );
	INCREMENT_COUNTER( 16, "handler_sasp_receive(), passed up" );
	return SASP_RET_TO_HIGHER_NEW;
}
 
#else
#error unexpected option
#endif





#endif // VERY_DEBUG_SIOT_SP_XXX
#endif // VERY_DEBUG
