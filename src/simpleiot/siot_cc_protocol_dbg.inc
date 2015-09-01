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
  and implements a highly simplified siot/gdp.
  
  In short, packet processing is reduced to just echoing a received packet without 
  modifications as intermediate, if the received packet is first or intermediate, 
  and as first otherwise.

  If this version is used, the other party should be ready to receive a copy of a
  packet that has been initially sent.
  
  Main purpose is, roughly, to avoid potential problems with the protocol
  (or its specific implementation).
*******************************************************************************/

#if !defined VERY_DEBUG
#error This file is intended for DEBUG PURPOSES ONLY and cannot be used when VERY_DEBUG is not defined
#else // VERY_DEBUG
#if (!defined VERY_DEBUG_SIOT_CCP)
#error This file is intended for DEBUG PURPOSES ONLY and can be used when one of VERY_DEBUG_SIOT_CCP is defined
#else // VERY_DEBUG_SIOT_CCP



#include "siot_cc_protocol.h"
#include "siot_gd_protocol.h" // for packet status in chain
#include "siot_uint48.h"
#include "siot_cc_protocol_constants.h"
#include "siot_bodypart_list.h"


void zepto_vm_init()
{
}

uint8_t handler_saccp_timer( MEMORY_HANDLE mem_h, sasp_nonce_type chain_id, sa_time_val* currt, waiting_for* wf )
{
	return SACCP_RET_NO_WAITING;
}

uint8_t handler_saccp_receive( MEMORY_HANDLE mem_h, sasp_nonce_type chain_id, sa_time_val* currt, waiting_for* wf )
{
	// in this DEBUG-only version we just return the received packet as response

	parser_obj po_start, po_end;
	zepto_parser_init( &po_start, mem_h );
	zepto_parser_init( &po_end, mem_h );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po_start ) );
	uint8_t first_byte = zepto_parse_uint8( &po_start );
	zepto_convert_part_of_request_to_response( mem_h, &po_start, &po_end );
	zepto_write_prepend_byte( mem_h, first_byte == 2 ? 1 : 0 ); // first, if the received is terminating, and intermediate otherwise

	return SACCP_RET_PASS_LOWER;
}



#endif // VERY_DEBUG_SIOT_CCP
#endif // VERY_DEBUG
