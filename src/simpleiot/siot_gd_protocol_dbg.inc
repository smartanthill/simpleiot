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
  
  In short, packet processing is reduced to just forwarding it to an intended 
  party without modifications assuming that everything on the way from Master
  to Slave and back works fine (no packet is lost, thus no packet is resent, etc).
  In addition, all states are considered to be good, or, better say, it is
  assumed that packets come when the protocol is in a state that allows packet
  forwarding to an intended party.
  
  Main purpose is, roughly, to exclude potential problems with the protocol
  (or its specific implementation) from consideration.
*******************************************************************************/

#if !defined VERY_DEBUG
#error This file is intended for DEBUG PURPOSES ONLY and cannot be used when VERY_DEBUG is not defined
#else // VERY_DEBUG
#if (!defined VERY_DEBUG_SIOT_GDP)
#error This file is intended for DEBUG PURPOSES ONLY and can be used when one of VERY_DEBUG_SIOT_GDP is defined
#else // VERY_DEBUG_SIOT_GDP

#include "siot_gd_protocol.h"



void sagdp_init( /*SAGDP_DATA* sagdp_data*/ )
{
}

uint8_t handler_sagdp_timer( sa_time_val* currt, waiting_for* wf, sasp_nonce_type nonce, REQUEST_REPLY_HANDLE mem_h, REQUEST_REPLY_HANDLE mem_h_addr/*, SAGDP_DATA* sagdp_data*/ )
{
	return SAGDP_RET_OK; // just ignore
}

uint8_t handler_sagdp_receive_up( sa_time_val* currt, waiting_for* wf, sasp_nonce_type nonce, uint8_t* pid, REQUEST_REPLY_HANDLE mem_h, REQUEST_REPLY_HANDLE mem_h_addr/*, SAGDP_DATA* sagdp_data*/ )
{
	// in this DEBUG-only version we just return the packet as response

	parser_obj po_start, po_end;
	zepto_parser_init( &po_start, mem_h );
	zepto_parser_init( &po_end, mem_h );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po_start ) );
	zepto_convert_part_of_request_to_response( mem_h, &po_start, &po_end );

	return SAGDP_RET_TO_HIGHER;
}

uint8_t handler_sagdp_receive_request_resend_lsp( sa_time_val* currt, waiting_for* wf, sasp_nonce_type nonce, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_h_addr/*, SAGDP_DATA* sagdp_data*/ )
{
	return SAGDP_RET_TO_LOWER_NONE;
}

uint8_t handler_sagdp_receive_hlp( sa_time_val* currt, waiting_for* wf, sasp_nonce_type nonce, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_h_addr/*, SAGDP_DATA* sagdp_data*/ )
{
	// in this DEBUG-only version we just return the packet as response

	parser_obj po_start, po_end;
	zepto_parser_init( &po_start, mem_h );
	zepto_parser_init( &po_end, mem_h );
	zepto_parse_skip_block( &po_end, zepto_parsing_remaining_bytes( &po_start ) );
	zepto_convert_part_of_request_to_response( mem_h, &po_start, &po_end );

	return SAGDP_RET_TO_LOWER_NEW;
}



#endif // VERY_DEBUG_SIOT_GDP
#endif // VERY_DEBUG
