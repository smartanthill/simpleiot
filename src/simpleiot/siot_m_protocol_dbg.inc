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
  and implements a highly simplified siot/mp.
  
  In short, packet processing is reduced to just passing a received packet without 
  modifications if if it were intended for an upper layer (or the other way around).

  If this version is used, the other party should be ready to receive a copy of a
  packet that has been passed from the upper layer.

  Note that bus_id is not changed within this simplification, which means that 
  a response will be sent to a bus from which a request has come.
  
  Main purpose is, roughly, to avoid potential problems with the protocol
  (or its specific implementation).

  Practical usage: using simulation (say, Desktop platform) prepare a reasonable 
  request packet, and record response. Generate firmware, pass a request packet to device, 
  get a response and compare it with that saved on the previous step in hope 
  they will be the same.
*******************************************************************************/

#if !defined VERY_DEBUG
#error This file is intended for DEBUG PURPOSES ONLY and cannot be used when VERY_DEBUG is not defined
#else // VERY_DEBUG
#if (!defined VERY_DEBUG_SIOT_SIOTMP)
#error This file is intended for DEBUG PURPOSES ONLY and can be used when one of VERY_DEBUG_SIOT_SIOTMP is defined
#else // VERY_DEBUG_SIOT_SIOTMP

#include "siot_m_protocol.h"


#ifdef USED_AS_MASTER

uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint16_t* src_id, uint16_t* bus_id, uint8_t conn_quality, uint8_t error_cnt )
{
	return SIOT_MESH_RET_PASS_TO_PROCESS;
}

uint8_t handler_siot_mesh_send_packet( uint8_t is_ctr, sa_time_val* currt, waiting_for* wf, uint16_t target_id, MEMORY_HANDLE mem_h, uint8_t resend_cnt, uint16_t* bus_id )
{
	return SIOT_MESH_RET_PASS_TO_SEND;
}

void handler_siot_mesh_process_route_update_response(  uint16_t source_dev_id, MEMORY_HANDLE mem_h )
{
}

uint8_t handler_siot_mesh_timer( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t* device_id, uint16_t* bus_id )
{
	return SIOT_MESH_RET_OK;
}

#else // USED_AS_MASTER

void siot_mesh_init_tables()
{
}

void handler_siot_process_route_update_request( parser_obj* po, MEMORY_HANDLE reply )
{
}

uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint8_t* mesh_val, uint8_t signal_level, uint8_t error_cnt, uint16_t* bus_id, uint16_t* ack_bus_id )
{
	return SIOT_MESH_RET_PASS_TO_PROCESS;
}

uint8_t handler_siot_mesh_packet_rejected_broken( /*MEMORY_HANDLE mem_h, */uint8_t* mesh_val )
{
	return SIOT_MESH_RET_OK;
}

uint8_t handler_siot_mesh_send_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint8_t mesh_val, uint8_t resend_cnt, uint16_t target_id, uint16_t* bus_id )
{
	return SIOT_MESH_RET_PASS_TO_SEND;
}

uint8_t handler_siot_mesh_timer( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t* bus_id )
{
	return SIOT_MESH_RET_OK;
}

#endif // USED_AS_MASTER



#endif // VERY_DEBUG_SIOT_SIOTMP
#endif // VERY_DEBUG
