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


#if !defined __SIOT_M_PROTOCOL_H__
#define __SIOT_M_PROTOCOL_H__

#include "siot_common.h"
#include "../simpleiot_hal/siot_mem_mngmt.h"
#include <hal_time_provider.h>
#include "../simpleiot_hal/hal_waiting.h"

extern uint16_t DEVICE_SELF_ID;

#define SIOT_MESH_ANY_PACKET 0 // (used for other purposes) 	Samp-Unicast-Data-Packet
#define SIOT_MESH_FROM_SANTA_DATA_PACKET 1 // 	Samp-From-Santa-Data-Packet
#define SIOT_MESH_TO_SANTA_DATA_OR_ERROR_PACKET 2 // 	Samp-To-Santa-Data-Packet
#define SIOT_MESH_FORWARD_TO_SANTA_DATA_OR_ERROR_PACKET 3 // 	Samp-Forward-To-Santa-Data-Or-Error-Packet
#define SIOT_MESH_ROUTING_ERROR_PACKET 4 // 	Samp-Routing-Error-Packet
#define SIOT_MESH_ACK_NACK_PACKET 5 // 	Samp-Ack-Nack-Packet

#define SIOT_MESH_GENERIC_EXTRA_HEADER_FLAGS 0
#define SIOT_MESH_GENERIC_EXTRA_HEADER_COLLISION_DOMAIN 1
#define SIOT_MESH_UNICAST_EXTRA_HEADER_LOOP_ACK 2
#define SIOT_MESH_TOSANTA_EXTRA_HEADER_LAST_INCOMING_HOP 3
#define SIOT_MESH_TTL_MAX 4
// Route table MODIFICATIONS-LIST entry types
#define ADD_OR_MODIFY_LINK_ENTRY 0
#define DELETE_LINK_ENTRY 1
#define ADD_OR_MODIFY_ROUTE_ENTRY 2
#define DELETE_ROUTE_ENTRY 3

// SIOT_MESH data structures

// link table item
typedef struct _SIOT_MESH_LINK
{
	uint16_t LINK_ID; // type is inspired in section "Communicating Routing Table Information over SACCP" by "Encoded-Unsigned-Int<max=2> bitfield substrate ... bits[4..] equal to LINK-ID"
	uint16_t NEXT_HOP; // note: this link will be common for all targets that are reachable through a device with NEXT_HOP device ID
	uint16_t BUS_ID; // type is inspired in section "Communicating Routing Table Information over SACCP" by "BUS-ID is an Encoded-Unsigned-Int<max=2> field"
	uint8_t INTRA_BUS_ID; // INTRA-BUS-ID=NULL means that the entry is for an incoming link. Incoming link entries are relatiely rare, and are used to specify LINK-DELAYs.
	uint8_t NEXT_HOP_ACKS; // NEXT-HOP-ACKS is a flag which is set if the nearest hop (over (BUS-ID,INTRA-BUS-ID)) is known to be able not only to receive packets, but to send ACKs back
	uint16_t LINK_DELAY_UNIT; // type is inspired: same section as above
	uint16_t LINK_DELAY; // type is inspired: same section as above
	uint16_t LINK_DELAY_ERROR; // type is inspired: same section as above
} SIOT_MESH_LINK;

#define SIOT_MESH_NEXT_HOP_UNDEFINED 0xFFFF
#define SIOT_MESH_TARGET_UNDEFINED 0xFFFF
#define SIOT_MESH_BUS_UNDEFINED 0xFFFF

// rout table item
typedef struct _SIOT_MESH_ROUTE
{
	uint16_t TARGET_ID;
	uint16_t LINK_ID;
} SIOT_MESH_ROUTE;

typedef struct _SIOT_MESH_RETRANSM_COMMON_DATA
{
	uint8_t MAX_TTL;
	uint8_t FORWARD_TO_SANTA_DELAY_UNIT;
	uint8_t FORWARD_TO_SANTA_DELAY;
	uint8_t MAX_FORWARD_TO_SANTA_DELAY; // (using same units as FORWARD-TO-SANTA-DELAY); indicates maximum "forward to santa" delay for all Retransmitting Devices in the PAN.
	uint8_t NODE_MAX_RANDOM_DELAY_UNIT;
	uint8_t NODE_MAX_RANDOM_DELAY;
} SIOT_MESH_RETRANSM_COMMON_DATA;


// SIOT_MESH ret codes
#define SIOT_MESH_RET_OK 0
#define SIOT_MESH_RET_ERROR_ANY 1
#define SIOT_MESH_RET_GARBAGE_RECEIVED 2
#define SIOT_MESH_RET_NOT_FOR_THIS_DEV_RECEIVED 3
#define SIOT_MESH_RET_PASS_TO_PROCESS 4
#define SIOT_MESH_RET_PASS_TO_SEND 5
#define SIOT_MESH_RET_PASS_TO_CCP 6
#define SIOT_MESH_RET_SEND_ACK_AND_PASS_TO_PROCESS 7
#define SIOT_MESH_RET_SEND_ACK_AND_PASS_TO_SEND 8
// internal errors (TODO: should not be exposed)
#define SIOT_MESH_RET_ERROR_NOT_FOUND 16
#define SIOT_MESH_RET_ERROR_OUT_OF_RANGE 17
#define SIOT_MESH_RET_ERROR_ROUTE_UNDER_CONSTRUCTION 18



#ifdef USED_AS_MASTER

#define SIOT_MESH_SUBJECT_FOR_ACK 2
#define SIOT_MESH_SUBJECT_FOR_MESH_RESEND 5
#define MESH_RESEND_PERIOD_MS 500

uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint16_t* src_id, uint8_t conn_quality, uint8_t error_cnt );
uint8_t handler_siot_mesh_send_packet( uint8_t is_ctr, sa_time_val* currt, waiting_for* wf, uint16_t target_id, MEMORY_HANDLE mem_h, uint8_t resend_cnt, uint16_t* bus_id );
uint8_t handler_siot_mesh_timer( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t* device_id, uint16_t* bus_id );
void handler_siot_mesh_process_route_update_response(  uint16_t source_dev_id, MEMORY_HANDLE mem_h );

#ifdef __cplusplus
extern "C" {
#endif

#define SIOT_MESH_AT_ROOT_RET_OK 0
#define SIOT_MESH_AT_ROOT_RET_FAILED 1
#define SIOT_MESH_AT_ROOT_RET_ALREADY_EXISTS 2
#define SIOT_MESH_AT_ROOT_RET_NOT_FOUND 3
#define SIOT_MESH_AT_ROOT_RET_NO_UPDATES 4
#define SIOT_MESH_AT_ROOT_RET_NO_READY_UPDATES 5
// ret codes for siot_mesh_at_root_get_resend_task
#define SIOT_MESH_AT_ROOT_RET_RESEND_TASK_NONE_EXISTS 6
#define SIOT_MESH_AT_ROOT_RET_RESEND_TASK_NOT_NOW 7
#define SIOT_MESH_AT_ROOT_RET_RESEND_TASK_INTERM 8
#define SIOT_MESH_AT_ROOT_RET_RESEND_TASK_FINAL 9
#define SIOT_MESH_AT_ROOT_RET_RESEND_TASK_FROM_SANTA 10


void siot_mesh_init_tables();  // TODO: this call reflects current development stage and may or may not survive in the future
uint8_t write_bus_types_for_device_for_from_santa_packet( MEMORY_HANDLE mem_h, uint16_t device_id );
uint16_t write_retransmitter_list_for_from_santa_packet( MEMORY_HANDLE mem_h );

uint8_t siot_mesh_at_root_target_to_link_id( uint16_t target_id, uint16_t* link_id );
uint8_t siot_mesh_get_link( uint16_t link_id, SIOT_MESH_LINK* link );
void siot_mesh_at_root_remove_link_to_target_no_ack_from_immediate_hop( uint16_t target_id, uint16_t next_hop_id ); // generates route table updates
void siot_mesh_at_root_remove_link_to_target_route_error_reported( uint16_t reporting_id, uint16_t failed_hop_id, uint16_t failed_target_id );

void siot_mesh_form_packets_from_santa_and_add_to_task_list( const sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t target_id );

void siot_mesh_at_root_add_last_hop_in_data( uint16_t src_id, uint16_t last_hop_id, uint16_t last_hop_bus_id, uint8_t conn_q );
uint8_t siot_mesh_at_root_find_best_route( uint16_t* target_id, uint16_t* bus_id_at_target, uint16_t* id_prev, uint16_t* bus_id_at_prev, uint16_t* id_next );
uint8_t siot_mesh_at_root_remove_last_hop_data( uint16_t target_id );
uint8_t siot_mesh_at_root_add_updates_for_device_when_route_is_added( uint16_t id_target, uint16_t bus_to_send_from_target, uint16_t id_prev, uint16_t bust_to_send_from_prev, uint16_t id_next /*more data may be required*/ );
void siot_mesh_at_root_add_last_hop_out_data( uint16_t src_id, uint16_t bus_id_at_src, uint16_t first_receiver_id, uint8_t conn_q );
uint8_t siot_mesh_at_root_load_update_to_packet( MEMORY_HANDLE mem_h, uint16_t* recipient );
uint8_t siot_mesh_at_root_update_done( uint16_t device_id );

void siot_mesh_at_root_add_resend_task( MEMORY_HANDLE packet, const sa_time_val* currt, uint16_t checksum, uint16_t target_id, uint16_t bus_id, uint16_t next_hop_id, sa_time_val* time_to_next_event );
void siot_mesh_at_root_add_send_from_santa_task( MEMORY_HANDLE packet, const sa_time_val* currt, uint16_t bus_id );
uint8_t siot_mesh_at_root_get_resend_task( MEMORY_HANDLE packet, const sa_time_val* currt, uint16_t* target_id, uint16_t* bus_id, uint16_t* next_hop_id, sa_time_val* time_to_next_event );
void siot_mesh_at_root_remove_resend_task_by_hash( uint16_t checksum, const sa_time_val* currt, sa_time_val* time_to_next_event );
void siot_mesh_at_root_remove_resend_task_by_device_id( uint16_t target_id, const sa_time_val* currt, sa_time_val* time_to_next_event );

uint16_t zepto_parser_calculate_checksum_of_part_of_response( MEMORY_HANDLE mem_h, uint16_t offset, uint16_t sz, uint16_t accum_val );
uint16_t zepto_parser_calculate_checksum_of_part_of_request( MEMORY_HANDLE mem_h, parser_obj* po_start, uint16_t sz, uint16_t accum_val );

#ifdef __cplusplus
}
#endif

#else // USED_AS_MASTER

#define SIOT_MESH_SUBJECT_FOR_ACK 2
#define SIOT_MESH_SUBJECT_FOR_MESH_RESEND 5
#define SIOT_MESH_SUBJECT_FOR_MESH_RESEND_UNICAST_IN_TRANSIT 4
#define MESH_RESEND_PERIOD_MS 500
#define MESH_RFEPLY_TO_FROMSANTA_PERIOD_MS_MAX 500

void siot_mesh_init_tables();  // TODO: this call reflects current development stage and may or may not survive in the future
void handler_siot_process_route_update_request( parser_obj* po, MEMORY_HANDLE reply );
#ifdef USED_AS_RETRANSMITTER
uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint8_t* mesh_val, uint8_t signal_level, uint8_t error_cnt, uint16_t* bus_id, uint16_t* ack_bus_id );
#else // USED_AS_RETRANSMITTER
uint8_t handler_siot_mesh_receive_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, MEMORY_HANDLE mem_ack_h, uint8_t* mesh_val, uint8_t signal_level, uint8_t error_cnt, uint16_t* ack_bus_id );
#endif // USED_AS_RETRANSMITTER
uint8_t handler_siot_mesh_packet_rejected_broken( /*MEMORY_HANDLE mem_h, */uint8_t* mesh_val );
uint8_t handler_siot_mesh_send_packet( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint8_t mesh_val, uint8_t resend_cnt, uint16_t target_id, uint16_t* bus_id );
uint8_t handler_siot_mesh_timer( sa_time_val* currt, waiting_for* wf, MEMORY_HANDLE mem_h, uint16_t* bus_id );

#endif // USED_AS_MASTER




#endif // __SIOT_M_PROTOCOL_H__
