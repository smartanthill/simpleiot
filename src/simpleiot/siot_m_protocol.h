/*******************************************************************************
Copyright (C) 2015 OLogN Technologies AG

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2 as
    published by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
ZEPTO_MEMMOV
    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*******************************************************************************/


#if !defined __SIOT_M_PROTOCOL_H__
#define __SIOT_M_PROTOCOL_H__

#include "siot_common.h"

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

// SIOT_MESH data structures

// link table item
typedef struct _SIOT_MESH_LINK
{
	uint8_t LINK_ID;
	uint8_t BUS_ID;
	uint8_t INTRA_BUS_ID; // INTRA-BUS-ID=NULL means that the entry is for an incoming link. Incoming link entries are relatiely rare, and are used to specify LINK-DELAYs.
	uint8_t NEXT_HOP_ACKS; // NEXT-HOP-ACKS is a flag which is set if the nearest hop (over (BUS-ID,INTRA-BUS-ID)) is known to be able not only to receive packets, but to send ACKs back
	uint8_t LINK_DELAY_UNIT;
	uint8_t LINK_DELAY;
	uint8_t LINK_DELAY_ERROR;
} SIOT_MESH_LINK;

// rout table item
typedef struct _SIOT_MESH_ROUTE
{
	uint8_t TARGET_ID;
	uint8_t LINK_ID;
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
#define SIOT_MESH_RET_ERROR_NOT_FOUND 2
#define SIOT_MESH_RET_ERROR_OUT_OF_RANGE 3




#endif // __SIOT_M_PROTOCOL_H__
