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

#if !defined __SASP_PROTOCOL_H__
#define __SASP_PROTOCOL_H__

#include "siot_common.h"
#include "siot_data_types.h"
#include "../simpleiot_hal/siot_eeprom.h"
#include "../simpleiot_hal/siot_mem_mngmt.h"

// RET codes
#define SASP_RET_IGNORE_PACKET_BROKEN 0 // has not passed decryption/authentication
#define SASP_RET_IGNORE_PACKET_LAST_REPEATED 1 // same nonce as last received
#define SASP_RET_IGNORE_PACKET_NONCE_LS_NOT_APPLIED 2 // packet with a recommended minimal value of nonce to send; the value is less than current
#define SASP_RET_TO_HIGHER_NEW 3 // new packet
#define SASP_RET_TO_HIGHER_LAST_SEND_FAILED 4 // sending of last packet failed (for instance, old nonce)
#define SASP_RET_TO_LOWER_REGULAR 5 // for regular sending
#define SASP_RET_TO_LOWER_ERROR 6 // for error messaging
#define SASP_RET_NONCE 7 // buffer out contains nonce


// sizes
#define SASP_NONCE_SIZE 6
#define SASP_HEADER_SIZE SASP_NONCE_SIZE
#define SASP_ENC_BLOCK_SIZE 16
#define SASP_TAG_SIZE SASP_ENC_BLOCK_SIZE


// data structures
typedef struct _SASP_DATA
{
	sasp_nonce_type nonce_lw;
	sasp_nonce_type nonce_ls;
} SASP_DATA;


#ifdef USED_AS_MASTER

// initializing and backup
void sasp_init_eeprom_data_at_lifestart( SASP_DATA* sasp_data, uint16_t storage_param );
void sasp_restore_from_backup( SASP_DATA* sasp_data, uint16_t storage_param );
void handler_sasp_save_state( SASP_DATA* sasp_data, uint16_t storage_param );

// handlers
uint8_t handler_sasp_receive( const uint8_t* key, uint8_t* packet_id, MEMORY_HANDLE mem_h, SASP_DATA* sasp_data, uint16_t storage_param );
uint8_t handler_sasp_send( const uint8_t* key, const uint8_t* packet_id, MEMORY_HANDLE mem_h, SASP_DATA* sasp_data );
uint8_t handler_sasp_get_packet_id( sa_uint48_t buffOut, SASP_DATA* sasp_data, uint16_t storage_param );

#else // USED_AS_MASTER

// initializing and backup
void sasp_init_eeprom_data_at_lifestart();
void sasp_restore_from_backup();
void handler_sasp_save_state();

// handlers
uint8_t handler_sasp_receive( const uint8_t* key, uint8_t* packet_id, MEMORY_HANDLE mem_h );
uint8_t handler_sasp_send( const uint8_t* key, const uint8_t* packet_id, MEMORY_HANDLE mem_h );
uint8_t handler_sasp_get_packet_id( sa_uint48_t buffOut );

#endif // USED_AS_MASTER


#endif // __SASP_PROTOCOL_H__