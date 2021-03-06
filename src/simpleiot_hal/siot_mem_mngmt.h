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

#if !defined __SIOT_MEM_MNGMT_H__
#define __SIOT_MEM_MNGMT_H__

#include "siot_mem_mngmt_base.h"

#ifdef ZEPTO_VM_USE_SIMPLE_FRAME
#define PLAIN_REPLY_FRAME
#endif

#if (defined USED_AS_RETRANSMITTER) || (defined USED_AS_MASTER)
#define MEMORY_HANDLE_ALLOW_ACQUIRE_RELEASE
#endif

// named memory handles
#ifdef USED_AS_MASTER

#define MEMORY_HANDLE_MAIN_LOOP_1 0
#define MEMORY_HANDLE_MAIN_LOOP_1_SAOUDP_ADDR 1
#define MEMORY_HANDLE_MAIN_LOOP_2 2
#define MEMORY_HANDLE_MAIN_LOOP_2_SAOUDP_ADDR 3

#define MEMORY_HANDLE_SAGDP_LSM_APP_BASE 4
#define MEMORY_HANDLE_SAGDP_LSM_APP_SAOUDP_ADDR_BASE 14
#define MEMORY_HANDLE_SAGDP_LSM_CTR_BASE 24
#define MEMORY_HANDLE_SAGDP_LSM_CTR_SAOUDP_ADDR_BASE 34
//#define MEMORY_HANDLE_ADDITIONAL_ANSWER 44

#define MEMORY_HANDLE_MESH_ACK 45
#define MEMORY_HANDLE_SECOND_PART_START 46

#else // USED_AS_MASTER
//#define MEMORY_HANDLE_ALLOW_ACQUIRE_RELEASE

#define MEMORY_HANDLE_MAIN_LOOP_1 0
#define MEMORY_HANDLE_MAIN_LOOP_2 1
#define MEMORY_HANDLE_MAIN_LOOP_1_SAOUDP_ADDR 2
#define MEMORY_HANDLE_MAIN_LOOP_2_SAOUDP_ADDR 3
#define MEMORY_HANDLE_SAGDP_LSM_APP 4
#define MEMORY_HANDLE_SAGDP_LSM_APP_SAOUDP_ADDR 5
#define MEMORY_HANDLE_SAGDP_LSM_CTR 6
#define MEMORY_HANDLE_SAGDP_LSM_CTR_SAOUDP_ADDR 7
#define MEMORY_HANDLE_ADDITIONAL_ANSWER 8
#define MEMORY_HANDLE_MESH_1 9
#define MEMORY_HANDLE_MESH_2 10
#define MEMORY_HANDLE_MESH_ACK 11
#define MEMORY_HANDLE_MESH_RESEND_TASK_HOLDER 12
#define MEMORY_HANDLE_SECOND_PART_START 13

#endif // USED_AS_MASTER

#if !defined PLAIN_REPLY_FRAME
#define MEMORY_HANDLE_DEFAULT_PLUGIN MEMORY_HANDLE_SECOND_PART_START
#ifdef SA_DEBUG
#define MEMORY_HANDLE_TEST_SUPPORT (MEMORY_HANDLE_SECOND_PART_START + 1)
#define MEMORY_HANDLE_DBG_TMP (MEMORY_HANDLE_SECOND_PART_START + 2)
#define MEMORY_HANDLE_MAX_PREDEFINED (MEMORY_HANDLE_SECOND_PART_START + 3) // TODO: keep updated!!!
#else
#define MEMORY_HANDLE_MAX_PREDEFINED (MEMORY_HANDLE_SECOND_PART_START + 1) // TODO: keep updated!!!
#endif
#else
#ifdef SA_DEBUG
#define MEMORY_HANDLE_TEST_SUPPORT MEMORY_HANDLE_SECOND_PART_START
#define MEMORY_HANDLE_DBG_TMP (MEMORY_HANDLE_SECOND_PART_START + 1)
#define MEMORY_HANDLE_MAX_PREDEFINED (MEMORY_HANDLE_SECOND_PART_START + 2) // TODO: keep updated!!!
#else
#define MEMORY_HANDLE_MAX_PREDEFINED MEMORY_HANDLE_SECOND_PART_START // TODO: keep updated!!!
#endif
#endif // PLAIN_REPLY_FRAME

#ifdef MEMORY_HANDLE_ALLOW_ACQUIRE_RELEASE
#define MEMORY_HANDLE_ACQUIRABLE_HANDLE_STORAGE MEMORY_HANDLE_MAX_PREDEFINED
#define MEMORY_HANDLE_MAX (MEMORY_HANDLE_MAX_PREDEFINED + 1)
#define MEMORY_HANDLE_ACQUIRABLE_START MEMORY_HANDLE_MAX
#else
#define MEMORY_HANDLE_MAX MEMORY_HANDLE_MAX_PREDEFINED
#endif // MEMORY_HANDLE_ALLOW_ACQUIRE_RELEASE



typedef struct _parser_obj
{
	MEMORY_HANDLE mem_handle;
	bool res_valid;
	uint16_t offset;
} parser_obj;


#ifdef __cplusplus
extern "C" {
#endif

void zepto_mem_man_init_memory_management();


// parsing functions
void zepto_parser_init( parser_obj* po, REQUEST_REPLY_HANDLE mem_h );
void zepto_parser_init_by_parser( parser_obj* po, const parser_obj* po_base );
INLINE bool zepto_parser_is_result_valid( parser_obj* po ) {return po->res_valid;}
uint8_t zepto_parse_uint8( parser_obj* po );
bool zepto_parse_read_block( parser_obj* po, uint8_t* block, uint16_t size );
bool zepto_parse_skip_block( parser_obj* po, uint16_t size );
bool zepto_is_parsing_done( parser_obj* po );
uint16_t zepto_parsing_remaining_bytes( parser_obj* po );

uint8_t zepto_parse_encoded_uint8( parser_obj* po );
uint16_t zepto_parse_encoded_uint16( parser_obj* po );
INLINE int16_t zepto_parse_encoded_signed_int16( parser_obj* po )
{
	uint16_t ux = zepto_parse_encoded_uint16( po );
	return (int16_t)((ux >> 1) ^ (-(ux & 1)));
}
uint32_t zepto_parse_encoded_uint32( parser_obj* po );
void zepto_parser_decode_uint( parser_obj* po, uint8_t* bytes_out, uint8_t target_size );

// writing functions
void zepto_write_uint8( REQUEST_REPLY_HANDLE mem_h, uint8_t val );
//void zepto_write_encoded_uint16( REQUEST_REPLY_HANDLE mem_h, uint16_t val );
void zepto_write_block( REQUEST_REPLY_HANDLE mem_h, const uint8_t* block, uint16_t size );

// extended writing functions
void zepto_response_to_request( MEMORY_HANDLE mem_h );
void zepto_convert_part_of_request_to_response( MEMORY_HANDLE mem_h, parser_obj* po_start, parser_obj* po_end );
void zepto_append_part_of_request_to_response( MEMORY_HANDLE mem_h, parser_obj* po_start, parser_obj* po_end );
void zepto_copy_request_to_response_of_another_handle( MEMORY_HANDLE mem_h, MEMORY_HANDLE target_mem_h );
void zepto_copy_response_to_response_of_another_handle( MEMORY_HANDLE mem_h, MEMORY_HANDLE target_mem_h );
void zepto_append_response_to_response_of_another_handle( MEMORY_HANDLE mem_h, MEMORY_HANDLE target_mem_h );
void zepto_copy_part_of_request_to_response_of_another_handle( MEMORY_HANDLE mem_h, parser_obj* po_start, parser_obj* po_end, MEMORY_HANDLE target_mem_h );
void zepto_append_part_of_request_to_response_of_another_handle( MEMORY_HANDLE mem_h, parser_obj* po_start, parser_obj* po_end, MEMORY_HANDLE target_mem_h );
void zepto_prepend_part_of_request_to_response_of_another_handle( MEMORY_HANDLE mem_h, parser_obj* po_start, parser_obj* po_end, MEMORY_HANDLE target_mem_h );
void zepto_write_prepend_byte( MEMORY_HANDLE mem_h, uint8_t bt );
void zepto_write_prepend_block( MEMORY_HANDLE mem_h, const uint8_t* block, uint16_t size );


void zepto_parser_free_memory( REQUEST_REPLY_HANDLE mem_h );
void zepto_parser_free_response( REQUEST_REPLY_HANDLE mem_h );
void zepto_parser_strip_beginning_of_request( parser_obj* po );

// special r/w functions for operations with locally generated data
bool zepto_memman_read_locally_generated_data_by_offset( MEMORY_HANDLE mem_h, uint16_t offset, uint16_t size, uint8_t* buff );
bool zepto_memman_write_locally_generated_data_by_offset( MEMORY_HANDLE mem_h, uint16_t offset, uint16_t size, const uint8_t* buff );
void zepto_memman_append_locally_generated_data( MEMORY_HANDLE mem_h, uint16_t size, const uint8_t* buff );
void zepto_memman_trim_locally_generated_data_at_right( MEMORY_HANDLE mem_h, uint16_t size );
uint16_t zepto_memman_get_currently_allocated_size_for_locally_generated_data( MEMORY_HANDLE mem_h );
/*
void zepto_writer_get_copy_of_response( MEMORY_HANDLE mem_h, uint8_t* buff );
*/

// specific encoded uint functions
//void zepto_parser_encode_uint( const uint8_t* num_bytes, uint8_t num_sz_max, uint8_t** bytes_out );
//void zepto_parser_decode_uint( const uint8_t** packed_num_bytes, uint8_t* bytes_out, uint8_t target_size );

void zepto_parser_encode_and_append_uint8( MEMORY_HANDLE mem_h, uint8_t num );
void zepto_parser_encode_and_append_uint16( MEMORY_HANDLE mem_h, uint16_t num );
INLINE void zepto_parser_encode_and_append_signed_int16( MEMORY_HANDLE mem_h, int16_t sx )
{
	uint16_t ux = (uint16_t)((sx << 1) ^ (sx>>15));
	zepto_parser_encode_and_append_uint16( mem_h, ux );
}
void zepto_parser_encode_and_append_uint32( MEMORY_HANDLE mem_h, uint32_t num );
void zepto_parser_encode_and_append_uint( MEMORY_HANDLE mem_h, const uint8_t* num_bytes, uint8_t num_sz_max );
void zepto_parser_encode_and_prepend_uint16( MEMORY_HANDLE mem_h, uint16_t num );
void zepto_parser_encode_and_prepend_uint( MEMORY_HANDLE mem_h, const uint8_t* num_bytes, uint8_t num_sz_max );

uint16_t memory_object_get_request_size( REQUEST_REPLY_HANDLE mem_h );
uint16_t memory_object_get_response_size( REQUEST_REPLY_HANDLE mem_h );
uint8_t memory_object_read_response_byte( REQUEST_REPLY_HANDLE mem_h, uint16_t offset );

#ifdef MEMORY_HANDLE_ALLOW_ACQUIRE_RELEASE
MEMORY_HANDLE acquire_memory_handle();
void release_memory_handle( MEMORY_HANDLE mem_h );
#else
#define acquire_memory_handle() ZEPTO_DEBUG_ASSERT( 0 == "acquire_memory_handle() is not implemented in this configuration" )
#define release_memory_handle(x) ZEPTO_DEBUG_ASSERT( 0 == "release_memory_handle() is not implemented in this configuration" )
#endif // MEMORY_HANDLE_ALLOW_ACQUIRE_RELEASE

#ifdef __cplusplus
}
#endif

#endif // __SIOT_MEM_MNGMT_H__