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


#if !defined __SA_SA_BODYPART_LIST_COMMON_H__
#define __SA_SA_BODYPART_LIST_COMMON_H__

#include "siot_common.h"
#include "siot_data_types.h"
#include "../simpleiot_hal/siot_mem_mngmt.h"
#include "../simpleiot_hal/hal_waiting.h"

#define PLUGIN_OK 0
#define PLUGIN_WAIT_TO_CONTINUE 1

typedef uint8_t (*plugin_handler_config_fn)(const void* plugin_config, void* plugin_persistent_state);
typedef uint8_t (*plugin_exec_init_fn)(const void* plugin_config, void* plugin_state);
typedef uint8_t (*plugin_exec_fn)( const void* plugin_config, void* plugin_persistent_state, void* plugin_state, parser_obj* command, MEMORY_HANDLE reply, waiting_for* wf, uint8_t first_byte );

typedef struct _bodypart_item
{
	plugin_handler_config_fn ph_config_fn;
	plugin_exec_init_fn ph_exec_init_fn;
	plugin_exec_fn ph_exec_fn;
	void* ph_config;
	void* ph_persistent_state;
	void* ph_state;
} bodypart_item;


#endif // __SA_SA_BODYPART_LIST_COMMON_H__