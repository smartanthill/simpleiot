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


#if !defined __SIOT_BUS_LIST_COMMON_H__
#define __SIOT_BUS_LIST_COMMON_H__

#include "siot_common.h"
#include "siot_data_types.h"

#define SIOT_COMMUNICATION_BUS_TYPE_UART

uint8_t hal_get_bus_count(); // bus IDs are then expected in the range 0..(ret_val-1)
uint8_t hal_get_bus_type( uint8_t bus_id );
// requests for other information, such as flags, etc, will be added in a similar manner as necessary


#endif // __SIOT_BUS_LIST_COMMON_H__