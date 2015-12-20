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

// Upper levels need some way to iterate over buses (more precisely, over something uniquely specifying a direction to send a packet, such as bus_id, or, potentially, even bus_id + intra_bus_id, etc )
// Here we provide a sample (and quite naive and simple) approach just to be able to go forward with implementation of upper levels 
// and with full understanding that selected approach can be revised more or less substantially assuming that the above-said goal is somehow reached
// (in other words: if you do not like it, just come with proposals).
// TODO: actual interface and its implementation

uint8_t hal_get_bus_count(); // bus IDs are then expected in the range 0..(ret_val-1)

#if 0 // another potential approach
uint16_t hal_get_first_bus_id();
uint16_t hal_get_first_bus_id( uint16_t prev_bus_id);
#endif // 0

#define BUS_TYPE_UNDEFINED 0XFF
#define BUS_ID_UNDEFINED 0XFFFF
uint8_t hal_get_bus_type_by_bus_id( uint16_t bus_id );
uint16_t hal_get_next_bus_of_type(uint8_t bus_type, uint16_t prev_bus_id );
// requests for other information, such as flags, etc, will be added in a similar manner as necessary


#endif // __SIOT_BUS_LIST_COMMON_H__