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
  
  Main purpose is, roughly, to avoid potential problems with the protocol
  (or its specific implementation).
*******************************************************************************/

#if !defined VERY_DEBUG
#error This file is intended for DEBUG PURPOSES ONLY and cannot be used when VERY_DEBUG is not defined
#else // VERY_DEBUG
#if (!defined VERY_DEBUG_SIOT_SIOTMP)
#error This file is intended for DEBUG PURPOSES ONLY and can be used when one of VERY_DEBUG_SIOT_SIOTMP is defined
#else // VERY_DEBUG_SIOT_SIOTMP





#endif // VERY_DEBUG_SIOT_SIOTMP
#endif // VERY_DEBUG
