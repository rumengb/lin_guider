/*
 * io_null.cpp
 *
 *  Created on: 01.06.2011
 *      Author: gm
 *
 *
 * This file is part of Lin_guider.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "io_null.h"
#include "maindef.h"
#include "utils.h"

namespace io_drv
{

//----------------------------------------
//
// Derived.... NULL
//
//----------------------------------------
cio_driver_null::cio_driver_null( bool stub )
{
	device_type = DT_NULL;
	min_pulse_length = 0;
	max_pulse_length = 0xFFFFFF;

	stub_mode = stub;
}


cio_driver_null::~cio_driver_null()
{
	//MUST BE called the first: stop thread and release resources
	stop();
}


int cio_driver_null::open_device( void )
{
	set_bit_map_template( device_bit_map_template[DT_NULL-1] );
	log_i( "cio_driver_null::open_device(): Success" );
 return 0;
}


int cio_driver_null::close_device( void )
{
	log_i( "cio_driver_null::close_device(): Success" );
 return 0;
}


void cio_driver_null::write_data( unsigned int dByte )
{
 unsigned int mapped;

 	mapped = bit_map_encoder[ (u_char)dByte ];

	//TODO: do software correction
 	(void)mapped;
}


u_char cio_driver_null::read_byte( void )
{
 return 0;
}

}
