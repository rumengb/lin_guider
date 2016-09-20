/*
 * io_lpt.cpp
 *
 *  Created on: 25.05.2011
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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/ppdev.h>
#include <linux/parport.h>
#include <unistd.h>

#include "io_lpt.h"
#include "maindef.h"
#include "utils.h"

namespace io_drv
{

//----------------------------------------
//
// Derived.... LPT
//
//----------------------------------------
cio_driver_lpt::cio_driver_lpt()
{
	parport = -1;
	pp_mode = 0;
	device_type = DT_LPT;
	min_pulse_length = 0;
	max_pulse_length = 0xFFFFFF;
}


cio_driver_lpt::~cio_driver_lpt()
{
	//MUST BE called the first: stop thread and release resources
	stop();
}


int cio_driver_lpt::open_device( void )
{

	parport = open( dev_name, O_RDWR );

	if( parport == -1 )
	{
		return 1;
	}

	if( ioctl( parport, PPCLAIM ) )
	{
		close( parport );
		return 2;
	}

	pp_mode = IEEE1284_MODE_COMPAT;
	if( ioctl( parport, PPNEGOT, &pp_mode ) )
	{
		close( parport );
		return 3;
	}

	// Down all pins
	write_data( 0x0 );

 return 0;
}


int cio_driver_lpt::close_device( void )
{
	if( parport == -1 )
		return -1;

	// Down all pins
	write_data( 0x0 );

	ioctl( parport, PPRELEASE );
	close( parport );

	parport = -1;

 return 0;
}


void cio_driver_lpt::write_data( unsigned int dByte )
{
 unsigned int mapped;

 	mapped = bit_map_encoder[ (u_char)dByte ];

	ioctl( parport, PPWDATA, &mapped );
}


u_char cio_driver_lpt::read_byte( void )
{
 return 0;
}

}
