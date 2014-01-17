/*
 * lusb.cpp
 *
 *  Created on: 23.05.2011
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

#include <libusb-1.0/libusb.h>
#include "lusb.h"
#include "utils.h"


int lusb::m_init_cnt = 0;

int lusb::initialize( void )
{
 int ret = 0;


 	if( m_init_cnt == 0 )
 	{
 		ret = libusb_init( NULL );
 		if( ret == 0 )
 		{
 			libusb_set_debug( NULL, 0 );
 			log_i( "lusb::initialize(): Success" );
 		}
 		else
 			log_e( "lusb::initialize(): Failed: error code = %d", ret );
 	}
 	if( ret == 0 )
 		m_init_cnt++;

 return ret;
}


int lusb::release( void )
{
 int ret = 0;

	if( m_init_cnt <= 0 )
	{
		log_e( "lusb::release(): Alreade released or not initialized yet!" );
		ret = -1;
	}
	else
	{
		m_init_cnt--;
		if( m_init_cnt == 0 )
		{
			libusb_exit( NULL );
			log_i( "lusb::release(): Success" );
		}
	}

 return ret;
}


bool lusb::is_initialized( void )
{
	return m_init_cnt > 0;
}
