/*
 * io_qhy6.cpp
 *
 *  Created on: 21.06.2011
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

#include "io_qhy6.h"
#include "maindef.h"
#include "utils.h"

namespace io_drv
{

//----------------------------------------
//
// Derived.... QHY6
//
//----------------------------------------
cio_driver_qhy6::cio_driver_qhy6()
{
	device_type = DT_QHY6;
	min_pulse_length = 0;
	max_pulse_length = 0xFFFF;

	m_qhy6_obj = new qhy6_core_shared();

	set_bit_map_template( device_bit_map_template[DT_QHY6-1] );
}


cio_driver_qhy6::~cio_driver_qhy6()
{
	//MUST BE called the first: stop thread and release resources
	stop();

	delete m_qhy6_obj;
}


int cio_driver_qhy6::open_device( void )
{
 return m_qhy6_obj->open_device();
}


int cio_driver_qhy6::close_device( void )
{
	m_qhy6_obj->close_device();

 return 0;
}


void cio_driver_qhy6::write_data( unsigned int dByte )
{
	int direction = 0;
	bool inverse_ra = false;
	bool inverse_dec = false;
	int ra_inc_pos = 0;
	int ra_dec_pos = 0;
	int dec_inc_pos = 0;
	int dec_dec_pos = 0;

	for( int i = 0;i < 8;i++ )
	{
		 if( bit_actions[i] == RA_INC_DIR )
			 ra_inc_pos = i;
		 if( bit_actions[i] == RA_DEC_DIR )
		 	 ra_dec_pos = i;
		 if( bit_actions[i] == DEC_INC_DIR )
		 	 dec_inc_pos = i;
		 if( bit_actions[i] == DEC_DEC_DIR )
		 	 dec_dec_pos = i;
	}

	inverse_ra = ra_inc_pos > ra_dec_pos;
	inverse_dec = dec_inc_pos > dec_dec_pos;

	u_char mapped = bit_map_encoder[ (u_char)dByte ];

	if( DBG_VERBOSITY )
		log_i("mapped = %d", (int)mapped);

	for( int i = 0;i < 8;i++ )
	{
		u_char mask = (1 << i);
		if( (mapped & mask) && bit_actions[i] != NO_DIR )
		{
			switch( bit_actions[i] )
			{
			case RA_INC_DIR:
				direction |= (inverse_ra ? QHY6_WEST : QHY6_EAST);
				break;
			case RA_DEC_DIR:
				direction |= (inverse_ra ? QHY6_EAST : QHY6_WEST);
				break;
			case DEC_INC_DIR:
				direction |= (inverse_dec ? QHY6_SOUTH : QHY6_NORTH);
				break;
			case DEC_DEC_DIR:
				direction |= (inverse_dec ? QHY6_NORTH : QHY6_SOUTH);
				break;
			default:
				;
			}
		}
	}

	if( !initialized )
		return;

	if( direction )
		m_qhy6_obj->guide( direction, max_pulse_length );
	else
		m_qhy6_obj->guide( 0, 0 );
}


u_char cio_driver_qhy6::read_byte( void )
{
 return 0;
}

}
