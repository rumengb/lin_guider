/*
 * io_qhy5.cpp
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

#include "io_qhy5.h"
#include "maindef.h"
#include "utils.h"

namespace io_drv
{

//----------------------------------------
//
// Derived.... QHY5
//
//----------------------------------------
cio_driver_qhy5::cio_driver_qhy5()
{
	device_type = DT_QHY5;
	min_pulse_length = 0;
	max_pulse_length = 0xFFFF;

	m_qhy5_obj = new qhy5_core_shared();

	set_bit_map_template( device_bit_map_template[DT_QHY5-1] );
}


cio_driver_qhy5::~cio_driver_qhy5()
{
	//MUST BE called the first: stop thread and release resources
	stop();

	delete m_qhy5_obj;
}


int cio_driver_qhy5::open_device( void )
{
 return m_qhy5_obj->open_device();
}


int cio_driver_qhy5::close_device( void )
{
	m_qhy5_obj->close_device();

 return 0;
}


void cio_driver_qhy5::write_data( unsigned int dByte )
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
		log_i("mapped =%d", (int)mapped);

	for( int i = 0;i < 8;i++ )
	{
		u_char mask = (1 << i);
		if( (mapped & mask) && bit_actions[i] != NO_DIR )
		{
			switch( bit_actions[i] )
			{
			case RA_INC_DIR:
				direction |= (inverse_ra ? QHY5_WEST : QHY5_EAST);
				break;
			case RA_DEC_DIR:
				direction |= (inverse_ra ? QHY5_EAST : QHY5_WEST);
				break;
			case DEC_INC_DIR:
				direction |= (inverse_dec ? QHY5_SOUTH : QHY5_NORTH);
				break;
			case DEC_DEC_DIR:
				direction |= (inverse_dec ? QHY5_NORTH : QHY5_SOUTH);
				break;
			default:
				;
			}
		}
	}

	if( !initialized )
		return;

	// stop all
	m_qhy5_obj->guide( QHY5_NORTH | QHY5_SOUTH | QHY5_EAST | QHY5_WEST, 0 );

	if( direction )
		m_qhy5_obj->guide( direction, max_pulse_length );
}


u_char cio_driver_qhy5::read_byte( void )
{
 return 0;
}

}
