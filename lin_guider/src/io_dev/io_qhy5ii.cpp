/*
 * io_qhy5ii.cpp
 *
 *  Created on: 10.10.2013
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

#include "io_qhy5ii.h"
#include "maindef.h"
#include "utils.h"

namespace io_drv
{

//----------------------------------------
//
// Derived.... QHY5II
//
//----------------------------------------
cio_driver_qhy5ii::cio_driver_qhy5ii()
{
	device_type = DT_QHY5II;
	min_pulse_length = 0;
	max_pulse_length = 0xFFFF;

	m_qhy5ii_obj = new qhy5ii_core_shared();

	set_bit_map_template( device_bit_map_template[DT_QHY5II-1] );
}


cio_driver_qhy5ii::~cio_driver_qhy5ii()
{
	//MUST BE called the first: stop thread and release resources
	stop();

	delete m_qhy5ii_obj;
}


int cio_driver_qhy5ii::open_device( void )
{
	return m_qhy5ii_obj->open_device();
}


int cio_driver_qhy5ii::close_device( void )
{
	m_qhy5ii_obj->close_device();

	return EXIT_SUCCESS;
}


void cio_driver_qhy5ii::write_data( unsigned int dByte )
{
	int direction = NO_DIR;
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
				direction |= (inverse_ra ? QHY5II_RA_DEC : QHY5II_RA_INC);
				break;
			case RA_DEC_DIR:
				direction |= (inverse_ra ? QHY5II_RA_INC : QHY5II_RA_DEC);
				break;
			case DEC_INC_DIR:
				direction |= (inverse_dec ? QHY5II_DEC_DEC : QHY5II_DEC_INC);
				break;
			case DEC_DEC_DIR:
				direction |= (inverse_dec ? QHY5II_DEC_INC : QHY5II_DEC_DEC);
				break;
			default:
				;
			}
		}
	}

	if( !initialized )
		return;

	m_qhy5ii_obj->guide( direction );
}


u_char cio_driver_qhy5ii::read_byte( void )
{
	return 0;
}

}
