/*
 * io_atik.cpp
 *
 *  Created on: 29.01.2015
 *      Author: Rumen G.Bogdanovski
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

#include "io_sx.h"
#include "sxccdusb.h"
#include "maindef.h"
#include "utils.h"
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>

namespace io_drv {

//----------------------------------------
//
// Derived.... sx
//
//----------------------------------------
cio_driver_sx::cio_driver_sx() {
	device_type = DT_SX;
	set_bit_map_template( device_bit_map_template[DT_SX-1] );
	min_pulse_length = 0;
	max_pulse_length = 0xFFFFFF;
}


cio_driver_sx::~cio_driver_sx() {

}


int cio_driver_sx::open_device( void ) {

	if (DBG_VERBOSITY) log_i("%s", __FUNCTION__);

	int res = open();
	if (res) return res;

	if((!m_caps.extra_caps) & SXCCD_CAPS_GUIDER) {
		log_e("This Starlight Xpress camera does not have a guider port.");
		return 1;
	} else {
		log_i("Starlight Xpress guider port initialized.");
	}

	return 0;
}


int cio_driver_sx::close_device( void ) {
	return close();
}


void cio_driver_sx::write_data( unsigned int dByte ) {
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
				direction |= (inverse_ra ? SX_GUIDE_WEST : SX_GUIDE_EAST);
				break;
			case RA_DEC_DIR:
				direction |= (inverse_ra ? SX_GUIDE_EAST : SX_GUIDE_WEST);
				break;
			case DEC_INC_DIR:
				direction |= (inverse_dec ? SX_GUIDE_SOUTH : SX_GUIDE_NORTH);
				break;
			case DEC_DEC_DIR:
				direction |= (inverse_dec ? SX_GUIDE_NORTH : SX_GUIDE_SOUTH);
				break;
			default:
				;
			}
		}
	}

	if( !initialized )
		return;

	if( DBG_VERBOSITY )
		log_i("direction = 0x%X", direction);

	set_guide_relays( direction );
}


u_char cio_driver_sx::read_byte( void ) {

	if (DBG_VERBOSITY) log_i("%s", __FUNCTION__);
	return 0;
}

}
