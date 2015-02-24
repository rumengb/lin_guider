/*
 * io_asi.cpp
 *
 *  Created on: 23 february 2015
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

#include "io_asi.h"
#include "maindef.h"
#include "utils.h"
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>

namespace io_drv {

//----------------------------------------
//
// Derived.... ASI
//
//----------------------------------------
cio_driver_asi::cio_driver_asi() {
	device_type = DT_ASI;
	set_bit_map_template( device_bit_map_template[DT_ASI-1] );
	min_pulse_length = 0;
	max_pulse_length = 0xFFFFFF;
}


cio_driver_asi::~cio_driver_asi() {

}


int cio_driver_asi::open_device( void ) {

	if (DBG_VERBOSITY) log_i("%s", __FUNCTION__);

	int res = open();
	if (res) return res;

	if(m_cam_info.ST4Port != ASI_TRUE) {
		log_e("This ASI camera does not have a guider port.");
		return 1;
	} else {
		log_i("ASI ST4 port initialized.");
	}

	return 0;
}


int cio_driver_asi::close_device( void ) {
	return close();
}


void cio_driver_asi::write_data( unsigned int dByte ) {
	bool inverse_ra = false;
	bool inverse_dec = false;
	int ra_inc_pos = 0;
	int ra_dec_pos = 0;
	int dec_inc_pos = 0;
	int dec_dec_pos = 0;

	ASI_GUIDE_DIRECTION ra_dir = ASI_GUIDE_NORTH;
	ASI_GUIDE_DIRECTION de_dir = ASI_GUIDE_NORTH;
	int ra_axis = 0;
	int de_axis = 0;

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
				ra_dir = (inverse_ra ? ASI_GUIDE_WEST : ASI_GUIDE_EAST);
				ra_axis = 1;
				break;
			case RA_DEC_DIR:
				ra_dir = (inverse_ra ? ASI_GUIDE_EAST : ASI_GUIDE_WEST);
				ra_axis = 1;
				break;
			case DEC_INC_DIR:
				de_dir = (inverse_dec ? ASI_GUIDE_SOUTH : ASI_GUIDE_NORTH);
				de_axis = 1;
				break;
			case DEC_DEC_DIR:
				de_dir = (inverse_dec ? ASI_GUIDE_NORTH : ASI_GUIDE_SOUTH);
				de_axis = 1;
				break;
			default:
				;
			}
		}
	}

	if( !initialized )
		return;

	ASI_ERROR_CODE result;
	lock();
	// Stop all
	result = pASIPulseGuideOff(m_camera, ASI_GUIDE_NORTH);
	if (result)
		log_e("ASIPulseGuideOff(): result = %d", result);

	result = pASIPulseGuideOff(m_camera, ASI_GUIDE_SOUTH);
	if (result)
		log_e("ASIPulseGuideOff(): result = %d", result);

	result = pASIPulseGuideOff(m_camera, ASI_GUIDE_EAST);
	if (result)
		log_e("ASIPulseGuideOff(): result = %d", result);

	result = pASIPulseGuideOff(m_camera, ASI_GUIDE_WEST);
	if (result)
		log_e("ASIPulseGuideOff(): result = %d", result);

	// Do corrections
	if (ra_axis) {
		if( DBG_VERBOSITY )
			log_i("guide RA = %d", ra_dir);

		result = pASIPulseGuideOn(m_camera, ra_dir);
		if (result)
			log_e("ASIPulseGuideOn(): result = %d", result);
	}

	if (de_axis) {
		if( DBG_VERBOSITY )
			log_i("guide DE = %d", de_dir);

		result = pASIPulseGuideOn(m_camera, de_dir);
		if (result)
			log_e("ASIPulseGuideOn(): result = %d", result);
	}
	unlock();
}


u_char cio_driver_asi::read_byte( void ) {

	if (DBG_VERBOSITY) log_i("%s", __FUNCTION__);
	return 0;
}

}
