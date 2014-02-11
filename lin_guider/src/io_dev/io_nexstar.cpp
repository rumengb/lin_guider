/*
 * io_nexstar.cpp
 *
 *  Created on: 01.10.2013
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

#include "io_nexstar.h"
#include "maindef.h"
#include "utils.h"
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <dlfcn.h>

namespace io_drv {

//----------------------------------------
//
// Derived.... nexstar
//
//----------------------------------------
cio_driver_nexstar::cio_driver_nexstar( bool stub ) :
	m_dev( -1 ),
	m_stub_mode( stub ),
	m_guide_rate( SR_HALF_SIDEREAL ),
	m_nexstar_handle( NULL )
{
	device_type = DT_NEXSTAR;

	const char *sym_error = NULL;

	if (DBG_VERBOSITY)
		log_i("%s, stub=%d", __FUNCTION__, stub);

	do
	{
		// open library
		m_nexstar_handle = dlopen("libnexstar.so", RTLD_LAZY);
		if (!m_nexstar_handle) {
			log_e("Cannot open library: %s\n", dlerror());
			return;
		}

		// reset errors
		dlerror();

		// Load symbols
		open_telescope = (int (*)(char*))dlsym(m_nexstar_handle, "open_telescope");
		sym_error = dlerror();
		if (sym_error)
			break;

		tc_get_model = (int (*)(int))dlsym(m_nexstar_handle, "tc_get_model");
		sym_error = dlerror();
		if (sym_error)
			break;

		tc_slew_fixed = (int (*)(int, char, char, char))dlsym(m_nexstar_handle, "tc_slew_fixed");
		sym_error = dlerror();
		if (sym_error)
			break;

		get_model_name = (char* (*)(int, char*, int))dlsym(m_nexstar_handle, "get_model_name");
		sym_error = dlerror();
		if (sym_error)
			break;

		// OK
		return;
	}while( 0 );

	// handle errors
	log_e("Cannot load symbol: %s", sym_error);
	dlclose(m_nexstar_handle);
	m_nexstar_handle = NULL;
}


cio_driver_nexstar::~cio_driver_nexstar() {

	const char *dl_error = NULL;

	if (DBG_VERBOSITY) log_i("%s", __FUNCTION__);
	
	stop();

	if (m_nexstar_handle) return;

	dlclose(m_nexstar_handle);
	dl_error = dlerror();
	if (dl_error) log_e( "Cannot close lib %s", dl_error );
}


int cio_driver_nexstar::open_device( void ) {

	if (DBG_VERBOSITY) log_i("%s", __FUNCTION__);

	if (!m_nexstar_handle) return -1;

	// open tty port to the mount
	m_dev = open_telescope(dev_name);
	if (m_dev == -1) {
		log_e("open_telescope(): failed to open %s (errno=%d)",  dev_name, errno);
		return 1;
	}
	
	// check if there is an actual mount on the other end
	int mount_id = tc_get_model(m_dev);
	if (mount_id < 0) {
		log_e("tc_get_model(): mount not found (errno=%d)", errno);
		return 1;
	}
	if (DBG_VERBOSITY) {
		char mount_name[30];
		get_model_name(mount_id, mount_name, 30);
		log_i("Mount: %s (id=%d)", mount_name, mount_id);
	}

	set_bit_map_template( device_bit_map_template[DT_NEXSTAR-1] );
	return 0;
}


int cio_driver_nexstar::close_device( void ) {

	if (DBG_VERBOSITY) log_i("%s", __FUNCTION__);

	if (!m_nexstar_handle) return -1;

	if (m_dev < 0) return 0;

	close_telescope(m_dev);
	m_dev = -1;

	return 0;
}


void cio_driver_nexstar::write_data( unsigned int dByte ) {

	u_char mapped;
	u_char mask;

	if (!m_nexstar_handle) return;
	
	// If mount is not initialized do nothing
	if (m_dev < 0) return;

	int ra_dir = TC_DIR_POSITIVE;
	int de_dir = TC_DIR_POSITIVE;
	int ra_axis = 0;
	int de_axis = 0;

	int ra_inc_pos = 0;
	int ra_dec_pos = 0;
	int dec_inc_pos = 0;
	int dec_dec_pos = 0;

	for( int i = 0;i < 8;i++ ) {
		 if( bit_actions[i] == RA_INC_DIR )
			 ra_inc_pos = i;
		 if( bit_actions[i] == RA_DEC_DIR )
		 	 ra_dec_pos = i;
		 if( bit_actions[i] == DEC_INC_DIR )
		 	 dec_inc_pos = i;
		 if( bit_actions[i] == DEC_DEC_DIR )
		 	 dec_dec_pos = i;
	}

	mapped = bit_map_encoder[ (u_char)dByte ];
	if(DBG_VERBOSITY) log_i("mapped = %x", (int)mapped);

	for( int i = 0;i < 8;i++ ) {
		mask = (1 << i);
		if( (mapped & mask) && bit_actions[i] != NO_DIR ) {
			switch( bit_actions[i] ) {
			case RA_INC_DIR:
				ra_dir = (ra_inc_pos < ra_dec_pos) ? TC_DIR_POSITIVE : TC_DIR_NEGATIVE;
				ra_axis = 1;
				break;
			case RA_DEC_DIR:
				ra_dir = (ra_inc_pos < ra_dec_pos) ? TC_DIR_NEGATIVE : TC_DIR_POSITIVE;
				ra_axis = 1;
				break;
			case DEC_INC_DIR:
				de_dir = (dec_inc_pos < dec_dec_pos) ? TC_DIR_POSITIVE : TC_DIR_NEGATIVE;
				de_axis = 1;
				break;
			case DEC_DEC_DIR:
				de_dir = (dec_inc_pos < dec_dec_pos) ? TC_DIR_NEGATIVE : TC_DIR_POSITIVE;
				de_axis = 1;
				break;
			default:
				;
			}
		}
	}
	
	// Stop all
	if (tc_slew_fixed(m_dev, TC_AXIS_RA, ra_dir, SR_STOP)) {
		log_i("tc_slew_fixed(): returned error (errno=%d)", errno);
	}
	if (tc_slew_fixed(m_dev, TC_AXIS_DE, de_dir, SR_STOP))
		log_i("tc_slew_fixed(): returned error (errno=%d)", errno);

	// Do corrections
	if (ra_axis) {
		if(tc_slew_fixed(m_dev, TC_AXIS_RA, ra_dir, m_guide_rate))
			log_i("tc_slew_fixed(): returned error (errno=%d)", errno);
		if( DBG_VERBOSITY )
			log_i("%s: tc_slew_fixed(dev, TC_AXIS_RA, %d, %d);", __FUNCTION__ , ra_dir, m_guide_rate);
	}
	if (de_axis) {
		if(tc_slew_fixed(m_dev, TC_AXIS_DE, de_dir, m_guide_rate))
			log_i("tc_slew_fixed(): returned error (errno=%d)", errno);
		if( DBG_VERBOSITY )
			log_i("%s: tc_slew_fixed(dev, TC_AXIS_DE, %d, %d);", __FUNCTION__ , de_dir, m_guide_rate);
	}
}


u_char cio_driver_nexstar::read_byte( void ) {

	if (DBG_VERBOSITY) log_i("%s", __FUNCTION__);

	return 0;
}

}
