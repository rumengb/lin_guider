/*
 * io_ftdi.cpp
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

#include <dlfcn.h>

#include "io_ftdi.h"
#include "maindef.h"
#include "utils.h"

namespace io_drv
{

//----------------------------------------
//
// Derived.... FTDI
//
//----------------------------------------
cio_driver_ftdi::cio_driver_ftdi()
{
 const char *dlsym_error;


 	device_type = DT_FTDI;
 	min_pulse_length = 0;
 	max_pulse_length = 0xFFFFFF;

 	lib_initialized = false;

	lib_init 		= NULL;
	lib_release 	= NULL;
	lib_write_data 	= NULL;

	chip_type = TYPE_R;

	// open library
	lib_handle = dlopen( "./libio_ftdi.so", RTLD_LAZY );

	if( !lib_handle )
	{
	    log_e( "Cannot open library: 'libio_ftdi.so' %s\n", dlerror() );
	    return;
	}

	// importing symbols

	// reset errors
	dlerror();
	lib_init = (int (*)(int *))dlsym( lib_handle, "io_ftdi_init" );
	dlsym_error = dlerror();
	if( dlsym_error )
	{
		if( DBG_VERBOSITY )
			log_e( "Cannot load symbol '%s'\n", dlsym_error );
	}

	lib_release = (int (*)())dlsym( lib_handle, "io_ftdi_release" );
	dlsym_error = dlerror();
	if( dlsym_error )
	{
		if( DBG_VERBOSITY )
			log_e( "Cannot load symbol '%s'\n", dlsym_error );
	}

	lib_write_data = (int (*)(unsigned int))dlsym( lib_handle, "io_ftdi_write" );
	dlsym_error = dlerror();
	if( dlsym_error )
	{
		if( DBG_VERBOSITY )
			log_e( "Cannot load symbol '%s'\n", dlsym_error );
	}

	if( !lib_init || !lib_release || !lib_write_data )
	{
		log_e( "Library is unavailable... closing... \n" );
		dlclose( lib_handle );
		lib_handle = NULL;
		return;
	}

	lib_initialized = true;

	log_i( "Library 'libio_ftdi.so' successully initialized...\n" );
}


cio_driver_ftdi::~cio_driver_ftdi()
{
 const char *dl_error;

 	//MUST BE called the first: stop thread and release resources
 	stop();

	if( !lib_handle )
		return ;

	dlclose( lib_handle );
	dl_error = dlerror();
	if( dl_error )
	{
		log_e( "Cannot close lib '%s'\n", dl_error );
	}

	lib_init 		= NULL;
	lib_release 	= NULL;
	lib_write_data 	= NULL;

	lib_initialized = false;

	log_i( "Library 'libio_ftdi.so' released.\n" );
}


int cio_driver_ftdi::open_device( void )
{
 int res = -1;

	if( !lib_initialized )
		return -1;

	chip_type = TYPE_R;

	res = lib_init( (int *)&chip_type );

	if( chip_type == TYPE_R )
		set_bit_map_template( device_bit_map_template[DT_FTDI-1] );
	else
		set_bit_map_template( device_bit_map_template[DT_NULL-1] );


 return res;
}


int cio_driver_ftdi::close_device( void )
{
 int res = -1;

	if( !lib_initialized )
		return -1;

	res = lib_release();

 return res;
}


void cio_driver_ftdi::write_data( unsigned int dByte )
{
 int res;
 unsigned int mapped;

	if( !lib_initialized )
		return;

	mapped = bit_map_encoder[ (u_char)dByte ];

	res = lib_write_data( mapped );

	res = res;
}


u_char cio_driver_ftdi::read_byte( void )
{
 return 0;
}

}
