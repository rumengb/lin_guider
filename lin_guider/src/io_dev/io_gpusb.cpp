/*
 * io_gpusb.cpp
 *
 *  Created on: 06.04.2013
 *      Author: gm
 *
 *  base code and testing by Giampiero Spezzano gspezzano@gmail.com
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

#include <unistd.h>

#include "io_gpusb.h"
#include "lusb.h"
#include "maindef.h"
#include "utils.h"

// disables hardware access for GPUSB
//#define NO_GPUSB

namespace io_drv
{

#define GPUSB_VENDOR_ID   0x134A
#define GPUSB_PRODUCT_ID  0x9020

#define ENDPOINT_INT_IN 0x81
#define ENDPOINT_INT_OUT 0x02

#define DIR_ANY(x)      (x & 0xF)
#define DIR_BOTH(x)     ((x & 0xC) && (x & 0x3))

//----------------------------------------
//
// Derived.... GPUSB
//
//----------------------------------------
cio_driver_gpusb::cio_driver_gpusb() :
	m_handle( NULL )
{
	int ret = lusb::initialize();

	if( ret != 0 )
		log_e( "cio_driver_gpusb::cio_driver_gpusb(): Could not initialize libusb" );

	device_type = DT_GPUSB;
	min_pulse_length = 0;
	max_pulse_length = 0xFFFF;

	set_bit_map_template( device_bit_map_template[DT_GPUSB-1] );
}


cio_driver_gpusb::~cio_driver_gpusb()
{
	//MUST BE called the first: stop thread and release resources
	stop();

	lusb::release();
}


libusb_device_handle *cio_driver_gpusb::locate_and_open_device( unsigned int vid, unsigned int pid )
{
    libusb_device_handle *device_handle = NULL;

    device_handle = libusb_open_device_with_vid_pid( NULL, vid, pid );

    if( device_handle == NULL )
        return NULL;

    if( libusb_kernel_driver_active( device_handle, 0 ) )
        libusb_detach_kernel_driver( device_handle, 0 );

    int open_status = libusb_set_configuration( device_handle, 1 );
    (void)open_status;

    open_status = libusb_claim_interface( device_handle, 0 );

    return device_handle;
}


int cio_driver_gpusb::open_device( void )
{
	int res = EXIT_SUCCESS;

#ifdef NO_GPUSB
	if( lusb::is_initialized() )
		log_i( "cio_driver_gpusb::open_device(): Success" );
#else
	if( m_handle == NULL && lusb::is_initialized() )
	{
		if( (m_handle = locate_and_open_device( GPUSB_VENDOR_ID, GPUSB_PRODUCT_ID)) == NULL )
		{
			log_e( "cio_driver_gpusb::open_device(): Could not open the GPUSB device" );
			res = EXIT_FAILURE;
		}
		else
			log_i( "GPUSB successfully opened." );
	}
#endif
	return res;
}


int cio_driver_gpusb::close_device( void )
{
#ifdef NO_GPUSB
	log_i( "cio_driver_gpusb::close_device(): Success" );
#else
	if( m_handle )
	{
		libusb_release_interface( m_handle, 0 );
		libusb_close( m_handle );
		m_handle = NULL;
		log_i( "cio_driver_gpusb::close_device(): Success" );
	}
#endif
	return 0;
}


void cio_driver_gpusb::write_data( unsigned int dByte )
{
	u_char mapped = bit_map_encoder[ (u_char)dByte ];

	// cut off all invalid
	mapped &= SAFE_MASK;

	if( DBG_VERBOSITY )
		log_i("mapped = %d", (int)mapped);

	{	// set LED
		if( DIR_ANY( mapped ) )
		{
			mapped |= LED_enable;
			if( DIR_BOTH( mapped ) )
				mapped |= LED_dbl_axes;
		}
	}

	{	// test
		if( mapped & LED_enable )
		{
			log_i( "LED ON " );
			if( mapped & LED_dbl_axes )
				log_i( "double (RED)" );
			else
				log_i( "single (GREEN)" );
		} // end of test
	}

	if( !m_handle )
		return;

	int rcode = 0, transferred = 0;
	rcode = libusb_bulk_transfer( m_handle, ENDPOINT_INT_OUT, &mapped, 1, &transferred, 1000 );
	if( rcode != 0 )
		log_e( "cio_driver_gpusb::write_data(): libusb_bulk_transfer() failed ret = %d", rcode );
}


u_char cio_driver_gpusb::read_byte( void )
{
	if( !m_handle )
		return 0;

	int rcode = 0;
	int transferred = 0;
	u_char byte = 0;

	//Must be read twice to get the right value, can't get why but it's recurring when programming libusb1.0
	rcode = libusb_interrupt_transfer( m_handle, ENDPOINT_INT_IN, &byte, 1, &transferred, 1000 );
	rcode = libusb_interrupt_transfer( m_handle, ENDPOINT_INT_IN, &byte, 1, &transferred, 1000 );
	if( rcode != 0 )
		log_e( "cio_driver_gpusb::read_byte(): libusb_interrupt_transfer() error ret = %d", rcode );

	return byte;
}

}
