/*
 * qhy5ii_core.cpp
 *
 *  Created on: 10.10.2013
 *      Author: gm
 *
 * Device access code is based on original QHY code from https://github.com/qhyccd-lzr
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <algorithm>

#include "maindef.h"
#include "utils.h"
#include "lusb.h"
#include "qhy5ii_core.h"


// disables hardware access for QHY5II
// #define NO_QHY5II

pthread_mutex_t qhy5ii_core_shared::m_mutex = PTHREAD_MUTEX_INITIALIZER;
libusb_device_handle *qhy5ii_core_shared::m_handle = NULL;
int qhy5ii_core_shared::m_init_cnt = 0;


qhy5ii_core_shared::qhy5ii_core_shared( void )
{
	int ret = lusb::initialize();

	if( ret != 0 )
		log_e( "qhy5ii_core_shared::qhy5ii_core_shared(): Could not initialize libusb" );
}


qhy5ii_core_shared::~qhy5ii_core_shared( void )
{
	lusb::release();
}


int qhy5ii_core_shared::open_device( void )
{
    int res = EXIT_SUCCESS;

    pthread_mutex_lock( &m_mutex );

#ifdef NO_QHY5II
    if( m_init_cnt == 0 && lusb::is_initialized() )
        log_i( "cvideo_qhy5ii::open_device(): Success" );
#else
    if( m_handle == NULL && m_init_cnt == 0 && lusb::is_initialized() )
    {
        if( (m_handle = locate_and_open_device( QHY5II_VENDOR_ID, QHY5II_PRODUCT_ID)) == NULL )
        {
            log_e( "cvideo_qhy5ii::open_device(): Could not open the QHY5II device" );
            res = EXIT_FAILURE;
        }
        else
        {
            log_i( "qhy5ii successfully opened." );
        }
    }
#endif
    if( res == EXIT_SUCCESS )
        m_init_cnt++;

    pthread_mutex_unlock( &m_mutex );

    return res;
}


void qhy5ii_core_shared::close_device( void )
{
 	pthread_mutex_lock( &m_mutex );

	if( m_init_cnt <= 0 )
	{
		log_e( "qhy5ii_core_shared::close_device(): Already closed or not opened yet" );
	}
	else
	{
		m_init_cnt--;
#ifdef NO_QHY5II
		if( m_init_cnt == 0 )
			log_i( "qhy5ii_core_shared::close_device(): Success" );
#else
		if( m_handle && m_init_cnt == 0 )
		{
			libusb_release_interface( m_handle, 0 );
			libusb_close( m_handle );
			m_handle = NULL;
			log_i( "qhy5ii_core_shared::close_device(): Success" );
		}
#endif
	}

	pthread_mutex_unlock( &m_mutex );
}


libusb_device_handle *qhy5ii_core_shared::locate_and_open_device( unsigned int vid, unsigned int pid )
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


void qhy5ii_core_shared::lock( void )
{
	pthread_mutex_lock( &m_mutex );
}


void qhy5ii_core_shared::unlock( void )
{
	pthread_mutex_unlock( &m_mutex );
}


int qhy5ii_core_shared::ctrl_msg( unsigned char request_type, unsigned char request, unsigned int value, unsigned int index, unsigned char *data, unsigned char len )
{
#ifdef NO_QHY5II
	if( m_handle == NULL )
		return EXIT_SUCCESS;
#endif

	assert( m_handle != NULL );

	int result = libusb_control_transfer( m_handle, request_type, request, value, index, data, len, 5000 );

	return result < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}


int qhy5ii_core_shared::eeprom_read( unsigned char addr, unsigned char* data, unsigned short len )
{
#ifdef NO_QHY5II
	if( m_handle == NULL )
		return EXIT_SUCCESS;
#endif

	assert( m_handle != NULL );

	int result = ctrl_msg( QHYCCD_REQUEST_READ, 0xCA, 0, addr, data, len );

	return result < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}


int qhy5ii_core_shared::get_dev_info( int *dev_type, bool *is_color )
{
#ifdef NO_QHY5II
	if( m_handle == NULL )
	{
		*dev_type = DEVICETYPE_QHY5LII;
		*is_color = false;
		return EXIT_SUCCESS;
	}
#endif

	assert( m_handle != NULL );

	unsigned char buf[16];
	if( eeprom_read( 0x10, buf, 16 ) != EXIT_SUCCESS )
	{
		log_e( "qhy5ii_core_shared::eeprom_read(): failed" );
		return EXIT_FAILURE;
	}
	if( buf[1] == 1 )
		*is_color = true;
	else
		*is_color = false;

	if( buf[0] == 6 )
	{
		unsigned char buf[4] = { 0, 0, 0, 0 };
		ctrl_msg( QHYCCD_REQUEST_WRITE, 0xc1, 0, 0, buf, 4 );

		*dev_type = DEVICETYPE_QHY5LII;
		log_i( "Detected DEVICETYPE_QHY5LII" );
	}
	else
	if( buf[0] == 1 )
	{
		*dev_type = DEVICETYPE_QHY5II;
		log_i( "Detected DEVICETYPE_QHY5II" );
	}
	else
		*dev_type = DEVICETYPE_UNKOWN;
	return EXIT_SUCCESS;
}


int qhy5ii_core_shared::get_frame( unsigned char *data, unsigned int data_size, unsigned int exposure_tout )
{
#ifdef NO_QHY5II
	if( m_handle == NULL )
		return EXIT_SUCCESS;
#endif

	assert( m_handle != NULL );

	int transfered = 0;
	int try_cnt = 0;
#if 1
	int pos = 0;
	int to_read = data_size + 5;

	while( to_read )
	{
		int ret = libusb_bulk_transfer( m_handle, QHYCCD_DATA_READ_ENDPOINT, data + pos,
			to_read, &transfered, (int)exposure_tout + 1500);

		if( ret != LIBUSB_SUCCESS )
		{
			if ( DBG_VERBOSITY )
				log_i("Retrying frame! read: %d, ret: %d.", transfered, ret);

			if( try_cnt > 3 )
			{
				log_e("Frame Failed! bytes read: %d, ret: %d.", transfered, ret);
				return EXIT_FAILURE;
			}
			try_cnt++;
                        continue;
		}

		pos += transfered;
		to_read -= transfered;

		/* Here we are using the pattern as a frame delimiter. If we still have bytes
		   to read and the pattern is found then the frames are missalined and we are at
		   the end of the previous framefram We have to start agin.
		*/
		unsigned char pat[4] = {0xaa, 0x11, 0xcc, 0xee};
		void *ppat = memmem(data+pos-5, 4, pat, 4);

		if ((to_read) && (ppat))
		{
			if( DBG_VERBOSITY )
				log_i("#### Aligning frame, pos=%d, to_read=%d.", pos, to_read);
			pos = 0;
			to_read = data_size + 5;
			continue;
		}
		/* If by accident to_read is 0 and we are not at the end of the frame
		   we have missed the alignment pattern, so look for the next one.
		*/
		if ((to_read <= 0) && (ppat == NULL))
		{
			if ( DBG_VERBOSITY )
				log_i("Frame seems to be invalid, retrying!");

			if( try_cnt > 3 )
			{
				log_e("Frame Failed - no pattern found!");
				return EXIT_FAILURE;
			}
			pos = 0;
			to_read = data_size + 5;
			try_cnt++;
			continue;
		}

//		if( DBG_VERBOSITY )
//			log_i("Read: %d of %d, try_cnt=%d, result=%d.", transfered, to_read + transfered, try_cnt, ret);
	}

	if( DBG_VERBOSITY )
		log_i( "Frame succeeded: %d bytes", pos );

	return EXIT_SUCCESS;
}
#else
	{
		data_size += 5;
		while( try_cnt < 5 )
		{
			int ret = libusb_bulk_transfer( m_handle, QHYCCD_DATA_READ_ENDPOINT, data, data_size, &transfered, std::max((int)exposure_tout+1500, 2000) );
			if( transfered != (int)data_size )
				try_cnt++;
			else
				break;

			if( ret != LIBUSB_SUCCESS )
				log_e("code: %d, %s", ret, strerror(errno));
		}
		if( try_cnt >= 5 )
		{
			log_e("image failed");
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}
#endif


int qhy5ii_core_shared::guide( int direction )
{
#ifdef NO_QHY5II
	if( m_handle == NULL )
		return EXIT_SUCCESS;
#endif

	assert( m_handle != NULL );

	int ret = EXIT_SUCCESS;

	//log_i("direction = 0x%.2X", direction );

	pthread_mutex_lock( &m_mutex );

	unsigned char buf[2] = { 0, 0 };

	if( direction & (QHY5II_RA_INC | QHY5II_RA_DEC) ) // RA axis
	{
		if( direction & QHY5II_RA_INC ) // ra+
			ctrl_msg( QHYCCD_REQUEST_WRITE, 0xc0, 0x0001, QHY5II_RA_INC, buf, 2 );
		else // ra-
			ctrl_msg( QHYCCD_REQUEST_WRITE, 0xc0, 0x0001, QHY5II_RA_DEC, buf, 2 );
	}
	else
		ctrl_msg( QHYCCD_REQUEST_WRITE, 0xc0, 0x0001, 0x0000, buf, 2 );

	if( direction & (QHY5II_DEC_INC | QHY5II_DEC_DEC) ) // DEC axis
	{
		if( direction & QHY5II_DEC_INC ) // dec+
			ctrl_msg( QHYCCD_REQUEST_WRITE, 0xc0, 0x0002, QHY5II_DEC_INC, buf, 2 );
		else // dec-
			ctrl_msg( QHYCCD_REQUEST_WRITE, 0xc0, 0x0002, QHY5II_DEC_DEC, buf, 2 );
	}
	else
		ctrl_msg( QHYCCD_REQUEST_WRITE, 0xc0, 0x0002, 0x0000, buf, 2 );

	pthread_mutex_unlock( &m_mutex );

	return ret;
}

