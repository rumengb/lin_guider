/*
 * qhy5_core.h
 *
 *  Created on: 10.02.2011
 *      Author: gm
 *
 * Original code: Copyright(c) 2009 Geoffrey Hausheer.
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

#ifndef QHY5_CORE_H_
#define QHY5_CORE_H_

#include <pthread.h>
#include <libusb-1.0/libusb.h>

#define QHY5_VENDOR_ID   0x16c0
#define QHY5_PRODUCT_ID  0x296d

enum {
        QHY5_NORTH = 0x20,
        QHY5_SOUTH = 0x40,
        QHY5_EAST  = 0x10,
        QHY5_WEST  = 0x80,
};


#define QHY5_MATRIX_WIDTH	1558
#define QHY5_MATRIX_HEIGHT  1048

#define QHY5_IMAGE_WIDTH	1280
#define QHY5_IMAGE_HEIGHT	1024


#define STORE_WORD_BE(var, val) *(var) = ((val) >> 8) & 0xff; *((var) + 1) = (val) & 0xff

class qhy5_core_shared
{
public:
	qhy5_core_shared();
	virtual ~qhy5_core_shared();

	int open_device( void );		// open device
	void close_device( void );		// close device

	void reset_camera( void );
	int start_exposure( unsigned int exposure );
	int read_exposure( unsigned char *image, unsigned int image_size );
	int set_size( int height, int gain, int first_time );
	int guide( int direction, int duration_msec );
private:
	int ctrl_msg( unsigned char request_type, unsigned char request, unsigned int value, unsigned int index, unsigned char *data, unsigned char len );
	libusb_device_handle *locate_and_open_device( unsigned int vid, unsigned int pid );

	// shared static data
	static pthread_mutex_t m_mutex;
	static struct libusb_device_handle *m_handle;
	static int m_init_cnt;
	static int gain_lut[74];
};


#endif /* QHY5_CORE_H_ */
