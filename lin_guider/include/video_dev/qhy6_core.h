/*
 * qhy6_core.h
 *
 *  Created on: 02.06.2011
 *      Author: gm
 *
 *  Original code by Tom Vandeneede
 *  Original code for ST-4 port by Vladimir Volynsky
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

#ifndef QHY6_CORE_H_
#define QHY6_CORE_H_

#include <pthread.h>
#include <libusb-1.0/libusb.h>


enum {
        QHY6_NORTH = 0x04,
        QHY6_SOUTH = 0x02,
        QHY6_EAST  = 0x01,
        QHY6_WEST  = 0x08,
};

#define QHY6_VENDOR_ID   0x1618
#define QHY6_PRODUCT_ID  0x025A

#define QHY6_MATRIX_WIDTH	800
#define QHY6_MATRIX_HEIGHT	596

#define QHY6_MATRIX_WIDTH_B2	400
#define QHY6_MATRIX_HEIGHT_B2	298

#define QHY6_SKIP_X	34
#define QHY6_SKIP_Y	14

#define QHY6_SKIP_X_B2	22
#define QHY6_SKIP_Y_B2	7

#define QHY6_EFFECTIVE_WIDTH	752
#define QHY6_EFFECTIVE_HEIGHT	582

#define QHY6_EFFECTIVE_WIDTH_B2 	375  // QHY6_EFFECTIVE_WIDTH / 2 - 1
#define QHY6_EFFECTIVE_HEIGHT_B2	290 // QHY6_EFFECTIVE_HEIGHT / 2 - 1

#define QHY6_PIXEL_WIDTH	6.50  // um
#define QHY6_PIXEL_HEIGHT	6.25  // um

#define V4L2_CID_USER_BLACK_POINT (V4L2_CID_USER_CLASS + 1)

//#define QHY6_BUFFER_LEN 480000	// 800*600


class qhy6_core_shared
{
public:
	qhy6_core_shared();
	virtual ~qhy6_core_shared();

	int open_device( void );		// open device
	void close_device( void );		// close device

	int start_exposure( unsigned int exposure );
	int read_exposure( unsigned char *image, unsigned int image_size );
	int set_params( int exposuretime, int binn, int gain, int offset, int speed, int amp, int vbe, int *out_width, int *out_height, int *out_buffer_size );
	int guide( int direction, int duration_msec );
private:
	int ctrl_msg( int request_type, int request, unsigned int value, unsigned int index, unsigned char *data, int len );
	libusb_device_handle *locate_and_open_device( unsigned int vid, unsigned int pid );
	int do_pulse( int dir, int duration_msec );

	unsigned char MSB( unsigned int i );
	unsigned char LSB( unsigned int i );

	// shared static data
	static pthread_mutex_t m_mutex;
	static struct libusb_device_handle *m_handle;
	static int m_init_cnt;
	//static int gain_lut[74];
	static int m_exposuretime;
	static unsigned m_magic_size; //Why the hell is that ?!?! should read m_magic_size less byted per image
};

#endif /* QHY6_CORE_H_ */
