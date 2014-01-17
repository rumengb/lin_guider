/*
 * qhy5ii_core.h
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

#ifndef QHY5II_CORE_H_
#define QHY5II_CORE_H_

#include <pthread.h>
#include <libusb-1.0/libusb.h>


#define QHY5II_VENDOR_ID   0x1618
#define QHY5II_PRODUCT_ID  0x0921

#define DEVICETYPE_QHY5II  51
#define DEVICETYPE_QHY5LII 56
#define DEVICETYPE_UNKOWN 0

#define QHYCCD_REQUEST_READ  0xC0
#define QHYCCD_REQUEST_WRITE  0x40


#define QHYCCD_INTERRUPT_READ_ENDPOINT 0x81
#define QHYCCD_INTERRUPT_WRITE_ENDPOINT 0x01
#define QHYCCD_DATA_READ_ENDPOINT 0x82

enum
{
	QHY5II_RA_INC  = 0x0080,
	QHY5II_DEC_INC = 0x0040,
	QHY5II_DEC_DEC = 0x0020,
	QHY5II_RA_DEC  = 0x0010
};

class qhy5ii_core_shared
{
public:
	qhy5ii_core_shared();
	virtual ~qhy5ii_core_shared();

	int open_device( void );		// open device
	void close_device( void );		// close device

	int get_dev_info( int *dev_type, bool *is_color );
	int get_frame( unsigned char *data, unsigned int data_size, unsigned int exposure_tout );

	void lock( void );
	void unlock( void );
	int ctrl_msg( unsigned char request_type, unsigned char request, unsigned int value, unsigned int index, unsigned char *data, unsigned char len );

	int guide( int direction );
private:
	libusb_device_handle *locate_and_open_device( unsigned int vid, unsigned int pid );
	int eeprom_read( unsigned char addr, unsigned char* data, unsigned short len );

	// shared static data
	static pthread_mutex_t m_mutex;
	static struct libusb_device_handle *m_handle;
	static int m_init_cnt;
};


#endif /* QHY5II_CORE_H_ */
