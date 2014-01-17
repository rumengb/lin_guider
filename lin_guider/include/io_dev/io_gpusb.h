/*
 * io_gpusb.h
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

#ifndef IO_GPUSB_H_
#define IO_GPUSB_H_

#include <stdint.h>
#include <libusb-1.0/libusb.h>

#include "io_driver.h"


namespace io_drv
{

class cio_driver_gpusb : public cio_driver_base
{
	union ctrl_byte
	{
		struct
		{
			unsigned char RA_DEC : 1;
			unsigned char RA_INC : 1;
			unsigned char DEC_DEC : 1;
			unsigned char DEC_INC : 1;
			unsigned char LED_dbl_axes : 1;
			unsigned char LED_enable : 1;
			unsigned char res1 : 1;
			unsigned char res2 : 1;
			} bits;
		unsigned char byte;
	};
	enum GPUSB_STUFF
	{
		RA_DEC  = 0x1,
		RA_INC  = 0x2,
		DEC_DEC = 0x4,
		DEC_INC = 0x8,

		LED_dbl_axes = 0x10,
		LED_enable   = 0x20,

		SAFE_MASK = 0x3F
	};
public:
	cio_driver_gpusb();
	virtual ~cio_driver_gpusb();

private:
	virtual int  open_device( void );				// open device
	virtual int  close_device( void );				// close dvice
	virtual void write_data( unsigned int dByte );	// write data to control telescope
	virtual u_char read_byte( void );				// read byte from gpusb box

	libusb_device_handle *locate_and_open_device( unsigned int vid, unsigned int pid );

	struct libusb_device_handle *m_handle;
};

}

#endif /* IO_GPUSB_H_ */
