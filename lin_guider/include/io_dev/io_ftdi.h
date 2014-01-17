/*
 * io_ftdi.h
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

#ifndef IO_FTDI_H_
#define IO_FTDI_H_

#include "io_driver.h"

namespace io_drv
{

class cio_driver_ftdi : public cio_driver_base
{
	// FTDI chip type
	enum ftdi_chip_type { TYPE_AM=0, TYPE_BM=1, TYPE_2232C=2, TYPE_R=3 };
public:
	cio_driver_ftdi();
	virtual ~cio_driver_ftdi();

private:
	virtual int  open_device( void );				// open device
	virtual int  close_device( void );				// close dvice
	virtual void write_data( unsigned int dByte );	// write data to control telescope
	virtual u_char read_byte( void );				// not implemented

	bool lib_initialized;
	void *lib_handle;

	ftdi_chip_type chip_type;

	// importing functions
	int (*lib_init)( int *chp_type );
	int (*lib_release)( void );
	int (*lib_write_data)( unsigned int dByte );

};

}

#endif /* IO_FTDI_H_ */
