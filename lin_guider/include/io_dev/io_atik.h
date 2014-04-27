/*
 * io_atik.h
 *
 *  Created on: april 2014
 *      Author: Rumen G.Bogdanovski
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

#ifndef IO_ATIK_H_
#define IO_ATIK_H_

#include <atik_core.h>
#include "io_driver.h"

namespace io_drv
{

class cio_driver_atik : public atik_core, public cio_driver_base
{
public:
	cio_driver_atik();
	virtual ~cio_driver_atik();

private:
	virtual int open_device( void );				// open device
	virtual int close_device( void );				// close dvice
	virtual void write_data( unsigned int dByte );	// write data to control telescope
	virtual u_char read_byte( void );				// not implemented
};

}

#endif /* IO_ATIK_H_ */
