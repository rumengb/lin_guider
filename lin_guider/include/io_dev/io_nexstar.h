/*
 * io_nexstar.h
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

#ifndef IO_NEXSTAR_H_
#define IO_NEXSTAR_H_

#include "io_driver.h"


#define TC_DIR_POSITIVE 1
#define TC_DIR_NEGATIVE 0

#define TC_AXIS_RA 1
#define TC_AXIS_DE 0

#define close_telescope(dev_fd) (close(dev_fd))

namespace io_drv {

class cio_driver_nexstar : public cio_driver_base {

public:
	cio_driver_nexstar( bool stub = false );
	virtual ~cio_driver_nexstar();

private:
	/* For Celestron mounts only fixed slew rates
	   1 (sidereal_rate/2) and 2 (sidereal_rate) can
	   be used for guiding. Slew rate 0 stops the slew.
	*/
	enum slew_rate {
		SR_STOP = 0,
		SR_HALF_SIDEREAL,
		SR_SIDEREAL
	};

	int  m_dev;
	bool m_stub_mode;
	enum slew_rate m_guide_rate;

	virtual int open_device( void );		// open device
	virtual int close_device( void );		// close dvice
	virtual void write_data( unsigned int dByte );	// write data to control telescope
	virtual u_char read_byte( void );		// not implemented

	// libnexstar stuff
	void *m_nexstar_handle;
	int (*open_telescope)(char *dev_file);
	int (*tc_get_model)(int dev);
	int (*tc_slew_fixed)(int dev, char axis, char direction, char rate);
	char* (*get_model_name)(int id, char *name, int len);
};

}

#endif /* IO_NEXSTAR_H_ */
