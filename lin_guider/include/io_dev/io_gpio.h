/*
 * io_gpio.h
 *
 *  Created on: 04.07.2013
 *      Author: bgilsrud
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

#ifndef IO_GPIO_H_
#define IO_GPIO_H_

#include <stdio.h>
#include "io_driver.h"


namespace io_drv
{

class cio_driver_gpio : public cio_driver_base
{
	enum pin_idx
	{
	        GPIO_RAP,
	        GPIO_RAN,
	        GPIO_DECP,
	        GPIO_DECN,
	        GPIO_MAX
	};
public:
	cio_driver_gpio();
	virtual ~cio_driver_gpio();

private:
	virtual int open_device( void );				// open device
	virtual int close_device( void );				// close dvice
	virtual void write_data( unsigned int dByte );	// write data to control telescope
	virtual u_char read_byte( void );				// not implemented

	/* This defines which GPIO's are used to implement the ST-4 interface. This
	   can be modified to select different GPIO's for different boards/configurations.
	 */
	// reads from io_gpio.conf
	int m_gpio_pin[ GPIO_MAX ];						// GPIO pin dev numbers
	FILE *m_gpio[ GPIO_MAX ];                       // GPIO SysFS file descriptors

	bool m_save_default_cfg;
};

}

#endif /* IO_LPT_H_ */
