/*
 * io_gpio.cpp
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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <QSettings>

#include "io_gpio.h"
#include "maindef.h"
#include "utils.h"


// disables hardware access for GPUSB
//#define NO_GPIO

/* This can be used to spoof the gpio directory structure and see which files
   are being written to.
 */
const char *dbg_pfx = "";

namespace io_drv
{

#define FSTRSZ(x) (x), sizeof(x), 1

//----------------------------------------
//
// Derived.... GPIO
//
//----------------------------------------
cio_driver_gpio::cio_driver_gpio()
{
	device_type = DT_GPIO;
	min_pulse_length = 0;
	max_pulse_length = 0xFFFFFF;

	// set default values
	m_gpio_pin[ GPIO_RAP ]  = 27; //RA+
	m_gpio_pin[ GPIO_RAN ]  = 22; //RA-
	m_gpio_pin[ GPIO_DECP ] = 23; //DEC+
	m_gpio_pin[ GPIO_DECN ] = 24;  //DEC-

	for( int i = 0;i < GPIO_MAX;i++ )
		m_gpio[ i ] = NULL;

	// try to load config
	QSettings settings( "GM_software", QString("io_gpio") );
	m_save_default_cfg = access( settings.fileName().toAscii().data(), R_OK|W_OK) != 0;
	settings.beginGroup("gpio");
		m_gpio_pin[ GPIO_RAP ]  = settings.value( "RAP",  27 ).toInt(); //RA+
		m_gpio_pin[ GPIO_RAN ]  = settings.value( "RAN",  22 ).toInt(); //RA-
		m_gpio_pin[ GPIO_DECP ] = settings.value( "DECP", 23 ).toInt(); //DEC+
		m_gpio_pin[ GPIO_DECN ] = settings.value( "DECN", 24 ).toInt();  //DEC-
	settings.endGroup();

	for( int i = 0;i < GPIO_MAX;i++ )
		log_i("%d", m_gpio_pin[ i ] );
}


cio_driver_gpio::~cio_driver_gpio()
{
	//MUST BE called the first: stop thread and release resources
	stop();

	// save default config
	if( m_save_default_cfg )
	{
		QSettings settings( "GM_software", QString("io_gpio") );
		settings.beginGroup("gpio");
			settings.setValue( "RAP",  m_gpio_pin[ GPIO_RAP ] );
			settings.setValue( "RAN",  m_gpio_pin[ GPIO_RAN ] );
			settings.setValue( "DECP", m_gpio_pin[ GPIO_DECP ] );
			settings.setValue( "DECN", m_gpio_pin[ GPIO_DECN ] );
		settings.endGroup();
	}
}


int cio_driver_gpio::open_device( void )
{
#ifdef NO_GPIO

#else
	for( int i = 0; i < GPIO_MAX; ++i )
		m_gpio[i] = NULL;

	for( int i = 0; i < GPIO_MAX; ++i )
	{
		char buf[256];

		/* Configure GPIO as output */
		snprintf( buf, sizeof(buf), "%s/sys/class/gpio/gpio%d/direction", dbg_pfx, m_gpio_pin[i] );
		if( access(buf, F_OK | R_OK | W_OK) )
		{
			log_e("Can't access %s. Has the GPIO been exported?", buf);
			return EXIT_FAILURE;
		}
		m_gpio[i] = fopen( buf, "w" );
		if (!m_gpio[i] )
		{
			log_e("Failed to open direction pin");
			return EXIT_FAILURE;
		}
		size_t n_wr = fwrite( FSTRSZ("out"), m_gpio[i] );
		if( n_wr != 1 )
			log_e( "cio_driver_gpio::open_device(): Write error into GPIO: %s", strerror(errno) );
		fclose( m_gpio[i] );
		m_gpio[i] = NULL;

		/* Configure pin as de-asserted by default */
		snprintf( buf, sizeof(buf), "%s/sys/class/gpio/gpio%d/value", dbg_pfx, m_gpio_pin[i] );
		if( access(buf, F_OK | R_OK | W_OK) )
		{
			log_e( "Can't access value pin" );
			return EXIT_FAILURE;
		}
		m_gpio[i] = fopen( buf, "w" );
		if( !m_gpio[i] )
		{
			log_e("Failed to open value pin");
			return EXIT_FAILURE;
		}
		// Down all pins
		n_wr = fwrite( FSTRSZ("0"), m_gpio[i] );
		if( n_wr != 1 )
			log_e( "cio_driver_gpio::open_device(): Write error into GPIO: %s", strerror(errno) );
	}
#endif

	return EXIT_SUCCESS;
}


int cio_driver_gpio::close_device( void )
{
	// Down all pins
	write_data( 0x0 );

	for( int i = 0; i < GPIO_MAX; ++i )
	{
		if( m_gpio[i] )
		{
			fclose( m_gpio[i] );
			m_gpio[i] = NULL;
		}
	}

	return EXIT_SUCCESS;
}


void cio_driver_gpio::write_data( unsigned int dByte )
{
	bool inverse_ra = false;
	bool inverse_dec = false;
	int ra_inc_pos = 0;
	int ra_dec_pos = 0;
	int dec_inc_pos = 0;
	int dec_dec_pos = 0;

	for( int i = 0;i < 8;i++ )
	{
		 if( bit_actions[i] == RA_INC_DIR )
			 ra_inc_pos = i;
		 if( bit_actions[i] == RA_DEC_DIR )
			 ra_dec_pos = i;
		 if( bit_actions[i] == DEC_INC_DIR )
			 dec_inc_pos = i;
		 if( bit_actions[i] == DEC_DEC_DIR )
			 dec_dec_pos = i;
	}

	inverse_ra = ra_inc_pos > ra_dec_pos;
	inverse_dec = dec_inc_pos > dec_dec_pos;

	u_char mapped = bit_map_encoder[ (u_char)dByte ];

	for( int i = 0 ; i < 8; ++i )
	{
		/* Skip NO_DIR */
		if( bit_actions[i] != NO_DIR )
		{
			u_char mask = (1 << i);
			FILE *out = NULL;

			switch( bit_actions[i] )
			{
			case RA_INC_DIR:
				out = m_gpio[ inverse_ra ? GPIO_RAN : GPIO_RAP ];
				log_i("RA_INC_DIR: %s", (mapped & mask) ? "1" : "0");
				break;
			case RA_DEC_DIR:
				out = m_gpio[ inverse_ra ? GPIO_RAP : GPIO_RAN ];
				log_i("RA_DEC_DIR: %s", (mapped & mask) ? "1" : "0");
				break;
			case DEC_INC_DIR:
				out = m_gpio[ inverse_dec ? GPIO_DECN : GPIO_DECP ];
				log_i("DEC_INC_DIR: %s", (mapped & mask) ? "1" : "0");
				break;
			case DEC_DEC_DIR:
				out = m_gpio[ inverse_dec ? GPIO_DECP: GPIO_DECN ];
				log_i("DEC_DEC_DIR: %s", (mapped & mask) ? "1" : "0");
				break;
			default:
				break;
			}

			if( out )
			{
				size_t n_wr = fwrite( FSTRSZ( (mapped & mask) ? "1" : "0"), out );
				if( n_wr != 1 )
					log_e( "cio_driver_gpio::write_data(): Write error into GPIO: %s", strerror(errno) );
				fflush( out );
			}
		}
	}
}


u_char cio_driver_gpio::read_byte( void )
{
	return 0;
}

}
