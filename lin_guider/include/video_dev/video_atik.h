/*
 * video_atik.h
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

#ifndef VIDEO_ATIK_H_
#define VIDEO_ATIK_H_

#include <vector>
#include <time.h>
#include "video.h"
#include "timer.h"
#include <atik_core.h>

namespace video_drv
{

/*
 * Atik camera
 */
class cvideo_atik : public cvideo_base, public atik_core
{
public:
	cvideo_atik();
	virtual ~cvideo_atik();

	virtual time_fract_t set_fps( const time_fract_t &new_fps );

protected:
	virtual int open_device( void );		// open device
	virtual int close_device( void );		// close device
	virtual int get_vcaps( void );

	virtual int set_control( unsigned int control_id, const param_val_t &val );
	virtual int get_control( unsigned int control_id, param_val_t *val );

private:
	virtual int init_device( void );		// get&check capabilities, apply format
	virtual int uninit_device( void );		// deinit device
	virtual int start_capturing( void );	// turn on stream
	virtual int stop_capturing( void );		// stop stream
	virtual int read_frame( void );			// read frame
	virtual int set_format( void );
	unsigned int get_pix_fmt( void );

	// local
	virtual int enum_controls( void );

	ctimer exp_timer;
	long m_expstart;
	bool m_do_debayer;
};

}

#endif /* VIDEO_ATIK_H_ */
