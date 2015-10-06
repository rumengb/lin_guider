/*
 * video_asi.h
 *
 *  Created on: february 2015
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

#ifndef VIDEO_ASI_H_
#define VIDEO_ASI_H_

#include <vector>
#include <time.h>
#include "video.h"
#include "timer.h"
#include "asi_core.h"

namespace video_drv
{

/*
 * ZWO ASI camera
 */
class cvideo_asi : public cvideo_base, public asi_core
{
public:
	cvideo_asi();
	virtual ~cvideo_asi();

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

	// local
	virtual int enum_controls( void );
	unsigned int get_pix_fmt( void );

	int m_transfer_bits; // 8 or 16 Bit Transfer Mode Transfer Mode
	int m_bandwidth;
	int m_wb_r;
	int m_wb_b;
	bool m_force_bw;

	ctimer exp_timer;
	long m_expstart;
};

}

#endif /* VIDEO_ASI_H_ */
