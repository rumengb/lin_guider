/*
 * video_qhy6.h
 *
 *  Created on: 02.06.2011
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

#ifndef VIDEO_QHY6_H_
#define VIDEO_QHY6_H_

#include "video.h"
#include "qhy6_core.h"

namespace video_drv
{

/*
 * qhy6 camera
 */
class cvideo_qhy6 : public cvideo_base
{
public:
	cvideo_qhy6();
	virtual ~cvideo_qhy6();

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

	virtual int enum_controls( void );

	qhy6_core_shared *m_qhy6_obj;
private:
	// current settings
	// int m_exposuretime;	// stores in capture_params.fps
	int m_binn;
	// int m_gain;	// stores in capture_params.gain
	int m_offset;
	int m_speed;
	int m_amp;
	int m_vbe;
	int m_data_size;
	bool m_use_black_point;
};

}

#endif /* VIDEO_QHY6_H_ */
