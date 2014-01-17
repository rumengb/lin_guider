/*
 * video_uvc.h
 *
 *  Created on: 24.05.2011
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

#ifndef VIDEO_UVC_H_
#define VIDEO_UVC_H_

#include "video.h"

namespace video_drv
{

/*
 * for UVC cameras
 */
class cvideo_uvc : public cvideo_base
{
public:
	cvideo_uvc();
	virtual ~cvideo_uvc();

	virtual time_fract_t set_fps( const time_fract_t &new_fps );

private:
	virtual int init_device( void );		// get&check capabilities, apply format
	virtual int uninit_device( void );		// deinit device
	virtual int start_capturing( void );	// turn on stream
	virtual int stop_capturing( void );		// stop stream
	virtual int read_frame( void );			// read frame
	virtual int set_format( void );

	// local
	int init_read( unsigned int buffer_size );	// init buffers
	int init_mmap( void );			// init buffers
};

}

#endif /* VIDEO_UVC_H_ */
