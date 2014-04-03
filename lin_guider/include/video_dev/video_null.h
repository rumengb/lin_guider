/*
 * video_gen.h
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

#ifndef VIDEO_NULL_H_
#define VIDEO_NULL_H_

#include <vector>
#include "video.h"

namespace video_drv
{


/*
 * generic NULL camera
 */
class cvideo_null : public cvideo_base
{
public:
	cvideo_null( bool stub = false );
	virtual ~cvideo_null();

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

	bool stub_mode;

	// emu
	enum emu_contsts
	{
		star_no = 5,
		bad_pixel_no = 4
	};
	typedef struct
	{
		int x, y, lum;
		double sigma;
	}emu_star_t;
	std::vector<emu_star_t> m_emu_stars;
	std::vector<emu_star_t> m_emu_badpix;
	void generate_emu_stars( void );
	void generate_emu_field( void );
};

}

#endif /* VIDEO_NULL_H_ */
