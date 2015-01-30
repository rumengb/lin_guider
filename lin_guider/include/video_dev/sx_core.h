/*
 * sx_core.h
 *
 *  Created on: january 2015
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

#ifndef SX_CORE_H_
#define SX_CORE_H_

#include <vector>
#include <time.h>
#include <assert.h>

#include "video.h"
#include "utils.h"
#include "timer.h"
#include "sxccdusb.h"


#define CAM_MAX 5

/*
 * Starlight Xpress camera core class
 */

class sx_core
{
public:
	sx_core() {}
	virtual ~sx_core() {}

protected:
	static struct t_sxccd_params m_caps;
	static int m_width;
	static int m_height;
	static int m_binX;
	static int m_binY;

	int open( void );
	int close( void );
	int set_guide_relays( int dir );
	bool start_exposure();
	bool abort_exposure();
	bool read_image(char *buf, int buf_size);
	void lock( void ) const;
	void unlock( void ) const;

private:
	unsigned short m_model;
	static int m_ref_count;
	static pthread_mutex_t m_mutex;
	static HANDLE m_camera;

	static bool m_is_interlaced;

	// if the camera has no shutter ans is interleased we need to know
	// the exact time between between wiping the odd and even rows to
	// in order to expose then the same time.
	ctimer delay_timer;
	static long m_wipe_delay;

	// if the camera is interleased we read the even and odd rows separately
	static char* m_oddBuf;
	static char* m_evenBuf;
};

#endif /* SX_CORE_H_ */
