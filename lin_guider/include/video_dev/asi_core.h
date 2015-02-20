/*
 * asi_core.h
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

#ifndef ASI_CORE_H_
#define ASI_CORE_H_

#include <vector>
#include <time.h>
#include <assert.h>

#include "video.h"
#include "utils.h"
#include "timer.h"
#include "ASICamera2.h"

#define CAM_MAX 5

/*
 * ZWO ASI camera core class
 */

class asi_core
{
public:
	asi_core() {}
	virtual ~asi_core() {}

protected:
	static int m_width;
	static int m_height;
	static int m_binX;
	static int m_binY;

	static ASI_CONTROL_CAPS m_expo_caps;
	static bool m_has_expo;
	static ASI_CONTROL_CAPS m_gain_caps;
	static bool m_has_gain;
	static ASI_CONTROL_CAPS m_bwidth_caps;
	static bool m_has_bwidth;

	static ASI_CAMERA_INFO m_cam_info;
	static int m_camera;

	int open( void );
	int close( void );
	int set_guide_relays( int dir );
	bool start_exposure();
	bool abort_exposure();
	bool read_image(char *buf, int buf_size);
	void lock( void ) const;
	void unlock( void ) const;

private:
	static int m_ref_count;
	static pthread_mutex_t m_mutex;
};

#endif /* ASI_CORE_H_ */
