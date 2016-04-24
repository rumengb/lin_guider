/*
 * atik_core.h
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

#ifndef ATIK_CORE_H_
#define ATIK_CORE_H_

#include <vector>
#include <time.h>
#include <assert.h>

#include "video.h"
#include "utils.h"
#include "atikccdusb.h"

#define V4L2_CID_ATIK_DODEBAYER (V4L2_CID_USER_CLASS + 1)

#define CAM_MAX 5

typedef int AtikCamera_list_t(AtikCamera**, int);
typedef void AtikCamera_destroy_t(AtikCamera*);

/*
 * Atik camera core class
 */

class atik_core
{
public:
	struct caps_s
	{
		caps_s() :
			name( NULL ),
			type( (enum CAMERA_TYPE)0 ),
			has_shutter( false),
			has_guide_port( false ),
			pixel_count_X( 0 ),
			pixel_count_Y( 0 ),
			pixel_size_X( 0 ),
			pixel_size_Y( 0 ),
			max_bin_X( 0 ),
			max_bin_Y( 0 ),
			color_type( COLOUR_NONE ),
			offsetX (0),
			offsetY (0)
		{}
		const char* name;
		CAMERA_TYPE type;
		bool has_shutter;
		bool has_guide_port;
		unsigned pixel_count_X, pixel_count_Y;
		double pixel_size_X, pixel_size_Y;
		unsigned max_bin_X, max_bin_Y;
		COLOUR_TYPE color_type;
		int offsetX;
		int offsetY;
	};
	atik_core() {}
	virtual ~atik_core() {}

	static unsigned int m_width;
	static unsigned int m_height;
	static unsigned int m_binX;
	static unsigned int m_binY;

protected:
	AtikCamera* get_camera() { return m_camera; }
	int open( void );		// get&check capabilities, apply format
	int close( void );		// deinit device
	const atik_core::caps_s& get_caps( void ) const;
	int set_guide_relays( int dir );
	void lock( void ) const;
	void unlock( void ) const;

private:
	static int m_ref_count;
	static AtikCamera *m_camera;
	static void* m_atik_sdk;
	static pthread_mutex_t m_mutex;
	static AtikCamera_list_t *AtikCamera_list;
	static AtikCamera_destroy_t *AtikCamera_destroy;
	static struct caps_s m_caps;
};

#endif /* ATIK_CORE_H_ */
