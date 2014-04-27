/*
 * atik_base.h
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
#include "video.h"
#include <utils.h>
#include <atikccdusb.h>

#define CAM_MAX 5

typedef int AtikCamera_list_t(AtikCamera**, int);
typedef void AtikCamera_destroy_t(AtikCamera*);

/*
 * Atik camera core class
 */
 
class atik_core
{
public:
	atik_core() {};
	virtual ~atik_core() {};
	AtikCamera* get_camera() { return m_camera; };

	static int m_ref_count;
	static AtikCamera *m_camera;
	static void* atik_sdk;
	static pthread_mutex_t m_mutex;
	static AtikCamera_list_t *AtikCamera_list;
	static AtikCamera_destroy_t *AtikCamera_destroy;
	
	int m_camera_count;
	AtikCamera *m_camera_list[CAM_MAX];
	const char* m_name;
	CAMERA_TYPE m_type;
	bool m_has_shutter;
	bool m_has_guide_port;
	unsigned m_pixel_count_X, m_pixel_count_Y;
	double m_pixel_size_X, m_pixel_size_Y;
	unsigned m_max_bin_X, m_max_bin_Y;
	COOLER_TYPE m_cooler;
	COOLING_STATE m_cooler_state;
	float m_target_temp;
	float m_current_temp;
	struct timeval m_expstart;

	int atik_open( void );		// get&check capabilities, apply format
	int atik_close( void );		// deinit device
};

#endif /* ATIK_CORE_H_ */
