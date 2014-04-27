/*
 * atik_core.cpp
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

#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include <unistd.h>
#include <dlfcn.h>
#include <atik_core.h>

AtikCamera* atik_core::m_camera = NULL;
int atik_core::m_ref_count = 0;
void* atik_core::atik_sdk = NULL;
pthread_mutex_t atik_core::m_mutex = PTHREAD_MUTEX_INITIALIZER;
AtikCamera_list_t* atik_core::AtikCamera_list = NULL;
AtikCamera_destroy_t* atik_core::AtikCamera_destroy = NULL;

int atik_core::atik_open( void )
{
	bool success;

	pthread_mutex_lock( &m_mutex );

	if (atik_sdk == NULL) {
		atik_sdk = dlopen("libatikccd.so", RTLD_LAZY);
		if (!atik_sdk) {
			log_e("Cannot load library: %s", dlerror());
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		AtikCamera_list = (AtikCamera_list_t *) dlsym(atik_sdk, "AtikCamera_list");
		const char* dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load AtikCamera_list(): %s", dlsym_error);
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		AtikCamera_destroy = (AtikCamera_destroy_t *) dlsym(atik_sdk, "AtikCamera_destroy");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load symbol AtikCamera_destroy(): %s", dlsym_error);
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		bool *AtikDebug = (bool *) dlsym(atik_sdk, "AtikDebug");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load symbol AtikDebug: %s", dlsym_error);
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}
		*AtikDebug = 0;
	}

	if (m_ref_count == 0) {
		m_camera_count = AtikCamera_list(m_camera_list, CAM_MAX);
		if (m_camera_count <=0) {
			log_e("No Atik camera found");
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		// TODO: select a camera with guiderport and delete all other
		m_camera = m_camera_list[0];
		log_i("Camera found: %s", m_camera->getName());

		success = m_camera->open();
		if (!success) {
			log_i("Can not open camera.");
			pthread_mutex_unlock( &m_mutex );
			return 2;
		}

		success = m_camera->setParam(QUICKER_START_EXPOSURE_DELAY, 1000);
		if (!success) log_i("Could not set timings.");

		success = m_camera->setParam(QUICKER_READ_CCD_DELAY, 1000);
		if (!success) log_i("Could not set timings.");
	}
	m_ref_count++;

	pthread_mutex_unlock( &m_mutex );

	success = m_camera->getCapabilities(&m_name, &m_type, &m_has_shutter, &m_has_guide_port,
		&m_pixel_count_X, &m_pixel_count_Y, &m_pixel_size_X, &m_pixel_size_Y, &m_max_bin_X, &m_max_bin_Y, &m_cooler);
	if (!success) return 3;

	return 0;
}


int atik_core::atik_close( void )
{
	pthread_mutex_lock( &m_mutex );

	if (m_ref_count == 1) {
		m_camera->close();
		AtikCamera_destroy(m_camera);
		dlclose(atik_sdk);
		atik_sdk = NULL;
		m_ref_count = 0;
	} else if (m_ref_count > 1) m_ref_count--;

	pthread_mutex_unlock( &m_mutex );

	return 0;
}
