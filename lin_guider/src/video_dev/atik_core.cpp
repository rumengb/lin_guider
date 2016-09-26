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

#include "atik_core.h"


AtikCamera* atik_core::m_camera = NULL;
int atik_core::m_ref_count = 0;
void* atik_core::m_atik_sdk = NULL;
pthread_mutex_t atik_core::m_mutex = PTHREAD_MUTEX_INITIALIZER;
AtikCamera_list_t* atik_core::AtikCamera_list = NULL;
AtikCamera_destroy_t* atik_core::AtikCamera_destroy = NULL;
atik_core::caps_s atik_core::m_caps = atik_core::caps_s();
unsigned int atik_core::m_width = 0;
unsigned int atik_core::m_height = 0;
unsigned int atik_core::m_binX = 1;
unsigned int atik_core::m_binY = 1;

int atik_core::open( void )
{
	bool success = false;
	struct AtikCapabilities caps;

	pthread_mutex_lock( &m_mutex );

	if (m_atik_sdk == NULL) {
		m_atik_sdk = dlopen("libatikccd.so", RTLD_LAZY);
		if (!m_atik_sdk) {
			log_e("Cannot load library: %s", dlerror());
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		AtikCamera_list = (AtikCamera_list_t *) dlsym(m_atik_sdk, "AtikCamera_list");
		const char* dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load AtikCamera_list(): %s", dlsym_error);
			dlclose(m_atik_sdk);
			m_atik_sdk = NULL;
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		AtikCamera_destroy = (AtikCamera_destroy_t *) dlsym(m_atik_sdk, "AtikCamera_destroy");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load symbol AtikCamera_destroy(): %s", dlsym_error);
			dlclose(m_atik_sdk);
			m_atik_sdk = NULL;
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		bool *AtikDebug = (bool *) dlsym(m_atik_sdk, "AtikDebug");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load symbol AtikDebug: %s", dlsym_error);
			dlclose(m_atik_sdk);
			m_atik_sdk = NULL;
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}
		*AtikDebug = 0;
	}

	if (m_ref_count == 0)
	{
		int camera_count;
		AtikCamera *camera_list[CAM_MAX];
		camera_count = AtikCamera_list( camera_list, CAM_MAX );
		if (camera_count <=0) {
			log_e("No Atik camera found");
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		// Select camera with ST4 port, if not found use the last camera found
		bool found = false;
		int index = 0;
		while (index < camera_count) {
			m_camera = camera_list[index];
			if( DBG_VERBOSITY ) {
				log_i("Trying camera: %s", m_camera->getName());
			}

			success = m_camera->open();
			if (!success) {
				log_e("Can not open camera.");
				AtikCamera_destroy(m_camera);
				index++;
				continue;
			}

			success = m_camera->getCapabilities(&m_caps.name, &m_caps.type, &caps);
			if (!success) {
				log_e("Can not get capabilities.");
				m_camera->close();
				AtikCamera_destroy(m_camera);
				index++;
				continue;
			}

			m_caps.has_shutter = caps.hasShutter;
			m_caps.has_guide_port = caps.hasGuidePort;
			m_caps.pixel_count_X = caps.pixelCountX;
			m_caps.pixel_count_Y = caps.pixelCountY;
			m_caps.pixel_size_X = caps.pixelSizeX;
			m_caps.pixel_size_Y = caps.pixelSizeY;
			m_caps.max_bin_X = caps.maxBinX;
			m_caps.max_bin_Y = caps.maxBinY;
			m_caps.color_type = caps.colour;
			m_caps.offsetX = caps.offsetX;
			m_caps.offsetY = caps.offsetY;

			if ((m_caps.has_guide_port) || (index == camera_count-1)) {
				found = true;
				break;
			} else {
				m_camera->close();
				AtikCamera_destroy(m_camera);
				index++;
			}
		}

		// Skip the selected and destroy the rest of the camera objects
		index++;
		while(index < camera_count) {
			AtikCamera_destroy(camera_list[index++]);
		}

		if (!found) {
			log_e("No usable Atik camera found.");
			pthread_mutex_unlock( &m_mutex );
			return 2;
		}

		if (m_caps.color_type == COLOUR_NONE) {
			log_i("Using camera: %s (Mono)", m_camera->getName());
			log_i("Note: Some older colour cameras report mono. If so, use \"Camera Settings\" to force colour mode.");
		} else {
			log_i("Using camera: %s (Colour)", m_camera->getName());
		}

		if (m_caps.type == QUICKER) {
			success = m_camera->setParam(QUICKER_START_EXPOSURE_DELAY, 1000);
			if (!success) log_e("Could not set timings.");

			success = m_camera->setParam(QUICKER_READ_CCD_DELAY, 1000);
			if (!success) log_e("Could not set timings.");
		}

		if (DBG_VERBOSITY) log_i("Camera name: %s, type: %d", m_caps.name, m_caps.type);
	}
	m_ref_count++;

	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


int atik_core::close( void )
{
	pthread_mutex_lock( &m_mutex );

	assert( m_ref_count > 0 );
	m_ref_count--;
	if( m_ref_count == 0)
	{
		m_camera->close();
		AtikCamera_destroy(m_camera);
		dlclose(m_atik_sdk);
		m_atik_sdk = NULL;
	}

	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


const atik_core::caps_s& atik_core::get_caps( void ) const
{
	return m_caps;
}


int atik_core::set_guide_relays( int dir )
{
	pthread_mutex_lock( &m_mutex );
	m_camera->setGuideRelays( dir );
	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


void atik_core::lock( void ) const
{
	pthread_mutex_lock( &m_mutex );
}


void atik_core::unlock( void ) const
{
	pthread_mutex_unlock( &m_mutex );
}
