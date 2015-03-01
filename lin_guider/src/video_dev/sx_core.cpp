/*
 * sx_core.cpp
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

#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include <unistd.h>
#include <dlfcn.h>

#include "sx_core.h"

int sx_core::m_ref_count = 0;
pthread_mutex_t sx_core::m_mutex = PTHREAD_MUTEX_INITIALIZER;
HANDLE sx_core::m_camera = NULL;
struct t_sxccd_params sx_core::m_caps = {0,0,0,0,0,0,0,0,0,0,0,0,0};
char* sx_core::m_oddBuf = NULL;
char* sx_core::m_evenBuf = NULL;
long sx_core::m_wipe_delay = 130000; //this will be updated in read_image();
bool sx_core::m_is_interlaced = false;
int sx_core::m_width = 0;
int sx_core::m_height = 0;
int sx_core::m_binX = 1;
int sx_core::m_binY = 1;

int sx_core::open( void )
{
	DEVICE devices[CAM_MAX];
	const char* names[CAM_MAX];
	bool success;

	pthread_mutex_lock( &m_mutex );

	if (m_ref_count == 0)
	{
		int camera_count;
		camera_count = sxList(devices, names, CAM_MAX);
		if (camera_count <=0) {
			log_e("No Starlight Xpress camera found.");
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		// Select camera with ST4 port, if not found use the last camera found
		bool found = false;
		int index = 0;
		while (index < camera_count) {
			if( DBG_VERBOSITY ) {
				log_i("Trying camera: %s", names[index]);
			}
			success = sxOpen(devices[index], &m_camera);
			if (!success) {
				log_e("sxOpen(): Error opening camera.");
				index++;
				continue;
			}

			memset(&m_caps, 0, sizeof(m_caps));
			success = sxGetCameraParams(m_camera, 0, &m_caps);
			if (!success) {
				log_e("sxGetCameraParams(): Error reading params.");
				sxClose(&m_camera);
				index++;
				continue;
			}

			if ((m_caps.extra_caps & SXCCD_CAPS_GUIDER) || (index == camera_count-1)) {
				found = true;
				break;
			} else {
				sxClose(&m_camera);
				index++;
			}
		}

		if (!found) {
			log_e("No usable Starlight Xpress camera found.");
			pthread_mutex_unlock( &m_mutex );
			return 2;
		}

		log_i("Using camera: %s", names[index]);
		sxReset(m_camera);

		m_model = sxGetCameraModel(m_camera);
		m_is_interlaced = sxIsInterlaced(m_model);
		if (m_is_interlaced) {
			m_caps.pix_height /= 2;
			m_caps.height *= 2;
			int bytes_pix = m_caps.bits_per_pixel / 8;
			m_oddBuf = (char*) malloc(m_caps.height/2 * m_caps.width * bytes_pix);
			if (m_oddBuf == NULL) {
				log_i("malloc(): no memeory for m_oddBuff.");
				pthread_mutex_unlock( &m_mutex );
				return 3;
			}
			m_evenBuf = (char*) malloc(m_caps.height/2 * m_caps.width * bytes_pix);
			if (m_evenBuf == NULL) {
				log_i("malloc(): no memeory for m_evenBuff.");
				pthread_mutex_unlock( &m_mutex );
				return 4;
			}
		}
	}
	m_ref_count++;
	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


int sx_core::close( void )
{
	pthread_mutex_lock( &m_mutex );

	assert( m_ref_count > 0 );
	m_ref_count--;
	if( m_ref_count == 0) {
		sxClose(&m_camera);
	}

	pthread_mutex_unlock( &m_mutex );

	if (m_is_interlaced) {
			if (m_oddBuf) free(m_oddBuf);
			if (m_evenBuf) free(m_evenBuf);
	}

	return EXIT_SUCCESS;
}


bool sx_core::start_exposure() {
	int rc;

	if (m_is_interlaced && m_binY == 1) {
		if( DBG_VERBOSITY ) {
			log_i("m_wipe_delay = %d us", m_wipe_delay );
		}
		pthread_mutex_lock( &m_mutex );
		rc = sxClearPixels(m_camera, CCD_EXP_FLAGS_FIELD_EVEN | CCD_EXP_FLAGS_NOWIPE_FRAME, 0);
		pthread_mutex_unlock( &m_mutex );
		usleep(m_wipe_delay); // measured in read_image()
		pthread_mutex_lock( &m_mutex );
		if (rc) sxClearPixels(m_camera, CCD_EXP_FLAGS_FIELD_ODD | CCD_EXP_FLAGS_NOWIPE_FRAME, 0);
		pthread_mutex_unlock( &m_mutex );
	} else {
		pthread_mutex_lock( &m_mutex );
		rc = sxClearPixels(m_camera, CCD_EXP_FLAGS_FIELD_BOTH, 0);
		pthread_mutex_unlock( &m_mutex );
	}
	pthread_mutex_lock( &m_mutex );
	if (m_caps.extra_caps & SXUSB_CAPS_SHUTTER)	sxSetShutter(m_camera, 0);
	pthread_mutex_unlock( &m_mutex );

	return (bool)rc;
}

bool sx_core::abort_exposure() {
    if (m_caps.extra_caps & SXUSB_CAPS_SHUTTER) {
		pthread_mutex_lock( &m_mutex );
		sxSetShutter(m_camera, 1);
		pthread_mutex_unlock( &m_mutex );
	}
    return true;
}


bool sx_core::read_image(char *buf, int buf_size) {
	int rc;
	int subX = 0;
	int subY = 0;
	int width_b = m_width * m_caps.bits_per_pixel/8;
	int size = (width_b * m_height) / (m_binX * m_binY);

	if (size > buf_size) return false; // data will fit in buf

	pthread_mutex_lock( &m_mutex );
	if (m_caps.extra_caps & SXUSB_CAPS_SHUTTER) sxSetShutter(m_camera, 1);
	pthread_mutex_unlock( &m_mutex );
	if (m_is_interlaced) {
		if (m_binY > 1) {
			pthread_mutex_lock( &m_mutex );
			rc = sxLatchPixels(m_camera, CCD_EXP_FLAGS_FIELD_BOTH, 0, subX, subY, m_width, m_height/2, m_binX, m_binY/2);
			if (rc) rc = sxReadPixels(m_camera, buf, size);
			pthread_mutex_unlock( &m_mutex );
		} else {
			pthread_mutex_lock( &m_mutex );
			rc = sxLatchPixels(m_camera, CCD_EXP_FLAGS_FIELD_EVEN | CCD_EXP_FLAGS_SPARE2 , 0, subX, subY/2, m_width, m_height/2, m_binX, 1);
			if (rc) {
				long start_time = delay_timer.gettime();
				rc = sxReadPixels(m_camera, m_evenBuf, size/2);
				// measure the delay needed between wiping even and odd rows to ensure uniform exposure
				m_wipe_delay = (delay_timer.gettime() - start_time) * 1000;
			}
			if (rc) rc = sxLatchPixels(m_camera, CCD_EXP_FLAGS_FIELD_ODD | CCD_EXP_FLAGS_SPARE2, 0, subX, subY/2, m_width, m_height/2, m_binX, 1);
			if (rc) rc = sxReadPixels(m_camera, m_oddBuf, size/2);
			pthread_mutex_unlock( &m_mutex );
			if (rc) {
				for (int i = 0, j = 0; i < m_height; i += 2, j++) {
					memcpy(buf + i * width_b, m_oddBuf + (j * width_b), width_b);
					memcpy(buf + ((i + 1) * width_b), m_evenBuf + (j * width_b), width_b);
				}
			}
		}
	} else {
		pthread_mutex_unlock( &m_mutex );
		rc = sxLatchPixels(m_camera, CCD_EXP_FLAGS_FIELD_BOTH, 0, subX, subY, m_width, m_height, m_binX, m_binY);
		if (rc) rc = sxReadPixels(m_camera, buf, size);
		pthread_mutex_unlock( &m_mutex );
	}
	return (bool)rc;
}


int sx_core::set_guide_relays( int dir )
{
	pthread_mutex_lock( &m_mutex );
	sxSetSTAR2000(m_camera, dir);
	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


void sx_core::lock( void ) const
{
	pthread_mutex_lock( &m_mutex );
}


void sx_core::unlock( void ) const
{
	pthread_mutex_unlock( &m_mutex );
}
