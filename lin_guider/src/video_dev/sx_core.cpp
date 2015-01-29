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

		// TODO: select a camera with guiderport and delete all other
		log_i("Camera found: %s", names[0]);

		success = sxOpen(devices[0], &m_camera);
		if (!success) {
			log_i("sxOpen(): Error opening camera.");
			pthread_mutex_unlock( &m_mutex );
			return 2;
		} 
		
		sxReset(m_camera);
		
		memset(&m_caps, 0, sizeof(m_caps));
		success = sxGetCameraParams(m_camera, 0, &m_caps);
		if (!success) {
			log_i("sxGetCameraParams(): Error reading params.");
			pthread_mutex_unlock( &m_mutex );
			return 3;
		}
		
		m_model = sxGetCameraModel(m_camera);
		m_is_interleaced = sxIsInterlaced(m_model);
		if (m_is_interleaced) {
			m_caps.pix_height /= 2;
			m_caps.height *= 2;
			int bytes_pix = m_caps.bits_per_pixel / 8;
			m_oddBuf = (char*) malloc(m_caps.height/2 * m_caps.width * bytes_pix);
			if (m_oddBuf == NULL) {
				log_i("malloc(): no memeory for m_oddBuff.");
				pthread_mutex_unlock( &m_mutex );
				return 4;
			}
			m_evenBuf = (char*) malloc(m_caps.height/2 * m_caps.width * bytes_pix);
			if (m_evenBuf == NULL) {
				log_i("malloc(): no memeory for m_evenBuff.");
				pthread_mutex_unlock( &m_mutex );
				return 4;
			}
		}
		m_wipe_delay = 130000; //this will be updated in read_image();
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
	if( m_ref_count == 0)
	{
		sxClose(&m_camera);
	}

	pthread_mutex_unlock( &m_mutex );
	
	if (m_is_interleaced) {		
			if (m_oddBuf) free(m_oddBuf);
			if (m_evenBuf) free(m_evenBuf);
	}

	return EXIT_SUCCESS;
}


bool sx_core::start_exposure() {
	int rc;
	
	pthread_mutex_lock( &m_mutex );
	if (m_is_interleaced) {
		if( DBG_VERBOSITY ) {
			log_i("wipe_delay = %d us", m_wipe_delay );
		}
		rc = sxClearPixels(m_camera, CCD_EXP_FLAGS_FIELD_EVEN | CCD_EXP_FLAGS_NOWIPE_FRAME, 0);
		usleep(m_wipe_delay); // measured in read_image()
		if (rc) sxClearPixels(m_camera, CCD_EXP_FLAGS_FIELD_ODD | CCD_EXP_FLAGS_NOWIPE_FRAME, 0);
	} else {
		rc = sxClearPixels(m_camera, CCD_EXP_FLAGS_FIELD_BOTH, 0);
	}
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
	int subW = m_caps.width;
	int subH = m_caps.height;
	int binX = 1;
	int binY = 1;
	int subWW = subW * 2;
    int size;
    
	// must chekck buf_size and figure out bbp stuff
	if (m_is_interleaced && binY > 1) {
		size = subW * subH / 2 / binX / (binY / 2);
	} else {
		size = subW * subH / binX / binY;
	}
	pthread_mutex_lock( &m_mutex );
	if (m_caps.extra_caps & SXUSB_CAPS_SHUTTER) sxSetShutter(m_camera, 1);
	if (m_is_interleaced) {
		if (binY > 1) {
			rc = sxLatchPixels(m_camera, CCD_EXP_FLAGS_FIELD_BOTH, 0, subX, subY, subW, subH / 2, binX, binY / 2);
			if (rc) rc = sxReadPixels(m_camera, buf, size * 2);
		} else {
			rc = sxLatchPixels(m_camera, CCD_EXP_FLAGS_FIELD_EVEN | CCD_EXP_FLAGS_SPARE2 , 0, subX, subY / 2, subW, subH / 2, binX, 1);
			if (rc) {
				long start_time = delay_timer.gettime(); 
				rc = sxReadPixels(m_camera, m_evenBuf, size);
				// measure the delay needed between wiping even and odd roes to ensure uniform exposure
				m_wipe_delay = (delay_timer.gettime() - start_time) * 1000;
			}
			if (rc) rc = sxLatchPixels(m_camera, CCD_EXP_FLAGS_FIELD_ODD | CCD_EXP_FLAGS_SPARE2, 0, subX, subY / 2, subW, subH / 2, binX, 1);
			if (rc) rc = sxReadPixels(m_camera, m_oddBuf, size);
			if (rc) {
				for (int i = 0, j = 0; i < subH; i += 2, j++) {
					memcpy(buf + i * subWW, m_oddBuf + (j * subWW), subWW);
					memcpy(buf + ((i + 1) * subWW), m_evenBuf + (j * subWW), subWW);
				}
			}
		}
	} else {
		rc = sxLatchPixels(m_camera, CCD_EXP_FLAGS_FIELD_BOTH, 0, subX, subY, subW, subH, binX, binY);
		if (rc) rc = sxReadPixels(m_camera, buf, size * 2);
	}
	pthread_mutex_unlock( &m_mutex );
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
