/*
 * asi_core.cpp
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

#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <math.h>
#include <unistd.h>
#include <dlfcn.h>

#include "asi_core.h"

int asi_core::m_ref_count = 0;
pthread_mutex_t asi_core::m_mutex = PTHREAD_MUTEX_INITIALIZER;
int asi_core::m_camera = -1;
ASI_CAMERA_INFO asi_core::m_cam_info = {{0},0,0,0,ASI_FALSE,ASI_BAYER_RG,{0},{ASI_IMG_END},0,ASI_FALSE,ASI_FALSE};
ASI_CONTROL_CAPS asi_core::m_expo_caps = {{0},{0},0,0,0,0,ASI_FALSE,ASI_FALSE,ASI_GAIN};
bool asi_core::m_has_expo = false;
ASI_CONTROL_CAPS asi_core::m_gain_caps = {{0},{0},0,0,0,0,ASI_FALSE,ASI_FALSE,ASI_GAIN};
bool asi_core::m_has_gain = false;
ASI_CONTROL_CAPS asi_core::m_bwidth_caps = {{0},{0},0,0,0,0,ASI_FALSE,ASI_FALSE,ASI_GAIN};
bool asi_core::m_has_bwidth = false;
int asi_core::m_width = 0;
int asi_core::m_height = 0;
int asi_core::m_binX = 1;
int asi_core::m_binY = 1;
int asi_core::m_bandwidth = 0;
unsigned char asi_core::m_bpp = 0;
ASI_IMG_TYPE asi_core::m_img_type = ASI_IMG_END;

int asi_core::open( void )
{
	int rc;
	ASI_CONTROL_CAPS ctrl_caps;

	pthread_mutex_lock( &m_mutex );
	if (m_ref_count == 0)
	{
		int camera_count = ASIGetNumOfConnectedCameras();
		if (camera_count <=0) {
			log_e("No ZWO ASI camera found.");
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		rc = ASIGetCameraProperty(&m_cam_info, 0);
		if (rc != ASI_SUCCESS) {
			log_e("ASIGetCameraProperty() returned %d", rc);
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}
		// TODO: select a camera with guiderport and delete all other
		log_i("Camera found: %s", m_cam_info.Name);

		m_camera=m_cam_info.CameraID;
		rc = ASIOpenCamera(m_camera);
		if (rc != ASI_SUCCESS) {
			log_i("ASIOpenCamera(): Error opening camera (rc = %d).",rc);
			pthread_mutex_unlock( &m_mutex );
			return 1;
        }

		int c_num;
		ASIGetNumOfControls(m_camera, &c_num);
		for (int i = 0; i < c_num; i++) {
			rc = ASIGetControlCaps(m_camera, i, &ctrl_caps);
			if (rc != ASI_SUCCESS) {
				log_e("ASIGetControlCaps(): error = %d\n", rc);
				pthread_mutex_unlock( &m_mutex );
				return 1;
			}
			switch (ctrl_caps.ControlID) {
			case ASI_GAIN:
				m_gain_caps = ctrl_caps;
				m_has_gain = true;
				break;
			case ASI_EXPOSURE:
				m_expo_caps = ctrl_caps;
				m_has_expo = true;
				break;
			case ASI_BANDWIDTHOVERLOAD:
				m_bwidth_caps = ctrl_caps;
				m_has_bwidth = true;
				break;
			}
		}

		int x,y,d;
		ASIGetROIFormat(m_camera, &x, &y,  &d, &m_img_type);
		switch (m_img_type) {
		case ASI_IMG_RAW8:
		case ASI_IMG_Y8:
			m_bpp = 8;
			break;
		case ASI_IMG_RAW16:
			m_bpp = 16;
			break;
		case ASI_IMG_RGB24:
			m_bpp = 24;
			break;
		}
	}

	m_ref_count++;
	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


int asi_core::close( void )
{
	pthread_mutex_lock( &m_mutex );

	assert( m_ref_count > 0 );
	m_ref_count--;
	if( m_ref_count == 0) {
		ASICloseCamera(m_camera);
	}

	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


bool asi_core::start_exposure() {
	int rc;

	pthread_mutex_lock( &m_mutex );
	rc = ASIStartVideoCapture(m_camera);
	pthread_mutex_unlock( &m_mutex );
	if(rc) return false;
	return true;
}

bool asi_core::abort_exposure() {
	int rc;
	pthread_mutex_lock( &m_mutex );
	rc = ASIStopVideoCapture(m_camera);
	pthread_mutex_unlock( &m_mutex );
	if(rc) return false;
    return true;
}


bool asi_core::set_camera_exposure(long exp_time) {
	exp_time *= 1000; //convert to us
	if((exp_time < m_expo_caps.MinValue) || (exp_time > m_expo_caps.MaxVale)) {
		log_e("Exposure time %d not supported", exp_time);
		return false;
	}
	pthread_mutex_lock( &m_mutex );
	int rc = ASISetControlValue(m_camera, m_expo_caps.ControlID, exp_time, ASI_FALSE);
	pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASISetControlValue(expossure): returned error %d", rc);
		return false;
	}
    return true;
}

bool asi_core::set_camera_gain(unsigned char gain) {
	if((gain < m_gain_caps.MinValue) || (gain > m_gain_caps.MaxVale)) {
		log_e("Gain %d not supported", gain);
		return false;
	}
	pthread_mutex_lock( &m_mutex );
	int rc = ASISetControlValue(m_camera, m_gain_caps.ControlID, gain, ASI_FALSE);
	pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASISetControlValue(gain): returned error %d", rc);
		return false;
	}
    return true;
}

bool asi_core::set_band_width(unsigned char bwidth) {
	if((bwidth < m_bwidth_caps.MinValue) || (bwidth > m_bwidth_caps.MaxVale)) {
		log_e("Bandwidth %d not supported", bwidth);
		return false;
	}
	pthread_mutex_lock( &m_mutex );
	int rc = ASISetControlValue(m_camera, m_bwidth_caps.ControlID, bwidth, ASI_FALSE);
	pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASISetControlValue(bandwidth): returned error %d", rc);
		return false;
	}
    return true;
}

bool asi_core::read_image(char *buf, int buf_size, long exp_time) {
	int rc;

	pthread_mutex_lock( &m_mutex );
	rc = ASIGetVideoData(m_camera,(unsigned char*)buf, buf_size, exp_time*2+1000);
	pthread_mutex_unlock( &m_mutex );
	if(rc) return false;
	return true;
}

int asi_core::set_guide_relays( int dir )
{
	pthread_mutex_lock( &m_mutex );
	//sxSetSTAR2000(m_camera, dir);
	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


void asi_core::lock( void ) const
{
	pthread_mutex_lock( &m_mutex );
}


void asi_core::unlock( void ) const
{
	pthread_mutex_unlock( &m_mutex );
}
