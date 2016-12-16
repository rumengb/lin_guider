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
ASI_CAMERA_INFO asi_core::m_cam_info = {{0},0,0,0,ASI_FALSE,ASI_BAYER_RG,{0},{ASI_IMG_END},0,ASI_FALSE,ASI_FALSE,ASI_FALSE,ASI_FALSE,ASI_FALSE,0,{0}};
ASI_CONTROL_CAPS asi_core::m_expo_caps = {{0},{0},0,0,0,ASI_FALSE,ASI_FALSE,ASI_GAIN,{0}};
bool asi_core::m_has_expo = false;
ASI_CONTROL_CAPS asi_core::m_gain_caps = {{0},{0},0,0,0,ASI_FALSE,ASI_FALSE,ASI_GAIN,{0}};
bool asi_core::m_has_gain = false;
ASI_CONTROL_CAPS asi_core::m_bwidth_caps = {{0},{0},0,0,0,ASI_FALSE,ASI_FALSE,ASI_GAIN,{0}};
bool asi_core::m_has_bwidth = false;
ASI_CONTROL_CAPS asi_core::m_wb_r_caps = {{0},{0},0,0,0,ASI_FALSE,ASI_FALSE,ASI_GAIN,{0}};
bool asi_core::m_has_wb_r = false;
ASI_CONTROL_CAPS asi_core::m_wb_b_caps = {{0},{0},0,0,0,ASI_FALSE,ASI_FALSE,ASI_GAIN,{0}};
bool asi_core::m_has_wb_b = false;
int asi_core::m_width = 0;
int asi_core::m_height = 0;
int asi_core::m_binX = 1;
int asi_core::m_binY = 1;
bool asi_core::m_clear_buffs = true;
unsigned char asi_core::m_bpp = 0;
ASI_IMG_TYPE asi_core::m_img_type = ASI_IMG_END;

void* asi_core::m_sdk_handle = NULL;
int (*asi_core::pASIGetNumOfConnectedCameras)() = NULL;
ASI_ERROR_CODE (*asi_core::pASIGetCameraProperty)(ASI_CAMERA_INFO *, int) = NULL;
ASI_ERROR_CODE (*asi_core::pASIOpenCamera)(int iCameraID) = NULL;
ASI_ERROR_CODE (*asi_core::pASIInitCamera)(int iCameraID) = NULL;
ASI_ERROR_CODE (*asi_core::pASICloseCamera)(int iCameraID) = NULL;
ASI_ERROR_CODE (*asi_core::pASIGetNumOfControls)(int iCameraID, int * piNumberOfControls) = NULL;
ASI_ERROR_CODE (*asi_core::pASIGetControlCaps)(int iCameraID, int iControlIndex, ASI_CONTROL_CAPS * pControlCaps) = NULL;
ASI_ERROR_CODE (*asi_core::pASIGetControlValue)(int  iCameraID, int  iControlID, long *plValue, ASI_BOOL *pbAuto) = NULL;
ASI_ERROR_CODE (*asi_core::pASISetControlValue)(int  iCameraID, int  iControlID, long lValue, ASI_BOOL bAuto) = NULL;
ASI_ERROR_CODE (*asi_core::pASISetROIFormat)(int iCameraID, int iWidth, int iHeight, int iBin, ASI_IMG_TYPE Img_type) = NULL;
ASI_ERROR_CODE (*asi_core::pASIGetROIFormat)(int iCameraID, int *piWidth, int *piHeight,  int *piBin, ASI_IMG_TYPE *pImg_type) = NULL;
ASI_ERROR_CODE (*asi_core::pASISetStartPos)(int iCameraID, int iStartX, int iStartY) = NULL;
ASI_ERROR_CODE (*asi_core::pASIGetStartPos)(int iCameraID, int *piStartX, int *piStartY) = NULL;
ASI_ERROR_CODE (*asi_core::pASIGetDroppedFrames)(int iCameraID,int *piDropFrames) = NULL;
ASI_ERROR_CODE (*asi_core::pASIStartVideoCapture)(int iCameraID) = NULL;
ASI_ERROR_CODE (*asi_core::pASIStopVideoCapture)(int iCameraID) = NULL;
ASI_ERROR_CODE (*asi_core::pASIGetVideoData)(int iCameraID, unsigned char* pBuffer, long lBuffSize, int iWaitms) = NULL;
ASI_ERROR_CODE (*asi_core::pASIPulseGuideOn)(int iCameraID, ASI_GUIDE_DIRECTION direction) = NULL;
ASI_ERROR_CODE (*asi_core::pASIPulseGuideOff)(int iCameraID, ASI_GUIDE_DIRECTION direction) = NULL;

int asi_core::open( void )
{
	int rc;
	ASI_CONTROL_CAPS ctrl_caps;

	pthread_mutex_lock( &m_mutex );

	if(! init_sdk()) {
		pthread_mutex_unlock( &m_mutex );
		return 1;
	}

	if (m_ref_count == 0)
	{
		int camera_count = pASIGetNumOfConnectedCameras();
		if (camera_count <=0) {
			log_e("No ZWO ASI camera found.");
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		rc = pASIGetCameraProperty(&m_cam_info, 0);
		if (rc != ASI_SUCCESS) {
			log_e("ASIGetCameraProperty() returned %d", rc);
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}
		// TODO: select a camera with guiderport and delete all other
		log_i("Camera found: %s", m_cam_info.Name);

		m_camera=m_cam_info.CameraID;
		rc = pASIOpenCamera(m_camera);
		if (rc != ASI_SUCCESS) {
			log_i("ASIOpenCamera(): Error opening camera (rc = %d).",rc);
			pthread_mutex_unlock( &m_mutex );
			return 1;
		}

		// If there is no ASIInitCamera() then we do not have to call it.
		// ASIOpenCamera() does all the initializations if SDK version < 0.4.
		if (pASIInitCamera) {
			rc = pASIInitCamera(m_camera);
			if (rc != ASI_SUCCESS) {
				log_i("ASIInitCamera(): Error opening camera (rc = %d).",rc);
				pthread_mutex_unlock( &m_mutex );
				return 1;
			}
		}

		int c_num;
		pASIGetNumOfControls(m_camera, &c_num);
		for (int i = 0; i < c_num; i++) {
			rc = pASIGetControlCaps(m_camera, i, &ctrl_caps);
			if (rc != ASI_SUCCESS) {
				log_e("ASIGetControlCaps(): error = %d\n", rc);
				pthread_mutex_unlock( &m_mutex );
				return 1;
			}
			switch (ctrl_caps.ControlType) {
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
			case ASI_WB_R:
				m_wb_r_caps = ctrl_caps;
				m_has_wb_r = true;
				break;
			case ASI_WB_B:
				m_wb_b_caps = ctrl_caps;
				m_has_wb_b = true;
				break;
			default:
				break;
			}
		}
	}

	m_ref_count++;
	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}

void asi_core::set_camera_image_type(ASI_IMG_TYPE img_type) {
	int x,y,d;
	int rc = pASIGetROIFormat(m_camera, &x, &y, &d, &m_img_type);
	if(rc) {
		log_e("ASIGetROIFormat(): returned error %d", rc);
		return;
	}
	m_img_type = img_type;
	rc = pASISetROIFormat(m_camera, x, y, d, m_img_type);
	if(rc) {
		log_e("ASIGetROIFormat(): returned error %d", rc);
		return;
	}
}

void asi_core::update_camera_image_type() {
	int x,y,d;
	int rc = pASIGetROIFormat(m_camera, &x, &y, &d, &m_img_type);
	if(rc) {
		log_e("ASIGetROIFormat(): returned error %d", rc);
		return;
	}
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
	default:
		m_bpp = 0;
		break;
	}
}

int asi_core::close( void )
{
	pthread_mutex_lock( &m_mutex );

	assert( m_ref_count > 0 );
	m_ref_count--;
	if( m_ref_count == 0) {
		pASICloseCamera(m_camera);
	}
	close_sdk();
	pthread_mutex_unlock( &m_mutex );

	return EXIT_SUCCESS;
}


bool asi_core::init_sdk() {
	if (m_sdk_handle == NULL) {
		m_sdk_handle = dlopen("libASICamera2.so", RTLD_LAZY);
		if (!m_sdk_handle) {
			log_e("Cannot load library: %s", dlerror());
			return false;
		}

		pASIGetNumOfConnectedCameras = (int (*)(void)) dlsym(m_sdk_handle, "ASIGetNumOfConnectedCameras");
		const char* dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetNumOfConnectedCameras(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		pASIGetCameraProperty = (ASI_ERROR_CODE (*)(ASI_CAMERA_INFO*, int)) dlsym(m_sdk_handle, "ASIGetCameraProperty");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetCameraProperty(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		pASIOpenCamera = (ASI_ERROR_CODE (*)(int)) dlsym(m_sdk_handle, "ASIOpenCamera");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIOpenCamera(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		// If there is no ASIInitCamera() then we do not have to call it.
		// ASIOpenCamera() does all the initializations if SDK version < 0.4.
		pASIInitCamera = (ASI_ERROR_CODE (*)(int)) dlsym(m_sdk_handle, "ASIInitCamera");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_i("Cannot load ASIInitCamera(): %s", dlsym_error);
			log_i("Assuming libasicamera version < 0.4");
			pASIInitCamera = (ASI_ERROR_CODE (*)(int)) NULL;
		}

		pASICloseCamera = (ASI_ERROR_CODE (*)(int)) dlsym(m_sdk_handle, "ASICloseCamera");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASICloseCamera(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIGetNumOfControls)(int iCameraID, int * piNumberOfControls);
		pASIGetNumOfControls = (ASI_ERROR_CODE (*)(int, int*)) dlsym(m_sdk_handle, "ASIGetNumOfControls");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetNumOfControls(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIGetControlCaps)(int iCameraID, int iControlIndex, ASI_CONTROL_CAPS * pControlCaps);
		pASIGetControlCaps = (ASI_ERROR_CODE (*)(int, int, ASI_CONTROL_CAPS*)) dlsym(m_sdk_handle, "ASIGetControlCaps");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetControlCaps(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIGetControlValue)(int  iCameraID, int  iControlID, long *plValue, ASI_BOOL *pbAuto);
		pASIGetControlValue = (ASI_ERROR_CODE (*)(int, int, long*, ASI_BOOL*)) dlsym(m_sdk_handle, "ASIGetControlValue");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetControlValue(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASISetControlValue)(int  iCameraID, int  iControlID, long lValue, ASI_BOOL bAuto);
		pASISetControlValue = (ASI_ERROR_CODE (*)(int, int, long, ASI_BOOL)) dlsym(m_sdk_handle, "ASISetControlValue");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASISetControlValue(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASISetROIFormat)(int iCameraID, int iWidth, int iHeight,  int iBin, ASI_IMG_TYPE Img_type);
		pASISetROIFormat = (ASI_ERROR_CODE (*)(int, int, int,  int, ASI_IMG_TYPE)) dlsym(m_sdk_handle, "ASISetROIFormat");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASISetROIFormat(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIGetROIFormat)(int iCameraID, int *piWidth, int *piHeight,  int *piBin, ASI_IMG_TYPE *pImg_type);
		pASIGetROIFormat = (ASI_ERROR_CODE (*)(int, int*, int*, int*, ASI_IMG_TYPE*)) dlsym(m_sdk_handle, "ASIGetROIFormat");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetROIFormat(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASISetStartPos)(int iCameraID, int iStartX, int iStartY);
		pASISetStartPos = (ASI_ERROR_CODE (*)(int, int, int)) dlsym(m_sdk_handle, "ASISetStartPos");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASISetStartPos(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIGetStartPos)(int iCameraID, int *piStartX, int *piStartY);
		pASIGetStartPos = (ASI_ERROR_CODE (*)(int, int*, int*)) dlsym(m_sdk_handle, "ASIGetStartPos");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetStartPos(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIGetDroppedFrames)(int iCameraID,int *piDropFrames);
		pASIGetDroppedFrames = (ASI_ERROR_CODE (*)(int, int*)) dlsym(m_sdk_handle, "ASIGetDroppedFrames");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetDroppedFrames(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIStartVideoCapture)(int iCameraID);
		pASIStartVideoCapture = (ASI_ERROR_CODE (*)(int)) dlsym(m_sdk_handle, "ASIStartVideoCapture");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIStartVideoCapture(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIStopVideoCapture)(int iCameraID);
		pASIStopVideoCapture = (ASI_ERROR_CODE (*)(int)) dlsym(m_sdk_handle, "ASIStopVideoCapture");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIStopVideoCapture(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIGetVideoData)(int iCameraID, unsigned char* pBuffer, long lBuffSize, int iWaitms);
		pASIGetVideoData = (ASI_ERROR_CODE (*)(int, unsigned char*, long, int)) dlsym(m_sdk_handle, "ASIGetVideoData");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIGetVideoData(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIPulseGuideOn)(int iCameraID, ASI_GUIDE_DIRECTION direction);
		pASIPulseGuideOn = (ASI_ERROR_CODE (*)(int, ASI_GUIDE_DIRECTION)) dlsym(m_sdk_handle, "ASIPulseGuideOn");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIPulseGuideOn(): %s", dlsym_error);
			close_sdk();
			return false;
		}

		//static ASI_ERROR_CODE (*pASIPulseGuideOff)(int iCameraID, ASI_GUIDE_DIRECTION direction);
		pASIPulseGuideOff = (ASI_ERROR_CODE (*)(int, ASI_GUIDE_DIRECTION)) dlsym(m_sdk_handle, "ASIPulseGuideOff");
		dlsym_error = dlerror();
		if (dlsym_error) {
			log_e("Cannot load ASIPulseGuideOff(): %s", dlsym_error);
			close_sdk();
			return false;
		}
	}
	return true;
}

bool asi_core::close_sdk() {
	if(m_sdk_handle) {
		dlclose(m_sdk_handle);
		m_sdk_handle = NULL;
	}
	return true;
}

bool asi_core::start_exposure() {
	int rc;

	usleep(150); // avoid broken frames
	pthread_mutex_lock( &m_mutex );
	rc = pASIStartVideoCapture(m_camera);
	pthread_mutex_unlock( &m_mutex );
	if(rc) return false;
	return true;
}

bool asi_core::abort_exposure() {
	int rc;
	pthread_mutex_lock( &m_mutex );
	rc = pASIStopVideoCapture(m_camera);
	pthread_mutex_unlock( &m_mutex );
	if(rc) return false;
	usleep(150); // aovid broken frames
    return true;
}

bool asi_core::set_camera_exposure(long exp_time) {
	if (!m_has_expo) return false;

	if( DBG_VERBOSITY )
		log_i( "Set exposure %d ms", exp_time);

	exp_time *= 1000; //convert to us
	if((exp_time < m_expo_caps.MinValue) || (exp_time > m_expo_caps.MaxValue)) {
		log_e("Exposure time %d not supported", exp_time);
		return false;
	}

	pthread_mutex_lock( &m_mutex );
	int rc = pASISetControlValue(m_camera, m_expo_caps.ControlType, exp_time, ASI_FALSE);
	pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASISetControlValue(expossure): returned error %d", rc);
		return false;
	}

    return true;
}

bool asi_core::set_camera_gain(unsigned int gain) {
	if (!m_has_gain) return false;
	if(((int)gain < m_gain_caps.MinValue) || ((int)gain > m_gain_caps.MaxValue)) {
		log_e("Gain %d not supported", gain);
		return false;
	}
	pthread_mutex_lock( &m_mutex );
	int rc = pASISetControlValue(m_camera, m_gain_caps.ControlType, gain, ASI_FALSE);
	pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASISetControlValue(gain): returned error %d", rc);
		return false;
	}
    return true;
}

bool asi_core::set_band_width(unsigned char bwidth) {
	if (!m_has_bwidth) return false;
	if((bwidth < m_bwidth_caps.MinValue) || (bwidth > m_bwidth_caps.MaxValue)) {
		log_e("Bandwidth %d not supported", bwidth);
		return false;
	}
	pthread_mutex_lock( &m_mutex );
	int rc = pASISetControlValue(m_camera, m_bwidth_caps.ControlType, bwidth, ASI_FALSE);
	pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASISetControlValue(bandwidth): returned error %d", rc);
		return false;
	}
    return true;
}

bool asi_core::set_wb_r(unsigned char wb_r) {
	if (!m_has_wb_r) return false;
	if((wb_r < m_wb_r_caps.MinValue) || (wb_r > m_wb_r_caps.MaxValue)) {
		log_e("WB Red value %d not supported", wb_r);
		return false;
	}
	pthread_mutex_lock( &m_mutex );
	int rc = pASISetControlValue(m_camera, m_wb_r_caps.ControlType, wb_r, ASI_FALSE);
	pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASISetControlValue(WB_R): returned error %d", rc);
		return false;
	}
	return true;
}

bool asi_core::set_wb_b(unsigned char wb_b) {
	if (!m_has_wb_b) return false;
	if((wb_b < m_wb_b_caps.MinValue) || (wb_b > m_wb_b_caps.MaxValue)) {
		log_e("WB Blue value %d not supported", wb_b);
		return false;
	}
	pthread_mutex_lock( &m_mutex );
	int rc = pASISetControlValue(m_camera, m_wb_b_caps.ControlType, wb_b, ASI_FALSE);
	pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASISetControlValue(WB_B): returned error %d", rc);
		return false;
	}
	return true;
}

bool asi_core::read_image(char *buf, int buf_size, long exp_time) {
	int rc;

	//pthread_mutex_lock( &m_mutex );
	if (m_clear_buffs) {
		rc = pASIGetVideoData(m_camera,(unsigned char*)buf, buf_size, 0); // there are 2 internal buffers that have to be cleaned
		rc = pASIGetVideoData(m_camera,(unsigned char*)buf, buf_size, 0); // This hack is suggested Sam Wen from ZWO
	}
	rc = pASIGetVideoData(m_camera,(unsigned char*)buf, buf_size, exp_time*2+1000);
	//pthread_mutex_unlock( &m_mutex );
	if(rc) {
		log_e("ASIGetVideoData(): returned error %d", rc);
		return false;
	}
	return true;
}


void asi_core::lock( void ) const
{
	pthread_mutex_lock( &m_mutex );
}


void asi_core::unlock( void ) const
{
	pthread_mutex_unlock( &m_mutex );
}
