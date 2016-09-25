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

#define V4L2_CID_USER_BANDWIDTH (V4L2_CID_USER_CLASS + 1)
#define V4L2_CID_USER_CLEAR_BUFFS (V4L2_CID_USER_CLASS + 2)
#define V4L2_CID_USER_FORCE_BW (V4L2_CID_USER_CLASS + 3)
#define V4L2_CID_USER_ASI8BIT (V4L2_CID_USER_CLASS + 4)

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
	static bool m_clear_buffs;
	static unsigned char m_bpp;
	static ASI_IMG_TYPE m_img_type;

	static ASI_CONTROL_CAPS m_expo_caps;
	static bool m_has_expo;
	static ASI_CONTROL_CAPS m_gain_caps;
	static bool m_has_gain;
	static ASI_CONTROL_CAPS m_bwidth_caps;
	static bool m_has_bwidth;
	static ASI_CONTROL_CAPS m_wb_r_caps;
	static bool m_has_wb_r;
	static ASI_CONTROL_CAPS m_wb_b_caps;
	static bool m_has_wb_b;

	static ASI_CAMERA_INFO m_cam_info;
	static int m_camera;

	int open( void );
	int close( void );
	bool start_exposure();
	bool abort_exposure();
	bool set_camera_exposure(long exp_time);
	bool set_camera_gain(unsigned int gain);
	void set_camera_image_type(ASI_IMG_TYPE img_type);
	void update_camera_image_type();
	bool set_band_width(unsigned char bwidth);
	bool set_wb_r(unsigned char wb_r);
	bool set_wb_b(unsigned char wb_b);
	bool read_image(char *buf, int buf_size, long exp_time);
	void lock( void ) const;
	void unlock( void ) const;

	//ASI SDK stuff
	bool init_sdk();
	bool close_sdk();
	static void *m_sdk_handle;
	static int (*pASIGetNumOfConnectedCameras)();
	static ASI_ERROR_CODE (*pASIGetCameraProperty)(ASI_CAMERA_INFO *pASICameraInfo, int iCameraIndex);
	static ASI_ERROR_CODE (*pASIOpenCamera)(int iCameraID);
	static ASI_ERROR_CODE (*pASIInitCamera)(int iCameraID);
	static ASI_ERROR_CODE (*pASICloseCamera)(int iCameraID);
	static ASI_ERROR_CODE (*pASIGetNumOfControls)(int iCameraID, int * piNumberOfControls);
	static ASI_ERROR_CODE (*pASIGetControlCaps)(int iCameraID, int iControlIndex, ASI_CONTROL_CAPS * pControlCaps);
	static ASI_ERROR_CODE (*pASIGetControlValue)(int  iCameraID, int  iControlID, long *plValue, ASI_BOOL *pbAuto);
	static ASI_ERROR_CODE (*pASISetControlValue)(int  iCameraID, int  iControlID, long lValue, ASI_BOOL bAuto);
	static ASI_ERROR_CODE (*pASISetROIFormat)(int iCameraID, int iWidth, int iHeight,  int iBin, ASI_IMG_TYPE Img_type);
	static ASI_ERROR_CODE (*pASIGetROIFormat)(int iCameraID, int *piWidth, int *piHeight,  int *piBin, ASI_IMG_TYPE *pImg_type);
	static ASI_ERROR_CODE (*pASISetStartPos)(int iCameraID, int iStartX, int iStartY);
	static ASI_ERROR_CODE (*pASIGetStartPos)(int iCameraID, int *piStartX, int *piStartY);
	static ASI_ERROR_CODE (*pASIGetDroppedFrames)(int iCameraID,int *piDropFrames);
	static ASI_ERROR_CODE (*pASIStartVideoCapture)(int iCameraID);
	static ASI_ERROR_CODE (*pASIStopVideoCapture)(int iCameraID);
	static ASI_ERROR_CODE (*pASIGetVideoData)(int iCameraID, unsigned char* pBuffer, long lBuffSize, int iWaitms);
	static ASI_ERROR_CODE (*pASIPulseGuideOn)(int iCameraID, ASI_GUIDE_DIRECTION direction);
	static ASI_ERROR_CODE (*pASIPulseGuideOff)(int iCameraID, ASI_GUIDE_DIRECTION direction);

private:
	static int m_ref_count;
	static pthread_mutex_t m_mutex;
};
#endif /* ASI_CORE_H_ */
