/*
 * video.cpp
 *
 *      Author: gm
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

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <asm/types.h>		/* for videodev2.h */
#include <linux/videodev2.h>

#include <map>
#include <utility>

#include "maindef.h"
#include "video.h"
#include "decoder.h"
#include "utils.h"
#include "timer.h"
#include "pwc-ioctl.h"
#include "bayer.h"

// tmp for test
#include "lin_guider.h"

namespace video_drv
{

device_desc_t device_desc_list[DEVICE_CNT] = {
												{
													DT_NULL,
													false,
													"Null (default)",
													"Simulated camera",
													"<html><head/><body><p>Simulated camera.</p></body></html>"
												},
												{
													DT_WEBCAM,
													true,
													"Web Camera",
													NULL,
													NULL
												},
												{
													DT_DSI2PRO,
													false,
													"DSI 2 Pro",
													"Camera firmware is required",
													"<html><head/><body><p>Camera <a href=\"https://sourceforge.net/projects/cccd/files/firmware/\"><span style=\"text-decoration: underline; color:#0000ff;\">firmware</span></a> is required.</p></body></html>"
												},
												{
													DT_QHY5,
													false,
													"QHY 5",
													"Camera firmware is required",
													"<html><head/><body><p>Camera <a href=\"https://sourceforge.net/projects/cccd/files/firmware/\"><span style=\"text-decoration: underline; color:#0000ff;\">firmware</span></a> is required.</p></body></html>"
												},
												{
													DT_QHY6,
													false,
													"QHY 6",
													"Camera firmware is required",
													"<html><head/><body><p>Camera <a href=\"https://sourceforge.net/projects/cccd/files/firmware/\"><span style=\"text-decoration: underline; color:#0000ff;\">firmware</span></a> is required.</p></body></html>"
												},
												{
													DT_QHY5II,
													false,
													"QHY 5II / 5LII",
													"Camera firmware is required",
													"<html><head/><body><p>Camera <a href=\"https://sourceforge.net/projects/cccd/files/firmware/\"><span style=\"text-decoration: underline; color:#0000ff;\">firmware</span></a> is required.</p></body></html>"
												},
												{
													DT_ATIK,
													false,
													"Atik",
													"Atikccdsdk is required",
													"<html><head/><body><p>External library <a href=\"https://sourceforge.net/projects/linguider/files/atik_sdk/\"><span style=\"text-decoration: underline; color:#0000ff;\">atikccdsdk</span></a> is required.</p></body></html>"
												},
												{
													DT_SX,
													false,
													"Starlight Xpress",
													"To use SX camera without root, firmware bundle is required",
													"<html><head/><body><p>To use SX camera without root, <a href=\"https://sourceforge.net/projects/cccd/files/firmware/\"><span style=\"text-decoration: underline; color:#0000ff;\">firmware</span></a> bundle is required.</p></body></html>"
												},
												{
													DT_ASI,
													false,
													"ZWO ASI",
													#ifdef __arm__
													"libasicamera is required (Highly unstable on ARM)",
													"<html><head/><body><p>External library <a href=\"https://sourceforge.net/projects/linguider/files/asi_sdk/\"><span style=\"text-decoration: underline; color:#0000ff;\">libasicamera</span></a> is required (Highly unstable on ARM).</p></body></html>"
													#else
													"Libasicamera is required",
													"<html><head/><body><p>External library <a href=\"https://sourceforge.net/projects/linguider/files/asi_sdk/\"><span style=\"text-decoration: underline; color:#0000ff;\">libasicamera</span></a> is required.</p></body></html>"
													#endif
												}

											};

int cvideo_base::enum_frame_intervals( unsigned int pixfmt, unsigned int width, unsigned int height, int fmt_idx, int frm_idx )
{
	int ret;
	struct v4l2_frmivalenum fival;

	memset(&fival, 0, sizeof(fival));
	fival.index = 0;
	fival.pixel_format = pixfmt;
	fival.width = width;
	fival.height = height;
	log_i("\tTime interval between frames: ");

	while( (ret = xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0 )
	{
		if( fival.type == V4L2_FRMIVAL_TYPE_DISCRETE )
		{
			log_i("%u/%u, ", fival.discrete.numerator, fival.discrete.denominator);

			if( fival.index < (unsigned)MAX_FMT )
			{
				device_formats[ fmt_idx ].frame_table[ frm_idx ].fps_table[fival.index].numerator = fival.discrete.numerator;
				device_formats[ fmt_idx ].frame_table[ frm_idx ].fps_table[fival.index].denominator = fival.discrete.denominator;
			}
			else
				log_e( "Refusing interval - param list is full." );
		}
		else
		if( fival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS )
		{
			log_i("{min { %u/%u } .. max { %u/%u } }, ",
					fival.stepwise.min.numerator, fival.stepwise.min.numerator,
					fival.stepwise.max.denominator, fival.stepwise.max.denominator);
			break;
		}
		else
		if( fival.type == V4L2_FRMIVAL_TYPE_STEPWISE )
		{
			log_i("{min { %u/%u } .. max { %u/%u } / "
					"stepsize { %u/%u } }, ",
					fival.stepwise.min.numerator, fival.stepwise.min.denominator,
					fival.stepwise.max.numerator, fival.stepwise.max.denominator,
					fival.stepwise.step.numerator, fival.stepwise.step.denominator);
			break;
		}
		fival.index++;
	}
	log_i("");

	if( ret != 0 && errno != EINVAL )
	{
		perror("ERROR enumerating frame intervals");
		return errno;
	}

	return 0;
}

int cvideo_base::enum_frame_sizes( unsigned int pixfmt, int fmt_idx )
{
	int ret;
	struct v4l2_frmsizeenum fsize;

	memset(&fsize, 0, sizeof(fsize));
	fsize.index = 0;
	fsize.pixel_format = pixfmt;

	while( (ret = xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0 )
	{
		if( fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE )
		{
			log_i("{ discrete: width = %u, height = %u }", fsize.discrete.width, fsize.discrete.height);

			if( fsize.index < (unsigned)MAX_FMT )
			{
				device_formats[ fmt_idx ].frame_table[fsize.index].size.x = fsize.discrete.width;
				device_formats[ fmt_idx ].frame_table[fsize.index].size.y = fsize.discrete.height;


				ret = enum_frame_intervals( pixfmt, fsize.discrete.width, fsize.discrete.height, fmt_idx, fsize.index );

				if( ret != 0 )
					log_e( "Unable to enumerate frame sizes." );
			}
			else
				log_e( "Refusing frame size - param list is full." );
		}
		else
		if( fsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS )
		{
			log_i("{ continuous: min { width = %u, height = %u } .. "
					"max { width = %u, height = %u } }",
					fsize.stepwise.min_width, fsize.stepwise.min_height,
					fsize.stepwise.max_width, fsize.stepwise.max_height);
			log_i( "Refusing to enumerate frame intervals." );
			break;
		}
		else
		if( fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE )
		{
			log_i("{ stepwise: min { width = %u, height = %u } .. "
					"max { width = %u, height = %u } / "
					"stepsize { width = %u, height = %u } }",
					fsize.stepwise.min_width, fsize.stepwise.min_height,
					fsize.stepwise.max_width, fsize.stepwise.max_height,
					fsize.stepwise.step_width, fsize.stepwise.step_height);
			log_i( "Refusing to enumerate frame intervals." );
			break;
		}
		fsize.index++;
	}
	if( ret != 0 && errno != EINVAL )
	{
		perror("ERROR enumerating frame sizes");
		return errno;
	}

	return 0;
}


int  cvideo_base::enum_frame_formats( void )
{
	int ret;
	struct v4l2_fmtdesc fmt;

	memset(&fmt, 0, sizeof(fmt));
	fmt.index = 0;
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	while( (ret = xioctl(fd, VIDIOC_ENUM_FMT, &fmt)) == 0 )
	{
		log_i("{ pixelformat = '%c%c%c%c', description = '%s' }",
						fmt.pixelformat & 0xFF, (fmt.pixelformat >> 8) & 0xFF,
						(fmt.pixelformat >> 16) & 0xFF, (fmt.pixelformat >> 24) & 0xFF,
						fmt.description);

		ret = enum_frame_sizes( fmt.pixelformat, fmt.index );

		if( ret != 0 )
			log_e( "Unable to enumerate frame sizes." );

		if( fmt.index < MAX_FMT )
		{
			device_formats[ fmt.index ].format = fmt.pixelformat;
		}
		else
			log_e( "Refusing format - param list is full." );

		fmt.index++;
	}
	if( errno != EINVAL )
	{
		perror("ERROR enumerating frame formats");
		return errno;
	}

	return 0;
}


cam_control_t *cvideo_base::add_control( int fd, struct v4l2_queryctrl *queryctrl, cam_control_t *control, int *nctrl, bool ext_ctl )
{
    int n = *nctrl;


    control = (cam_control_t *)realloc( control, sizeof( cam_control_t )*(n+1) );
    control[n].i = n;
    control[n].id = queryctrl->id;
    control[n].type = (cam_control_type_t)queryctrl->type;
    //allocate control name (must free it on exit)
    control[n].name = strdup((const char *)queryctrl->name);
    control[n].min = queryctrl->minimum;
    control[n].max = queryctrl->maximum;
    control[n].step = queryctrl->step;
    control[n].default_val = queryctrl->default_value;
    control[n].enabled = (queryctrl->flags & V4L2_CTRL_FLAG_GRABBED) ? 0 : 1;
    control[n].entries = NULL;

    log_i("Control %s id=%u", queryctrl->name, queryctrl->id);

    if( queryctrl->type == V4L2_CTRL_TYPE_BOOLEAN )
    {
            control[n].min = 0;
            control[n].max = 1;
            control[n].step = 1;
            /*get the first bit*/
            control[n].default_val=(queryctrl->default_value & 0x0001);
    }
    else
    if( queryctrl->type == V4L2_CTRL_TYPE_MENU && fd != -1 )
    {
            struct v4l2_querymenu querymenu;
            memset(&querymenu,0,sizeof(struct v4l2_querymenu));
            control[n].min = 0;
            querymenu.id = queryctrl->id;
            querymenu.index = 0;
            while ( xioctl(fd, VIDIOC_QUERYMENU, &querymenu) == 0 )
            {
                    //allocate entries list
                    control[n].entries = (char**)realloc( control[n].entries, sizeof(char*)*(querymenu.index+1) );
                    //allocate entrie name
                    control[n].entries[querymenu.index] = strdup( (char *) querymenu.name );
                    querymenu.index++;
            }
            control[n].max = querymenu.index - 1;
    }
    n++;
    *nctrl = n;

    if( ext_ctl )
    	m_ext_ctls.insert( std::make_pair( queryctrl->id, std::string((const char *)queryctrl->name) ) );

    // log
    switch( queryctrl->id )
    {
    case V4L2_CID_AUTOGAIN:
    	log_i("Autogain supported");
    	break;
    case V4L2_CID_GAIN:
    	log_i("Gain supported");
    	break;
    case V4L2_CID_EXPOSURE:
        log_i("Exposure supported");
        break;
    }

 return control;
}


int cvideo_base::enum_controls( void )
{
	int ret = 0;
	int tries = IOCTL_RETRY;
	//InputControl * control = NULL;
	int n = 0;
	struct v4l2_queryctrl queryctrl;


	memset(&queryctrl,0,sizeof(struct v4l2_queryctrl));

	queryctrl.id = 0 | V4L2_CTRL_FLAG_NEXT_CTRL;

	if( (ret=xioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) == 0 )
	{
		log_i("V4L2_CTRL_FLAG_NEXT_CTRL supported");
		// The driver supports the V4L2_CTRL_FLAG_NEXT_CTRL flag
		queryctrl.id = 0;
		unsigned currentctrl = queryctrl.id;
		queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;

		// Loop as long as ioctl does not return EINVAL
		// don't use xioctl here since we must reset queryctrl.id every retry (is this realy true ??)
		while( (ret = /*v4l2_*/ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)), ret ? errno != EINVAL : 1 )
		{

			if( ret && (errno == EIO || errno == EPIPE || errno == ETIMEDOUT) )
			{
				// I/O error RETRY
				queryctrl.id = currentctrl | V4L2_CTRL_FLAG_NEXT_CTRL;
				tries = IOCTL_RETRY;
				while(tries-- && (ret = ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) && (errno == EIO || errno == EPIPE || errno == ETIMEDOUT))
				{
					queryctrl.id = currentctrl | V4L2_CTRL_FLAG_NEXT_CTRL;
				}
				if ( ret && ( tries <= 0))
					log_e("Failed to query control id=%d tried %i times - giving up: %s", currentctrl, IOCTL_RETRY, strerror(errno));
			}
			// Prevent infinite loop for buggy NEXT_CTRL implementations
			if( ret && queryctrl.id <= currentctrl )
			{
				currentctrl++;
				goto next_control;
			}
			currentctrl = queryctrl.id;

			// skip if control is disabled or failed
			if( ret || (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) )
				goto next_control;

			// Add control to control list
			controls = add_control( fd, &queryctrl, controls, &n );

next_control:
			queryctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
		}
	}
	else //NEXT_CTRL flag not supported, use old method
	{
		log_i("V4L2_CTRL_FLAG_NEXT_CTRL not supported");
		unsigned currentctrl;
		for( currentctrl = V4L2_CID_BASE; currentctrl < V4L2_CID_LASTP1; currentctrl++ )
		{
			queryctrl.id = currentctrl;
			ret = xioctl(fd, VIDIOC_QUERYCTRL, &queryctrl);

			if( ret || (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) )
				continue;
			// Add control to control list
			controls = add_control( fd, &queryctrl, controls, &n );
		}

		for( currentctrl = V4L2_CID_CAMERA_CLASS_BASE; currentctrl < V4L2_CID_CAMERA_CLASS_LAST; currentctrl++ )
		{
			queryctrl.id = currentctrl;
			// Try querying the control
			ret = xioctl(fd, VIDIOC_QUERYCTRL, &queryctrl);

			if( ret || (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) )
				continue;
			// Add control to control list
			controls = add_control( fd, &queryctrl, controls, &n );
		}

		for( currentctrl = V4L2_CID_BASE_LOGITECH; currentctrl < V4L2_CID_LAST_EXTCTR; currentctrl++ )
		{
			queryctrl.id = currentctrl;
			// Try querying the control
			ret = xioctl(fd, VIDIOC_QUERYCTRL, &queryctrl);

			if( ret || (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) )
				continue;
			// Add control to control list
			controls = add_control( fd, &queryctrl, controls, &n );
		}
	}

	num_controls = n;

 return 0;
}


void cvideo_base::free_controls( void )
{
 int i = 0;


	for( i = 0; i < num_controls;i++ )
	{
		//clean control name
		free( controls[i].name);
		if( controls[i].type == INPUT_CONTROL_TYPE_MENU )
		{
			int j;
			for( j = 0; j <= controls[i].max; j++ )
			{
				//clean entrie name
				free( controls[i].entries[j] );
			}
			//clean entries list
			free( controls[i].entries );
		}
	}
	//clean control lists
	free( controls );
	controls = NULL;
}




cvideo_base::cvideo_base()
{
	init_decoder();

	initialized = false;

	memset( dev_name, 0, sizeof(dev_name) );
	strcpy( dev_name, "/dev/video0" );
	device_type = DT_NULL;
	next_device_type = DT_NULL;

	fd = -1;

	next_params.type    = 0;
	next_params.width   = 0;
	next_params.height  = 0;
	
	force_fps = false;

	controls = NULL;
	num_controls = 0;

	frame_delay = time_fract::to_msecs( capture_params.fps );

	buffers 	       = NULL;
	n_buffers 	       = 0;
	
    calibration_frame_cnt = 1;
	calibration_frame     = 0;
	is_calibrating        = false;
	have_calibration      = false;

	is_streaming = false;

	memset( &posted_param, 0, sizeof(posted_param) );

	memset( device_formats, 0, sizeof(device_formats) );
	for( int i = 0;i < MAX_FMT;i++ )
	{
		for( int j = 0;j < MAX_FMT;j++ )
		{
			for( int k = 0;k < 3 && k < MAX_FMT;k++ )
				device_formats[i].frame_table[j].fps_table[k] = time_fract::mk_fps( 1, (k+1)*5 );
		}
	}

	lut_to8bit.start = malloc( sizeof(unsigned char) * lut_to8bit_len );
	init_lut_to8bit();

	is_v4l_1 = false;

}


cvideo_base::~cvideo_base()
{
	if( tmp_buffer.ptr )
		free( tmp_buffer.ptr );

	free_decoder();

	if( calibration_buffer.start.ptr )
		free( calibration_buffer.start.ptr );

	if( controls )
		free_controls();

	if( lut_to8bit.start.ptr )
		free( lut_to8bit.start.ptr );

}


bool cvideo_base::start( const char *devname )
{
 int len;
 int ret;

 	if( initialized )
 		return false;

 	// prepare...
	len =  strlen(devname);
	if( len > (int)(ARRAY_SIZE(dev_name))-1 || len <= 0 )
		return false;
	snprintf( dev_name, sizeof(dev_name), "%s", devname );

	// open device
	ret = open_device();
	if( ret != 0 )
		return false;

	// get capabilities
	ret = get_vcaps();
	if( ret != 0 )
	{
		close_device();
	  	return false;
	}

	// init device
	ret = init_device();
	if( ret != 0 )
	{
		close_device();
		return false;
	}

	// init calibration buffer
	if( is_color() )
	{
		calibration_buffer.length = sizeof(double) * 3 * capture_params.width * capture_params.height;
	}
	else
	{
		calibration_buffer.length = sizeof(double) * capture_params.width * capture_params.height;
	}
	calibration_buffer.start.ptr = malloc( calibration_buffer.length );

	// setup for capturing
	ret = start_capturing();
	if( ret != 0 )
	{
		uninit_device();
		close_device();
		return false;
	}

	// init thread staff
	pthread_mutex_init( &cv_mutex, NULL );
	pthread_cond_init( &cv, NULL );
	start_thread_flag 	= 0;
	quit_thread_flag 	= 0;
	data_thread_flag	= 0;

	pthread_create( &dev_thread, NULL, &cvideo_base::video_thread, this );

	// ok
	initialized = true;

 return true;
}


void cvideo_base::stop( void )
{
	if( !initialized )
		return;

	// stop thread
	pthread_mutex_lock( &cv_mutex );
	start_thread_flag = 1;
	quit_thread_flag = 1;
	data_thread_flag = 1;
	pthread_cond_signal( &cv );
	pthread_mutex_unlock( &cv_mutex );

	pthread_join( dev_thread, NULL );

	// release stuff
	pthread_mutex_destroy( &cv_mutex );
	pthread_cond_destroy( &cv );


	stop_capturing();

	uninit_device();

	close_device();

	initialized = false;
}


captureparams_t cvideo_base::get_capture_params( void ) const
{
 return capture_params;
}


void cvideo_base::set_capture_params( const captureparams_t &v )
{
	capture_params = v;
}


capture_next_params_t cvideo_base::get_next_params( void ) const
{
 return next_params;
}


void cvideo_base::set_next_params( const capture_next_params_t &v )
{
	next_params = v;
}


current_format_state_t cvideo_base::get_current_format_params( void )
{
 current_format_state_t fmt_state;
 int idx = get_format_idx();

	fmt_state.format_desc	= idx >= 0 ? &device_formats[ idx ] : NULL;

	fmt_state.format_idx 	= idx;
	fmt_state.frame_idx 	= get_frame_idx();
	fmt_state.fps_idx		= get_fps_idx();

 return fmt_state;
}


void cvideo_base::get_current_format_params_string( char *str, size_t str_sz, int ovrr_fps_idx )
{
	if( !str || !str_sz )
		return;

	current_format_state_t format_state = get_current_format_params();
	int len = 0;
	if( format_state.format_desc )
	{
		int frame_idx = format_state.frame_idx;
		int fps_idx   = ovrr_fps_idx == -1 ? format_state.fps_idx : ovrr_fps_idx;
		if( frame_idx != -1 )
		{
			len += snprintf( str+len, str_sz-len, "Frame: %dx%d", format_state.format_desc->frame_table[frame_idx].size.x,
															 format_state.format_desc->frame_table[frame_idx].size.y );
			if( fps_idx != -1 )
				len += snprintf( str+len, str_sz-len, ", Expo: %.2gs", video_drv::time_fract::to_msecs( format_state.format_desc->frame_table[frame_idx].fps_table[fps_idx] ) / 1000.0 );
		}
	}
}


int cvideo_base::get_format_idx( void ) const
{
 int i;

	for( i = 0;i < MAX_FMT;i++ )
	{
		if( device_formats[i].format == capture_params.pixel_format )
			return i;
	}

 return -1;
}


int cvideo_base::get_frame_idx( void ) const
{
 int i;
 int format_idx = get_format_idx();

	 if( format_idx < 0 )
		 return -1;

	 for( i = 0;i < MAX_FMT;i++ )
	 {
		 if( device_formats[format_idx].frame_table[i].size.x == (int)capture_params.width &&
			 device_formats[format_idx].frame_table[i].size.y == (int)capture_params.height )
			 return i;
	 }

 return -1;
}


int cvideo_base::get_fps_idx( void ) const
{
 int i;
 int format_idx = get_format_idx();
 int frame_idx = get_frame_idx();

	 if( format_idx < 0 || frame_idx < 0 )
 		 return -1;

	 for( i = 0;i < MAX_FMT;i++ )
	 {
		 if( device_formats[format_idx].frame_table[frame_idx].fps_table[i] == capture_params.fps )
			 return i;
	 }

 return -1;
}


void cvideo_base::pause( bool set )
{
	if( !initialized )
	 	return;

	pthread_mutex_lock( &cv_mutex );

	if( set != (bool)!start_thread_flag )
	{
		start_thread_flag = !set;
		pthread_cond_signal( &cv );
	}

	pthread_mutex_unlock( &cv_mutex );

}


void cvideo_base::continue_capture( void )
{
	if( !initialized )
		return;

	pthread_mutex_lock( &cv_mutex );

	data_thread_flag = 1;

	pthread_cond_signal( &cv );
	pthread_mutex_unlock( &cv_mutex );
}


void cvideo_base::process_frame( void *video_dst, int video_dst_size, void *math_dst, int math_dst_size, const void *src, bool is_grey, int bytesused )
{
#define HEADERFRAME1 0xaf

	int i, j, pix_no;
	u_char *pdst = (u_char*)video_dst;	// video buffer destination
	double *mdst = (double *)math_dst; // math buffer destination
	data_ptr psrc( src );
	data_ptr pdecoded;
	int bits = bpp();
	bool is_webcam = false;
	bool render_calibrated = capture_params.use_calibration && have_calibration;
	//bool threat_as_color = (capture_params.pixel_format != V4L2_PIX_FMT_SGRBG8) && is_color();

	unsigned char *py, *pu, *pv;

	int width, height;
	bool reverse = false;

	pix_no = capture_params.width * capture_params.height;

	// check params
	if( pix_no * sizeof(uint32_t) != (unsigned)video_dst_size )
	{
		log_e( "cvideo_base::process_frame(): video_dst_size = %d != estimated size = %d", video_dst_size, pix_no * sizeof(uint32_t) );
		return;
	}
	if( pix_no * sizeof(double) != (unsigned)math_dst_size )
	{
		log_e( "cvideo_base::process_frame(): math_dst_size = %d != estimated size = %d", math_dst_size, pix_no * sizeof(double) );
		return;
	}

	// perform decoding
	// NOTE!!!
	// decoder MUST can render rgba 32bit image if no calibration used. If calibration is used so calibration code renders rgba 32bit output image
	// decoder MUST store decoded data in 'pdecoded' pointer
	switch( capture_params.pixel_format )
	{
	case 0:
	case V4L2_PIX_FMT_YVU420:
		reverse = true;
	case V4L2_PIX_FMT_YUV420:
	{
		if( is_grey )
		{
			for( i = 0, j = 0;i < pix_no;i++, j+=4 )
				pdst[j]   = 
				pdst[j+1] = 
				pdst[j+2] = psrc.ptr8[i];
		}
		else
		{
			py = psrc.ptr8;
			if( !reverse )
			{
				pu = py + pix_no;
				pv = pu + pix_no / 4;
			}
			else
			{
				pv = py + pix_no;
				pu = pv + pix_no / 4;
			}
			convert_yuv420p_to_rgb32( capture_params.width, capture_params.height, py, pu, pv, pdst );
		}
		pdecoded.ptr8 = pdst;
		is_webcam = true;
		break;
	}
	case V4L2_PIX_FMT_JPEG:
	case V4L2_PIX_FMT_MJPEG:
	{
		if( bytesused <= HEADERFRAME1 )
		{	/* Prevent crash on empty image */
			/*	    if(debug)*/
			log_e("Ignoring empty buffer ...");
			return;
		}
		width = capture_params.width;
		height = capture_params.height;

		if( jpeg_decode(&tmp_buffer.ptr8, psrc.ptr8, &width, &height) < 0 )
		{
			log_e("jpeg decode errors");
		}
		else
		if( tmp_buffer.ptr )
		{
			width = capture_params.width;
			height = capture_params.height;

			if( is_grey )
			{
				for( i = 0, j = 0;i < pix_no;i++, j+=4 )
					pdst[j]   =
					pdst[j+1] =
					pdst[j+2] = tmp_buffer.ptr8[i<<1];
			}
			else
			{
				convert_yuv422_to_rgb32( tmp_buffer.ptr8, pdst, width, height );
			}
			pdecoded.ptr8 = pdst;
		}
		is_webcam = true;
		break;
	}
	case V4L2_PIX_FMT_YUYV:
	{
		if( is_grey )
		{
			int pix_no2 = pix_no << 1;
			for( i = 1; i < pix_no2; i += 2 )//just operating on the chroma
				*(psrc.ptr8 + i) = 0x80;     //set all chroma to midpoint 0x80
		}
		convert_yuv422_to_rgb32( psrc.ptr8, pdst, capture_params.width, capture_params.height );
		pdecoded.ptr8 = pdst;
		is_webcam = true;
		break;
	}
	case V4L2_PIX_FMT_GREY:
		pdecoded.ptr8 = psrc.ptr8;
		break;
	case V4L2_PIX_FMT_Y16:
		pdecoded.ptr16 = psrc.ptr16;
		break;
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR12:
	{
		int data_len = pix_no * 6;
		if( !tmp_buffer.ptr ) tmp_buffer.ptr = malloc(data_len);
		if( tmp_buffer.ptr == NULL)
		{
			log_e("%s(): Can not allocate tmp_buffer", __FUNCTION__ );
			return;
		}
#ifndef __arm__
		int cell_no = pix_no * 3;
		bayer_to_rgb48(psrc.ptr16, tmp_buffer.ptr16, capture_params.width, capture_params.height, capture_params.pixel_format);
		if( is_grey )
		{
			for( i = 0;i < cell_no;i +=3 )
			{
				tmp_buffer.ptr16[i]   =
				tmp_buffer.ptr16[i+1] =
				tmp_buffer.ptr16[i+2] = (uint16_t)((tmp_buffer.ptr16[i+2] + tmp_buffer.ptr16[i+1] + tmp_buffer.ptr16[i]) / 3);
			}
		}
#else
		assert( tmp_buffer.ptr );
		for( i = 0, j = 0;i < pix_no; i ++, j += 3 )
		{
			tmp_buffer.ptr16[j] =
			tmp_buffer.ptr16[j+1] =
			tmp_buffer.ptr16[j+2] = psrc.ptr16[i];
		}
#endif
		pdecoded.ptr16 = tmp_buffer.ptr16;
	}
		break;
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SBGGR8:
	{
		int data_len = pix_no * 3;
		if( !tmp_buffer.ptr ) tmp_buffer.ptr = malloc(data_len);
		if( tmp_buffer.ptr == NULL )
		{
			log_e("%s(): Can not allocate tmp_buffer", __FUNCTION__ );
			return;
		}
#ifndef __arm__
		bayer_to_rgb24(psrc.ptr8, tmp_buffer.ptr8, capture_params.width, capture_params.height, capture_params.pixel_format);
		if( is_grey )
		{
			for( i = 0, j = 0;i < data_len; i +=3, j += 4 )
			{
				tmp_buffer.ptr8[i]   =
				tmp_buffer.ptr8[i+1] =
				tmp_buffer.ptr8[i+2] = (u_char)((tmp_buffer.ptr8[i+2] + tmp_buffer.ptr8[i+1] + tmp_buffer.ptr8[i]) / 3);
			}
		}
#else
		assert( tmp_buffer.ptr );
		for( i = 0, j = 0;i < pix_no; i ++, j += 3 )
		{
			tmp_buffer.ptr8[j] =
			tmp_buffer.ptr8[j+1] =
			tmp_buffer.ptr8[j+2] = psrc.ptr8[i];
		}
#endif
		pdecoded.ptr8 = tmp_buffer.ptr8;
	}
		break;
	default:
	{
		log_e("Frame of unknown format grabbed!");
		return;
	}
	}

	// apply calibration frame to data buffer and render output rgba 32bit image
	if( render_calibrated )
	{
		int val = 0;

		int cell_no = pix_no;
		if( is_color() ) cell_no = pix_no * 3;

		if( is_webcam )
		{
			for( i = 0, j = 0;i < cell_no;i+=3, j+=4 )
			{
				val = (int)pdecoded.ptr8[j]   - (int)calibration_buffer.start.ptrDBL[i];
				pdecoded.ptr8[j]   = (u_char)(val < 0 ? 0 : val);
				val = (int)pdecoded.ptr8[j+1] - (int)calibration_buffer.start.ptrDBL[i+1];
				pdecoded.ptr8[j+1] = (u_char)(val < 0 ? 0 : val);
				val = (int)pdecoded.ptr8[j+2] - (int)calibration_buffer.start.ptrDBL[i+2];
				pdecoded.ptr8[j+2] = (u_char)(val < 0 ? 0 : val);
			}
		}
		else
		if( bits == 8 )
		{
			for( i = 0; i < cell_no; i++ )
			{
				val = (int)pdecoded.ptr8[i] - (int)calibration_buffer.start.ptrDBL[i];
				pdecoded.ptr8[i] = (u_char)(val < 0 ? 0 : val);
			}
		}
		else
		if( bits == 16 )
		{
			for( i = 0; i < cell_no; i++ )
			{
				val = (int)pdecoded.ptr16[i] - (int)calibration_buffer.start.ptrDBL[i];
				pdecoded.ptr16[i] = (uint16_t)(val < 0 ? 0 : val);
			}
		}
	}

	// perform calibration
	if( is_calibrating )
	{
		int cell_no = pix_no;
		if( is_color() ) cell_no = pix_no * 3;

		// accumulating frames
		if ( is_webcam )
		{
			for( i = 0, j = 0;i < cell_no;i+=3, j+=4 )
			{
				calibration_buffer.start.ptrDBL[i]   += (double)pdecoded.ptr8[j];
				calibration_buffer.start.ptrDBL[i+1] += (double)pdecoded.ptr8[j+1];
				calibration_buffer.start.ptrDBL[i+2] += (double)pdecoded.ptr8[j+2];
			}
		}
		else
		if( bits == 8 )
		{
			for( i = 0; i < cell_no; i++ )
				calibration_buffer.start.ptrDBL[i] += (double)pdecoded.ptr8[i];
		}
		else
		if( bits == 16 )
		{
			for( i = 0; i < cell_no; i++ )
				calibration_buffer.start.ptrDBL[i] += (double)pdecoded.ptr16[i];
		}

		if( calibration_frame < calibration_frame_cnt )
			calibration_frame++;
		else	// done!
		{
			for( i = 0; i < cell_no; i++ )
				calibration_buffer.start.ptrDBL[i] /= (double)calibration_frame_cnt;

			is_calibrating = false;
			have_calibration = true;

			emit calibrationFinished();
		}
	}

	// fill floating point math buffer
	if( mdst )
	{
		if( is_color() )
		{
			if ( is_webcam )
			{
				for( i = 0, j = 0;i < pix_no;i++, j+=4 )
					mdst[i] = (double)(pdecoded.ptr8[j] + pdecoded.ptr8[j+1] + pdecoded.ptr8[j+2]);
			}
			else
			if( bits == 8 )
			{
				for( i = 0, j = 0;i < pix_no;i++, j+=3 )
					mdst[i] = (double)(pdecoded.ptr8[j] + pdecoded.ptr8[j+1] + pdecoded.ptr8[j+2]);
			}
			else
			if( bits == 16 )
			{
				for( i = 0, j = 0;i < pix_no;i++, j+=3 )
					mdst[i] = (double)(pdecoded.ptr16[j] + pdecoded.ptr16[j+1] + pdecoded.ptr16[j+2]);
			}
		}
		else
		{
			if( bits == 8 )
			{
				for( i = 0;i < pix_no;i++ )
					mdst[i] = (double)pdecoded.ptr8[i];
			}
			else
			if( bits == 16 )
			{
				for( i = 0;i < pix_no;i++ )
					mdst[i] = (double)pdecoded.ptr16[i];
			}
		}
	}

	// finalize - apply LUT
	if( is_color() )
	{
		if( is_webcam )
		{
			int cell_no = pix_no << 2;
			for( i = 0;i < cell_no;i+=4 )
			{
				pdst[i]   = lut_to8bit.start.ptr8[ pdecoded.ptr8[i] ];
				pdst[i+1] = lut_to8bit.start.ptr8[ pdecoded.ptr8[i+1] ];
				pdst[i+2] = lut_to8bit.start.ptr8[ pdecoded.ptr8[i+2] ];
			}
		}
		else
		if( bits == 8 )
		{
			int cell_no = pix_no << 2;
			for( i = 0, j = 0;i < cell_no;i+=4, j+=3 )
			{
				pdst[i]   = lut_to8bit.start.ptr8[ pdecoded.ptr8[j+2] ];
				pdst[i+1] = lut_to8bit.start.ptr8[ pdecoded.ptr8[j+1] ];
				pdst[i+2] = lut_to8bit.start.ptr8[ pdecoded.ptr8[j] ];
			}
		}
		else
		if( bits == 16 )
		{
			int cell_no = pix_no << 2;
			for( i = 0, j = 0; i < cell_no; i+=4, j+=3 )
			{
				pdst[i]   = lut_to8bit.start.ptr8[ pdecoded.ptr16[j+2] ];
				pdst[i+1] = lut_to8bit.start.ptr8[ pdecoded.ptr16[j+1] ];
				pdst[i+2] = lut_to8bit.start.ptr8[ pdecoded.ptr16[j] ];
			}
		}
	}
	else
	{
		if( bits == 8 )
		{
			for( i = 0, j = 0;i < pix_no;i++, j += 4 )
				pdst[j] = pdst[j+1] = pdst[j+2] = lut_to8bit.start.ptr8[ pdecoded.ptr8[i] ];
		}
		else
		if( bits == 16 )
		{
			for( i = 0, j = 0;i < pix_no;i++, j += 4 )
				pdst[j] = pdst[j+1] = pdst[j+2] = lut_to8bit.start.ptr8[ pdecoded.ptr16[i] ];
		}
	}
}


bool cvideo_base::is_initialized( void ) const
{
 return initialized;
}


int cvideo_base::pack_params( control_id_t ctrl, const param_val_t &val, post_param_t *prm )
{
	switch( ctrl )
	{
	case CI_FPS:
		prm->params |= PP_FPS;
		prm->values[0] = val.values[0];
		prm->values[1] = val.values[1];
		return 1;
	case CI_AUTOGAIN:
		prm->params |= PP_AUTOGAIN;
		prm->values[2] = val.values[0];
		return 1;
	case CI_GAIN:
		prm->params |= PP_GAIN;
		prm->values[3] = val.values[0];
		return 1;
	case CI_EXPO:
		prm->params |= PP_EXPO;
		prm->values[4] = val.values[0];
		return 1;
	case CI_EXTCTL:
		prm->params |= PP_EXTPARAM;
		prm->values[5] = val.values[0]; // ext ctrl number
		prm->values[6] = val.values[1]; // param value
		return 1;
	}

 return 0;
}


int cvideo_base::post_params( const post_param_t &prm )
{
	pthread_mutex_lock( &cv_mutex );

	posted_param = prm;

	pthread_mutex_unlock( &cv_mutex );

 return 0;
}


int cvideo_base::check_posted_params( void )
{
 post_param_t prm;


	 pthread_mutex_lock( &cv_mutex );

	 prm = posted_param;		// get local copy of the task
	 posted_param.params = 0;	// reset task

	 pthread_mutex_unlock( &cv_mutex );


	 // set posted params
	 if( prm.params )
	 {
		 if( prm.params & PP_FPS )
		 {
			 time_fract_t new_fps;
			 new_fps.numerator = prm.values[0];
			 new_fps.denominator = prm.values[1];

			 set_fps( new_fps );
		 }
		 if( prm.params & PP_AUTOGAIN )
		 {
			 set_autogain( prm.values[2] );
		 }
		 if( prm.params & PP_GAIN )
		 {
			 set_gain( prm.values[3] );
		 }
		 if( prm.params & PP_EXPO )
		 {
		 	 set_exposure( prm.values[4] );
		 }
		 if( prm.params & PP_EXTPARAM )
		 {
			 set_ext_param( (unsigned int)prm.values[5], prm.values[6] );
		 }
	 }

 return 0;
}


cam_control_t *cvideo_base::get_cam_control( int ctrl, unsigned int low_ctl_id ) const
{
	for( int i = 0;i < num_controls;i++ )
	{
		if( ctrl == CI_AUTOGAIN && controls[i].id == V4L2_CID_AUTOGAIN )
			return &controls[i];
		else
		if( ctrl == CI_GAIN && controls[i].id == V4L2_CID_GAIN )
			return &controls[i];
		else
		if( ctrl == CI_EXPO && controls[i].id == V4L2_CID_EXPOSURE )
			return &controls[i];
		if( ctrl == CI_EXTCTL && controls[i].id == low_ctl_id )
			return &controls[i];
	}

	return NULL;
}


const std::map<unsigned int, const std::string>& cvideo_base::get_cam_ext_ctl_list( void ) const
{
	return m_ext_ctls;
}


int cvideo_base::set_autogain( int val )
{
 int ret = -1;
 cam_control_t *ctrl;

	 ctrl = get_cam_control( CI_AUTOGAIN );
	 if( ctrl )
	 {
		 param_val_t v;
		 v.set( val ? 1 : 0 );

		 ret = set_control( ctrl->id, v );
		 if( ret == 0 )
			 capture_params.autogain = val;
	 }

 return ret;
}


int cvideo_base::get_autogain( void )
{
 int ret = -1;
 cam_control_t *ctrl;

	ctrl = get_cam_control( CI_AUTOGAIN );
	if( ctrl )
	{
		param_val_t val;

		ret = get_control( ctrl->id, &val );
		if( ret == 0 )
			capture_params.autogain = val.values[0];
	}

 return ret == 0 ? capture_params.autogain : ret;
}


int cvideo_base::set_gain( int val )
{
 int ret = -1;
 cam_control_t *ctrl;

 	ctrl = get_cam_control( CI_GAIN );
 	if( ctrl )
 	{
 		param_val_t v;
 		v.set( val );

		ret = set_control( ctrl->id, v );
		if( ret == 0 )
			capture_params.gain = v.values[0];
	}

 return ret;
}


int cvideo_base::get_gain( void )
{
 int ret = -1;
 cam_control_t *ctrl;

  	ctrl = get_cam_control( CI_GAIN );
  	if( ctrl )
  	{
  		param_val_t val;

		ret = get_control( ctrl->id, &val );
		if( ret == 0 )
			capture_params.gain = val.values[0];
  	}

 return ret == 0 ? capture_params.gain : ret;
}


int cvideo_base::set_exposure( int val )
{
 int ret = -1;
 cam_control_t *ctrl;

	ctrl = get_cam_control( CI_EXPO );
	if( ctrl )
	{
		param_val_t v;
		v.set( val );

		ret = set_control( ctrl->id, v );
		if( ret == 0 )
			capture_params.exposure = v.values[0];
	}

	return ret;
}


int cvideo_base::get_exposure( void )
{
 int ret = -1;
 cam_control_t *ctrl;

	ctrl = get_cam_control( CI_EXPO );
	if( ctrl )
	{
		param_val_t val;

		ret = get_control( ctrl->id, &val );
		if( ret == 0 )
			capture_params.exposure = val.values[0];
	}

 return ret == 0 ? capture_params.exposure : ret;
}


int cvideo_base::set_ext_param( unsigned int ctrl_id, int val )
{
	int ret = -1;

	cam_control_t *ctrl = get_cam_control( CI_EXTCTL, ctrl_id );
	if( ctrl )
	{
		param_val_t v;
		v.set( val );

		ret = set_control( ctrl->id, v );
		//if( ret == 0 )
		//	capture_params.exposure = v.values[0];
	}

	return ret;
}


int cvideo_base::get_ext_param( unsigned int ctrl_id )
{
	int ret = -1;

	cam_control_t *ctrl = get_cam_control( CI_EXPO, ctrl_id );
	if( ctrl )
	{
		param_val_t val;

		ret = get_control( ctrl->id, &val );
		//if( ret == 0 )
		//	capture_params.exposure = val.values[0];
	}

	return ret == 0 ? 0 : ret;
}


int cvideo_base::set_control( unsigned int control_id, const param_val_t &val )
{
 int ret = 0;
 struct v4l2_control c;


	c.id  = control_id;
	c.value = val.values[0];
	ret = xioctl (fd, VIDIOC_S_CTRL, &c);
	if( ret < 0 )
		log_e("VIDIOC_S_CTRL - Unable to set control");

	return ret;
}


int cvideo_base::get_control( unsigned int control_id, param_val_t *val )
{
 int ret = 0;
 struct v4l2_control c;

	memset( &c,0,sizeof(struct v4l2_control) );
	val->set();

	c.id  = control_id;
	ret = xioctl (fd, VIDIOC_G_CTRL, &c);
	if( ret == 0 )
		val->values[0] = c.value;
	else
		log_e("VIDIOC_G_CTRL - Unable to get control");

	return ret;
}


void cvideo_base::start_calibration( int frame_cnt )
{
	calibration_frame = 0;
	calibration_frame_cnt = frame_cnt;
	is_calibrating = true;
	have_calibration = false;

	memset( calibration_buffer.start.ptr, 0, calibration_buffer.length );
}


void cvideo_base::cancel_calibration( void )
{
	if( !is_calibrating )
		return;

	is_calibrating = false;
	have_calibration = false;
}


void cvideo_base::set_use_calibration( bool use )
{
	capture_params.use_calibration = use;
}


bool cvideo_base::is_calibrated( void ) const
{
	return have_calibration;
}


const char* cvideo_base::get_name( void ) const
{
	if( !initialized )
		return "error";

	return device_type == DT_WEBCAM ? (const char*)dev_name : device_desc_list[device_type-1].desc;
}


const struct sensor_info_s& cvideo_base::get_sensor_info( void ) const
{
	return m_sensor_info;
}


//-------------------- LOW-LEVEL IO API ------------------------------
// Must be moved into library later
//--------------------------------------------------------------------
//---------------------------
//	IO request sender
//---------------------------
int cvideo_base::xioctl( int fd, int IOCTL_X, void *arg )
{
 int ret = 0;
 int tries = IOCTL_RETRY;


	do
    {
		 ret = ioctl(fd, IOCTL_X, arg);
	}while( ret && tries-- && ((errno == EINTR) || (errno == EAGAIN) || (errno == ETIMEDOUT)) );

	if( ret && (tries <= 0) )
		log_e("ioctl (%i) retried %i times - giving up: %s)", IOCTL_X, IOCTL_RETRY, strerror(errno));

 return ret;
}


int cvideo_base::open_device( void )
{
 struct stat st;


 	if( stat(dev_name, &st) == -1 )
    {
 		log_e( "Cannot identify '%s': %d, %s", dev_name, errno, strerror(errno));
 		return EXIT_FAILURE;
    }

 	// is symbolic device
 	if( !S_ISCHR(st.st_mode) )
    {
 		log_e( "%s is not valid device", dev_name);
 		return EXIT_FAILURE;
    }

 	fd = open( dev_name, O_RDWR | O_NONBLOCK, 0 ); // required NON-Block

 	if( fd == -1 )
    {
 		log_e( "Can't open '%s': %d, %s", dev_name, errno, strerror(errno) );
 		return EXIT_FAILURE;
    }

 return 0;
}


int cvideo_base::close_device( void )
{
	if( fd == -1 )
		return -1;

	close( fd );

	fd = -1;

 return 0;
}


int cvideo_base::get_vcaps( void )
{
 int ret;

	memset( &vcap, 0, sizeof(vcap) );
	ret = xioctl( fd, VIDIOC_QUERYCAP, &vcap );

	if( ret != 0 )
		return ret;

	if( enum_frame_formats() )
	{
		log_e("Unable to enumerate frame formats");
		return EXIT_FAILURE;
	}

	if( enum_controls() )
	{
		log_e("Unable to enumerate controls");
		return EXIT_FAILURE;
	}

 return 0;
}


void cvideo_base::init_lut_to8bit( int top )
{
	if( top == 0 )
		top = (1 << bpp());
	top = top > 0 ? top : 1;

	double hist_k = 256.0 / (double)top;
	for( int i = 0;i < lut_to8bit_len;i++ )
	{
		double v = (double)i*hist_k;
		v = v < 255 ? v : 255;
		lut_to8bit.start.ptr8[i] = (unsigned char)v;
	}
}


int cvideo_base::bpp( void ) const
{
	switch( capture_params.pixel_format )
	{
	case V4L2_PIX_FMT_YVU420:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_JPEG:
	case V4L2_PIX_FMT_MJPEG:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_GREY:
		return 8;
	case V4L2_PIX_FMT_Y16:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR12:
		return 16;
	default:
		log_e("Bpp request for unknown format");
	}
	return 0;
}


bool cvideo_base::is_color( void ) const
{
 	if( capture_params.pixel_format == V4L2_PIX_FMT_GREY || 
		capture_params.pixel_format == V4L2_PIX_FMT_Y16 )
		// || capture_params.pixel_format == V4L2_PIX_FMT_SGRBG12 )
		return false;

 return true;
}


// thread function
void *cvideo_base::video_thread( void *param )
{
 cvideo_base *vid = reinterpret_cast<cvideo_base*>(param);
 int quit = 0;

 fd_set fds;
 struct timeval tv;
 int res;
 //bool err_break = false;
 //int err_code = 0;

 	while( true )
	{
 		// check for pause
 		pthread_mutex_lock( &vid->cv_mutex );
 		while( !vid->start_thread_flag )
 			   pthread_cond_wait( &vid->cv, &vid->cv_mutex );  // infinite wait
 		quit = vid->quit_thread_flag;
 		pthread_mutex_unlock( &vid->cv_mutex );

 		// check for quit command
 		if( quit )
 			break;

 		// get frame...
 		while( true )
	    {
 			if( vid->fd != -1 )
 			{
 				FD_ZERO(&fds);
 				FD_SET(vid->fd, &fds);

 				// set frame timeout
 				tv.tv_sec = 5;
 				tv.tv_usec = 0;

 				res = select( vid->fd + 1, &fds, NULL, NULL, &tv );

 				if( res == -1 ) // error
 				{
 					if( errno == EINTR )
 						continue;

 					//err_break = true;
 					//err_code = 1;
 					break;
 				}

 				if( res == 0 ) // timeout
 				{
 					log_e( "video thread: select timeout");
 					//err_break = true;
 					//err_code = 2;
 					break;
 				}
 			}

 			// OK. try to read frame (res = 1)
 			if( vid->read_frame() == 0 )
 				break;
 			else
 			{
 				log_e( "video thread: read_frame error");
 			 	//err_break = true;
 			 	//err_code = 3;
 			 	break;
 			}
 			// EAGAIN - continue select loop.
		}

 		vid->check_posted_params(); // check for delayed task

 		//if( err_break )
 		//	break;
 		//err_break = false;

	}

 return NULL;
}


//-----------------------------------------------------------------------------------------------------
int cvideo_base::detect_best_device( int devtype, const char *devname )
{
 int cam_fd;
 struct v4l2_capability vcap;
 struct v4l2_fmtdesc fmt;
 struct pwc_probe probe;
 int ret;
 bool is_philips = false;


 	// try to detect predefined camera names
 	switch( devtype )
 	{
 	case DT_QHY5II:
		log_i( "Trying QHY5II..." );
		return DRV_QHY5II;
 	case DT_QHY5:
 		log_i( "Trying QHY5..." );
 		return DRV_QHY5;
 	case DT_DSI2PRO:
		log_i( "Trying DSI2PRO..." );
		return DRV_DSI2PRO;
 	case DT_QHY6:
		log_i( "Trying QHY6..." );
		return DRV_QHY6;
	case DT_ATIK:
		log_i( "Trying ATIK..." );
		return DRV_ATIK;
	case DT_SX:
		log_i( "Trying Starlight Xpress..." );
		return DRV_SX;
	case DT_ASI:
		log_i( "Trying ZWO ASI..." );
		return DRV_ASI;
 	case DT_NULL:
		log_i( "Trying NULL-camera..." );
		return DRV_NULL;
 	case DT_WEBCAM:
 	{
 		if( -1 == (cam_fd = open(devname, O_RDONLY)) )
 		{
 			perror( devname );
 			return -1;
 		}

 		// check for something...
 		memset(&vcap, 0, sizeof(vcap));
 		if( ioctl( cam_fd, VIDIOC_QUERYCAP, &vcap ) != 0 )
 		{
 			close( cam_fd );
 			return -1;
 		}

 		// only for DBG
 		//close( cam_fd );
 		//return UVC_CAM;

 		// try philips driver...
 		memset(&probe, 0, sizeof(probe));
 		if( ioctl(cam_fd, VIDIOCPWCPROBE, &probe) == 0 )
 		{
 			if( strcmp( (char *)vcap.card, probe.name) == 0 )
 				is_philips = true;
 		}

 		if( is_philips )
 		{
 			log_i( "Found '%s'", vcap.card );
 			close( cam_fd );
 			return DRV_PWC;
 		}

 		// check for UVC
 		memset(&fmt, 0, sizeof(fmt));
 		fmt.index = 0;
 		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 		while( (ret = ioctl(cam_fd, VIDIOC_ENUM_FMT, &fmt)) == 0 )
 		{
 			// try to find any supported format
 			if( fmt.pixelformat == V4L2_PIX_FMT_MJPEG || fmt.pixelformat == V4L2_PIX_FMT_JPEG ||
 					fmt.pixelformat == V4L2_PIX_FMT_YUYV || fmt.pixelformat == V4L2_PIX_FMT_YUV420 )
 			{
 				log_i( "Found UVC camera. Format: '%s'", fmt.description );
 				close( cam_fd );
 				return DRV_UVC;
 			}

 			fmt.index++;
 		}

 		// 	// v4l1 call was successful, but the first v4l2 call failed - so camera exists, but it's not v4l2
 		// 	if( ret != 0 && errno != EINVAL )
 		// 	{
 		// 		close( cam_fd );
 		// 		log_i( "Unknown '%s' camera detected.Using generic V4L driver.", vcap.name );
 		// 		return GENERIC_CAM;
 		// 	}

 		close( cam_fd );

 		log_i( "Unknown '%s' camera detected.", vcap.card );

 	}
 		break;
 	}

 return -1;
}

}
