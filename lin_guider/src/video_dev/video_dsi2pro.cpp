/*
 * video_dsi2pro.cpp
 *
 *  Created on: 24.05.2011
 *      Author: gm
 *
 *  Original DSI2PRO source code by Maxim Parygin
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
#include <stdlib.h>
#include <netinet/in.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>

#include "video_dsi2pro.h"
#include "lusb.h"
#include "timer.h"
#include "utils.h"


namespace video_drv
{

// disables hardware access for DSI2PRO
//#define NO_DSI2PRO

//-------------------------------------------- DSI2PRO ---------------------------------------------------------
int cvideo_dsi2pro::m_seq = 1;


// dsi2pro stuff
cvideo_dsi2pro::cvideo_dsi2pro() :
		m_handle( NULL )
{
	device_type = DT_DSI2PRO;

	int ret = lusb::initialize();

	if( ret != 0 )
		log_e( "cvideo_dsi2pro::cvideo_dsi2pro(): Could not open initialize libusb" );

	// this may be placed inside of initialization code
	m_sensor_info = video_drv::sensor_info_s( 8.6, 8.3, DSI2PRO_IMG_WIDTH, DSI2PRO_IMG_HEIGHT );
}


cvideo_dsi2pro::~cvideo_dsi2pro()
{
	//MUST BE called the first: stop thread and release resources
	stop();

	lusb::release();
}


time_fract_t cvideo_dsi2pro::set_fps( const time_fract_t &new_fps )
{
 internal_params_t iparam;
 bool rs = false;
 int ticks = 0;
 unsigned delay = 0;

	time_fract_t set_fps = time_fract::mk_fps( 1, 1 );
	int frame_idx = get_frame_idx();

	if( frame_idx != -1 )
	{
		for( int i = 0;i < MAX_FMT;i++ )
		{
			if( new_fps != time_fract::mk_fps( 0, 0 ) &&
				device_formats[0].frame_table[frame_idx].fps_table[i] == new_fps )
			{
				set_fps = new_fps;
				break;
			}
		}
	}

	delay = time_fract::to_msecs( set_fps );
	ticks = delay * 10;	// convert msecs to 1/10000 secs

 	// set EXP_TIME
	iparam.cmd = CMD_SET_EXP_TIME, iparam.arg = ticks, iparam.in = 4, iparam.out = 0, iparam.response = 0;
	rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
	if( !rs )
	{
		log_e( "cvideo_dsi2pro::set_fps(): CMD_SET_EXP_TIME failed." );
		return time_fract::mk_fps( 1, 1 );
	}

	if( initialized )
		pthread_mutex_lock( &cv_mutex );

 	//force_fps = true;
 	capture_params.fps = set_fps;
 	frame_delay = time_fract::to_msecs( capture_params.fps );

 	if( initialized )
 		pthread_mutex_unlock( &cv_mutex );

 return capture_params.fps;
}


int cvideo_dsi2pro::open_device( void )
{
	if( m_handle == NULL && lusb::is_initialized()  )
	{
#ifndef NO_DSI2PRO
		int ret = -1;

		// search
		m_handle = libusb_open_device_with_vid_pid( NULL, VID, PID );
		if( !m_handle )
		{
			log_e( "cvideo_dsi2pro::open_device(): unable to find device" );
			return -1;
		}

		if( libusb_kernel_driver_active( m_handle, 0 ) )
			libusb_detach_kernel_driver( m_handle, 0 );

		// claim
		ret = libusb_claim_interface( m_handle, 0 );
		if( ret != 0 )
		{
			log_e( "cvideo_dsi2pro::open_device(): unable claim interface" );
			return ret;
		}
#endif
	}

 return 0;
}


int cvideo_dsi2pro::close_device( void )
{
	if( m_handle )
	{
		libusb_release_interface( m_handle, 0 );
		libusb_close( m_handle );
		m_handle = NULL;
	}

 return 0;
}


int cvideo_dsi2pro::get_vcaps( void )
{
	int i = 0;
	point_t pt;

	device_formats[0].format = V4L2_PIX_FMT_Y16;

	pt.x = DSI2PRO_IMG_WIDTH;
	pt.y = DSI2PRO_IMG_HEIGHT;
	device_formats[0].frame_table[ i ].size =  pt;
	device_formats[0].frame_table[ i ].fps_table[ 0 ] = time_fract::mk_fps( 4, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 1 ] = time_fract::mk_fps( 2, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 2 ] = time_fract::mk_fps( 1, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 3 ] = time_fract::mk_fps( 1, 2 );
	device_formats[0].frame_table[ i ].fps_table[ 4 ] = time_fract::mk_fps( 1, 3 );
	device_formats[0].frame_table[ i ].fps_table[ 5 ] = time_fract::mk_fps( 1, 5 );
	device_formats[0].frame_table[ i ].fps_table[ 6 ] = time_fract::mk_fps( 1, 10 );
	i++;

	// add empty tail
	pt.x = pt.y = 0;
	device_formats[0].frame_table[ i++ ].size = pt;

	if( enum_controls() )
	{
		log_e("Unable to enumerate controls");
		return EXIT_FAILURE;
	}

 return 0;
}


int cvideo_dsi2pro::set_control( unsigned int control_id, const param_val_t &val )
{
	switch( control_id )
	{
	case V4L2_CID_GAIN:
	{
		internal_params_t iparam;
		bool rs = false;

		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 1023 ) v = 1023;
		// set GAIN
		iparam.cmd = CMD_SET_GAIN, iparam.arg = v, iparam.in = 1, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::set_control(): CMD_SET_GAIN failed." );
			return -1;
		}
		capture_params.gain = v;
	}
		break;
	case V4L2_CID_EXPOSURE:
	{
		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 65535 ) v = 65535;
		int top = 65536 - v;
		if( top <= 0 )
		{
			log_e( "cvideo_dsi2pro::set_control(): invalid exposure" );
			return -1;
		}
		init_lut_to8bit( top );

		capture_params.exposure = v;
	}
		break;
	default:
		return -1;
	}
	return 0;
}


int cvideo_dsi2pro::get_control( unsigned int control_id, param_val_t *val )
{
	switch( control_id )
	{
	case V4L2_CID_GAIN:
		val->values[0] = capture_params.gain;
		break;
	case V4L2_CID_EXPOSURE:
	{
		val->values[0] = capture_params.exposure;
		break;
	}
	default:
		return -1;
	}
	return 0;
}


int cvideo_dsi2pro::init_device( void )
{
 int sizeimage = 0;


	// set desired size
	sizeimage = set_format();
	if( sizeimage <= 0 )
		return EXIT_FAILURE;

	set_fps( capture_params.fps );

	n_buffers = 1;
	buffers = (buffer *)calloc( n_buffers, sizeof(*buffers) );

	if( !buffers )
	{
		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
		return EXIT_FAILURE;
	}

	buffers[0].length = sizeimage;
	buffers[0].start.ptr = malloc( sizeimage );

	if( !buffers[0].start.ptr )
	{
		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
		free( buffers );
		return EXIT_FAILURE;
	}

	// prepare...
	do
	{
		// set defaults
		bool rs = false;
		//bool out_rs = false;
		internal_params_t iparam;

		// reset
		iparam.cmd = CMD_RESET, iparam.arg = 0, iparam.in = 0, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_RESET failed." );
			break;
		}

		// set GAIN
		int v = capture_params.gain;
		if( v < 0 ) v = 0;
		if( v > 1023 ) v = 1023;
		iparam.cmd = CMD_SET_GAIN, iparam.arg = v, iparam.in = 1, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_GAIN failed." );
			break;
		}

		// set OFFSET
		iparam.cmd = CMD_SET_OFFSET, iparam.arg = 300, iparam.in = 2, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_OFFSET failed." );
			break;
		}

		// set ROW_COUNT_EVEN
		iparam.cmd = CMD_SET_ROW_COUNT_EVEN, iparam.arg = IMG_EVEN, iparam.in = 2, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_ROW_COUNT_EVEN failed." );
			break;
		}

		// set ROW_COUNT_ODD
		iparam.cmd = CMD_SET_ROW_COUNT_ODD, iparam.arg = IMG_ODD, iparam.in = 2, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_ROW_COUNT_ODD failed." );
			break;
		}

		// set VDD_MODE
		iparam.cmd = CMD_SET_VDD_MODE, iparam.arg = 1, iparam.in = 1, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_VDD_MODE failed." );
			break;
		}

		// set FLUSH_MODE
		iparam.cmd = CMD_SET_FLUSH_MODE, iparam.arg = 0, iparam.in = 1, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_FLUSH_MODE failed." );
			break;
		}

		//	set CLEAN_MODE
		iparam.cmd = CMD_SET_CLEAN_MODE, iparam.arg = 0, iparam.in = 1, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_CLEAN_MODE failed." );
			break;
		}

		// set READOUT_MODE
		iparam.cmd = CMD_SET_READOUT_MODE, iparam.arg = 0, iparam.in = 1, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_READOUT_MODE failed." );
			break;
		}

		// set READOUT_SPD
		iparam.cmd = CMD_SET_READOUT_SPD, iparam.arg = 1, iparam.in = 1, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_READOUT_SPD failed." );
			break;
		}

		// set EXP_MODE
		iparam.cmd = CMD_SET_EXP_MODE, iparam.arg = 1, iparam.in = 1, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_EXP_MODE failed." );
			break;
		}

		// set EXP_TIME
		iparam.cmd = CMD_SET_EXP_TIME, iparam.arg = 50, iparam.in = 4, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_EXP_TIME failed." );
			break;
		}

		// set NORM_READOUT_DELAY
		iparam.cmd = CMD_SET_NORM_READOUT_DELAY, iparam.arg = 1, iparam.in = 2, iparam.out = 0, iparam.response = 0;
		rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
		//out_rs = iparam.response == RES_ACK;
		if( !rs )
		{
			log_e( "cvideo_dsi2pro::init_device(): CMD_SET_NORM_READOUT_DELAY failed." );
			break;
		}

		set_exposure( capture_params.exposure );

		get_autogain();
		get_gain();
		get_exposure();

		initialized = true;
		return EXIT_SUCCESS;

	}while( 0 );

	uninit_device();

 return EXIT_FAILURE;
}


int cvideo_dsi2pro::uninit_device( void )
{
	if( buffers )
	{
		for( int i = 0;i < (int)n_buffers;i++ )
		{
			if( buffers[i].start.ptr )
				free( buffers[i].start.ptr );
		}
		free( buffers );
		buffers = NULL;
	}

 return 0;
}


int cvideo_dsi2pro::start_capturing( void )
{
 return 0;
}


int cvideo_dsi2pro::stop_capturing( void )
{
 return 0;
}


int cvideo_dsi2pro::read_frame( void )
{
 struct timespec tv;
 ctimer tm;
 int ret = 0;


	//-------- force FPS if needed -----------
  	if( force_fps )
  		tm.start();

 	//------------------------
  	internal_params_t iparam;
  	bool rs = false;
  	bool out_rs = false;

  	data_ptr raw = buffers[0].start;

  	// get IMAGE
  	int test = 0;	// 0 or 1 - for some test purposes

  	iparam.cmd = test ? CMD_TEST_PATTERN : CMD_TRIGGER, iparam.arg = 0, iparam.in = 0, iparam.out = 0, iparam.response = 0;
  	rs = send_command( NULL, 0, (char *)&iparam, sizeof(internal_params_t) );
  	out_rs = iparam.response == RES_ACK;
  	if( !rs
#ifndef NO_DSI2PRO
  			|| !out_rs
#endif
  	)
  	{
  		log_e( "cvideo_dsi2pro::read_frame(): CMD_TEST_PATTERN/CMD_TRIGGER failed." );
  		return 1;
  	}

  	// wait for exposure
  	usleep( frame_delay * 1000 );

  	// time has gone... read frame
#ifdef NO_DSI2PRO
  		memset( m_rawA, rand()%255, IMG_CHUKN_EVEN );
  		memset( m_rawB, rand()%255, IMG_CHUKN_ODD );
#else
  		// get image by two blocks
  		int t;

  		// transfer
  		ret = libusb_bulk_transfer( m_handle, EP_DATA, m_rawA, IMG_CHUKN_EVEN, &t, 0 );
  		if( ret < 0 )
  		{
  			log_e( "ccamera_dsi2pro::do_command(): read rawA error. ret = %d", ret );
  			return ret;
  		}
  		// also we must analyse t

  		ret = libusb_bulk_transfer( m_handle, EP_DATA, m_rawB, IMG_CHUKN_ODD, &t, 0 );
  		if( ret < 0 )
  		{
  			log_e( "ccamera_dsi2pro::do_command(): read rawB error. ret = %d", ret );
  			return ret;
  		}
  		// also we must analyse t
#endif

  		// interlaced block union
  		int i, j, k, l;
  		for( i = 5, j = 0, k = 5 * IMG_CHUNK + 58; i < 296; i++, j += DSI2PRO_IMG_WIDTH * 4, k += IMG_CHUNK )
  		{
  			for( l = 0; l < DSI2PRO_IMG_WIDTH * 2; l++ )
  				raw.ptr8[j + l] = m_rawA[k + l];
  		}
  		for( i = 5, j = DSI2PRO_IMG_WIDTH * 2, k = 5 * IMG_CHUNK + 58; i < 296; i++, j += DSI2PRO_IMG_WIDTH * 4, k += IMG_CHUNK )
  		{
  			for( l = 0; l < DSI2PRO_IMG_WIDTH * 2; l++ )
  				raw.ptr8[j + l] = m_rawB[k + l];
  		}

#define SWAP(a,b) { a ^= b; a ^= (b ^= a); }
  		unsigned data_size = buffers[0].length;
  		if( htons(0x55aa) != 0x55aa )
  			for( unsigned i = 0;i < data_size;i+=2 )
  				SWAP( raw.ptr8[i], raw.ptr8[i+1] );
#undef SWAP

 	// synchronize data with GUI
 	emit renderImage( buffers[ 0 ].start.ptr, buffers[0].length );

 	pthread_mutex_lock( &cv_mutex );
 	while( !data_thread_flag )
 		pthread_cond_wait( &cv, &cv_mutex );
 	data_thread_flag = 0;
 	pthread_mutex_unlock( &cv_mutex );

 	//-------- force FPS if needed -----------
 	if( force_fps )
 	{
 		tv.tv_sec = 0;
 		long tick = (long)tm.gettime();
 		long delay = (long)frame_delay - tick;
 		if( delay < 50 )
 			delay = 50;
 		tv.tv_nsec = delay * 1000000;
 		nanosleep( &tv, NULL );
 	}

 return 0;
}


int cvideo_dsi2pro::set_format( void )
{
	int i, j;
	point_t pt = {0, 0};

	capture_params.pixel_format = V4L2_PIX_FMT_Y16;/*  16  Greyscale     */	// this is a fake format. to work only with 1st item in array device_formats[]

	for( i = 0; i < MAX_FMT && device_formats[i].format;i++ )
	{
		if( device_formats[i].format != capture_params.pixel_format )
			continue;
		for( j = 0;j < MAX_FMT && device_formats[i].frame_table[j].size.x;j++ )
		{
			if( device_formats[i].frame_table[j].size.x == (int)capture_params.width &&
				device_formats[i].frame_table[j].size.y == (int)capture_params.height )
			{
				pt = device_formats[i].frame_table[j].size;
				break;
			}
		}
		if( pt.x == 0 && device_formats[i].frame_table[0].size.x )
			pt = device_formats[i].frame_table[0].size;

		break;
	}

	// set desired size
	capture_params.width  = pt.x;
	capture_params.height = pt.y;

 return capture_params.width * capture_params.height * sizeof(unsigned short);
}


int cvideo_dsi2pro::enum_controls( void )
{
	int n = 0;
	struct v4l2_queryctrl queryctrl;

	memset( &queryctrl, 0, sizeof(v4l2_queryctrl) );

	// create virtual control
	queryctrl.id = V4L2_CID_GAIN;
	queryctrl.type = V4L2_CTRL_TYPE_INTEGER;
	snprintf( (char*)queryctrl.name, sizeof(queryctrl.name)-1, "gain" );
	queryctrl.minimum = 0;
	queryctrl.maximum = 1023;
	queryctrl.step = 1;
	queryctrl.default_value = capture_params.gain;
	queryctrl.flags = 0;
	// Add control to control list
	controls = add_control( -1, &queryctrl, controls, &n );

	// create virtual control
	queryctrl.id = V4L2_CID_EXPOSURE;
	queryctrl.type = V4L2_CTRL_TYPE_INTEGER;
	snprintf( (char*)queryctrl.name, sizeof(queryctrl.name)-1, "exposure" );
	queryctrl.minimum = 0;
	queryctrl.maximum = 65535;
	queryctrl.step = 1;
	queryctrl.default_value = capture_params.exposure;
	queryctrl.flags = 0;
	// Add control to control list
	controls = add_control( -1, &queryctrl, controls, &n );

	num_controls = n;

	return 0;
}


bool cvideo_dsi2pro::send_command( const char */*cmd*/, int /*cmd_len*/, char *param, int param_len )
{
 internal_params_t *pp = reinterpret_cast<internal_params_t *>(param);
 int response = 0;
 int ret = 0, t;

	if( sizeof(internal_params_t) != param_len )
		return false;

#ifdef NO_DSI2PRO
	return true;
#endif

	unsigned char data[7];

	// rq
	switch( pp->in )
	{
	case 0:
		data[0] = 3;
		data[1] = (unsigned char)m_seq++;
		data[2] = pp->cmd;
		ret = libusb_bulk_transfer( m_handle, EP_IN, data, 3, &t, 0 );
		break;
	case 1:
		data[0] = 4;
		data[1] = (unsigned char)m_seq++;
		data[2] = pp->cmd;
		data[3] = pp->arg;
		ret = libusb_bulk_transfer( m_handle, EP_IN, data, 4, &t, 0 );
		break;
	case 2:
		data[0] = 5;
		data[1] = (unsigned char)m_seq++;
		data[2] = pp->cmd;
		data[3] = pp->arg;
		data[4] = (pp->arg >> 8);
		ret = libusb_bulk_transfer( m_handle, EP_IN, data, 5, &t, 0 );
		break;
	case 4:
		data[0] = 7;
		data[1] = (unsigned char)m_seq++;
		data[2] = pp->cmd;
		data[3] = pp->arg;
		data[4] = (pp->arg >> 8);
		data[5] = (pp->arg >> 16);
		data[6] = (pp->arg >> 24);
		ret = libusb_bulk_transfer( m_handle, EP_IN, data, 7, &t, 0 );
		break;
	default:
		log_e( "Unknown IN: %d", pp->in );
		pp->response = 0;
		return false;
	}

	// also we must analyse t

	if( ret )
	{
		log_e( "ccamera_dsi2pro::send_command(): 1. libusb_bulk_transfer error. ret = %d", ret );
		return false;
	}

	// resp
	switch( pp->out )
	{
	case 0:
		ret = libusb_bulk_transfer(m_handle, EP_OUT, data, 3, &t, 0);
		response = data[2];
		break;
	case 1:
		ret = libusb_bulk_transfer(m_handle, EP_OUT, data, 4, &t, 0);
		response = data[3] & 0xFF;
		break;
	case 2:
		ret = libusb_bulk_transfer(m_handle, EP_OUT, data, 5, &t, 0);
		response = (data[3] & 0xFF) + ((data[4] & 0xFF) << 8);
		break;
	case 4:
		ret = libusb_bulk_transfer(m_handle, EP_OUT, data, 7, &t, 0);
		response = (data[3] & 0xFF) + ((data[4] & 0xFF) << 8) + ((data[5] & 0xFF) << 16) + ((data[6] & 0xFF) << 24);
		break;
	default:
		log_e( "Unknown OUT: %d", pp->out );
		return false;
	}

	// also we must analyse t

	if( ret )
	{
		log_e( "cvideo_dsi2pro::send_command(): 2. libusb_bulk_transfer error. ret = %d", ret );
		return false;
	}

	pp->response = response;

 return true;
}

}
