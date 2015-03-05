/*
 * video_qhy5.cpp
 *
 *  Created on: 24.05.2011
 *      Author: gm
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
#include <unistd.h>

#include "video_qhy5.h"
#include "timer.h"
#include "utils.h"

namespace video_drv
{

// enable image QHY5_SCALER instead of crop
#define QHY5_SCALER

// enables test image
//#define QHY5_TEST_IMAGE

//-------------------------------------------- QHY5 ---------------------------------------------------------
// QHY5 stuff
cvideo_qhy5::cvideo_qhy5() :
	m_qhy5_obj( NULL ),
	m_real_height( 0 )
{
	device_type = DT_QHY5;

	m_qhy5_obj = new qhy5_core_shared();
}


cvideo_qhy5::~cvideo_qhy5()
{
	//MUST BE called the first: stop thread and release resources
	stop();

	delete m_qhy5_obj;
}


time_fract_t cvideo_qhy5::set_fps( const time_fract_t &new_fps )
{
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

	if( initialized )
		pthread_mutex_lock( &cv_mutex );

	//force_fps = true;
	capture_params.fps = set_fps;
	frame_delay = time_fract::to_msecs( capture_params.fps );

	log_i( "  Frame rate:   %u/%u fps", set_fps.denominator, set_fps.numerator );

	if( initialized )
		pthread_mutex_unlock( &cv_mutex );

	return capture_params.fps;
}


int cvideo_qhy5::open_device( void )
{
	return m_qhy5_obj->open_device();
}


int cvideo_qhy5::close_device( void )
{
	m_qhy5_obj->close_device();

	return 0;
}


int cvideo_qhy5::get_vcaps( void )
{
	int i = 0;
	point_t pt;

	device_formats[0].format = V4L2_PIX_FMT_GREY;

	pt.x = QHY5_IMAGE_WIDTH / 4;
	pt.y = QHY5_IMAGE_HEIGHT / 4;
	device_formats[0].frame_table[ i ].size =  pt;
	device_formats[0].frame_table[ i ].fps_table[ 0 ] = time_fract::mk_fps( 4, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 1 ] = time_fract::mk_fps( 2, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 2 ] = time_fract::mk_fps( 1, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 3 ] = time_fract::mk_fps( 1, 2 );
	device_formats[0].frame_table[ i ].fps_table[ 4 ] = time_fract::mk_fps( 1, 3 );
	device_formats[0].frame_table[ i ].fps_table[ 5 ] = time_fract::mk_fps( 1, 5 );
	device_formats[0].frame_table[ i ].fps_table[ 6 ] = time_fract::mk_fps( 1, 10 );
	i++;

	pt.x = QHY5_IMAGE_WIDTH / 2;
	pt.y = QHY5_IMAGE_HEIGHT / 2;
	device_formats[0].frame_table[ i ].size =  pt;
	device_formats[0].frame_table[ i ].fps_table[ 0 ] = time_fract::mk_fps( 4, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 1 ] = time_fract::mk_fps( 2, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 2 ] = time_fract::mk_fps( 1, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 3 ] = time_fract::mk_fps( 1, 2 );
	device_formats[0].frame_table[ i ].fps_table[ 4 ] = time_fract::mk_fps( 1, 3 );
	device_formats[0].frame_table[ i ].fps_table[ 5 ] = time_fract::mk_fps( 1, 5 );
	device_formats[0].frame_table[ i ].fps_table[ 6 ] = time_fract::mk_fps( 1, 10 );
	i++;

	pt.x = QHY5_IMAGE_WIDTH;
	pt.y = QHY5_IMAGE_HEIGHT;
	device_formats[0].frame_table[ i ].size =  pt;
	device_formats[0].frame_table[ i ].fps_table[ 0 ] = time_fract::mk_fps( 4, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 1 ] = time_fract::mk_fps( 2, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 2 ] = time_fract::mk_fps( 1, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 3 ] = time_fract::mk_fps( 1, 2 );
	device_formats[0].frame_table[ i ].fps_table[ 4 ] = time_fract::mk_fps( 1, 3 );
	device_formats[0].frame_table[ i ].fps_table[ 5 ] = time_fract::mk_fps( 1, 5 );
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


int cvideo_qhy5::set_control( unsigned int control_id, const param_val_t &val )
{
	switch( control_id )
	{
	case V4L2_CID_GAIN:
	{
		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 100 ) v = 100;
		capture_params.gain = v;
		m_qhy5_obj->set_size(
#ifdef QHY5_SCALER
			QHY5_IMAGE_HEIGHT,
#else
			capture_params.height,
#endif
			capture_params.gain, 0 );
		break;
	}
	case V4L2_CID_EXPOSURE:
	{
		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 255 ) v = 255;
		int top = 256 - v;
		if( top <= 0 )
		{
			log_e( "cvideo_qhy5::set_control(): invalid exposure" );
			return -1;
		}
		init_lut_to8bit( top );

		capture_params.exposure = v;
		break;
	}
	default:
		return -1;
	}
	return 0;
}


int cvideo_qhy5::get_control( unsigned int control_id, param_val_t *val )
{
	switch( control_id )
	{
	case V4L2_CID_GAIN:
	{
		val->values[0] = capture_params.gain;
		break;
	}
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


int cvideo_qhy5::init_device( void )
{
 int sizeimage = 0;


	// set desired size
	sizeimage = set_format();
	if( sizeimage <= 0 )
		return EXIT_FAILURE;

	set_fps( capture_params.fps );

#ifdef QHY5_SCALER
	int binn_factor = 1;
	switch( capture_params.width )
	{
	case QHY5_IMAGE_WIDTH/2:
		binn_factor = 2;
		break;
	case QHY5_IMAGE_WIDTH/4:
		binn_factor = 4;
		break;
	}
#endif
	m_sensor_info = video_drv::sensor_info_s(
#ifdef QHY5_SCALER
			5.2 * binn_factor,
			5.2 * binn_factor,
#else
			5.2,
			5.2,
#endif
			capture_params.width,
			capture_params.height
		);


	n_buffers = 2;
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

	// init internal buffer. now full frame for scale! without crop!!!
	m_real_height = m_qhy5_obj->set_size(
#ifdef QHY5_SCALER
			QHY5_IMAGE_HEIGHT,
#else
			capture_params.height,
#endif
			8, 1 );
	buffers[1].length = m_real_height * QHY5_MATRIX_WIDTH;
	buffers[1].start.ptr = malloc( buffers[1].length );

	if( !buffers[1].start.ptr )
	{
		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
		free( buffers[0].start.ptr );
		free( buffers );
		return EXIT_FAILURE;
	}

	// prepare...
	int tmp_gain = 10;
	m_qhy5_obj->start_exposure( frame_delay );
	usleep( frame_delay * 1000 );
	m_qhy5_obj->read_exposure( buffers[1].start.ptr8, buffers[1].length );
	m_qhy5_obj->set_size(
#ifdef QHY5_SCALER
			QHY5_IMAGE_HEIGHT,
#else
			capture_params.height,
#endif
			tmp_gain, 0 );

	set_gain( capture_params.gain );
	set_exposure( capture_params.exposure );

	get_autogain();
	get_gain();
	get_exposure();

 return 0;
}


int cvideo_qhy5::uninit_device( void )
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


int cvideo_qhy5::start_capturing( void )
{
 return 0;
}


int cvideo_qhy5::stop_capturing( void )
{
 return 0;
}


int cvideo_qhy5::read_frame( void )
{
 struct timespec tv;
 ctimer tm;
 long delay;
 int ret = 0;


	//-------- force FPS if needed -----------
  	if( force_fps )
  		tm.start();

 	//------------------------
  	ret = m_qhy5_obj->start_exposure( frame_delay );
	if( ret )
		return ret;

	usleep( frame_delay * 1000 );
	ret = m_qhy5_obj->read_exposure( buffers[1].start.ptr8, buffers[1].length );
	if( ret )
		return ret;

  	// convert to std buffer
  	uint8_t *dst = buffers[0].start.ptr8;
  	uint8_t *src = buffers[1].start.ptr8;
  	uint8_t *d_ptr = dst;	// writer
  	unsigned int row_end, col_start;
  	unsigned int x_offset = (QHY5_IMAGE_WIDTH -
#ifdef QHY5_SCALER
  			QHY5_IMAGE_WIDTH
#else
  			capture_params.width
#endif
  			) / 2;

  	row_end = m_real_height - 26;
  	col_start = x_offset + 20;

#ifdef QHY5_TEST_IMAGE
  	for( unsigned int row = 0; row < row_end; row++ )
	{
		for( unsigned int col = col_start; col < QHY5_IMAGE_WIDTH + col_start; col++ )
		{
			if( col-20 == row )
				src[row * QHY5_MATRIX_WIDTH + col] = 128;
		}
	}
#endif

#ifdef QHY5_SCALER
  	assert( row_end == QHY5_IMAGE_HEIGHT );

  	// copy or scale (new feature)
  	if( capture_params.width == QHY5_IMAGE_WIDTH && capture_params.height == QHY5_IMAGE_HEIGHT )
  	{
  		unsigned int col_end = capture_params.width + col_start;

  		for( unsigned int row = 0; row < row_end; row++ )
  		{
  			for( unsigned int col = col_start; col < col_end; col++ )
  			{
  				*d_ptr = src[row * QHY5_MATRIX_WIDTH + col];
  				d_ptr++;
  			}
  		}
  	}
  	else
  	{
  		int new_wd = capture_params.width;
  		int new_ht = capture_params.height;
  		float kx = (float)QHY5_IMAGE_WIDTH / (float)new_wd;
  		float ky = (float)QHY5_IMAGE_HEIGHT / (float)new_ht;

  		for( int j = 0;j < new_ht;j++ )
  		{
  			uint8_t *s_ptr = src + (int)((float)j * ky) * QHY5_MATRIX_WIDTH + col_start;
  			for( int i = 0;i < new_wd;i++ )
  			{
  				*d_ptr = *(s_ptr + (int)((float)i * kx));
  				d_ptr++;
  			}
  		}
  	}
#else	// old copy algorithm with cropping
  	assert( row_end == capture_params.height );

  	unsigned int col_end = capture_params.width + col_start;

  	for( unsigned int row = 0; row < row_end; row++ )
  	{
  		for( unsigned int col = col_start; col < col_end; col++ )
  		{
  			*d_ptr = src[row * QHY5_MATRIX_WIDTH + col];
  			d_ptr++;
  		}
  	}
#endif

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
 		delay = (long)frame_delay - tick;
 		if( delay < 50 )
 			delay = 50;
 		tv.tv_nsec = delay * 1000000;
 		nanosleep( &tv, NULL );
 	}

 return 0;
}


int cvideo_qhy5::set_format( void )
{
 int i, j;
 point_t pt = {0, 0};

	capture_params.pixel_format = V4L2_PIX_FMT_GREY;/*  8  Greyscale     */	// this is a fake format. to work only with 1st item in array device_formats[]

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

 return capture_params.width * capture_params.height;
}


int cvideo_qhy5::enum_controls( void )
{
	int n = 0;
	struct v4l2_queryctrl queryctrl;

	memset( &queryctrl, 0, sizeof(v4l2_queryctrl) );

	// create virtual control
	queryctrl.id = V4L2_CID_GAIN;
	queryctrl.type = V4L2_CTRL_TYPE_INTEGER;
	snprintf( (char*)queryctrl.name, sizeof(queryctrl.name)-1, "gain" );
	queryctrl.minimum = 0;
	queryctrl.maximum = 100;
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
	queryctrl.maximum = 255;
	queryctrl.step = 1;
	queryctrl.default_value = capture_params.exposure;
	queryctrl.flags = 0;
	// Add control to control list
	controls = add_control( -1, &queryctrl, controls, &n );

	num_controls = n;

	return 0;
}

}
