/*
 * video_atik.cpp
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

#include "video_atik.h"
#include "timer.h"
#include "utils.h"
#include "filters.h"


long time_diff(struct timeval *start, struct timeval *end) {
	long msec;

	msec = (end->tv_sec - start->tv_sec) * 1000;
	msec += (end->tv_usec - start->tv_usec) / 1000;

	return msec;
}


namespace video_drv
{


cvideo_atik::cvideo_atik()
{
	device_type = DT_ATIK;
}


cvideo_atik::~cvideo_atik()
{
	stop();
}


time_fract_t cvideo_atik::set_fps( const time_fract &new_fps )
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

	capture_params.fps = set_fps;
	frame_delay = time_fract::to_msecs( capture_params.fps );

	if( initialized )
		pthread_mutex_unlock( &cv_mutex );

	return capture_params.fps;
}


int cvideo_atik::open_device( void )
{
	return atik_open();
}


int cvideo_atik::close_device( void )
{
	return atik_close();
}


int  cvideo_atik::get_vcaps( void )
{
	int i = 0;
	point_t pt;

	device_formats[0].format = V4L2_PIX_FMT_Y16;

	pt.x = m_pixel_count_X;
	pt.y = m_pixel_count_Y;
	device_formats[0].frame_table[ i ].size =  pt;
	device_formats[0].frame_table[ i ].fps_table[ 0 ] = time_fract::mk_fps( 10, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 1 ] = time_fract::mk_fps( 5, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 2 ] = time_fract::mk_fps( 3, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 3 ] = time_fract::mk_fps( 2, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 4 ] = time_fract::mk_fps( 1, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 5 ] = time_fract::mk_fps( 1, 2 );
	device_formats[0].frame_table[ i ].fps_table[ 6 ] = time_fract::mk_fps( 1, 3 );
	device_formats[0].frame_table[ i ].fps_table[ 7 ] = time_fract::mk_fps( 1, 5 );
	device_formats[0].frame_table[ i ].fps_table[ 8 ] = time_fract::mk_fps( 1, 10 );
	i++;

	// add empty tail
	pt.x = pt.y = 0;
	device_formats[0].frame_table[ i++ ].size = pt;

	if( enum_controls() ) {
		log_e("Unable to enumerate controls");
		return EXIT_FAILURE;
	}

	return 0;
}


int  cvideo_atik::set_control( unsigned int control_id, const param_val_t &val )
{
	switch( control_id ) {
	case V4L2_CID_EXPOSURE: {
		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 65536 ) v = 65536;
		int top = 65536 - v;
		if( top <= 0 ) {
			log_e( "cvideo_atik::set_control(): invalid exposure" );
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


int  cvideo_atik::get_control( unsigned int control_id, param_val_t *val )
{
	switch( control_id ) {
	case V4L2_CID_EXPOSURE:
		val->values[0] = capture_params.exposure;
		break;
	default:
		return -1;
	}
	return 0;
}


int cvideo_atik::init_device( void )
{
	int sizeimage = 0;

	// set desired size
	sizeimage = set_format();
	if( sizeimage <= 0 )
		return EXIT_FAILURE;

	set_fps( capture_params.fps );

	n_buffers = 1;
	buffers = (buffer *)calloc( n_buffers, sizeof(*buffers) );

	if( !buffers ) {
		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
		return EXIT_FAILURE;
	}

	buffers[0].length = sizeimage;
	buffers[0].start.ptr = malloc( sizeimage );

	if( !buffers[0].start.ptr ) {
		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
		free( buffers );
		return EXIT_FAILURE;
	}

	set_exposure( capture_params.exposure );
	get_exposure();

	return 0;
}


int cvideo_atik::uninit_device( void )
{
	if( buffers ) {
		for( int i = 0;i < (int)n_buffers;i++ ) {
			if( buffers[i].start.ptr16 )
				free( buffers[i].start.ptr16 );
		}
		free( buffers );
		buffers = NULL;
	}

	return 0;
}


int cvideo_atik::start_capturing( void )
{
	bool success = m_camera->startExposure(false);
	if( !success ) {
		log_e("startExposure(): failed");
		return 1;
	}
	if( DBG_VERBOSITY )
		log_i( "Exposure started" );

	gettimeofday(&m_expstart, NULL);

	return 0;
}


int cvideo_atik::stop_capturing( void )
{
	m_camera->abortExposure();

	return 0;
}


int cvideo_atik::read_frame( void )
{
	struct timeval now;
	long time_elapsed;
	bool success;
	data_ptr raw = buffers[0].start;
        (void)raw;

	gettimeofday(&now, NULL);
	time_elapsed = time_diff(&m_expstart, &now);
	// if exposure time is not elapsed
	// wait for some time to offload the CPU and return
	if (time_elapsed < (long)frame_delay) {
		usleep((frame_delay - time_elapsed) / 2);
		return 0;
	}

	success = m_camera->readCCD(0, 0, m_pixel_count_X, m_pixel_count_Y, 1, 1);
	if( !success ) {
		log_e("readCCD(): failed");
		return 1;
	}
	if( DBG_VERBOSITY )
		log_i("Exposure finished. Reading %d bytes", buffers[0].length);

	success = m_camera->getImage(raw.ptr16, m_pixel_count_X * m_pixel_count_Y);
	if( !success ) {
		log_e("getImage(): failed");
		return 1;
	}
	if( DBG_VERBOSITY )
		log_i( "Downloading finished. Read: %d bytes", buffers[0].length);

	success = m_camera->startExposure(false);
	if( !success ) {
		log_e("startExposure(): failed");
		return 1;
	}

	struct timeval prev = m_expstart;
	gettimeofday(&m_expstart, NULL);

	if( DBG_VERBOSITY ) {
		long exptime = time_diff(&prev, &m_expstart);
		log_i( "Exposure started. Last frame took %d ms", exptime);
	}

	// synchronize data with GUI
	emit renderImage( buffers[ 0 ].start.ptr, buffers[0].length );

	pthread_mutex_lock( &cv_mutex );
	while( !data_thread_flag )
		pthread_cond_wait( &cv, &cv_mutex );
	data_thread_flag = 0;
	pthread_mutex_unlock( &cv_mutex );

	return 0;
}


int cvideo_atik::set_format( void )
{
	int i, j;
	point_t pt = {0, 0};

	capture_params.pixel_format = V4L2_PIX_FMT_Y16;
	for( i = 0; i < MAX_FMT && device_formats[i].format;i++ ) {
		if( device_formats[i].format != capture_params.pixel_format )
			continue;
		for( j = 0;j < MAX_FMT && device_formats[i].frame_table[j].size.x;j++ ) {
			if( device_formats[i].frame_table[j].size.x == (int)capture_params.width &&
					device_formats[i].frame_table[j].size.y == (int)capture_params.height ) {
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

	return capture_params.width * capture_params.height * 2;
}


int cvideo_atik::enum_controls( void )
{
	int n = 0;
	struct v4l2_queryctrl queryctrl;

	memset( &queryctrl, 0, sizeof(v4l2_queryctrl) );
	// create virtual control
	queryctrl.id = V4L2_CID_EXPOSURE;
	queryctrl.type = V4L2_CTRL_TYPE_INTEGER;
	snprintf( (char*)queryctrl.name, sizeof(queryctrl.name)-1, "exposure" );
	queryctrl.minimum = 0;
	queryctrl.maximum = 65535;
	queryctrl.step = 1;
	queryctrl.default_value = 0;
	queryctrl.flags = 0;
	// Add control to control list
	controls = add_control( -1, &queryctrl, controls, &n );

	num_controls = n;

	return 0;
}

}
