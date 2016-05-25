/*
 * video_gen.cpp
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
#include <math.h>
#include <unistd.h>

#include "video_null.h"
#include "timer.h"
#include "utils.h"
#include "filters.h"

namespace video_drv
{

//-------------------------------------------- GENERIC ---------------------------------------------------------
// GENERIC stuff
cvideo_null::cvideo_null( bool stub )
{
	device_type = DT_NULL;

	is_v4l_1 = true;

	stub_mode = stub;

	// this may be placed inside of initialization code
	//m_sensor_info = video_drv::sensor_info_s( 4.5, 4.5, 640, 480 );
}


cvideo_null::~cvideo_null()
{
	stop();
}


time_fract_t cvideo_null::set_fps( const time_fract &new_fps )
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

	if( initialized )
		pthread_mutex_unlock( &cv_mutex );

 return capture_params.fps;
}


int cvideo_null::open_device( void )
{
 return stub_mode ? -1 : 0;
}


int cvideo_null::close_device( void )
{
 return 0;
}


int  cvideo_null::get_vcaps( void )
{
	int i = 0;
	point_t pt;

	device_formats[0].format = V4L2_PIX_FMT_SGRBG12; //V4L2_PIX_FMT_GREY;

	pt.x = 320;
	pt.y = 240;
	device_formats[0].frame_table[ i ].size =  pt;
	device_formats[0].frame_table[ i ].fps_table[ 0 ] = time_fract::mk_fps( 5, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 1 ] = time_fract::mk_fps( 3, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 2 ] = time_fract::mk_fps( 2, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 3 ] = time_fract::mk_fps( 1, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 4 ] = time_fract::mk_fps( 1, 2 );
	device_formats[0].frame_table[ i ].fps_table[ 5 ] = time_fract::mk_fps( 1, 5 );
	device_formats[0].frame_table[ i ].fps_table[ 6 ] = time_fract::mk_fps( 1, 10 );
	device_formats[0].frame_table[ i ].fps_table[ 7 ] = time_fract::mk_fps( 1, 15 );
	i++;

	pt.x = 640;
	pt.y = 480;
	device_formats[0].frame_table[ i ].size =  pt;
	device_formats[0].frame_table[ i ].fps_table[ 0 ] = time_fract::mk_fps( 5, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 1 ] = time_fract::mk_fps( 3, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 2 ] = time_fract::mk_fps( 2, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 3 ] = time_fract::mk_fps( 1, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 4 ] = time_fract::mk_fps( 1, 2 );
	device_formats[0].frame_table[ i ].fps_table[ 5 ] = time_fract::mk_fps( 1, 5 );
	device_formats[0].frame_table[ i ].fps_table[ 6 ] = time_fract::mk_fps( 1, 10 );
	device_formats[0].frame_table[ i ].fps_table[ 7 ] = time_fract::mk_fps( 1, 15 );
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


int  cvideo_null::set_control( unsigned int control_id, const param_val_t &val )
{
	switch( control_id )
	{
	case V4L2_CID_AUTOGAIN:
		capture_params.autogain = val.values[0];
		break;
	case V4L2_CID_GAIN:
	{
		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 100 ) v = 100;
		capture_params.gain = v;
	}
		break;
	case V4L2_CID_EXPOSURE:
	{
		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 100 ) v = 100;
		//int top = 256 - 2.55*v;
		int top = 65536 - 655.35*v;
		if( top <= 0 )
		{
			log_e( "cvideo_null::set_control(): invalid exposure" );
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


int  cvideo_null::get_control( unsigned int control_id, param_val_t *val )
{
	switch( control_id )
	{
	case V4L2_CID_AUTOGAIN:
		val->values[0] = capture_params.autogain;
		break;
	case V4L2_CID_GAIN:
		val->values[0] = capture_params.gain;
		break;
	case V4L2_CID_EXPOSURE:
		val->values[0] = capture_params.exposure;
		break;
	default:
		return -1;
	}
	return 0;
}


int cvideo_null::init_device( void )
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
 	buffers[0].start.ptr = malloc( sizeimage * 2 );

 	if( !buffers[0].start.ptr )
 	{
 		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
 		free( buffers );
 		return EXIT_FAILURE;
 	}

 	set_autogain( capture_params.autogain );
 	set_gain( capture_params.gain );
 	set_exposure( capture_params.exposure );

 	get_autogain();
 	get_gain();
 	get_exposure();

 	{
 		int factor = capture_params.width == 640 ? 1 : 2;
 		m_sensor_info = video_drv::sensor_info_s( 4.5 * factor, 4.5 * factor, 640 / factor, 480 / factor );
 	}

 	// init emu
 	for( int i = 0;i < star_no;i++ )
 	{
 		emu_star_t star;
 		star.x = rand()%capture_params.width;
 		star.y = rand()%capture_params.height;
 		star.sigma = 1+(double)rand()/(double)RAND_MAX * 2;
 		star.lum = 2 * (1/3.0 * star.sigma * (1 << bpp()));

 		m_emu_stars.push_back( star );
 	}
 	int pix_max = (1 << bpp())-1;
 	for( int i = 0;i < bad_pixel_no;i++ )
 	{
 		emu_star_t bp;
 		bp.x = rand()%capture_params.width;
 		bp.y = rand()%capture_params.height;
 		bp.sigma = 1;
 		bp.lum = pix_max - rand()%(pix_max/2);

 		m_emu_badpix.push_back( bp );
 	}

 return 0;
}


int cvideo_null::uninit_device( void )
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


int cvideo_null::start_capturing( void )
{
 return 0;
}


int cvideo_null::stop_capturing( void )
{
 return 0;
}


int cvideo_null::read_frame( void )
{
 struct timespec tv;
 ctimer tm;

 	 if( 0 && DBG_VERBOSITY )
 		log_i( "Exposure started" );

	//-------- force FPS if needed -----------
	if( force_fps )
		tm.start();

	//------------------------
	data_ptr raw = buffers[0].start;
	(void)raw;

	// send command 'get IMAGE'
	// ...

	// wait for exposure
	if( !force_fps )
		usleep( frame_delay * 1000 );

	if( 0 && DBG_VERBOSITY )
		log_i("Exposure finished. Reading %d bytes", buffers[0].length);

	// time has gone... emulate and read frame
	generate_emu_field();
	generate_emu_stars();

	//filters::medianfilter( (uint8_t*)raw.ptr8, (uint8_t*)NULL, capture_params.width, capture_params.height );
	//filters::medianfilter( (uint16_t*)raw.ptr16, (uint16_t*)NULL, capture_params.width, capture_params.height );
	//filters::medianfilter( (uint16_t*)raw.ptr16, (uint16_t*)NULL, capture_params.width, capture_params.height );

	if( 0 && DBG_VERBOSITY )
		log_i( "Downloading finished. Read: %d bytes", buffers[0].length);

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


int cvideo_null::set_format( void )
{
 int i, j;
 point_t pt = {0, 0};

	capture_params.pixel_format = V4L2_PIX_FMT_SGRBG12; //V4L2_PIX_FMT_GREY;/*  8  Greyscale     */	// this is a fake format. to work only with 1st item in array device_formats[]

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


int cvideo_null::enum_controls( void )
{
	int n = 0;
	struct v4l2_queryctrl queryctrl;

	memset( &queryctrl, 0, sizeof(v4l2_queryctrl) );

	// create virtual control
	queryctrl.id = V4L2_CID_AUTOGAIN;
	queryctrl.type = V4L2_CTRL_TYPE_BOOLEAN;
	snprintf( (char*)queryctrl.name, sizeof(queryctrl.name)-1, "autogain" );
	queryctrl.minimum = 0;
	queryctrl.maximum = 1;
	queryctrl.step = 1;
	queryctrl.default_value = 1;
	queryctrl.flags = 0;
	// Add control to control list
	controls = add_control( -1, &queryctrl, controls, &n );

	// create virtual control
	queryctrl.id = V4L2_CID_GAIN;
	queryctrl.type = V4L2_CTRL_TYPE_INTEGER;
	snprintf( (char*)queryctrl.name, sizeof(queryctrl.name)-1, "gain" );
	queryctrl.minimum = 0;
	queryctrl.maximum = 100;
	queryctrl.step = 1;
	queryctrl.default_value = 10;
	queryctrl.flags = 0;
	// Add control to control list
	controls = add_control( -1, &queryctrl, controls, &n );

	// create virtual control
	queryctrl.id = V4L2_CID_EXPOSURE;
	queryctrl.type = V4L2_CTRL_TYPE_INTEGER;
	snprintf( (char*)queryctrl.name, sizeof(queryctrl.name)-1, "exposure" );
	queryctrl.minimum = 0;
	queryctrl.maximum = 100;
	queryctrl.step = 1;
	queryctrl.default_value = 0;
	queryctrl.flags = 0;
	// Add control to control list
	controls = add_control( -1, &queryctrl, controls, &n );

	num_controls = n;

	return 0;
}


void cvideo_null::generate_emu_stars( void )
{
	data_ptr raw = buffers[0].start;
	int pix_max = (1 << bpp())-1;

	// Create stars
	for( size_t k = 0;k < m_emu_stars.size();k++ )
	{
		emu_star_t star = m_emu_stars[k];
		int vx = rand()%10;
		int vy = rand()%10;
		int dx = rand()%3-1;
		int dy = rand()%3-1;
		star.x += vx > 7 ? dx : 0;
		star.y += vy > 7 ? dy : 0;
		double sigma = star.sigma;
		double star_size = ceil(sigma*3);
		double lum_rand = 1 - (double)rand()/(double)RAND_MAX * 0.2;

		for( int i = -star_size;i < star_size;i++ )
		{
			for( int j = -star_size;j < star_size;j++ )
			{
				if( star.x+j < 0 || star.x+j >= (int)capture_params.width ||
					star.y+i < 0 || star.y+i >= (int)capture_params.height )
					continue;

				size_t idx = (star.y+i)*capture_params.width+star.x+j;

				double val = (double)star.lum*lum_rand / sqrt(2*M_PI)*exp( -(i*i+j*j)/2/(sigma*sigma) );
				// add some noise to star
				// add background
				val += (double)raw.ptr16[idx];

				raw.ptr16[idx] = (uint16_t)((int)val < pix_max ? (int)val : pix_max);
			}
		}
	}

	for( size_t k = 0;k < m_emu_badpix.size();k++ )
	{
		emu_star_t &bp = m_emu_badpix[k];
		raw.ptr16[bp.y*capture_params.width+bp.x] = bp.lum;
	}

	// Create noise
/*
	for i=1:m
	for j=1:n
	SF(i,j) = SF(i,j) + 4096*rand(); // Noise simulation
	SF_CMPR(i,j) = SF_CMPR(i,j) + 4096*(rand() - 0.5);
	end
	end
*/
}


void cvideo_null::generate_emu_field( void )
{
	data_ptr raw = buffers[0].start;
	for( size_t i = 0;i < buffers[0].length;i++ )
	{
		raw.ptr16[i] = (32+rand()%(capture_params.gain+1))*256;
		//raw.ptr16[i] = 32*256 + rand()%100;
	}
}

}
