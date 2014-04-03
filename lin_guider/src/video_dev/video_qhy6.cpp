/*
 * video_qhy6.cpp
 *
 *  Created on: 02.06.2011
 *      Author: gm
 *
 * Original QHY6 video capturing code by Tom Vandeneede
 * Original code for ST-4 port by Vladimir Volynsky
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
#include <netinet/in.h>

#include "video_qhy6.h"
#include "timer.h"
#include "utils.h"

namespace video_drv
{

// enables test image
//#define QHY6_TEST_IMAGE

//-------------------------------------------- QHY6 ---------------------------------------------------------
// QHY6 stuff
cvideo_qhy6::cvideo_qhy6() :
	m_binn( 1 ),
	m_offset( 115 ),
	m_speed( 1 ), // 0 - slow, 1 - fast readout
 	m_amp( 2 ),
	m_vbe( 1 ),	// must be 1. else correct exposure time in read_frame()
	m_data_size( 0 )
{
	device_type = DT_QHY6;

	capture_params.fps = time_fract::mk_fps( 1, 1 );
	capture_params.gain = 50;
	capture_params.exposure = 1;

	m_qhy6_obj = new qhy6_core_shared();
}


cvideo_qhy6::~cvideo_qhy6()
{
	//MUST BE called the first: stop thread and release resources
	stop();

	delete m_qhy6_obj;
}


time_fract_t cvideo_qhy6::set_fps( const time_fract_t &new_fps )
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

	int ret = EXIT_FAILURE;
	unsigned frm_delay = time_fract::to_msecs( set_fps );
	ret = m_qhy6_obj->set_params( frm_delay, m_binn, capture_params.gain, m_offset, m_speed, m_amp, m_vbe, NULL, NULL, NULL );
	if( ret != EXIT_SUCCESS )
	{
		log_e( "cvideo_qhy6::set_fps(): set_params() failed." );
	}
	else
	{
		//force_fps = true;
		capture_params.fps = set_fps;
		frame_delay = frm_delay;

		log_i( "  Frame rate:   %u/%u fps", set_fps.denominator, set_fps.numerator );
	}

	if( initialized )
		pthread_mutex_unlock( &cv_mutex );

 return capture_params.fps;
}


int cvideo_qhy6::open_device( void )
{
 return m_qhy6_obj->open_device();
}


int cvideo_qhy6::close_device( void )
{
	m_qhy6_obj->close_device();

 return 0;
}


int cvideo_qhy6::get_vcaps( void )
{
	int i = 0;
	point_t pt;

	device_formats[0].format = V4L2_PIX_FMT_Y16;

	pt.x = QHY6_WIDTH_B1;
	pt.y = QHY6_HEIGHT_B1;
	device_formats[0].frame_table[ i ].size =  pt;
	device_formats[0].frame_table[ i ].fps_table[ 0 ] = time_fract::mk_fps( 4, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 1 ] = time_fract::mk_fps( 2, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 2 ] = time_fract::mk_fps( 1, 1 );
	device_formats[0].frame_table[ i ].fps_table[ 3 ] = time_fract::mk_fps( 1, 2 );
	device_formats[0].frame_table[ i ].fps_table[ 4 ] = time_fract::mk_fps( 1, 3 );
	device_formats[0].frame_table[ i ].fps_table[ 5 ] = time_fract::mk_fps( 1, 4 );
	device_formats[0].frame_table[ i ].fps_table[ 6 ] = time_fract::mk_fps( 1, 5 );
	device_formats[0].frame_table[ i ].fps_table[ 7 ] = time_fract::mk_fps( 1, 10 );
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


int cvideo_qhy6::set_control( unsigned int control_id, const param_val_t &val )
{
	switch( control_id )
	{
	case V4L2_CID_GAIN:
	{
		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 100 ) v = 100;
		int ret = EXIT_FAILURE;
		ret = m_qhy6_obj->set_params( frame_delay, m_binn, v, m_offset, m_speed, m_amp, m_vbe, NULL, NULL, NULL );
		if( ret != EXIT_SUCCESS )
		{
			log_e( "cvideo_qhy6::set_control(): set_params() failed." );
			return -1;
		}
		capture_params.gain = v;
		break;
	}
	case V4L2_CID_EXPOSURE:
	{
		int v = val.values[0];
		if( v < 0 ) v = 0;
		if( v > 65535 ) v = 65535;
		int top = 65536 - v;
		if( top <= 0 )
		{
			log_e( "cvideo_qhy6::set_control(): invalid exposure" );
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


int cvideo_qhy6::get_control( unsigned int control_id, param_val_t *val )
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


int cvideo_qhy6::init_device( void )
{
 int sizeimage = 0;
 int ret = EXIT_FAILURE;


	// set desired size
	sizeimage = set_format();
	if( sizeimage <= 0 )
		return EXIT_FAILURE;

	set_fps( capture_params.fps );

	// prepare...
	int wd, ht;
	ret = m_qhy6_obj->set_params( time_fract::to_msecs( capture_params.fps ), m_binn, capture_params.gain, m_offset, m_speed, m_amp, m_vbe, &wd, &ht, &m_data_size );

	if( ret != EXIT_SUCCESS )
	{
		log_e( "cvideo_qhy6::init_device(): set_params() failed." );
		return -1;
	}
	assert( wd == (int)capture_params.width && ht == (int)capture_params.height
#ifdef QHY6_WITH_ST4
			&& m_data_size == sizeimage
#endif
			 );

	n_buffers = 2;
	buffers = (buffer *)calloc( n_buffers, sizeof(*buffers) );

	if( !buffers )
	{
		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
		return EXIT_FAILURE;
	}

	buffers[0].length = m_data_size; //sizeimage;
	buffers[0].start.ptr = malloc( buffers[0].length );
	if( !buffers[0].start.ptr )
	{
		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
		free( buffers );
		return EXIT_FAILURE;
	}

	// init internal buffer.
	buffers[1].length = QHY6_BUFFER_LEN * sizeof(unsigned short);
	buffers[1].start.ptr = malloc( buffers[1].length );
	if( !buffers[1].start.ptr )
	{
		log_e( "Out of memory %s, %s", __FUNCTION__, __LINE__ );
		free( buffers[0].start.ptr );
		free( buffers );
		return EXIT_FAILURE;
	}

	set_exposure( capture_params.exposure );

	get_autogain();
	get_gain();
	get_exposure();

	return 0;
}


int cvideo_qhy6::uninit_device( void )
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


int cvideo_qhy6::start_capturing( void )
{
 return 0;
}


int cvideo_qhy6::stop_capturing( void )
{
 return 0;
}

#ifndef QHY6_WITH_ST4
void inline swap(uint8_t *x)
{
  uint8_t tmp;
  tmp = *x;
  *x=*(x+1);
  *(x+1) = tmp;
}
#endif

int cvideo_qhy6::read_frame( void )
{
 struct timespec tv;
 ctimer tm;
 long delay;
 int ret = 0;


	//-------- force FPS if needed -----------
  	if( force_fps )
  		tm.start();

 	//------------------------
  	ret = m_qhy6_obj->start_exposure( frame_delay );
	if( ret )
		return ret;

	ret = m_qhy6_obj->read_exposure( buffers[1].start.ptr8, m_data_size );
	if( ret )
		return ret;

#ifdef QHY6_TEST_IMAGE
	int pix_cnt = m_data_size / 2;
  	for( int i = 0;i < pix_cnt;i++ )
	{
  		buffers[1].start.ptr16[i] = rand();
	}
#endif

  	// convert to std buffer
  	uint16_t *src1, *src2, *tgt;
  	int line_sz = capture_params.width * sizeof( uint16_t );
  	unsigned int t;

  	// swap
#ifdef QHY6_WITH_ST4
#define SWAP(a,b) { a ^= b; a ^= (b ^= a); }
  	data_ptr raw = buffers[1].start;
  	unsigned data_size = m_data_size;
  	if( htons(0x55aa) != 0x55aa )
  		for( unsigned i = 0;i < data_size;i += 2 )
  			SWAP( raw.ptr8[i], raw.ptr8[i+1] );
#undef SWAP
#else
  	{
  	  int idx;
  	  uint8_t  *swb;
  	  swb = (uint8_t *)buffers[1].start.ptr8;
  	  idx = m_data_size>>1;
  	  while (idx--)
  	    {
  	      swap(swb);
  	      swb+=2;
  	    }
  	}
#endif

  	// decode
  	switch( m_binn )
  	{
  	case 1:  //1X1 binning
  		t = capture_params.height >> 1;
  		src1 = buffers[1].start.ptr16;
  		src2 = buffers[1].start.ptr16 + capture_params.width * t;
  		tgt = buffers[0].start.ptr16;

  		while( t-- )
  		{
#ifdef QHY6_WITH_ST4
  			memcpy( tgt, src1, line_sz );
  			tgt += capture_params.width;
  			memcpy( tgt, src2, line_sz );
  			tgt += capture_params.width;
#else
  			memcpy( tgt, src2, line_sz );
  			tgt += capture_params.width;
  			memcpy( tgt, src1, line_sz );
  			tgt += capture_params.width;
#endif

  			src1 += capture_params.width;
  			src2 += capture_params.width;
  		}
  		break;
  	case 2:  //2X2 binning
  		log_e( "cvideo_qhy6::read_frame(): binning x2 not supported" );
  		break;
  	}

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


int cvideo_qhy6::set_format( void )
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


int cvideo_qhy6::enum_controls( void )
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
	queryctrl.maximum = 65535;
	queryctrl.step = 1;
	queryctrl.default_value = capture_params.exposure;
	queryctrl.flags = 0;
	// Add control to control list
	controls = add_control( -1, &queryctrl, controls, &n );

	num_controls = n;

	return 0;
}

}
