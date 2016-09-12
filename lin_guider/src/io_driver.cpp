/*
 * io_driver.cpp
 *
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
#include <unistd.h>
#include <pthread.h>
#include <time.h>

#include <errno.h>

#include "io_driver.h"
#include "timer.h"
#include "utils.h"
#include "maindef.h"

namespace io_drv
{

struct guide_dir_desc  bit_list_data[5] = {
									{NO_DIR, "---"},
									{RA_INC_DIR, "ra+"},
									{RA_DEC_DIR, "ra-"},
									{DEC_INC_DIR, "dec+"},
									{DEC_DEC_DIR, "dec-"}
									};

device_desc_t device_desc_list[DEVICE_CNT] = {
									{
										DT_NULL,
										false,
										8,
										"Null (Default)",
										NULL,
										NULL
									},
									{
										DT_LPT,
										true,
										8,
										"LPT",
										NULL,
										NULL
									},
									{
										DT_FTDI,
										false,
										8,
										"FTDI",
										"External library libio_ftdi.so is required.",
										"External library libio_ftdi.so is required."
									},
									{
										DT_GPUSB,
										false,
										4,
										"GPUSB",
										NULL,
										NULL
									},
									{
										DT_GPIO,
										false,
										4,
										"GPIO",
										NULL,
										NULL
									},
									{
										DT_QHY5,
										false,
										4,
										"QHY 5",
										NULL,
										NULL
									},
									{
										DT_QHY6,
										false,
										4,
										"QHY 6",
										NULL,
										NULL
									},
									{
										DT_QHY5II,
										false,
										4,
										"QHY 5II / 5LII",
										"Works only in 8-bit video mode.",
										"Works only in 8-bit video mode."
									},
									{
										DT_NEXSTAR,
										true,
										4,
										"Celestron / NexStar compat.",
										"External library libnexstar is required. http://sourceforge.net/projects/libnexstar/",
										"<html><head/><body><p>The guiding rate of the driver is 0.5 sidereal rate.<br><br>External library <a href=\"http://sourceforge.net/projects/libnexstar/\"><span style=\"text-decoration: underline; color:#0000ff;\">libnexstar</span></a> is required.</p></body></html>"
									},
									{
										DT_ATIK,
										false,
										4,
										"Atik",
										"Atikccdsdk is required",
										"<html><head/><body><p>External library <a href=\"https://sourceforge.net/projects/linguider/files/atik_sdk/\"><span style=\"text-decoration: underline; color:#0000ff;\">atikccdsdk</span></a> is required.</p></body></html>"
									},
									{
										DT_SX,
										false,
										4,
										"Starlight Xpress",
										NULL,
										NULL
									},
									{
										DT_ASI,
										false,
										4,
										"ZWO ASI",
										#ifdef __arm__
										"libasicamera is required (Highly unstable on ARM)",
										"<html><head/><body><p>External library <a href=\"https://sourceforge.net/projects/linguider/files/asi_sdk/\"><span style=\"text-decoration: underline; color:#0000ff;\">libasicamera</span></a> is required (Highly unstable on ARM).</p></body></html>"
										#else
										"Libasicamera is required",
										"<html><head/><body><p>External library <a href=\"https://sourceforge.net/projects/linguider/files/asi_sdk/\"><span style=\"text-decoration: underline; color:#0000ff;\">libasicamera</span></a> is required.</p></body></html>"
										#endif
									},
									{
										DT_SKYWATCHER,
										true,
										4,
										"Sky-Watcher / Orion",
										"External library libnexstar (version >= 0.15) is required. http://sourceforge.net/projects/libnexstar/",
										"<html><head/><body><p>The guiding rate of the driver is 0.5 sidereal rate.<br><br>External library <a href=\"http://sourceforge.net/projects/libnexstar/\"><span style=\"text-decoration: underline; color:#0000ff;\">libnexstar</span></a> is required (version >= 0.15).</p></body></html>"
									},
								};

device_bit_map_t device_bit_map_template[DEVICE_CNT] = {
									{{0, 1, 2, 3, 4, 5, 6, 7}}, // DT_NULL
									{{0, 1, 2, 3, 4, 5, 6, 7}}, // DT_LPT
									{{3, 6, 5, 7, 1, 2, 4, 0}}, // DT_FTDI
									{{1, 3, 2, 0, 7, 7, 7, 7}}, // DT_GPUSB
									{{0, 1, 2, 3, 4, 5, 6, 7}}, // DT_GPIO
									{{0, 1, 2, 3, 4, 5, 6, 7}}, // DT_QHY5
									{{0, 1, 2, 3, 4, 5, 6, 7}}, // DT_QHY6
									{{0, 1, 2, 3, 4, 5, 6, 7}}, // DT_QHY5II
									{{0, 1, 2, 3, 4, 5, 6, 7}}, // DT_NEXSTAR
									{{3, 2, 1, 0, 4, 5, 6, 7}}, // DT_ATIK
									{{0, 1, 2, 3, 4, 5, 6, 7}}, // DT_SX
									{{3, 2, 1, 0, 4, 5, 6, 7}}, // DT_ASI
									{{0, 1, 2, 3, 4, 5, 6, 7}}  // DT_SKYWATCHER
									};


//----------------------------------------
//
// Init section
//
//----------------------------------------
cio_driver_base::cio_driver_base()
{
	initialized = false;

	memset( dev_name, 0, sizeof(dev_name) );
	strcpy( dev_name, "/dev/parport0" );
	device_type = DT_NULL;
	next_device_type = -1;
	is_inverse = false;
	min_pulse_length = 0;
	max_pulse_length = 0;

	// initialize default lowlevel bit map encoder
	set_bit_map_template( device_bit_map_template[ device_type-1 ] );

	bit_actions[0] =
	bit_actions[1] =
	bit_actions[2] =
	bit_actions[3] =
	bit_actions[4] =
	bit_actions[5] =
	bit_actions[6] =
	bit_actions[7] = NO_DIR;

	port_data = 0x0;

	use_DT = false;
	DT = 100;
	clock_gettime( CLOCK_REALTIME, &relay_DT[0] );
	relay_DT[1] =
	relay_DT[2] =
	relay_DT[3] =
	relay_DT[4] =
	relay_DT[5] =
	relay_DT[6] =
	relay_DT[7] = relay_DT[0];
}


cio_driver_base::~cio_driver_base()
{
}


bool cio_driver_base::start( const char *devname )
{
 int len = 0;
 int ret = -1;

 	if( initialized )
 		return false;

 	// prepare...
	len =  strlen(devname);
	if( len > (int)(ARRAY_SIZE(dev_name))-1 || len <= 0 )
		return false;

	snprintf( dev_name, sizeof(dev_name), "%s", devname );

	// open device
	ret = open_device();
	if( ret )
	{
		const char *dev_desc = dev_name;

		if( device_type != DT_LPT )
		{
			for( size_t i = 0;i < ARRAY_SIZE( device_desc_list );i++ )
			{
				if( device_desc_list[i].type == device_type )
				{
					dev_desc = device_desc_list[i].desc;
					break;
				}
			}
		}
		log_e( "Error opening device: %s: code=%d", dev_desc, ret );
		return false;
	}

	// try to replace bitmap template by config template
	set_bit_map_template( cfg_bit_map_template );

	// set initial state
	send_device_command( NO_DIR, NO_DIR );

	// init thead staff
	pthread_mutex_init( &sync_mutex, NULL );
	pthread_mutex_init( &cv_mutex, NULL );
	pthread_cond_init( &cv, NULL );
	start_thread_flag 	= 0;
	quit_thread_flag 	= 0;

	pthread_create( &dev_thread, NULL, &cio_driver_base::controller_thread, this );

	// ok
	initialized = true;

 return true;
}


void cio_driver_base::stop( void )
{
	if( !initialized )
		return;

	// stop thread
	pthread_mutex_lock( &cv_mutex );
	start_thread_flag = 1;
	quit_thread_flag = 1;
	pthread_cond_signal( &cv );
	pthread_mutex_unlock( &cv_mutex );

	pthread_join( dev_thread, NULL );


	// release stuff
	pthread_mutex_destroy( &sync_mutex );
	pthread_mutex_destroy( &cv_mutex );
	pthread_cond_destroy( &cv );

	close_device();

	initialized = false;
}


bool cio_driver_base::check_device_dir_map( const bit_direction_t &dir )
{
 int i;
 int dir_cnt[4+1]; // no-dir + 4 directions
 int idx;

	 memset( dir_cnt, 0, sizeof(dir_cnt) );

	 // collect dir-info
	 for( i = 0;i < 8;i++ )
	 {
		 idx = dir.bit_direction[i];
		 idx = (idx >= 0 && idx < (4+1)) ? idx : 0;
		 dir_cnt[ idx ]++;
	 }

	 // analyse only not-empty directions. each dir must be one!
	 for( i = 1; i < (4+1);i++ )
	 {
		 if( dir_cnt[i] > 1 )	// alarm... possibly incorrect dir_map!!!
			 return false;
	 }

 return true;
}


bool cio_driver_base::set_deviceparams( const device_init_params_t &params )
{
 bool success = false;

	success = check_device_dir_map( params.dir_map );

	memcpy( bit_actions, params.dir_map.bit_direction, sizeof( bit_actions ) );
	is_inverse = params.dir_map.is_inverse;

	use_DT 	= params.death_time.use;
	DT		= params.death_time.delay;

	device_type = params.type;
	if( device_type <= DT_MIN || device_type >= DT_MAX )
	{
		device_type = DT_NULL;
		log_e( "cio_driver_base::set_deviceparams(): invalid device_type = %d, fall back to null device", device_type );
	}
	next_device_type = params.next_device_type != -1 ? params.next_device_type : device_type;

	cfg_bit_map_template = params.bit_map_template;

 return success;
}


device_init_params_t cio_driver_base::get_deviceparams( void )
{
	device_init_params_t params;

 	memcpy( params.dir_map.bit_direction, bit_actions, sizeof( bit_actions ) );
 	params.dir_map.is_inverse = is_inverse;

 	params.death_time.use 	= use_DT;
 	params.death_time.delay = DT;

 	params.type = device_type;
 	params.next_device_type = next_device_type;

 	params.bit_map_template = bit_map_template;

 return params;
}


device_ro_params_t cio_driver_base::get_ro_params( void )
{
	device_ro_params_t ro_params;

	ro_params.type = device_type;
	ro_params.min_pulse_length = min_pulse_length;
	ro_params.max_pulse_length = max_pulse_length;

 return ro_params;
}


void cio_driver_base::test_dir_map( const bit_direction_t &test_map, guide_dir test_dir )
{
 device_init_params_t current_params, test_params;

 	test_params = current_params = get_deviceparams();
 	test_params.dir_map = test_map;
 	set_deviceparams( test_params );

 	send_device_command( test_dir, NO_DIR );

 	set_deviceparams( current_params );
}


bool cio_driver_base::set_bit_map_template( const device_bit_map_t &tmpl )
{
 int i;

	for( i = 0;i < 8;i++ )
		if( tmpl.bit_map[i] > 7 )
		{
			if( DBG_VERBOSITY )
				log_e( "device bit map template is bad at %d bit: (using defaults)", i );
			return false;
		}

	bit_map_template = tmpl;

	init_bit_map_encoder();

 return true;
}


device_bit_map_t cio_driver_base::get_bit_map_template( void ) const
{
 return bit_map_template;
}



bool cio_driver_base::do_pulse( guide_dir ra_dir, int ra_msecs, guide_dir dec_dir, int dec_msecs )
{
 pulse_event evRA( ra_dir, ra_msecs );
 pulse_event evDEC( dec_dir, dec_msecs );

 	if( !initialized )
  		return false;

	// set thread data
	pthread_mutex_lock( &sync_mutex );

	if( evRA.is_RA() )
		request_RA = evRA;
	if( evDEC.is_DEC() )
		request_DEC = evDEC;

	pthread_mutex_unlock( &sync_mutex );

	// signal to pulse
	pthread_mutex_lock( &cv_mutex );
	start_thread_flag = 1;
	pthread_cond_signal( &cv );
	pthread_mutex_unlock( &cv_mutex );

 return true;
}


bool cio_driver_base::do_pulse( guide_dir dir, int msecs )
{
 pulse_event evData( dir, msecs );

 	if( !initialized )
  		return false;

	// set thread data
	pthread_mutex_lock( &sync_mutex );

	if( evData.is_RA() )
		request_RA = evData;
	if( evData.is_DEC() )
		request_DEC = evData;

	pthread_mutex_unlock( &sync_mutex );

	// signal to pulse
	pthread_mutex_lock( &cv_mutex );
	start_thread_flag = 1;
	pthread_cond_signal( &cv );
	pthread_mutex_unlock( &cv_mutex );


 return true;
}


bool cio_driver_base::reset( void )
{
	if( !initialized )
		return false;

	// stop guiding
	do_pulse( RA_INC_DIR, 0, DEC_INC_DIR, 0 );

	// apply map (useless call as soon as thread do all)
	//send_device_command( NO_DIR, NO_DIR );

 return true;
}


void cio_driver_base::init_bit_map_encoder( void )
{
 int i, j;
 u_char mapped;

	for( i = 0;i < 256;i++ )
	{
		mapped = 0;
		for( j = 0;j < 8;j++ )
			mapped |= (((i>>j) & 0x1) << bit_map_template.bit_map[j]);
		bit_map_encoder[i] = mapped;
	}
}


bool cio_driver_base::is_initialized( void )
{
 return initialized;
}


const char* cio_driver_base::get_name( void ) const
{
	if( !initialized )
		return "error";

	return device_type == DT_LPT ? (const char*)dev_name : device_desc_list[device_type-1].desc;
}


bool cio_driver_base::swap_dec_bits( void )
{
 int dec_inc_idx = -1;
 int dec_dec_idx = -1;

	for( int i = 0;i < 8;i++ )
	{
		if( bit_actions[i] == DEC_INC_DIR && dec_inc_idx == -1 )
			dec_inc_idx = i;
		else
		if( bit_actions[i] == DEC_DEC_DIR && dec_dec_idx == -1 )
			dec_dec_idx = i;
	}
	if( dec_inc_idx == -1 || dec_dec_idx == -1 )
	{
		log_e( "cio_driver_base::swap_dec_bits(): DEC_INC_DIR or DEC_DEC_DIR is not defined" );
		return false;
	}
	bit_actions[ dec_inc_idx ] = DEC_DEC_DIR;
	bit_actions[ dec_dec_idx ] = DEC_INC_DIR;

 return true;
}


inline u_char cio_driver_base::make_device_command( guide_dir dir1, guide_dir dir2 )
{
 int i;
 u_char data = 0;

	  for( i = 0;i < 8;i++ )
	  {
	    if( dir1 != NO_DIR && dir1 == bit_actions[i] )
	    	data |= (1 << i);

	    if( dir2 != NO_DIR && dir2 == bit_actions[i] )
	    	data |= (1 << i);
	  }

	  if( is_inverse )
		  data = ~data;

 return data;
}


inline void cio_driver_base::send_device_command( guide_dir dir1, guide_dir dir2 )
{
 u_char data = make_device_command( dir1, dir2 );
 struct timespec time;

 	if( DBG_VERBOSITY )
 	{
 		if( data != port_data )
 		{
 			clock_gettime( CLOCK_REALTIME, &time );
 			log_i( "outp tick %ld",  time.tv_nsec );
 		}
 	}

	if( use_DT )
		port_data = check_command_for_DT( data );
	else
		port_data = data;

 	write_data( (unsigned int)port_data );
}


inline unsigned char cio_driver_base::check_command_for_DT( unsigned char cmd )
{
 int i;
 u_char data, mask = 1;
 struct timespec now;
 long delta_t;

 	data = cmd;

 	clock_gettime( CLOCK_REALTIME, &now );

 	for( i = 0;i < 8;i++ )
 	{
 		// try to set pin
 		if( data & mask )
 		{
 			delta_t = 1000*(now.tv_sec - relay_DT[i].tv_sec) + (now.tv_nsec - relay_DT[i].tv_nsec)*(1e-6);

 			if( (port_data & mask) == 0 && delta_t < DT ) // try to set pin too fast
 				data &= (~mask);	// remove pin and don't update  time
 		}
 		else
 		{
 			// clear pin
 			if( port_data & mask )
 				relay_DT[i] = now;
 		}

 		mask <<= 1;
 	}

 return data;
}



#define INFINITE_TIME 0xFFFFFFFF

void *cio_driver_base::controller_thread( void *param )
{
 cio_driver_base *drv = (cio_driver_base*)param;
 ctimer timerRA, timerDEC;
 pulse_event pulseRA;
 pulse_event pulseDEC;
 int quit = 0;
 double time2stop;
 uint32_t wait_time;
 bool need_send, ra_timer_start, dec_timer_start;

 // api specific
 struct timespec thread_wait_time;
 long int nsecs;

 int res;

 	while( 1 )
 	{
 		wait_time = INFINITE_TIME;

 		if( pulseRA.is_set() )
 		{
 		    time2stop = (double)pulseRA.wait_time() - (double)timerRA.gettime();

 		    if( time2stop < 1.0 )
 		        wait_time = 0;
 		    else
 		        wait_time = (uint32_t)time2stop;
 		}

 		if( pulseDEC.is_set() )
 		{
 		  	time2stop = (double)pulseDEC.wait_time() - (double)timerDEC.gettime();

 		   	if( time2stop < 1.0 )
 		   		wait_time = 0;
 		   	else
 		   		wait_time = MIN((uint32_t)time2stop, wait_time);
 		}


 		// wait for start
 		if( wait_time > 0 )
 		{
	 		pthread_mutex_lock( &drv->cv_mutex );
	 		if( !drv->start_thread_flag )
	 		{
	 			//if( wait_time != INFINITE_TIME )	// pulse timeout
	 			{
		 			clock_gettime( CLOCK_REALTIME, &thread_wait_time ); // get absolute system time!!!
		 			// add pulse timeout to absolute time
		 			thread_wait_time.tv_sec  += wait_time / 1000;
		 			nsecs = (wait_time % 1000) * 1000000;
		 			if( nsecs + thread_wait_time.tv_nsec > 999999999 )
		 			{
		 				thread_wait_time.tv_sec++;
		 				nsecs = nsecs + thread_wait_time.tv_nsec - 1000000000;
		 				thread_wait_time.tv_nsec = nsecs;
		 			}
		 			else
		 				thread_wait_time.tv_nsec += nsecs;

		 			res = pthread_cond_timedwait( &drv->cv, &drv->cv_mutex, &thread_wait_time );

		 			//qDebug( strerror(res) );
		 			switch(res)
		 			{
		 			case EINVAL:
		 				log_e("EINVAL");
		 				break;
		 			case EPERM:
		 				log_e("EPERM");
		 				break;
		 			case ETIMEDOUT:
		 				if( DBG_VERBOSITY )
		 					log_i("ETIMEDOUT");
		 				break;
		 			}

	 			}
	 			//else
	 			//	pthread_cond_wait( &drv->cv, &drv->cv_mutex );  // infinite wait
	 		}
	 		drv->start_thread_flag = 0;
	 		pthread_mutex_unlock( &drv->cv_mutex );
 		}


 		// init after sync data
 		need_send = ra_timer_start = dec_timer_start = false;

 		// sync lock to access data
 		pthread_mutex_lock( &drv->sync_mutex );

 		quit = drv->quit_thread_flag;

 		if( drv->request_RA.is_set() )
 		{
 		    pulseRA = drv->request_RA;
 		    drv->request_RA.clear();

 		    need_send = ra_timer_start = true;
 		}

 		if( drv->request_DEC.is_set() )
 		{
 		    pulseDEC = drv->request_DEC;
 		    drv->request_DEC.clear();

 		   need_send = dec_timer_start = true;
 		}
 		pthread_mutex_unlock( &drv->sync_mutex );


 		// check for quit command
 		if( quit )
 			break;

 		if( ra_timer_start )
 			timerRA.start();
 		if( dec_timer_start )
 		 	timerDEC.start();

 		// track pulse end by timer
	    pulseRA.set_current_time( timerRA.gettime() );
	    if( pulseRA.is_set() && pulseRA.current_time() >= pulseRA.wait_time() )
	    {
	    	pulseRA.clear();
	    	need_send = true;
	    }

	    // track pulse end by timer
	    pulseDEC.set_current_time( timerDEC.gettime() );
	    if( pulseDEC.is_set() && pulseDEC.current_time() >= pulseDEC.wait_time() )
	    {
	    	pulseDEC.clear();
	    	need_send = true;
	    }

	    if( need_send )
	    	drv->send_device_command( pulseRA.pulse_dir(), pulseDEC.pulse_dir() );

 	}

 return NULL;
}

}
