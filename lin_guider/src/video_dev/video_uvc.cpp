/*
 * video_uvc.cpp
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
#include <sys/mman.h>
#include <assert.h>
#include <unistd.h>

#include "video_uvc.h"
#include "timer.h"
#include "utils.h"

namespace video_drv
{

//-------------------------------------------- UVC ---------------------------------------------------------
cvideo_uvc::cvideo_uvc()
{
	device_type = DT_WEBCAM;
}


cvideo_uvc::~cvideo_uvc()
{
	stop();
}


time_fract_t cvideo_uvc::set_fps( const time_fract_t &new_fps )
{
 struct v4l2_streamparm setfps;
 int ret = -1;
 bool t_stream = is_streaming;


	if( t_stream )
		stop_capturing();

 	memset(&setfps, 0, sizeof(struct v4l2_streamparm));

 	setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 	setfps.parm.capture.timeperframe.numerator = new_fps.numerator;
 	setfps.parm.capture.timeperframe.denominator = new_fps.denominator;

 	ret =  xioctl(fd, VIDIOC_S_PARM, &setfps);
 	if( ret == -1 )
 	{
 		perror("Unable to set frame rate");
 		//return 0;
 	}

 	ret =  xioctl(fd, VIDIOC_G_PARM, &setfps);
 	if( ret == 0 )
 	{
 		if( setfps.parm.capture.timeperframe.numerator != new_fps.numerator || setfps.parm.capture.timeperframe.denominator != new_fps.denominator )
		{
 			force_fps = true;

 			log_e( "  Frame rate:   %u/%u fps (requested frame rate %u/%u fps is not supported by device)\nForcing software delays for frame rate %u/%u fps",
 					setfps.parm.capture.timeperframe.denominator,
 					setfps.parm.capture.timeperframe.numerator,
 					new_fps.denominator,
 					new_fps.numerator,
 					new_fps.denominator,
 					new_fps.numerator
 					);
 		}
 		else
 		{
 			log_i( "  Frame rate:   %u/%u fps", new_fps.denominator, new_fps.numerator );
 		}
 	}
 	else
 	{
 		perror("Unable to read out current frame rate");
 		return time_fract::mk_fps( 0, 0 );
 	}

 	if( initialized )
 		pthread_mutex_lock( &cv_mutex );

 	capture_params.fps = time_fract::mk_fps( setfps.parm.capture.timeperframe.numerator, setfps.parm.capture.timeperframe.denominator );
 	if( force_fps )
 		capture_params.fps = new_fps;
 	capture_params.fps = (capture_params.fps.numerator != 0 && capture_params.fps.denominator != 0) ? capture_params.fps : time_fract::mk_fps( 1, 1 );
 	frame_delay = time_fract::to_msecs( capture_params.fps );

 	if( initialized )
 		pthread_mutex_unlock( &cv_mutex );


 	if( t_stream )
 		start_capturing();

 return capture_params.fps;
}



// UVC stuff
int cvideo_uvc::init_device( void )
{
 struct v4l2_capability cap;
 struct v4l2_cropcap cropcap;
 struct v4l2_crop crop;
 int sizeimage = 0;
 int i;
 int res = 0;


	memset( &cap, 0, sizeof(struct v4l2_capability) );
 	memset( &cropcap, 0, sizeof(struct v4l2_cropcap) );
 	memset( &crop, 0, sizeof(struct v4l2_crop) );

	capture_params.pixel_format = V4L2_PIX_FMT_YUYV;

 	// aquire capabilities
 	if( xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1 )
    {
 		if( errno == EINVAL )
 		{
 			log_e( "%s is no V4L2 device", dev_name);
 			return EXIT_FAILURE;
 		}
 		else
 		{
 			log_e( "VIDIOC_QUERYCAP");
 			return EXIT_FAILURE;
 		}
    }
 	// capture check
 	if( !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) )
    {
 		log_e( "%s is no video capture device", dev_name);
 		return EXIT_FAILURE;
    }
 	// access check
 	switch( capture_params.io_mtd )
    {
     case IO_METHOD_READ:

    	 if( !(cap.capabilities & V4L2_CAP_READWRITE) )
    	 {
    		 log_e( "%s does not support read i/o", dev_name);
    		 return EXIT_FAILURE;
    	 }
      break;
     case IO_METHOD_MMAP:
    	 if( !(cap.capabilities & V4L2_CAP_STREAMING) )
    	 {
    		 log_e( "%s does not support streaming i/o", dev_name);
    		 return EXIT_FAILURE;
    	 }
      break;
    }


 	// Enumerate the supported formats to check whether the requested one
	// is available. If not, we try to fall back to YUYV.
	int requested_format_found = 0, fallback_format = -1;

	capture_params.pixel_format = V4L2_PIX_FMT_YUYV;

	for( i = 0; i < MAX_FMT && device_formats[i].format;i++ )
	{
		if( device_formats[i].format == capture_params.pixel_format )
		{
			requested_format_found = 1;
			break;
		}

		if( device_formats[i].format == V4L2_PIX_FMT_MJPEG || device_formats[i].format == V4L2_PIX_FMT_JPEG ||
			device_formats[i].format == V4L2_PIX_FMT_YUYV || device_formats[i].format == V4L2_PIX_FMT_YUV420
		)
			fallback_format = i;
	}

	if( requested_format_found )
	{
		// The requested format is supported
		log_i( "  Frame format: " FOURCC_FORMAT, FOURCC_ARGS(capture_params.pixel_format) );
	}
	else
	if( fallback_format >= 0 )
	{
		// The requested format is not supported but there's a fallback format
		log_i("  Frame format: " FOURCC_FORMAT " (" FOURCC_FORMAT " is not supported by device)",
				FOURCC_ARGS(device_formats[ fallback_format ].format), FOURCC_ARGS(capture_params.pixel_format) );

		capture_params.pixel_format = device_formats[ fallback_format ].format;
	}
	else
	{
		// The requested format is not supported and no fallback format is available
		log_e("ERROR: Requested frame format " FOURCC_FORMAT " is not available and no fallback format was found.",
				FOURCC_ARGS(capture_params.pixel_format));
		return EXIT_FAILURE;
	}

	param_val_t val;
	val.set( V4L2_EXPOSURE_AUTO );
	if( set_control( V4L2_CID_EXPOSURE_AUTO, val ) == -1 )
	{
		log_e("Unable to set exposure into autoexposure mode");
	}



 	// Select video input, video standard and tune here.
 	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 	xioctl(fd, VIDIOC_CROPCAP, &cropcap);


 	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 	crop.c = cropcap.defrect;	// reset to default

 	if( xioctl(fd, VIDIOC_S_CROP, &crop) == -1 )
    {
 		switch( errno )
 		{
 		 case EINVAL:
 			 // Cropping not supported.
 		  break;
 		 default:
 			 // Errors ignored.
 		  break;
 		}
    }

 	sizeimage = set_format();
 	if( sizeimage == EXIT_FAILURE )
 		return EXIT_FAILURE;

 	set_fps( capture_params.fps );

 	set_gain( capture_params.gain );
 	set_exposure( capture_params.exposure );

 	get_autogain();
 	get_gain();
 	get_exposure();

 	switch( capture_params.io_mtd )
    {
     case IO_METHOD_READ:
    	 res = init_read( sizeimage );
      break;
     case IO_METHOD_MMAP:
    	 res = init_mmap();
      break;
    }

 return res;
}


int cvideo_uvc::uninit_device( void )
{
 unsigned int i;

 	if( !buffers )
	{
 		log_e( "buffers not valid %s, %s", __FUNCTION__, __LINE__ );
		return EXIT_FAILURE;
	}

 	switch( capture_params.io_mtd )
    {
     case IO_METHOD_READ:
    	 if( buffers[0].start.ptr )
    		 free( buffers[0].start.ptr );
      break;
     case IO_METHOD_MMAP:
    	 for( i = 0; i < n_buffers; ++i )
    		 if( !buffers[i].start.ptr || munmap(buffers[i].start.ptr, buffers[i].length) == -1 )
    		 {
    			 log_e( "munmap %s, %s", __FUNCTION__, __LINE__ );
    			 return EXIT_FAILURE;
    		 }
      break;
    }

 	free( buffers );
 	buffers = NULL;

 return 0;
}


int cvideo_uvc::start_capturing( void )
{
 unsigned int i;
 enum v4l2_buf_type type;
 struct v4l2_buffer buf;


 	switch( capture_params.io_mtd )
    {
     case IO_METHOD_READ:
    	 // Nothing to do.
      break;

     case IO_METHOD_MMAP:
    	 for( i = 0; i < n_buffers; ++i )
    	 {
    		 memset( &buf, 0, sizeof(buf) );

    		 buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    		 buf.memory = V4L2_MEMORY_MMAP;
    		 buf.index = i;

    		 if( xioctl(fd, VIDIOC_QBUF, &buf) == -1 )
    		 {
    			 log_e( "VIDIOC_QBUF %s %d", __FILE__, __LINE__);
    			 return EXIT_FAILURE;

    		 }
    	 }

    	 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    	 // turn on stream
    	 if( xioctl(fd, VIDIOC_STREAMON, &type) == -1 )
    	 {
    		 log_e( "VIDIOC_STREAMON");
    		 return EXIT_FAILURE;
    	 }
      break;
    }

 	is_streaming = true;

 return 0;
}


int cvideo_uvc::stop_capturing( void )
{
 enum v4l2_buf_type type;

 	switch( capture_params.io_mtd )
    {
     case IO_METHOD_READ:
    	 // Nothing to do.
      break;
     case IO_METHOD_MMAP:
    	 // stop stream
    	 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    	 if( xioctl(fd, VIDIOC_STREAMOFF, &type) == -1 )
    	 {
    		 log_e( "VIDIOC_STREAMOFF");
    		 return EXIT_FAILURE;
    	 }
      break;
    }

 	is_streaming = false;

 return 0;
}


int cvideo_uvc::init_read( unsigned int buffer_size )
{

	buffers = (buffer *)calloc( 1, sizeof(*buffers) );

	if( !buffers )
    {
		log_e( "Out of memory");
		return EXIT_FAILURE;
    }

	buffers[0].length = buffer_size;
	buffers[0].start = (void *)malloc( sizeof(u_char)*buffer_size );

	if( !buffers[0].start.ptr )
    {
		log_e( "Out of memory");
		free( buffers );
		return EXIT_FAILURE;
    }

 return 0;
}



int cvideo_uvc::init_mmap( void )
{
 struct v4l2_requestbuffers req;
 struct v4l2_buffer buf;
 unsigned int i;


 	memset( &req, 0, sizeof(req) );

 	// set requested buffers
 	req.count = 4;
 	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 	req.memory = V4L2_MEMORY_MMAP;

 	if( xioctl(fd, VIDIOC_REQBUFS, &req) == -1 )
    {
 		if( errno == EINVAL )
 		{
 			log_e( "%s does not support memory mapping", dev_name);
 			return EXIT_FAILURE;
 		}
 		else
 		{
 			log_e( "VIDIOC_REQBUFS");
 			return EXIT_FAILURE;
 		}
    }

 	if( req.count < 2 )
    {
 		log_e( "Insufficient buffer memory on %s", dev_name);
 		return EXIT_FAILURE;
    }

 	// allocate N pointers
 	buffers = (buffer *)calloc( req.count, sizeof(*buffers) );

 	if( !buffers )
    {
 		log_e( "Out of memory");
 		return EXIT_FAILURE;
    }

 	// perform memory mapping
 	for( n_buffers = 0; n_buffers < req.count; ++n_buffers )
    {
 		memset( &buf, 0, sizeof(buf) );

 		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 		buf.memory = V4L2_MEMORY_MMAP;
 		buf.index = n_buffers;

 		if( xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1 )
 		{
 			free( buffers );
 			log_e( "VIDIOC_QUERYBUF");
 			return EXIT_FAILURE;
 		}

 		buffers[n_buffers].length = buf.length;
 		buffers[n_buffers].start = mmap(NULL, // start anywhere
				      					buf.length,
				      					PROT_READ | PROT_WRITE, // required
				      					MAP_SHARED,	// recommended
				      					fd, buf.m.offset);

 		if( buffers[n_buffers].start.ptr == MAP_FAILED )
 		{
 			 for( i = 0; i < n_buffers; ++i)
 			 {
 				  if( munmap( buffers[i].start.ptr, buffers[i].length ) == -1 )
 				  {
 					  log_e( "mmap - munmap");
 					  break;
 				  }
 			 }

 			free( buffers );
 			log_e( "mmap");
 			return EXIT_FAILURE;
 		}
    }

 return 0;
}



int cvideo_uvc::read_frame( void )
{
 struct v4l2_buffer buf;
 ssize_t read_bytes;
 unsigned int total_read_bytes;

 struct timespec tv;
 ctimer tm;
 long delay;


 	//-------- force FPS if needed -----------
 	if( force_fps )
 		tm.start();

 	switch( capture_params.io_mtd )
    {
     case IO_METHOD_READ:

    	 total_read_bytes = 0;
    	 do
    	 {
    		 read_bytes = read( fd, buffers[0].start.ptr, buffers[0].length );
    		 if( read_bytes < 0 )
    		 {
    			 switch( errno )
    			 {
    			  case EIO:
    			  case EAGAIN:
    				  continue;
    			  default:
    				  log_e( "read");
    				  return EXIT_FAILURE;
    			 }
    		 }
    		 total_read_bytes += read_bytes;

    	 }while( total_read_bytes < buffers[0].length );

    	 buf.index = 0;
    	 buf.length = buffers[0].length;
      break;

     case IO_METHOD_MMAP:

    	 memset( &buf, 0, sizeof(buf) );

    	 buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    	 buf.memory = V4L2_MEMORY_MMAP;

    	 if( xioctl(fd, VIDIOC_DQBUF, &buf) == -1 )
    	 {
    		 switch( errno )
    		 {
    		  case EAGAIN:
    			  return 0;
    		  case EIO:
    			  /* Could ignore EIO, see spec. */
    			  /* fall through */

    		  default:
    			  log_e( "VIDIOC_QBUF %s %d", __FILE__, __LINE__ );
    			  return EXIT_FAILURE;
    		 }
    	 }

    	 assert(buf.index < n_buffers);

      break;
    }


 	// synchronize data with main GUI thread
 	emit renderImage( buffers[ buf.index ].start.ptr, buf.bytesused /*buf.length*/ );

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

 	// finalize mmap metod
 	if( capture_params.io_mtd == IO_METHOD_MMAP )
 	{
 		if( xioctl(fd, VIDIOC_QBUF, &buf) == -1 )
 		{
 			log_e( "VIDIOC_QBUF %s %d", __FILE__, __LINE__ );
 		    return EXIT_FAILURE;
 		}
 	}

 return 0;
}


int cvideo_uvc::set_format( void )
{
 // setup format
 struct v4l2_format fmt;


	 memset( &fmt, 0, sizeof(fmt) );

	 fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	 fmt.fmt.pix.width = capture_params.width;
	 fmt.fmt.pix.height = capture_params.height;
	 fmt.fmt.pix.pixelformat = capture_params.pixel_format;
	 fmt.fmt.pix.field = V4L2_FIELD_ANY;

//	 if( (int)fmt.fmt.pix.width > vcap.maxwidth || (int)fmt.fmt.pix.height > vcap.maxheight )
//	 {
//		 fmt.fmt.pix.width = vcap.maxwidth;
//		 fmt.fmt.pix.height = vcap.maxheight;
//	 }
//	 if( (int)fmt.fmt.pix.width < vcap.minwidth || (int)fmt.fmt.pix.height < vcap.minheight )
//	 {
//		 fmt.fmt.pix.width = vcap.minwidth;
//		 fmt.fmt.pix.height = vcap.minheight;
//	 }

	 if( xioctl(fd, VIDIOC_S_FMT, &fmt) == -1 )
	 {
		 log_e( "VIDIOC_S_FMT %s", dev_name);
		 return EXIT_FAILURE;
	 }

	 // !!!!! Note VIDIOC_S_FMT may change width and height. !!!!!
	 capture_params.width  = fmt.fmt.pix.width;
	 capture_params.height = fmt.fmt.pix.height;


 return fmt.fmt.pix.sizeimage;
}

}
