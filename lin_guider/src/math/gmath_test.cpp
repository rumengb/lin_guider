/*
 * cgmath_test.cpp
 *
 *      Author: gm
 */

#include <stdlib.h>

#include "gmath.h"
#include "common.h"
#include "vect.h"
#include "utils.h"
#include "gmath_test.h"


cgmath_test::cgmath_test( const common_params &comm_params ) :
	cgmath( comm_params ),
	m_osf_pos( Vector(20, 10, 0) ),
	m_osf_size( Vector(1, 1, 0) ),
	m_osf_vis_size( (point_t){1, 1} )
{
}



cgmath_test::~cgmath_test()
{
}


bool cgmath_test::set_video_params( int vid_wd, int vid_ht )
{
	// NOTE! Base implementation must be called first!
	bool res = cgmath::set_video_params( vid_wd, vid_ht );

	move_osf( 0, 0 );
	resize_osf( m_common_params.osf_size_kx, m_common_params.osf_size_ky );

	return res;
}


ovr_params_t *cgmath_test::prepare_overlays( void )
{
	ovr_params_t *ovr = cgmath::prepare_overlays();

	ovr->osf_pos.x = (int)m_osf_pos.x;
	ovr->osf_pos.y = (int)m_osf_pos.y;

	ovr->osf_size.x = m_osf_vis_size.x;
	ovr->osf_size.y = m_osf_vis_size.y;

	ovr->locked |= ovr_params_t::OVR_SQUARE;

	return ovr;
}


int cgmath_test::get_default_overlay_set( void ) const
{
	// NOTE!
	// I turned on OVR_SQUARE to make shift visible
	// It doesn't look like a cross yet.
	// it will be later
	return ovr_params_t::OVR_SQUARE | ovr_params_t::OVR_RETICLE | ovr_params_t::OVR_OSF;
}


void cgmath_test::move_osf( double newx, double newy )
{
	int video_width, video_height;
	get_data_buffer( &video_width, &video_height, NULL, NULL );

	m_osf_pos.x = newx;
	m_osf_pos.y = newy;

	// check frame ranges
	if( m_osf_pos.x < 0 )
		m_osf_pos.x = 0;
	if( m_osf_pos.y < 0 )
		m_osf_pos.y = 0;
	if( m_osf_pos.x+(double)m_osf_vis_size.x > (double)video_width )
		m_osf_pos.x = (double)(video_width - m_osf_vis_size.x);
	if( m_osf_pos.y+(double)m_osf_vis_size.y > (double)video_height )
		m_osf_pos.y = (double)(video_height - m_osf_vis_size.y);
}


void cgmath_test::resize_osf( double kx, double ky )
{
	int video_width, video_height;
	get_data_buffer( &video_width, &video_height, NULL, NULL );

	kx = kx < 0.1 ? 1 : kx;
	kx = kx > 1 ? 1 : kx;
	ky = ky < 0.1 ? 1 : ky;
	ky = ky > 1 ? 1 : ky;

	Vector oldc = Vector( m_osf_pos.x - m_osf_vis_size.x/2, m_osf_pos.y - m_osf_vis_size.y/2, 0 );

	m_osf_vis_size.x = video_width * kx;
	m_osf_vis_size.y = video_height * ky;

	// check position
	move_osf( oldc.x - m_osf_vis_size.x/2, oldc.y -  m_osf_vis_size.y/2 );
}


void cgmath_test::get_osf_params( double *x, double *y, double *kx, double *ky ) const
{
	if( x )
		*x = m_osf_pos.x;
	if( y )
		*y = m_osf_pos.y;
	if( kx )
		*kx = (double)m_osf_size.x;
	if( ky )
		*ky = (double)m_osf_size.y;
}


Vector cgmath_test::find_star_local_pos( void ) const
{
	int wd, ht;
	const double *data = get_data_buffer( &wd, &ht, NULL, NULL );
	double r_x, r_y;
	get_reticle_params( &r_x, &r_y, NULL );

	double x, y;
	x = r_x +5+ rand()%10 - 5;
	y = r_y +5+ rand()%10 - 5;
	if( x < 0 ) x = 0;
	if( x > (double)wd-1 ) x = wd-1;
	if( y < 0 ) y = 0;
	if( y > (double)ht-1 ) y = ht-1;

	return Vector( x, y, 0 );
}


void cgmath_test::on_start( void )
{
	log_i( "cgmath_test::%s", __FUNCTION__ );
}


void cgmath_test::on_stop( void )
{
	log_i( "cgmath_test::%s", __FUNCTION__ );
}
