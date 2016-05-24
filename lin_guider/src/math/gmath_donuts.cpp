/*
 * cgmath_donuts.cpp
 *
 *      Author: Andrew Stepanenko, Rumen Bogdanovski
 */

#include <stdlib.h>

#include "gmath.h"
#include "common.h"
#include "vect.h"
#include "utils.h"
#include "gmath_donuts.h"
#include "donuts_guide.h"


cgmath_donuts::cgmath_donuts( const common_params &comm_params ) :
	cgmath( comm_params ),
	m_osf_pos( Vector(20, 10, 0) ),
	m_osf_size( Vector(1, 1, 0) ),
	m_osf_vis_size( (point_t){1, 1} )
{
	m_guiding = false;
}



cgmath_donuts::~cgmath_donuts()
{
}


bool cgmath_donuts::set_video_params( int vid_wd, int vid_ht )
{
	// NOTE! Base implementation must be called first!
	bool res = cgmath::set_video_params( vid_wd, vid_ht );

	move_osf( 0, 0 );
	resize_osf( m_common_params.osf_size_kx, m_common_params.osf_size_ky );

	return res;
}


ovr_params_t *cgmath_donuts::prepare_overlays( void )
{
	ovr_params_t *ovr = cgmath::prepare_overlays();

	ovr->osf_pos.x = (int)m_osf_pos.x;
	ovr->osf_pos.y = (int)m_osf_pos.y;

	ovr->osf_size.x = m_osf_vis_size.x;
	ovr->osf_size.y = m_osf_vis_size.y;

	ovr->locked |= ovr_params_t::OVR_SQUARE | ovr_params_t::OVR_RETICLE;

	return ovr;
}


int cgmath_donuts::get_default_overlay_set( void ) const
{
	// NOTE!
	// I turned on OVR_SQUARE to make shift visible
	// It doesn't look like a cross yet.
	// it will be later
	return ovr_params_t::OVR_SQUARE | ovr_params_t::OVR_RETICLE | ovr_params_t::OVR_OSF;
}


void cgmath_donuts::move_osf( double newx, double newy )
{
	int video_width, video_height;
	double ang;
	double *buf = get_data_buffer( &video_width, &video_height, NULL, NULL );

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

	get_reticle_params( NULL, NULL, &ang );
	set_reticle_params(m_osf_pos.x + m_osf_vis_size.x/2 , m_osf_pos.y + m_osf_vis_size.y/2,ang);
}


void cgmath_donuts::resize_osf( double kx, double ky )
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


void cgmath_donuts::get_osf_params( double *x, double *y, double *kx, double *ky ) const
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


Vector cgmath_donuts::find_star_local_pos( void ) const
{
	int wd, ht, res;
	double r_x, r_y;
	frame_digest dg_new;
	corrections d_corr;
	const double *data = get_data_buffer( &wd, &ht, NULL, NULL );

	get_reticle_params( &r_x, &r_y, NULL );
	if (!m_guiding) return Vector( r_x, r_y, 0 );

	res = dg_new_frame_digest(data, wd, ht, &dg_new);
	if (res < 0) {
		log_e("dg_new_frame_digest(): failed");
		return Vector( r_x, r_y, 0 );
	}

	res = dg_calculate_corrections(&m_dg_ref, &dg_new, &d_corr);
	if (res < 0) {
		log_e("dg_calculate_corrections(): failed");
		dg_delete_frame_digest(&dg_new);
		return Vector( r_x, r_y, 0 );
	}

	dg_delete_frame_digest(&dg_new);
	if (res < 0) {
		log_e("dg_delete_frame_digest(): failed");
		return Vector( r_x, r_y, 0 );
	}

	log_i("%s(): %d %d",__FUNCTION__,wd, ht);

	log_i("corr = %f %f", r_x + d_corr.x, r_y + d_corr.y);

	return Vector( r_x + d_corr.x, r_y + d_corr.y, 0 );
}


void cgmath_donuts::on_start( void )
{
	int wd, ht;
	if (!m_guiding) {
		const double *data = get_data_buffer( &wd, &ht, NULL, NULL );
		dg_new_frame_digest(data, wd, ht, &m_dg_ref);
		m_guiding = true;
	}
	log_i( "cgmath_donuts::%s", __FUNCTION__ );
}


void cgmath_donuts::on_stop( void )
{
	if (m_guiding) {
		dg_delete_frame_digest(&m_dg_ref);
		m_guiding = false;
	}
	log_i( "cgmath_donuts::%s", __FUNCTION__ );
}
