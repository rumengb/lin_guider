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
#include "filters.h"
#include "gmath_donuts.h"
#include "donuts_guide.h"


namespace lg_math
{

cgmath_donuts::cgmath_donuts( const common_params &comm_params ) :
	cgmath_helper( comm_params ),

	m_osf_pos( Vector(20, 10, 0) ),
	m_osf_size( Vector(1, 1, 0) ),
	m_osf_vis_size( (point_t){1, 1} ),

	m_sub_frame( NULL ),
	m_guiding( false ),
	m_video_width( 0 ),
	m_video_height( 0 ),
	m_ref_x( 0 ),
	m_ref_y( 0 )
{
	m_type = GA_DONUTS;
}


cgmath_donuts::~cgmath_donuts()
{
}


bool cgmath_donuts::set_video_params( int vid_wd, int vid_ht )
{
	// NOTE! Base implementation must be called first!
	bool res = cgmath::set_video_params( vid_wd, vid_ht );

	m_video_width = vid_wd;
	m_video_height = vid_ht;

	resize_osf( m_common_params.osf_size_kx, m_common_params.osf_size_ky );
	move_osf( (vid_wd - vid_wd * m_common_params.osf_size_kx)/2, (vid_ht - vid_ht * m_common_params.osf_size_ky)/2 );

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

	if (m_guiding)
		ovr->visible = ovr_params_t::OVR_SQUARE | ovr_params_t::OVR_RETICLE | ovr_params_t::OVR_OSF;
	else
		ovr->visible =  ovr_params_t::OVR_RETICLE | ovr_params_t::OVR_OSF;

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
	double ang;

	m_osf_pos.x = newx;
	m_osf_pos.y = newy;

	// check frame ranges
	if( m_osf_pos.x < 0 )
		m_osf_pos.x = 0;
	if( m_osf_pos.y < 0 )
		m_osf_pos.y = 0;
	if( m_osf_pos.x+(double)m_osf_vis_size.x > (double)m_video_width )
		m_osf_pos.x = (double)(m_video_width - m_osf_vis_size.x);
	if( m_osf_pos.y+(double)m_osf_vis_size.y > (double)m_video_height )
		m_osf_pos.y = (double)(m_video_height - m_osf_vis_size.y);

	get_reticle_params( NULL, NULL, &ang );
	set_reticle_params(m_osf_pos.x + m_osf_vis_size.x/2 , m_osf_pos.y + m_osf_vis_size.y/2,ang);
}


void cgmath_donuts::resize_osf( double kx, double ky )
{
	kx = kx < 0.1 ? 1 : kx;
	kx = kx > 1 ? 1 : kx;
	ky = ky < 0.1 ? 1 : ky;
	ky = ky > 1 ? 1 : ky;

	Vector oldc = Vector( m_osf_pos.x + m_osf_vis_size.x/2, m_osf_pos.y + m_osf_vis_size.y/2, 0 );

	m_osf_vis_size.x = m_video_width * kx;
	m_osf_vis_size.y = m_video_height * ky;

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
	int res;
	double r_x, r_y;
	frame_digest dg_new;
	corrections d_corr;

	/* if guiding is not started - no corrections needed */
	if (!m_guiding) {
		get_reticle_params( &r_x, &r_y, NULL );
		return Vector( r_x, r_y, 0 );
	}

	/* m_subframe malloc failed? */
	if (!m_sub_frame) {
		log_e("Subframe not initialized!");
		get_reticle_params( &r_x, &r_y, NULL );
		return Vector( r_x, r_y, 0 );
	}

	copy_subframe(m_sub_frame, m_osf_pos.x, m_osf_pos.y, m_osf_vis_size.x, m_osf_vis_size.y);

	filters::medianfilter( (double*) m_sub_frame, (double*)NULL, m_osf_vis_size.x, m_osf_vis_size.y);

	res = dg_new_frame_digest(m_sub_frame, m_osf_vis_size.x, m_osf_vis_size.y, &dg_new);
	if (res < 0) {
		log_e("dg_new_frame_digest(): failed");
		return Vector( m_ref_x, m_ref_y, 0 );
	}

	res = dg_calculate_corrections(&m_dg_ref, &dg_new, &d_corr);
	if (res < 0) {
		log_e("dg_calculate_corrections(): failed");
		dg_delete_frame_digest(&dg_new);
		return Vector( m_ref_x, m_ref_y, 0 );
	}

	res = dg_delete_frame_digest(&dg_new);
	if (res < 0) {
		log_e("dg_delete_frame_digest(): failed");
		return Vector( m_ref_x, m_ref_y, 0 );
	}

	log_i("%s()",__FUNCTION__);

	log_i("corr = %f %f", m_ref_x + d_corr.x, m_ref_y + d_corr.y);

	set_quality_params( double(rand()%10+10)/20.0, 0.1 );

	return Vector( m_ref_x - d_corr.x, m_ref_y - d_corr.y, 0 );
}


void cgmath_donuts::on_start( void )
{
	if (!m_guiding) {
		m_sub_frame = (double *)realloc(m_sub_frame, m_osf_vis_size.x * m_osf_vis_size.y * sizeof(double));
		if(!m_sub_frame) {
			log_e( "cgmath_donuts::%s - can not allocate memory for subframe", __FUNCTION__ );
			return;
		}

		/* get reticle refference point so that we can use dithering */
		get_reticle_params(&m_ref_x, &m_ref_y, NULL);

		copy_subframe(m_sub_frame, m_osf_pos.x, m_osf_pos.y, m_osf_vis_size.x, m_osf_vis_size.y);

		/* clear one pixel spikes */
		filters::medianfilter( (double*) m_sub_frame, (double*)NULL, m_osf_vis_size.x, m_osf_vis_size.y);

		dg_new_frame_digest(m_sub_frame, m_osf_vis_size.x, m_osf_vis_size.y, &m_dg_ref);
		m_guiding = true;
	}
	log_i( "cgmath_donuts::%s", __FUNCTION__ );
}


void cgmath_donuts::on_stop( void )
{
	if (m_guiding) {
		if (m_sub_frame) {
			free(m_sub_frame);
			m_sub_frame = NULL;
		}

		dg_delete_frame_digest(&m_dg_ref);

		m_ref_x = 0;
		m_ref_y = 0;
		m_guiding = false;
	}
	log_i( "cgmath_donuts::%s", __FUNCTION__ );
}

}
