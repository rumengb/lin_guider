/*
 * gmath.cpp
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

#include <math.h>
#include <string.h>
#include <stdint.h>

#include <algorithm>
#include <vector>
#include <utility>

#include "gmath.h"
#include "filters.h"
#include "vect.h"
#include "matr.h"
#include "common.h"
#include "io_driver.h"
#include "utils.h"

#define DEF_SQR_0	(16-0)
#define DEF_SQR_1	(32-0)
#define DEF_SQR_2	(64-0)
#define DEF_SQR_3	(128-0)

const guide_square_t guide_squares[] = { 	{DEF_SQR_0, DEF_SQR_0*DEF_SQR_0*1.0},
											{DEF_SQR_1, DEF_SQR_1*DEF_SQR_1*1.0},
											{DEF_SQR_2, DEF_SQR_2*DEF_SQR_2*1.0},
											{DEF_SQR_3, DEF_SQR_3*DEF_SQR_3*1.0},
											{-1, -1}
											};

const square_alg_t guide_square_alg[] = {
											{ SMART_THRESHOLD, "Smart" },
											{ AUTO_THRESHOLD, "Auto" },
											{ NO_THRESHOLD, "No thresh." },
											{ -1, "" }
											};

const q_control_t q_control_mtd[] = {
											{ Q_CTRL_OFF, "Off" },
											{ Q_CTRL_NOTIFY, "Notify" },
											{ Q_CTRL_FULL, "Full" },
											{ -1, "" }
											};


cgmath::cgmath( const common_params &comm_params ) :
	m_common_params( comm_params )
{
	// sys...
	ticks = 0;
	pdata = NULL;
	video_width  = -1;
	video_height = -1;
	ccd_pixel_width  = 0;
	ccd_pixel_height = 0;
	focal = 1;
	ROT_Z = Matrix(0);
	preview_mode = true;
	suspended	 = false;

	// square variables
	square_idx = fix_square_index( m_common_params.square_index );
	square_alg_idx	= SMART_THRESHOLD;
	square_size		= guide_squares[square_idx].size;
	square_square 	= guide_squares[square_idx].square;
	square_pos 	 = Vector(0);

	// sky coord. system vars.
	star_pos 	 	= Vector(0);
	scr_star_pos	= Vector(0);
	reticle_pos 	= Vector(0);
	reticle_org     = Vector(0);
	reticle_orts[0] = Vector(0);
	reticle_orts[1] = Vector(0);
	reticle_angle	= 0;

	// overlays
	memset( &overlays, 0, sizeof(overlays) );

	// processing
	in_params.reset();
	out_params.reset();
	channel_ticks[RA] = channel_ticks[DEC] = 0;
	accum_ticks[RA] = accum_ticks[DEC] = 0;
	drift[RA]  = new double[MAX_ACCUM_CNT];
	drift[DEC] = new double[MAX_ACCUM_CNT];
	memset( drift[RA], 0, sizeof(double)*MAX_ACCUM_CNT );
	memset( drift[DEC], 0, sizeof(double)*MAX_ACCUM_CNT );
	drift_integral[RA] = drift_integral[DEC] = 0;

	calc_dir_checker();

	// statistics
	do_statistics = true;
	sum = sqr_sum = 0;
	delta_prev = sigma_prev = sigma = 0;

	// quality estimation
	q_star_max = 0;
	q_bkgd = 0;
	memset( q_stat, 0, sizeof(double)*q_stat_len );
	q_control_idx = Q_CTRL_OFF;

	// info
	m_ra_drift_v  = 0;
	m_dec_drift_v = 0;

	m_hfd_sqr_info = NULL;

	hfd_init();
}


cgmath::~cgmath()
{
	delete [] drift[RA];
	delete [] drift[DEC];

	delete [] pdata;

	hfd_destroy();
}


bool cgmath::set_video_params( int vid_wd, int vid_ht )
{
	if( vid_wd <= 0 || vid_ht <= 0 )
		return false;

	if( pdata )
		delete [] pdata;
	pdata = new double[ vid_wd*vid_ht ];
	memset( pdata, 0, vid_wd*vid_ht*sizeof(double) );

	video_width  = vid_wd;
	video_height = vid_ht;

	set_reticle_params( video_width/2, video_height/2, m_common_params.reticle_angle );

	return true;
}


double *cgmath::get_data_buffer( int *width, int *height, int *length, int *size )
{
	if( width )
		*width = video_width;
	if( height )
		*height = video_height;
	if( length )
		*length = video_width * video_height;
	if( size )
		*size = video_width * video_height * sizeof(double);

	return pdata;
}


bool cgmath::set_guider_params( double ccd_pix_wd, double ccd_pix_ht, double guider_aperture, double guider_focal )
{
	if( ccd_pix_wd < 0 )
		ccd_pix_wd = 0;
	if( ccd_pix_ht < 0 )
		ccd_pix_ht = 0;
	if( guider_focal <= 0 )
		guider_focal = 1;

	ccd_pixel_width		= ccd_pix_wd / 1000.0; // from mkm to mm
	ccd_pixel_height	= ccd_pix_ht / 1000.0; // from mkm to mm
	aperture			= guider_aperture;
	focal 				= guider_focal;

	return true;
}


bool cgmath::set_reticle_params( double x, double y, double ang )
{
	// check frame ranges
 	if( x < 0 )
 		x = 0;
 	if( y < 0 )
 		y = 0;
 	if( x >= (double)video_width-1 )
 		x = (double)video_width-1;
 	if( y >= (double)video_height-1 )
 		y = (double)video_height-1;

	reticle_pos = reticle_org = Vector( x, y, 0 );

	if( ang >= 0 )
		reticle_angle = ang;

	ROT_Z = RotateZ( -M_PI*reticle_angle/180.0 ); // NOTE!!! sing '-' derotates star coordinate system

	reticle_orts[0] = Vector(1, 0, 0) * 100;
	reticle_orts[1] = Vector(0, 1, 0) * 100;

	reticle_orts[0] = reticle_orts[0] * ROT_Z;
	reticle_orts[1] = reticle_orts[1] * ROT_Z;

	// lets position static overlay
	overlays.reticle_axis_ra.x = reticle_orts[0].x;
	overlays.reticle_axis_ra.y = reticle_orts[0].y;

	overlays.reticle_axis_dec.x = -reticle_orts[1].x;
	overlays.reticle_axis_dec.y = -reticle_orts[1].y;	// invert y-axis

	overlays.reticle_pos.x = overlays.reticle_org.x = reticle_pos.x;
	overlays.reticle_pos.y = overlays.reticle_org.y = reticle_pos.y;

 	return true;
}


bool cgmath::get_reticle_params( double *x, double *y, double *ang ) const
{
	if( x )
		*x = reticle_pos.x;
	if( y )
		*y = reticle_pos.y;
	if( ang )
		*ang = reticle_angle;

	return true;
}


int  cgmath::fix_square_index( int square_index ) const
{
	if( square_index < 0 || square_index >= (int)(sizeof(guide_squares)/sizeof(guide_square_t))-1 )
		return DEFAULT_SQR;

	 return square_index;
}


int  cgmath::get_square_index( void ) const
{
	return square_idx;
}


int  cgmath::get_square_algorithm_index( void ) const
{
	return square_alg_idx;
}



cproc_in_params * cgmath::get_in_params( void )
{
	return &in_params;
}


void cgmath::set_in_params( const cproc_in_params *v )
{
	//in_params.threshold_alg_idx     = v->threshold_alg_idx;
	set_square_algorithm_index( v->threshold_alg_idx );
	in_params.guiding_rate 			         	= v->guiding_rate;
	in_params.guiding_normal_coef		     	= cgmath::precalc_proportional_gain(v->guiding_rate);
	in_params.normalize_gain		         	= v->normalize_gain;
	in_params.enabled_dir[RA] 		         	= v->enabled_dir[RA];
	in_params.enabled_dir[DEC]		         	= v->enabled_dir[DEC];
	in_params.enabled_dir_sign[RA][SGN_POS]	 	= v->enabled_dir_sign[RA][SGN_POS];
	in_params.enabled_dir_sign[RA][SGN_NEG]	 	= v->enabled_dir_sign[RA][SGN_NEG];
	in_params.enabled_dir_sign[DEC][SGN_POS] 	= v->enabled_dir_sign[DEC][SGN_POS];
	in_params.enabled_dir_sign[DEC][SGN_NEG]	= v->enabled_dir_sign[DEC][SGN_NEG];
	calc_dir_checker();
	in_params.average 							= v->average;
	in_params.accum_frame_cnt[RA] 				= v->accum_frame_cnt[RA];
	in_params.accum_frame_cnt[DEC] 				= v->accum_frame_cnt[DEC];
	in_params.proportional_gain[RA]  			= v->proportional_gain[RA];
	in_params.proportional_gain[DEC] 			= v->proportional_gain[DEC];
	in_params.integral_gain[RA] 				= v->integral_gain[RA];
	in_params.integral_gain[DEC] 				= v->integral_gain[DEC];
	in_params.derivative_gain[RA] 				= v->derivative_gain[RA];
	in_params.derivative_gain[DEC] 				= v->derivative_gain[DEC];
	in_params.max_pulse_length[RA] 				= v->max_pulse_length[RA];
	in_params.max_pulse_length[DEC] 			= v->max_pulse_length[DEC];
	in_params.min_pulse_length[RA]				= v->min_pulse_length[RA];
	in_params.min_pulse_length[DEC]				= v->min_pulse_length[DEC];
	set_q_control_index( v->q_control_idx );
	in_params.quality_threshold1    = v->quality_threshold1;
	in_params.quality_threshold2    = v->quality_threshold2;
	in_params.stability_limit_factor = v->stability_limit_factor;
	// need to check ranges (range values are Sigmas, so may be hardcoded)
	if( in_params.stability_limit_factor < 1.0 ) in_params.stability_limit_factor = 1.0;
	if( in_params.stability_limit_factor > 3.0 ) in_params.stability_limit_factor = 3.0;
}


void cgmath::calc_dir_checker( void )
{
	dir_checker[ io_drv::NO_DIR ] = io_drv::NO_DIR;

	if( in_params.enabled_dir[RA] )
	{
		dir_checker[ io_drv::RA_INC_DIR ] = in_params.enabled_dir_sign[RA][SGN_POS] ? io_drv::RA_INC_DIR : io_drv::NO_DIR;
		dir_checker[ io_drv::RA_DEC_DIR ] = in_params.enabled_dir_sign[RA][SGN_NEG] ? io_drv::RA_DEC_DIR : io_drv::NO_DIR;
	}
	else
		dir_checker[ io_drv::RA_INC_DIR ] = dir_checker[ io_drv::RA_DEC_DIR ] = io_drv::NO_DIR;

	if( in_params.enabled_dir[DEC] )
	{
		dir_checker[ io_drv::DEC_INC_DIR ] = in_params.enabled_dir_sign[DEC][SGN_POS] ? io_drv::DEC_INC_DIR : io_drv::NO_DIR;
		dir_checker[ io_drv::DEC_DEC_DIR ] = in_params.enabled_dir_sign[DEC][SGN_NEG] ? io_drv::DEC_DEC_DIR : io_drv::NO_DIR;
	}
	else
		dir_checker[ io_drv::DEC_INC_DIR ] = dir_checker[ io_drv::DEC_DEC_DIR ] = io_drv::NO_DIR;
}


const cproc_out_params * cgmath::get_out_params( void ) const
{
	return &out_params;
}


info_params_t cgmath::get_info_params( void ) const
{
	info_params_t ret;
	Vector p;

 	ret.aperture	= aperture;
 	ret.focal		= focal;
 	ret.focal_ratio	= focal / aperture;
 	p = Vector(video_width, video_height, 0);
 	p = point2arcsec( p );
 	p /= 60;	// convert to minutes
 	ret.fov_wd	= p.x;
 	ret.fov_ht	= p.y;

 return ret;
}


uint32_t cgmath::get_ticks( void ) const
{
	return ticks;
}


void cgmath::get_star_drift( double *dx, double *dy ) const
{
	*dx = star_pos.x;
	*dy = star_pos.y;
}


void cgmath::get_star_screen_pos( double *dx, double *dy ) const
{
	*dx = scr_star_pos.x;
	*dy = scr_star_pos.y;
}


bool cgmath::reset( void )
{
	square_idx 		= fix_square_index( m_common_params.square_index );
	square_alg_idx	= AUTO_THRESHOLD;
	square_size		= guide_squares[square_idx].size;
	square_square 	= guide_squares[square_idx].square;
	square_pos 	 	= Vector(0);

	// sky coord. system vars.
	star_pos 	 	= Vector(0);
	scr_star_pos	= Vector(0);

	set_reticle_params( video_width/2, video_height/2, 0.0 );

	return true;
}


void cgmath::move_square( double newx, double newy )
{
	square_pos.x = newx;
	square_pos.y = newy;

	// check frame ranges
	if( square_pos.x < 0 )
		square_pos.x = 0;
	if( square_pos.y < 0 )
		square_pos.y = 0;
	if( square_pos.x+(double)square_size > (double)video_width )
		square_pos.x = (double)(video_width - square_size);
	if( square_pos.y+(double)square_size > (double)video_height )
		square_pos.y = (double)(video_height - square_size);
}


void cgmath::resize_square( int size_idx )
{
	size_idx = fix_square_index( size_idx );

	int old_hsz = square_size / 2;

	square_size = guide_squares[size_idx].size;
	square_square = guide_squares[size_idx].square;
	square_idx = size_idx;

	// check position
	move_square( square_pos.x + double(old_hsz - square_size/2), square_pos.y + double(old_hsz - square_size/2) );
}


int cgmath::dither( void )
{
	if( preview_mode )
	{
		log_i( "cgmath::dither(): Guiding is not started" );
		return GUIDING_NOT_STARTED;
	}
	if( m_ra_drift_v == 0 && m_dec_drift_v == 0 )
	{
		log_i( "cgmath::dither(): No speed info. Perform calibration" );
		return NO_SPEED_INFO;
	}

	double newx = reticle_pos.x, newy = reticle_pos.y;

	if( m_ra_drift_v )
		do
		{
			newx = reticle_org.x + (double)(rand()%m_common_params.dithering_range) - (double)m_common_params.dithering_range / 2.0;
		}while( newx == reticle_pos.x );
	if( m_dec_drift_v )
		do
		{
			newy = reticle_org.y + (double)(rand()%m_common_params.dithering_range) - (double)m_common_params.dithering_range / 2.0;
		}while( newy == reticle_pos.y );

	Vector delta( fabs(newx - reticle_pos.x), fabs(newy - reticle_pos.y), 0 );

	reticle_pos.x = newx;
	reticle_pos.y = newy;

	double min_v = (m_ra_drift_v && m_dec_drift_v) ? std::min( m_ra_drift_v, m_dec_drift_v ) :
			(m_ra_drift_v ? m_ra_drift_v : m_dec_drift_v);
	min_v = min_v > 0 ? min_v : 0.5;

	int tout = (int)(std::max( delta.x, delta.y ) / min_v) +
					m_common_params.dithering_rest_tout; // adding N secs to have a rest

	if( tout < DITHER_FIXED_TOUT )
		tout = DITHER_FIXED_TOUT;

	log_i( "cgmath::dither(): TOUT: %d", tout );

	if( tout > DITHER_FIXED_TOUT_CLIP )
	{
		tout = DITHER_FIXED_TOUT_CLIP;
		log_i( "cgmath::dither(): TOUT is long, clipped to %d. Check setting", tout );
	}

	return tout;
}


int cgmath::dither_no_wait_xy(double rx, double ry)
{
	if( preview_mode ) {
		log_i( "cgmath::dither_no_wait_xy(): Guiding is not started" );
		return GUIDING_NOT_STARTED;
	}

	double newx = reticle_pos.x, newy = reticle_pos.y;
	// we shall avoid division by zero and infinite loops
	if( (int)rx < 2 ) rx = 2;
	if( (int)ry < 2 ) ry = 2;

	do {
		newx = reticle_org.x + (double)(rand()%(int)rx) - (double)rx / 2.0;
	} while( newx == reticle_pos.x );

	do {
		newy = reticle_org.y + (double)(rand()%(int)ry) - (double)ry / 2.0;
	} while( newy == reticle_pos.y );

	reticle_pos.x = newx;
	reticle_pos.y = newy;

	return 0;
}


const char *cgmath::get_dither_errstring( int err_code ) const
{
	switch( err_code )
	{
	case GUIDING_NOT_STARTED:
		return "Guiding is not started";
	case NO_SPEED_INFO:
		return "No speed info. Perform calibration";
	default:
		return "Unknown error";
	}
	return "Unknown error";
}


int cgmath::get_distance(double *dx, double *dy) const
{
	if( preview_mode ) {
		log_i( "cgmath::get_distance(): Guiding is not started" );
		return GUIDING_NOT_STARTED;
	}

	Vector drift_px = arcsec2point(star_pos);
	*dx = fabs(drift_px.x);
	*dy = fabs(drift_px.y);

	return 0;
}

void cgmath::set_square_algorithm_index( int alg_idx )
{
	if( alg_idx < 0 || alg_idx >= (int)(sizeof(guide_square_alg)/sizeof(square_alg_t))-1 )
		return;

	square_alg_idx = alg_idx;

	in_params.threshold_alg_idx = square_alg_idx;
}


int  cgmath::get_q_control_index( void ) const
{
	return q_control_idx;
}


void cgmath::set_q_control_index( int idx )
{
	if( idx < 0 || idx >= (int)(sizeof(q_control_mtd)/sizeof(q_control_t))-1 )
		return;

	q_control_idx = idx;

	in_params.q_control_idx = q_control_idx;
}


ovr_params_t *cgmath::prepare_overlays( void )
{
	// square
	overlays.square_size  = square_size;
	overlays.square_pos.x = (int)square_pos.x;
	overlays.square_pos.y = (int)square_pos.y;

	// reticle
	overlays.reticle_pos.x = reticle_pos.x;
	overlays.reticle_pos.y = reticle_pos.y;

	return &overlays;
}


Vector cgmath::point2arcsec( const Vector &p ) const
{
	Vector arcs;

 	// arcs = 3600*180/pi * (pix*ccd_pix_sz) / focal_len
 	arcs.x = 206264.8062470963552 * p.x * ccd_pixel_width / focal;
 	arcs.y = 206264.8062470963552 * p.y * ccd_pixel_height / focal;

 	return arcs;
}


Vector cgmath::arcsec2point( const Vector &asec ) const
{
	Vector px;

	px.x = focal * asec.x / (206264.8062470963552 * ccd_pixel_width);
	px.y = focal * asec.y / (206264.8062470963552 * ccd_pixel_height);

	return px;
}


double cgmath::precalc_proportional_gain( double g_rate )
{
	if( g_rate <= 0.01 )
		return 0;

	return 1000.0 / (g_rate * 15.0);
}


bool cgmath::calc_and_set_reticle( double start_x,
								   double start_y,
								   double end_x,
								   double end_y,
								   unsigned drift_tm )
{
	 double phi = calc_phi( start_x, start_y, end_x, end_y );

	 if( phi < 0 )
		 return false;

	 set_reticle_params( start_x, start_y, phi );

	 // fill info
	 if( drift_tm )
	 {
		 m_ra_drift_v = !Vector( end_x - start_x, end_y - start_y, 0 ) / (double)drift_tm;
		 m_dec_drift_v = 0;
	 }

	 return true;
}


bool cgmath::calc_and_set_reticle2( double start_ra_x, double start_ra_y,
									double end_ra_x, double end_ra_y,
									double start_dec_x, double start_dec_y,
									double end_dec_x, double end_dec_y,
									bool *swap_dec,
									unsigned ra_drift_tm,
									unsigned dec_drift_tm )
{
	double len_threshold = 0.5; // if drift is shorter than 0.5 pix - so error
	double phi_ra = 0;	 // angle calculated by RA drift
	double phi_dec = 0; // angle calculated by DEC drift
	double phi = 0;

	Vector ra_v  = Vector(end_ra_x - start_ra_x, -(end_ra_y - start_ra_y), 0);
	Vector dec_v = Vector(end_dec_x - start_dec_x, -(end_dec_y - start_dec_y), 0);

	double ra_len  = !ra_v;
	double dec_len = !dec_v;

	// reject processing by threshold
	if( ra_len < len_threshold || dec_len < len_threshold )
		return false;

	Vector ra_vect  = Normalize( ra_v );
	Vector dec_vect = Normalize( dec_v );

	Vector try_increase = dec_vect * RotateZ( M_PI/2 );
	Vector try_decrease = dec_vect * RotateZ( -M_PI/2 );

	double cos_increase = try_increase & ra_vect;
	double cos_decrease = try_decrease & ra_vect;

	bool do_increase = cos_increase > cos_decrease;
	bool use_ra  = true;
	bool use_dec = true;

	// reject axis by drift length
	if( ra_len > dec_len )
	{
		if( dec_len < ra_len*0.58 ) // implicit taking account of error
		{
			use_dec = false;
			if( DBG_VERBOSITY )
				log_i( "calc_and_set_reticle2(): DEC axis rejected as its drift is too short" );
		}
	}
	else
	{
		if( ra_len < dec_len*0.58 ) // implicit taking account of error
		{
			use_ra = false;
			if( DBG_VERBOSITY )
				log_i( "calc_and_set_reticle2(): RA axis rejected as its drift is too short" );
		}
	}

	phi_ra = calc_phi( start_ra_x, start_ra_y, end_ra_x, end_ra_y, len_threshold );
	if( phi_ra < 0 )
		return false;

	phi_dec = calc_phi( start_dec_x, start_dec_y, end_dec_x, end_dec_y, len_threshold );
	if( phi_dec < 0 )
		return false;

	if( do_increase )
		phi_dec += 90;
	else
		phi_dec -= 90;

	if( phi_dec > 360 )phi_dec -= 360.0;
	if( phi_dec < 0 )phi_dec += 360.0;

	if( fabs(phi_dec - phi_ra) > 180 )
	{
		if( phi_ra > phi_dec )
			phi_ra -= 360;
		else
			phi_dec -= 360;
	}

	// now angles are unidirectional
	if( !use_ra )
	{
		phi = phi_dec;
	}
	else
		if( !use_dec )
		{
			phi = phi_ra;
		}
		else
		{
			// average angles
			phi = (phi_ra + phi_dec) / 2;
		}
	if( phi < 0 )phi += 360.0;

	if( DBG_VERBOSITY )
		log_i( "PHI_RA = %f\nPHI_DEC = %f\nPHI_AVG = %f", phi_ra, phi_dec, phi );

	set_reticle_params( start_ra_x, start_ra_y, phi );

	// check DEC
	if( swap_dec )
		*swap_dec = do_increase ? false : true;

	// fill info
	if( ra_drift_tm )
		m_ra_drift_v = ra_len / (double)ra_drift_tm; // pixels per second
	if( dec_drift_tm )
		m_dec_drift_v = dec_len / (double)dec_drift_tm; // pixels per second

	return true;
}


double cgmath::calc_phi( double start_x, double start_y, double end_x, double end_y, double len_threshold ) const
{
	double delta_x, delta_y;
	double phi;

	delta_x = end_x - start_x;
	delta_y = -(end_y - start_y);

	if( (!Vector(delta_x, delta_y, 0)) < len_threshold )
		return -1;

	// 90 or 270 degrees
	if( fabs(delta_x) < fabs(delta_y) / 1000000.0 )
	{
		phi = delta_y > 0 ? 90.0 : 270;
	}
	else
	{
		phi = 180.0/M_PI*atan2( delta_y, delta_x );
		if( phi < 0 )phi += 360.0;
	}

 	return phi;
}


void cgmath::do_ticks( void )
{
	ticks++;

	channel_ticks[RA]++;
	channel_ticks[DEC]++;
	if( channel_ticks[RA] >= MAX_ACCUM_CNT )
		channel_ticks[RA] = 0;
	if( channel_ticks[DEC] >= MAX_ACCUM_CNT )
		channel_ticks[DEC] = 0;	
	
	accum_ticks[RA]++;
	accum_ticks[DEC]++;
	if( accum_ticks[RA] >= in_params.accum_frame_cnt[RA] )
		accum_ticks[RA] = 0;
	if( accum_ticks[DEC] >= in_params.accum_frame_cnt[DEC] )
		accum_ticks[DEC] = 0;
}


//-------------------- Processing ---------------------------
void cgmath::start( void )
{
	ticks = 0;
	channel_ticks[RA] = channel_ticks[DEC] = 0;
	accum_ticks[RA] = accum_ticks[DEC] = 0;
	drift_integral[RA] = drift_integral[DEC] = 0;
	out_params.reset();

	memset( drift[RA], 0, sizeof(double)*MAX_ACCUM_CNT );
    memset( drift[DEC], 0, sizeof(double)*MAX_ACCUM_CNT );

	// cleanup stat vars.
	sum = sqr_sum = 0;
	delta_prev = sigma_prev = sigma = 0;

	memset( q_stat, 0, sizeof(double)*q_stat_len );

	preview_mode = false;

	// restore position
	reticle_pos = reticle_org;
}


void cgmath::stop( void )
{
	preview_mode = true;

	// restore position
	reticle_pos = reticle_org;
}


void cgmath::suspend( bool mode )
{
	suspended = mode;
}


bool cgmath::is_suspended( void ) const
{
	return suspended;
}


Vector cgmath::find_star_local_pos( void )
{
	int i, j;
	double resx, resy, mass, threshold, pval;
	double *psrc = NULL, *porigin = NULL;
	double *pptr;
	Vector ret;

 	psrc = porigin = pdata + (int)square_pos.y*video_width + (int)square_pos.x;

	resx = resy = 0;
	threshold = mass = 0;

	// several threshold adaptive smart agorithms
	switch( square_alg_idx )
	{
	// Alexander's Stepanenko smart threshold algorithm
	case SMART_THRESHOLD:
	{
		point_t bbox_lt = { (int)square_pos.x-SMART_FRAME_WIDTH, (int)square_pos.y-SMART_FRAME_WIDTH };
		point_t bbox_rb = { (int)square_pos.x+square_size+SMART_FRAME_WIDTH, (int)square_pos.y+square_size+SMART_FRAME_WIDTH };
		int offset = 0;

		// clip frame
		if( bbox_lt.x < 0 )
			bbox_lt.x = 0;
		if( bbox_lt.y < 0 )
			bbox_lt.y = 0;
		if( bbox_rb.x > video_width )
			bbox_rb.x = video_width;
		if( bbox_rb.y > video_height )
			bbox_rb.y = video_height;

		// calc top bar
		int box_wd = bbox_rb.x - bbox_lt.x;
		int box_ht = (int)square_pos.y - bbox_lt.y;
		int pix_cnt = 0;
		if( box_wd > 0 && box_ht > 0 )
		{
			pix_cnt += box_wd * box_ht;
			for( j = bbox_lt.y;j < (int)square_pos.y;j++ )
			{
				offset = j*video_width;
				for( i = bbox_lt.x;i < bbox_rb.x;i++ )
				{
					pptr = pdata + offset + i;
					threshold += *pptr;
				}
			}
		}
		// calc left bar
		box_wd = (int)square_pos.x - bbox_lt.x;
		box_ht = square_size;
		if( box_wd > 0 && box_ht > 0 )
		{
			pix_cnt += box_wd * box_ht;
			for( j = (int)square_pos.y;j < (int)square_pos.y+box_ht;j++ )
			{
				offset = j*video_width;
				for( i = bbox_lt.x;i < (int)square_pos.x;i++ )
				{
					pptr = pdata + offset + i;
					threshold += *pptr;
				}
			}
		}
		// calc right bar
		box_wd = bbox_rb.x - (int)square_pos.x - square_size;
		box_ht = square_size;
		if( box_wd > 0 && box_ht > 0 )
		{
			pix_cnt += box_wd * box_ht;
			for( j = (int)square_pos.y;j < (int)square_pos.y+box_ht;j++ )
			{
				offset = j*video_width;
				for( i = (int)square_pos.x+square_size;i < bbox_rb.x;i++ )
				{
					pptr = pdata + offset + i;
					threshold += *pptr;
				}
			}
		}
		// calc bottom bar
		box_wd = bbox_rb.x - bbox_lt.x;
		box_ht = bbox_rb.y - (int)square_pos.y - square_size;
		if( box_wd > 0 && box_ht > 0 )
		{
			pix_cnt += box_wd * box_ht;
			for( j = (int)square_pos.y+square_size;j < bbox_rb.y;j++ )
			{
				offset = j*video_width;
				for( i = bbox_lt.x;i < bbox_rb.x;i++ )
				{
					pptr = pdata + offset + i;
					threshold += *pptr;
				}
			}
		}
		// find maximum
		double max_val = 0;
		for( j = 0;j < square_size;j++ )
		{
			for( i = 0;i < square_size;i++ )
			{
				pptr = psrc+i;
				if( *pptr > max_val )
					max_val = *pptr;
			}
			psrc += video_width;
		}
		threshold /= (double)pix_cnt;

		q_bkgd = threshold;

		// cut by 10% higher then average threshold
		if( max_val > threshold )
			threshold += (max_val - threshold) * SMART_CUT_FACTOR;

		//log_i("smart thr. = %f cnt = %d", threshold, pix_cnt);
		break;
	}
	// simple adaptive threshold
	case AUTO_THRESHOLD:
	{
		for( j = 0;j < square_size;j++ )
		{
			for( i = 0;i < square_size;i++ )
			{
				pptr = psrc+i;
				threshold += *pptr;
			}
			psrc += video_width;
		}
		threshold /= square_square;

		q_bkgd = threshold;

		break;
	}
	// no threshold subtracion
	default:
	{
		q_bkgd = 1.0;
	}
	}

	psrc = porigin;
	for( j = 0;j < square_size;j++ )
	{
		for( i = 0;i < square_size;i++ )
		{
			pptr = psrc+i;
			pval = *pptr - threshold;
			pval = pval < 0 ? 0 : pval;

			resx += (double)i * pval;
			resy += (double)j * pval;

			mass += pval;
		}
		psrc += video_width;
	}

	if( mass == 0 )mass = 1;

	resx /= mass;
	resy /= mass;

	ret = square_pos + Vector( resx, resy, 0 );

	q_star_max = *(pdata + (int)ret.y*video_width + (int)ret.x);

 return ret;
}


void cgmath::process_axes( void  )
{
	int cnt = 0;
	double t_delta = 0;

 	// process axes...
 	for( int k = RA;k <= DEC;k++ )
 	{
 		// zero all out commands
 		out_params.pulse_dir[k] = io_drv::NO_DIR;

 		if( accum_ticks[k] < in_params.accum_frame_cnt[k]-1 )
 			continue;

 		t_delta = 0;
		drift_integral[k] = 0;

 		cnt = in_params.accum_frame_cnt[ k ];
	
 		for( int i = 0, idx = channel_ticks[k];i < cnt;i++ )
 		{
 			t_delta += drift[k][idx];
 		
			if( idx > 0 )
				--idx;
			else
				idx = MAX_ACCUM_CNT-1;
		}
		
		for( int i = 0;i < MAX_ACCUM_CNT;i++ )
 			drift_integral[k] += drift[k][i];
		
		out_params.delta[k] = t_delta / (double)cnt;
		drift_integral[k] /= (double)MAX_ACCUM_CNT;
 	
		//if( k == RA )
		//	log_i( "PROP = %f INT = %f", out_params.delta[k], drift_integral[k] );

		out_params.pulse_length[k] = fabs(out_params.delta[k]*in_params.proportional_gain[k] + drift_integral[k]*in_params.integral_gain[k]);
		out_params.pulse_length[k] = out_params.pulse_length[k] <= in_params.max_pulse_length[k] ? out_params.pulse_length[k] : in_params.max_pulse_length[k];

 		// calc direction
 		if( !in_params.enabled_dir[k] )
 		{
 			out_params.pulse_dir[k] = io_drv::NO_DIR;
 			continue;
 		}

 		if( out_params.pulse_length[k] >= in_params.min_pulse_length[k] )
 		{
 			io_drv::guide_dir dir = io_drv::NO_DIR;
 			if( k == RA )
 				dir = out_params.delta[k] > 0 ? io_drv::RA_DEC_DIR : io_drv::RA_INC_DIR;   // RA. right dir - decreases RA
 			else
 				dir = out_params.delta[k] > 0 ? io_drv::DEC_INC_DIR : io_drv::DEC_DEC_DIR; // DEC.

 			out_params.pulse_dir[k] = dir_checker[ dir ];
 			//log_i("CK_DIR: %d", out_params.pulse_dir[k]);
 		}
 		else
 			out_params.pulse_dir[k] = io_drv::NO_DIR;
 	}
}


void cgmath::do_processing( void )
{
 	// do nothing if suspended
 	if( suspended )
 		return;

	// find guiding star location in
 	scr_star_pos = star_pos = find_star_local_pos();

	// move square overlay
 	move_square( round(star_pos.x) - (double)square_size/2, round(star_pos.y) - (double)square_size/2 );

 	if( m_common_params.hfd_on )
 		hfd_calc();

	if( preview_mode )
		return;

	// translate star coords into sky coord. system

	// convert from pixels into arcsecs
	Vector arc_star_pos    = point2arcsec( star_pos );
	Vector arc_reticle_pos = point2arcsec( reticle_pos );

	// translate into sky coords.
	star_pos = arc_star_pos - arc_reticle_pos;
	star_pos.y = -star_pos.y; // invert y-axis as y picture axis is inverted

	star_pos = star_pos * ROT_Z;

	// both coords are ready for math processing
	//put coord to drift list
	drift[RA][channel_ticks[RA]]   = star_pos.x;
	drift[DEC][channel_ticks[DEC]] = star_pos.y;

	// make decision by axes
	process_axes();

	// process statistics
	calc_square_err();

	// find quality of picture
	calc_quality();

	// finally process tickers
	do_ticks();
}


void cgmath::calc_square_err( void )
{
	if( !do_statistics )
		return;

	// through MAX_ACCUM_CNT values
	if( ticks == 0 )
		return;

	for( int k = RA;k <= DEC;k++ )
	{
		double sqr_avg = 0;
		for( int i = 0;i < MAX_ACCUM_CNT;i++ )
			sqr_avg += drift[k][i] * drift[k][i];

		out_params.sigma[k] = sqrt( sqr_avg / (double)MAX_ACCUM_CNT );
	}
}


void cgmath::calc_quality( void )
{
	uint32_t q_tick = ticks % q_stat_len;

	if( q_star_max <= 0 )
		q_star_max = 1;

	if( q_bkgd > q_star_max )
		q_star_max = q_bkgd+1;

	double cur_quality = 1 - q_bkgd / q_star_max;

	q_stat[ q_tick ] = cur_quality;

	int cnt = 0;
	double q_avg = 0;
	for( int i = 0;i < q_stat_len;i++ )
	{
		if( q_stat[i] > 0 )
		{
			q_avg += q_stat[i];
			cnt++;
		}
	}
	// not enough data
	if( cnt < q_stat_len )
	{
		out_params.quality = -1;
		return;
	}
	q_avg /= (double)cnt;

	out_params.quality = q_avg * 100.0;
}


void cgmath::hfd_init( void ) const
{
	if( m_hfd_sqr_info )
		return;

	int sqr_cnt = (int)(sizeof(guide_squares)/sizeof(guide_square_t));
	m_hfd_sqr_info = new struct hfd_sqr_s [ sqr_cnt ];
	for( int i = 0;i < sqr_cnt;i++ )
	{
		if( guide_squares[ i ].size != -1 )
			m_hfd_sqr_info[ i ].data = new struct hfd_item_s [ guide_squares[ i ].size * guide_squares[ i ].size ];
	}

	for( int k = 0;k < sqr_cnt;k++ )
	{
		struct hfd_sqr_s *hfd_sqr = &m_hfd_sqr_info[ k ];
		if( !hfd_sqr )
			continue;

		int sqr_size = guide_squares[ k ].size/*-1*/;
		int sqr_cx = sqr_size / 2;
		int sqr_cy = sqr_cx;
		double r = sqr_size / 2;

		for( int j = 0;j < sqr_size;j++ )
		{
			for( int i = 0;i < sqr_size;i++ )
			{
				int idx = j * sqr_size + i;
				double dist = sqrt( (i-sqr_cx)*(i-sqr_cx) + (j-sqr_cy)*(j-sqr_cy) );
				bool in =  dist <= r;
				if( in )
				{
					hfd_sqr->dist_sum += dist;
					hfd_sqr->area_cnt++;
				}
				else
					hfd_sqr->bkgd_cnt++;
				hfd_sqr->data[ idx ].in_circle = in;
				hfd_sqr->data[ idx ].distance  = dist;
			}
		}
	}
}


void cgmath::hfd_destroy( void ) const
{
	if( !m_hfd_sqr_info )
		return;

	for( int i = 0;m_hfd_sqr_info[ i ].data;i++ )
		delete [] m_hfd_sqr_info[ i ].data;
	delete [] m_hfd_sqr_info;
}


void cgmath::hfd_calc( void )
{
	if( !m_hfd_sqr_info )
		return;
	struct hfd_sqr_s *hfd_sqr = &m_hfd_sqr_info[ square_idx ];
	struct hfd_item_s *hfd_sqr_data = hfd_sqr->data;
	assert( hfd_sqr_data );

	double *psrc   = NULL;
	double *pptr   = NULL;
	double v_sum   = 0;
	double vd_sum  = 0;
	double bkgd    = 0;
	double lum_max = 0;

	psrc = pdata + (int)square_pos.y*video_width + (int)square_pos.x;
	for( int j = 0, idx = 0;j < square_size;j++ )
	{
		for( int i = 0;i < square_size;i++, idx++ )
		{
			pptr = psrc+i;
			double val = *pptr;
			if( hfd_sqr_data[ idx ].in_circle )
			{
				v_sum  += val;
				vd_sum += val * hfd_sqr_data[ idx ].distance;
				if( val > lum_max )
					lum_max = val;
			}
			else
				bkgd += val;
		}
		psrc += video_width;
	}
	bkgd /= (double)hfd_sqr->bkgd_cnt;

	double numerator   = vd_sum - bkgd * hfd_sqr->dist_sum;
	double denominator = v_sum - hfd_sqr->area_cnt * bkgd;
	if( denominator == 0 )
		denominator = 1;
	double H = 2 * fabs( numerator / denominator );

	if( H <= (double)square_size )
	{
		Vector arc_h = point2arcsec( Vector(H, 0, 0) );
		out_params.hfd_h = arc_h.x;
	}
	else
		out_params.hfd_h = -1;
	out_params.hfd_lum_max = lum_max;
}


//=========================== utility methods ==================================
int cgmath::calc_quality_rate( void ) const
{
	double notify_threshold   = in_params.quality_threshold1;
	double critical_threshold = in_params.quality_threshold2;

	if( out_params.quality == -1 )
		return -1;

	if( notify_threshold < critical_threshold )
		std::swap( notify_threshold, critical_threshold );

	if( out_params.quality > notify_threshold )
		return QUALITY_OK;
	else
	if( out_params.quality > critical_threshold )
		return QUALITY_NOTIFY;

	return QUALITY_CRITICAL;
}


bool cgmath::find_stars( std::vector< std::pair<Vector, double> > *stars ) const
{
#define USE_MEDIAN
	int  buf_len = video_width*video_height;
	double *buf = new double[ buf_len ];
	int star_size = 10;
	int clip_edge = video_height >= FIND_STAR_CLIP_EDGE*4 ? FIND_STAR_CLIP_EDGE : (video_height / 4);
	int clip_width = video_width - clip_edge;
	int clip_height = video_height - clip_edge;
	double  lmax = 1;

#ifdef USE_MEDIAN
	filters::medianfilter( pdata, buf, video_width, video_height );
#else
	memcpy( buf, pdata, buf_len*sizeof(double) );
#endif

	double threshold = 0;
	for( int i = 0;i < buf_len;i++ )
		threshold += buf[i];
	threshold /= (double)buf_len;
	threshold *= 1.25;

	stars->clear();

	while( lmax > 0 )
	{
		lmax = 0;
		Vector p( 0 );

		for( int j = clip_edge;j < clip_height;j++ )
		{
			for( int i = clip_edge;i < clip_width;i++ )
			{
				int off = j*video_width+i;
				if( buf[off] > threshold && lmax < buf[off] )
				{
					lmax = buf[off];
					p.x = (double)i;
					p.y = (double)j;
					p.z = 0;
					p.z = !(p - Vector( video_width/2, video_height/2, 0 ));
					if( p.z / std::min(video_width, video_height)/2 > 0.5 )
						p.z = p.z / std::min(video_width, video_height)/2;
					else
						p.z = 0.5;
				}
			}
		}
		if( lmax > 0 )
		{
			stars->push_back( std::make_pair(p, 0) );

			std::pair< Vector, double > &p = stars->back();
			for( int j = -star_size;j < star_size;j++ )
			{
				for( int i = -star_size;i < star_size;i++ )
				{
					if( (int)p.first.x+i < 0 || (int)p.first.x+i >= video_width ||
						(int)p.first.y+j < 0 || (int)p.first.y+i >= video_height )
						continue;
					int off = ((int)p.first.y+j)*video_width + (int)p.first.x+i;
					if( buf[ off ] > p.second )
						p.second = buf[ off ];
					buf[ off ] = 0;
				}
			}
			p.second = log(fabs(p.second));
		}
		if( stars->size() >= 3 )
			break;
	}

	delete [] buf;

	if( DBG_VERBOSITY )
	{
		for( size_t i = 0;i < stars->size();i++ )
			log_i("star #%u: x = %lf, y = %lf, z = %lf, lum = %lf",
					i+1, (*stars)[i].first.x, (*stars)[i].first.y, (*stars)[i].first.z, (*stars)[i].second );
	}

	return stars->size() ? true : false;
}


bool cgmath::check_drift_dec( void ) const
{
	double diff       = 0;
	double drift_data[ MAX_ACCUM_CNT ];
	double diff_data[ MAX_ACCUM_CNT ];
	int    data_cnt = ((MAX_ACCUM_CNT-1) / 2) * 2;

	if( data_cnt <= 0 )
	{
		log_e( "cgmath::check_drift_dec(): critical error. data_cnt <= 0" );
		return false;
	}

	// get aligned data
	for( int i = 0, idx = channel_ticks[DEC];i < MAX_ACCUM_CNT;i++ )
	{
		drift_data[i] = fabs( drift[DEC][idx] );

		if( idx < MAX_ACCUM_CNT )
			++idx;
		else
			idx = 0;
	}

	filters::medianfilter( diff_data, (double*)NULL, MAX_ACCUM_CNT );

	for( int i = 0;i < data_cnt;i++ )
	{
		diff_data[i] = drift_data[i+1] - drift_data[i];
		diff += diff_data[i];
	}
 	diff /= (double)data_cnt;

 	log_i( "diff avg = %lf", diff );

 	if( diff > 0.05 )
 	{

 		return false;
 	}

 	return true;
}


int cgmath::calc_stability_rate( void ) const
{
 	 if( ticks < MAX_ACCUM_CNT )
 		 return -1;

 	double limits[2] = {0};
 	for( int k = RA;k <= DEC;k++ )
 	{
 		limits[k] = in_params.stability_limit_factor * fabs(in_params.min_pulse_length[k]) / (in_params.proportional_gain[k] > 0 ? in_params.proportional_gain[k] : 1);
 	}

 	if( out_params.sigma[RA] >= limits[RA] || out_params.sigma[DEC] >= limits[DEC] )
 	{
 		if( DBG_VERBOSITY )
 			log_i( "Guiding unstable: lim[RA] = %lf, lim[DEC] = %lf SLF = %lf", limits[RA], limits[DEC], in_params.stability_limit_factor );
 		return STABILITY_BAD;
 	}

 	 return STABILITY_GOOD;
}


bool cgmath::is_valid_pos( double x, double y, double edge_width ) const
{
	if( x <= edge_width ||
		x >= video_width - edge_width ||
		y < edge_width ||
		y >= video_height - edge_width )
		return false;

	return true;
}


void cgmath::clear_speed_info( void )
{
	m_ra_drift_v = m_dec_drift_v = 0;
}


void cgmath::get_speed_info( double *ra_v, double *dec_v ) const
{
	*ra_v  = m_ra_drift_v;
	*dec_v = m_dec_drift_v;
}




//---------------------------------------------------------------------------------------
cproc_in_params::cproc_in_params()
{
	reset();
}


void cproc_in_params::reset( void )
{
	threshold_alg_idx = SMART_THRESHOLD;
	guiding_rate = 0.5;
	normalize_gain = false;
	guiding_normal_coef = cgmath::precalc_proportional_gain( guiding_rate );
	average = true;

	for( int k = RA;k <= DEC;k++ )
	{
		enabled_dir[k] 				 = true;
		enabled_dir_sign[k][SGN_POS] = true;
		enabled_dir_sign[k][SGN_NEG] = true;
		accum_frame_cnt[k] 	         = 1;
		proportional_gain[k]         = guiding_normal_coef;
		integral_gain[k] 	         = 0;
		derivative_gain[k] 	         = 0;
		max_pulse_length[k]          = 5000;
		min_pulse_length[k]          = 100;
	}

	q_control_idx = Q_CTRL_OFF;
	quality_threshold1 = 50; // notification threshold in %
	quality_threshold2 = 15; // critical threshold in %
	stability_limit_factor = STABILITY_LIMIT_FACTOR;
}


cproc_out_params::cproc_out_params()
{
	reset();
}


void cproc_out_params::reset( void )
{
	for( int k = RA;k <= DEC;k++ )
	{
		delta[k] 		= 0;
		pulse_dir[k] 	= io_drv::NO_DIR;
		pulse_length[k] = 0;
		sigma[k] 		= 0;
	}
	quality = 0;
	hfd_h = 0;
	hfd_lum_max = 0;
}
