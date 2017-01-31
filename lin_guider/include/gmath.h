/*
 * gmath.h
 *
 *      Author: gm
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

#ifndef GMATH_H_
#define GMATH_H_

#include <stdint.h>
#include <sys/types.h>

#include <string>
#include <map>
#include <vector>
#include <utility>

#include "maindef.h"
#include "io_driver.h"
#include "vect.h"
#include "matr.h"


class common_params;

namespace lg_math
{

enum guider_algorithm
{
	GA_MIN = 0,
	GA_CENTROID,
	GA_DONUTS,
	GA_MAX,
	ALG_CNT = GA_MAX-1
};

enum capabilities
{
	CAP_HFD = 0x1,
	CAP_QUALITY = 0x2
};

typedef struct
{
	const int type;
	const bool use_osf_ui;
	const char *desc;
	const char *info;	    // any necessary text info
	const char *hyper_info;	// any necessary hypertext info
}algorithm_desc_t;

extern algorithm_desc_t alg_desc_list[ALG_CNT];

// core constants
enum axes
{
	RA = 0,
	DEC = 1
};
enum signs
{
	SGN_POS = 0,
	SGN_NEG = 1
};
static const double STABILITY_LIMIT_FACTOR = 2.0;
static const double FIND_STAR_CLIP_EDGE = 48;
static const double STAR_CLIP_EDGE = 8;
static const int MAX_ACCUM_CNT = 50;

enum dither_err_codes
{
	GUIDING_NOT_STARTED = -1,
	NO_SPEED_INFO = -2
};

enum thresold_alg
{
	SMART_THRESHOLD = 0,
	AUTO_THRESHOLD,
	NO_THRESHOLD
};

enum quality_ctrl
{
	Q_CTRL_OFF = 0,
	Q_CTRL_NOTIFY,
	Q_CTRL_FULL
};

enum quality_types	// used as zero-based index
{
	QUALITY_OK = 0,
	QUALITY_NOTIFY,
	QUALITY_CRITICAL
};

enum stability_types
{
	STABILITY_GOOD = 0,
	STABILITY_BAD
};

typedef struct
{
	int size;
	double square;
}guide_square_t;

typedef struct
{
	int idx;
	const char *name;
}square_alg_t;

typedef struct
{
	int idx;
	const char *name;
}q_control_t;

typedef struct
{
	enum consts
	{
		OVR_DRAGGABLE_CNT = 3,
		OVR_ALTERSQUARE_FLAG = 0x80000000
	};
	enum type_t // values must be power of 2
	{
		OVR_SQUARE = 1,
		OVR_RETICLE = 2,
		OVR_RETICLE_ORG = 4,
		OVR_OSF = 8,	// optional subframe
		OVR_ALTERSQUARE = (OVR_SQUARE | OVR_ALTERSQUARE_FLAG)
	};
	int visible;
	int locked;
	int square_size;
	point_t square_pos;
	point_t reticle_axis_ra;
	point_t reticle_axis_dec;
	point_t reticle_pos;
	point_t reticle_org;
	point_t osf_pos;
	point_t osf_size;
}ovr_params_t;


// input params
class cproc_in_params
{
	enum consts
	{
		CHANNEL_CNT = 2
	};

public:
	cproc_in_params();
	void reset( void );

	int       threshold_alg_idx;
	double    guiding_rate;
	double    guiding_normal_coef;
	bool      normalize_gain;
	bool      enabled_dir[CHANNEL_CNT];
	bool      enabled_dir_sign[CHANNEL_CNT][CHANNEL_CNT];
	bool      average;
	uint32_t  accum_frame_cnt[CHANNEL_CNT];
	double    proportional_gain[CHANNEL_CNT];
	double    integral_gain[CHANNEL_CNT];
	double    derivative_gain[CHANNEL_CNT];
	int       max_pulse_length[CHANNEL_CNT];
	int       min_pulse_length[CHANNEL_CNT];
	int       q_control_idx;
	double    quality_threshold1; // notification threshold in %
	double    quality_threshold2; // critical threshold in %
	double    stability_limit_factor; // 1 - 3
};


//output params
class cproc_out_params
{
public:
	cproc_out_params();
	void reset( void );

	double  			delta[2];
	io_drv::guide_dir 	pulse_dir[2];
	int	    			pulse_length[2];
	double				sigma[2];
	double      		quality;
	double				hfd_h;
	double				hfd_lum_max;
};


typedef struct
{
	double focal_ratio;
	double fov_wd, fov_ht;
	double focal, aperture;
}info_params_t;

extern const guide_square_t guide_squares[];
extern const square_alg_t guide_square_alg[];
extern const q_control_t q_control_mtd[];


class cgmath
{
	// smart threshold algorithm param
	// width of outer frame for backgroung calculation
	static const int SMART_FRAME_WIDTH = 4;
	// cut-factor above avarage threshold
	static const double SMART_CUT_FACTOR;

	static const int DITHER_FIXED_TOUT = 2;
	static const int DITHER_FIXED_TOUT_CLIP = 20;

public:
	enum status_level
	{
		STATUS_LEVEL_INFO = 0, // may be of standard color
		STATUS_LEVEL_WARNING,  // may be orange
		STATUS_LEVEL_ERROR     // may by red
	};
	static const int DEFAULT_SQR = 1;

	cgmath( const common_params &comm_params );
	virtual ~cgmath();
	
	// functions
	virtual bool set_video_params( int vid_wd, int vid_ht );
	double *get_data_buffer( int *width, int *height, int *length, int *size ) const;
	bool set_guider_params( double ccd_pix_wd, double ccd_pix_ht, double guider_aperture, double guider_focal );
	bool set_reticle_params( double x, double y, double ang );
	bool get_reticle_params( double *x, double *y, double *ang ) const;
	int  get_square_index( void ) const;
	int  get_square_algorithm_index( void ) const;
	void set_square_algorithm_index( int alg_idx );
	int  get_q_control_index( void ) const;
	void set_q_control_index( int idx );
	cproc_in_params *get_in_params( void );
	void set_in_params( const cproc_in_params *v );
	void calc_dir_checker( void );
	const cproc_out_params *get_out_params( void ) const;
	info_params_t get_info_params( void ) const;
	uint32_t get_ticks( void ) const;
	void get_star_drift( double *dx, double *dy ) const;
	void get_star_screen_pos( double *dx, double *dy ) const;
	int  get_distance(double *dx, double *dy) const;
	virtual bool reset( void );
	
	virtual const ovr_params_t *prepare_overlays( void );
	virtual int get_default_overlay_set( void ) const;
	void set_visible_overlays( int ovr_mask, bool set );
	void move_square( double newx, double newy );
	void resize_square( int size_idx );
	virtual void move_osf( double newx, double newy )
	{
		std::map< std::string, double >::iterator it = m_misc_vars.find( "osf_x" );
		if( it != m_misc_vars.end() )
			m_misc_vars.erase( it );
		m_misc_vars.insert( std::make_pair("osf_x", newx) );

		it = m_misc_vars.find( "osf_y" );
		if( it != m_misc_vars.end() )
			m_misc_vars.erase( it );
		m_misc_vars.insert( std::make_pair("osf_y", newy) );
	}
	virtual void resize_osf( double kx, double ky )
	{
		std::map< std::string, double >::iterator it = m_misc_vars.find( "osf_kx" );
		if( it != m_misc_vars.end() )
			m_misc_vars.erase( it );
		m_misc_vars.insert( std::make_pair("osf_kx", kx) );

		it = m_misc_vars.find( "osf_ky" );
		if( it != m_misc_vars.end() )
			m_misc_vars.erase( it );
		m_misc_vars.insert( std::make_pair("osf_ky", ky) );
	}
	virtual void get_osf_params( double *x, double *y, double *kx, double *ky ) const
	{
		if( x )
		{
			std::map< std::string, double >::const_iterator it = m_misc_vars.find( "osf_x" );
			if( it != m_misc_vars.end() )
				*x = it->second;
		}
		if( y )
		{
			std::map< std::string, double >::const_iterator it = m_misc_vars.find( "osf_y" );
			if( it != m_misc_vars.end() )
				*y = it->second;
		}
		if( kx )
		{
			std::map< std::string, double >::const_iterator it = m_misc_vars.find( "osf_kx" );
			if( it != m_misc_vars.end() )
				*kx = it->second;
		}
		if( ky )
		{
			std::map< std::string, double >::const_iterator it = m_misc_vars.find( "osf_ky" );
			if( it != m_misc_vars.end() )
				*ky = it->second;
		}
	}
	int  dither( void );
	int  dither_no_wait_xy( double rx, double ry );
	const char *get_dither_errstring( int err_code ) const;
	
	// proc
	void start( void );
	void stop( void );
	void suspend( bool mode );
	bool is_suspended( void ) const;
	bool is_guiding( void ) const;
	void do_processing( void );
	static double precalc_proportional_gain( double g_rate );
	bool calc_and_set_reticle( double start_x,
								double start_y,
								double end_x,
								double end_y,
								unsigned drift_tm = 0 ); // optional param
	bool calc_and_set_reticle2( double start_ra_x, double start_ra_y,
								double end_ra_x, double end_ra_y,
								double start_dec_x, double start_dec_y,
								double end_dec_x, double end_dec_y,
								bool *swap_dec,
								unsigned ra_drift_tm = 0, unsigned dec_drift_tm = 0 );// optional params
	static double calc_phi( double start_x, double start_y, double end_x, double end_y, double len_threshold = 5.0 );

	// utility
	int  calc_quality_rate( void ) const;
	bool find_stars( std::vector< std::pair<Vector, double> > *stars ) const;
	bool check_drift_dec( void ) const;
	int  calc_stability_rate( void ) const;
	bool is_valid_pos( double x, double y, double edge_width = STAR_CLIP_EDGE ) const;
	void clear_speed_info( void );
	void get_speed_info( double *ra_v, double *dec_v ) const;
	int  get_type( void ) const;
	const char *get_name( void ) const;
	const std::pair< enum cgmath::status_level, std::string >* get_status_info_for_key( unsigned int *key ) const;

protected:
	const common_params &m_common_params;
	int m_type;
	int m_caps;

	/*! This method should return position of star as vector(x, y, 0) relative to the left top corner of buffer.
        Note! Reticle position is a center of guiding
	*/
	virtual Vector find_star_local_pos( void ) const;
	virtual void on_start( void ) {}
	virtual void on_stop( void ) {}
	void add_quality( double q_val ) const;
	void set_status_info( enum status_level level, const std::string &txt ) const;

private:
	struct hfd_item_s
	{
		hfd_item_s() :
			in_circle( false ),
			distance( 0 )
		{}
		bool   in_circle;
		double distance;
	};
	struct hfd_sqr_s
	{
		hfd_sqr_s() :
			data( NULL ),
			area_cnt( 0 ),
			bkgd_cnt( 0 ),
			dist_sum( 0 )
		{}
		struct hfd_item_s *data;
		double area_cnt;
		double bkgd_cnt;
		double dist_sum;
	};

	// sys...
	uint32_t m_ticks;		// global channel ticker
	mutable double *m_pdata;		// pointer to data buffer
	int m_video_width, m_video_height;	// video frame dimensions
	double m_ccd_pixel_width, m_ccd_pixel_height, m_aperture, m_focal;
	Matrix	m_ROT_Z;
	bool m_preview_mode, m_suspended;
	
	// square variables
	int    m_square_size;	// size of analysing square
	double m_square_square; // square of guiding rect
	Vector m_square_pos;	// integer values in double vars.
	int    m_square_idx;		// index in size list
	int    m_square_alg_idx;		// index of threshold algorithm
	
	// sky coord. system vars.
	Vector m_star_pos;	// position of star in reticle coord. system
	Vector m_scr_star_pos; // screen star position
	Vector m_reticle_pos;
	Vector m_reticle_org; // origin position
	Vector m_reticle_orts[2];
	double m_reticle_angle;
	
	// processing
	uint32_t  m_channel_ticks[2];
	uint32_t  m_accum_ticks[2];
	double   *m_drift[2];
	double    m_drift_integral[2];
	io_drv::guide_dir m_dir_checker[5];
	
	// overlays...
	ovr_params_t m_overlays;
	cproc_in_params  m_in_params;
	cproc_out_params m_out_params;
	
	// stat math...
	bool   m_do_statistics;
	double m_sum, m_sqr_sum;
	double m_delta_prev, m_sigma_prev, m_sigma;

	// quality estimation
	enum quality
	{
		q_stat_len = 5
	};
	mutable double m_q_value;
	double         m_q_stat[q_stat_len]; //
	int            m_q_control_idx;

	// info
	double m_ra_drift_v; // pixels per second (fills by calibration procedure)
	double m_dec_drift_v;// pixels per second

	// hfd
	mutable struct hfd_sqr_s *m_hfd_sqr_info;

	// misc
	std::map< std::string, double > m_misc_vars;
	mutable std::pair< enum status_level, std::string > m_status_info;
	mutable unsigned int m_status_hash;

	int fix_square_index( int square_index ) const;

	// proc
	void do_ticks( void );
	Vector point2arcsec( const Vector &p ) const;
	Vector arcsec2point( const Vector &asec ) const;
	void process_axes( void );
	void calc_square_err( void );
	void calc_quality( void );
	
	void hfd_init( void ) const;
	void hfd_destroy( void ) const;
	void hfd_calc( void );

	cgmath( const cgmath& );
	cgmath& operator=( const cgmath& );
};

}

#endif /*GMATH_H_*/
