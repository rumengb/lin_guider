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
#include <vector>
#include <utility>

#include "maindef.h"
#include "io_driver.h"
#include "vect.h"
#include "matr.h"


class common_params;


#define FIND_STAR_CLIP_EDGE 48
#define STAR_CLIP_EDGE 8

// smart threshold algorithm param
// width of outer frame for backgroung calculation
#define SMART_FRAME_WIDTH	4
// cut-factor above avarage threshold
#define SMART_CUT_FACTOR	0.1

// core constants
#define RA	0
#define DEC	1
#define SGN_POS  0
#define SGN_NEG  1
#define CHANNEL_CNT	2
#define DEFAULT_SQR	1

#define  MAX_ACCUM_CNT	50

#define DITHER_FIXED_TOUT 2
#define DITHER_FIXED_TOUT_CLIP 20

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

#define STABILITY_LIMIT_FACTOR  2.0

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
	enum type_t // values must be power of 2
	{
		OVR_SQUARE = 1,
		OVR_RETICLE = 2,
		OVR_RETICLE_ORG = 4
	};
	int visible;
	int square_size;
	point_t square_pos;
	point_t reticle_axis_ra;
	point_t reticle_axis_dec;
	point_t reticle_pos;
	point_t reticle_org;
}ovr_params_t;


// input params
class cproc_in_params
{
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
public:
	cgmath( const common_params &comm_params );
	virtual ~cgmath();
	
	// functions
	bool set_video_params( int vid_wd, int vid_ht );
	double *get_data_buffer( int *width, int *height, int *length, int *size );
	bool set_guider_params( double ccd_pix_wd, double ccd_pix_ht, double guider_aperture, double guider_focal );
	bool set_reticle_params( double x, double y, double ang );
	bool get_reticle_params( double *x, double *y, double *ang ) const;
	int  fix_square_index( int square_index ) const;
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
	int get_distance(double *dx, double *dy) const;
	bool reset( void );
	
	ovr_params_t *prepare_overlays( void );
	void move_square( double newx, double newy );
	void resize_square( int size_idx );
	int  dither( void );
	int  dither_no_wait_xy( double rx, double ry );
	const char *get_dither_errstring( int err_code ) const;
	
	// proc
	void start( void );
	void stop( void );
	void suspend( bool mode );
	bool is_suspended( void ) const;
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
	double calc_phi( double start_x, double start_y, double end_x, double end_y, double len_threshold = 5.0 ) const;

	// utility
	int  calc_quality_rate( void ) const;
	bool find_stars( std::vector< std::pair<Vector, double> > *stars ) const;
	bool check_drift_dec( void ) const;
	int  calc_stability_rate( void ) const;
	bool is_valid_pos( double x, double y, double edge_width = STAR_CLIP_EDGE ) const;
	void clear_speed_info( void );
	void get_speed_info( double *ra_v, double *dec_v ) const;

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

	const common_params &m_common_params;

	// sys...
	uint32_t ticks;		// global channel ticker
	double *pdata;		// pointer to data buffer
	int video_width, video_height;	// video frame dimensions
	double ccd_pixel_width, ccd_pixel_height, aperture, focal;
	Matrix	ROT_Z;
	bool preview_mode, suspended;
	
	// square variables
	int square_size;	// size of analysing square
	double square_square; // square of guiding rect
	Vector square_pos;	// integer values in double vars.
	int square_idx;		// index in size list
	int square_alg_idx;		// index of threshold algorithm
	
	// sky coord. system vars.
	Vector star_pos;	// position of star in reticle coord. system
	Vector scr_star_pos; // screen star position
	Vector reticle_pos;
	Vector reticle_org; // origin position
	Vector reticle_orts[2];
	double reticle_angle;
	
	// processing
	uint32_t  channel_ticks[2];
	uint32_t  accum_ticks[2];
	double *drift[2];
	double drift_integral[2];
	io_drv::guide_dir dir_checker[5];
	
	// overlays...
	ovr_params_t overlays;
	cproc_in_params  in_params;
	cproc_out_params out_params;
	
	// stat math...
	bool do_statistics;
	double sum, sqr_sum;
	double delta_prev, sigma_prev, sigma;

	// quality estimation
	enum quality
	{
		q_stat_len = 5
	};
	double q_star_max;
	double q_bkgd;
	double q_stat[q_stat_len]; //
	int    q_control_idx;

	// info
	double m_ra_drift_v; // pixels per second (fills by calibration procedure)
	double m_dec_drift_v;// pixels per second

	// hfd
	mutable struct hfd_sqr_s *m_hfd_sqr_info;

	// proc
	void do_ticks( void );
	Vector point2arcsec( const Vector &p ) const;
	Vector arcsec2point( const Vector &asec ) const;
	Vector find_star_local_pos( void );
	void process_axes( void );
	void calc_square_err( void );
	void calc_quality( void );
	
	void hfd_init( void ) const;
	void hfd_destroy( void ) const;
	void hfd_calc( void );

	cgmath( const cgmath& );
	cgmath& operator=( const cgmath& );
};

#endif /*GMATH_H_*/
