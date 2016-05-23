/*
 * cgmath_test.h
 *
 *      Author: gm
 */

#ifndef GMATH_TEST_H_
#define GMATH_TEST_H_

#include "gmath.h"
#include <donuts_guide.h>


class common_params;

class cgmath_test : public cgmath
{
public:
	cgmath_test( const common_params &comm_params );
	virtual ~cgmath_test();

	virtual bool set_video_params( int vid_wd, int vid_ht );
	virtual ovr_params_t *prepare_overlays( void );
	virtual int get_default_overlay_set( void ) const;
	virtual void move_osf( double newx, double newy );
	virtual void resize_osf( double kx, double ky );
	virtual void get_osf_params( double *x, double *y, double *kx, double *ky ) const;

protected:
	virtual Vector find_star_local_pos( void ) const;
	virtual void on_start( void );
	virtual void on_stop( void );

private:
	Vector m_osf_pos;
	Vector m_osf_size;
	point_t m_osf_vis_size;
	bool m_guiding;
	frame_digest m_dg_ref;
};

#endif /* GMATH_TEST_H_ */
