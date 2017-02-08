/*
 * common.h
 *
 *  Created on: 21.11.2010
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

#ifndef COMMON_H_
#define COMMON_H_

#include <QtGui>
#include <assert.h>

#include "maindef.h"
#include "gmath.h"

#define ZOOM_MAX (2.0)
#define ZOOM_MIN (0.1)
#define ZOOM_STEP (0.1)


class complex_delegate
{
public:
	virtual ~complex_delegate() {}
	virtual void mouse_press( QMouseEvent *event ) = 0;
	virtual void mouse_release( QMouseEvent *event ) = 0;
	virtual void mouse_move( QMouseEvent *event ) = 0;
	virtual void mouse_doubleclick( QMouseEvent *event ) = 0;
	virtual void draw_overlays( QPainter &painter) = 0;
};


class custom_drawer : public QWidget
{
    Q_OBJECT

public:
    explicit custom_drawer(QWidget *parent = NULL ) :
		QWidget( parent ),
		m_cd( NULL ),
		m_image( NULL ),
		m_sz( QSize() ),
		m_scale_k( 1.0 ),
		m_scale_inv_k( 1.0 )
	{
	}
	~custom_drawer()
	{
	}
	bool set_source( QImage *image, complex_delegate *cd )
	{
		m_image = image;
		m_cd = cd;
		if( !m_image )
			return false;

		return set_scale( m_scale_k );
	}
	bool set_scale( float k )
	{
		if( k > ZOOM_MAX )
			m_scale_k = ZOOM_MAX;
		else
		if( k <= ZOOM_MIN )
			m_scale_k = ZOOM_MIN;
		else
			m_scale_k = k;
		m_scale_inv_k = 1.0 / m_scale_k;

		if( !m_image )
		    return false;

		int w = m_image->width();
		int h = m_image->height();
		xy2scr( &w, &h );
		m_sz = QSize( w, h );
		resize( m_sz );

		return true;
	}
	inline void xy2scr( int *x, int *y ) const
	{
		if( m_scale_k == 1.0 ) return;
		*x *= m_scale_k;
		*y *= m_scale_k;
	}
	inline void scr2xy( int *x, int *y ) const
	{
		if( m_scale_k == 1.0 ) return;
		*x *= m_scale_inv_k;
		*y *= m_scale_inv_k;
	}
	inline void x2scr( int *x ) const
	{
		if( m_scale_k == 1.0 ) return;
		*x *= m_scale_k;
	}
	inline void scr2x( int *x ) const
	{
		if( m_scale_k == 1.0 ) return;
		*x *= m_scale_inv_k;
	}
	inline void ovr2scr( lg_math::ovr_params_t *ovr ) const
	{
		if( m_scale_k == 1.0 ) return;
		ovr->square_size        *= m_scale_k;
		ovr->square_pos.x       *= m_scale_k;
		ovr->square_pos.y       *= m_scale_k;
		ovr->reticle_axis_ra.x  *= m_scale_k;
		ovr->reticle_axis_ra.y  *= m_scale_k;
		ovr->reticle_axis_dec.x *= m_scale_k;
		ovr->reticle_axis_dec.y *= m_scale_k;
		ovr->reticle_pos.x      *= m_scale_k;
		ovr->reticle_pos.y      *= m_scale_k;
		ovr->reticle_org.x      *= m_scale_k;
		ovr->reticle_org.y      *= m_scale_k;
		ovr->osf_pos.x          *= m_scale_k;
		ovr->osf_pos.y          *= m_scale_k;
		ovr->osf_size.x         *= m_scale_k;
		ovr->osf_size.y         *= m_scale_k;
	}
	QSize get_size( void ) const
	{
		return m_sz;
	}
protected:
	void paintEvent(QPaintEvent *)
	{
		if( !m_image )
			return;
		QPainter painter;
		painter.begin(this);
		if( m_scale_k == 1.0 )
			painter.drawImage( 0, 0, *m_image );
		else
		{
			QImage si = m_image->scaled( m_sz, Qt::KeepAspectRatio, Qt::FastTransformation );
			painter.drawImage( 0, 0, si );
		}
		if( m_cd )
			m_cd->draw_overlays( painter );
		painter.end();
	};
	void mouseMoveEvent ( QMouseEvent *event )
	{
		if( !m_cd )
			return;
		m_cd->mouse_move( event );
	}
	void mousePressEvent ( QMouseEvent *event )
	{
		if( !m_cd )
			return;
		m_cd->mouse_press( event );
	}
	void mouseReleaseEvent ( QMouseEvent *event )
	{
		if( !m_cd )
			return;
		m_cd->mouse_release( event );
	}
	void mouseDoubleClickEvent ( QMouseEvent *event )
	{
		if( !m_cd )
			return;
		m_cd->mouse_doubleclick( event );
	}
private:
	complex_delegate *m_cd;
	QImage           *m_image;
	QSize m_sz;
	float m_scale_k;
	float m_scale_inv_k;
};


class common_params
{
public:
	common_params() :
		//DBG_VERBOSITY( false ),
		udp_send_start_stop( false ),
		udp_send_image_quality( false ),
		udp_send_guiding_stability( false ),
		udp_send_drift_data( false ),

		dithering_range( 5 ),
		dithering_rest_tout( 3 ),

		hfd_on( false ),
		square_index( lg_math::cgmath::DEFAULT_SQR ),
		reticle_angle( 0 ),

		osf_size_kx( 1.0 ),
		osf_size_ky( 1.0 ),

		guider_algorithm( lg_math::GA_CENTROID )
	{
	}

	//bool DBG_VERBOSITY;
	bool udp_send_start_stop;
	bool udp_send_image_quality;
	bool udp_send_guiding_stability;
	bool udp_send_drift_data;

	int dithering_range;
	int dithering_rest_tout;

	bool hfd_on;
	int square_index;
	double reticle_angle;

	double osf_size_kx;
	double osf_size_ky;

	int guider_algorithm;
};


typedef struct uiparams_s
{
	static const float MIN_SCALE;
	static const float MAX_SCALE;
	static const float SCALE_STEP;

	uiparams_s() :
		half_refresh_rate( false ),
		show_helper_TB( false ),
		viewport_scale( 1.0 )
	{}
	bool half_refresh_rate;
	bool show_helper_TB;
	float viewport_scale;
}uiparams_t;

#endif /* COMMON_H_ */
