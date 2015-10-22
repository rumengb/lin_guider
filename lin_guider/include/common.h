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


class complex_delegate
{
public:
	virtual ~complex_delegate() {}
	virtual void mouse_press( QMouseEvent *event ) = 0;
	virtual void mouse_release( QMouseEvent *event ) = 0;
	virtual void mouse_move( QMouseEvent *event ) = 0;
	virtual void draw_overlays( QPainter &painter) = 0;
};


class custom_drawer : public QWidget
{
    Q_OBJECT

public:
    explicit custom_drawer(QWidget *parent = NULL ) :
		QWidget( parent ),
		m_cd( NULL ),
		m_image( NULL )
	{
	}
	~custom_drawer()
	{
	}
	bool set_source( QImage *image, complex_delegate *cd )
	{
		m_image = image;
		if( !m_image )
			return false;
		resize( m_image->size() );
		m_cd = cd;
	return true;
	}
protected:
	void paintEvent(QPaintEvent *)
	{
		if( !m_image )
			return;
		QPainter painter;
		painter.begin(this);
		painter.drawImage( 0, 0, *m_image );
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
private:
	complex_delegate *m_cd;
	QImage           *m_image;
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
		square_index( DEFAULT_SQR ),
		reticle_angle(0)
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

};

#endif /* COMMON_H_ */
