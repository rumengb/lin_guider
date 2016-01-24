/*
 * target_graph.cpp
 *
 *  Created on: 21 янв. 2016 г.
 *      Author: stepanenko
 */

#include <stdlib.h>

#include <QtGui>
#include <QWidget>

#include "scroll_graph.h"
#include "target_graph.h"
#include "utils.h"


target_graph::target_graph( int client_width, int client_height, int cell_nx, int cell_ny ) :
	cscroll_graph( client_width, client_height, cell_nx, cell_ny )
{

}


target_graph::~target_graph()
{
}


void target_graph::refresh( void )
{
	double kx, ky;

	if( !m_need_refresh )
		return;

	// fill background
	m_canvas.fillRect( 0, 0, m_client_rect_wd, m_client_rect_ht, m_brush );

	// Rasterizing coefficients
	kx = (double)m_client_rect_wd / m_vis_range_x;
	ky = (double)m_client_rect_ht / m_vis_range_y;

	draw_grid( kx, ky );

	for( int i = 0;i < m_data_cnt;i++ )
	{
		int x = m_half_buffer_size_wd + (int)(m_data.line[RA_LINE][i] * kx);
		int y = m_half_buffer_size_ht - (int)(m_data.line[DEC_LINE][i] * ky);
		m_canvas.drawPoint( x, y );
	}
}


void target_graph::draw_grid( double kx, double ky )
{
	cscroll_graph::draw_grid( kx, ky );
}


void target_graph::init_render_vars( void )
{
	cscroll_graph::init_render_vars();
}

