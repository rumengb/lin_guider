/*
 * scroll_graph.cpp
 *
 *  Created on: 04 May 2016.
 *  Authors: Rumen G.Bogdanovski, Andrew Stepanenko
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

#include <stdlib.h>

#include <QtGui>
#include <QWidget>

#include "drift_graph.h"
#include "scroll_graph.h"
#include "utils.h"


scroll_graph::scroll_graph( int client_width, int client_height, int cell_nx, int cell_ny) :
	cdrift_graph( client_width, client_height, cell_nx, cell_ny )
{

}


scroll_graph::~scroll_graph()
{

}


void scroll_graph::refresh( void )
{
	int i, min_x, max_x;
	double kx, ky;
	double data_ra, data_dec;

	int x, y_ra, y_dec;
	int py_ra, py_dec;
	double px;

	if( !m_need_refresh )
		return;

	// Rasterizing coefficients
	kx = (double)m_client_rect_wd / m_vis_range_x;
	ky = (double)m_client_rect_ht / m_vis_range_y;

	max_x = (m_data_count > m_data_len) ? m_data_len : m_data_count;
	min_x = (max_x > m_vis_range_x) ? (max_x - m_vis_range_x) : 0;

	get_point(max_x, &data_ra, &data_dec);
	px = (double)m_client_rect_wd;
	py_ra = m_half_buffer_size_wd - (int)(data_ra * ky);
	py_dec = m_half_buffer_size_ht - (int)(data_dec * ky);

	// fill background
	m_canvas.fillRect( 0, 0, m_client_rect_wd, m_client_rect_ht, m_brush );
	draw_grid( kx );

	// Draw Graphs
	for(i = max_x-1; i >= min_x; i--) {
		get_point(i, &data_ra, &data_dec);

		y_ra = m_half_buffer_size_ht - (int)(data_ra * ky);
		y_dec = m_half_buffer_size_ht - (int)(data_dec * ky);
		x = (int)(px - kx);

		m_pen.setColor( RA_COLOR );
		m_canvas.setPen( m_pen );
		m_canvas.drawLine( px, py_ra, x, y_ra );

		m_pen.setColor( DEC_COLOR );
		m_canvas.setPen( m_pen );
		m_canvas.drawLine( (int)px, py_dec, x, y_dec );

		px -= kx;
		py_ra = y_ra;
		py_dec = y_dec;
	}
	m_need_refresh = false;
}


void scroll_graph::draw_grid( double kx )
{
	int i, x, sx, y;
	int grid_column, val;
	QString str;

	m_pen.setColor( GRID_COLOR );
	m_canvas.setPen( m_pen );

	grid_column = m_data_count / (int)m_grid_step_x * (int)m_grid_step_x;
	sx = m_client_rect_wd - (double)(m_data_count % (int)m_grid_step_x)*kx;

	for( i = 0;i < m_gridx_N;i++ )
	{
		x = sx - (double)i*m_grid_view_step_x;
		m_canvas.drawLine( x, 0, x, m_client_rect_ht );
	}
	for( i = 0;i < m_gridy_N;i++ )
	{
		y = (double)i*m_grid_view_step_y;
		if( i == m_gridy_N/2 )
		{
			m_pen.setColor( WHITE_COLOR );
			m_canvas.setPen( m_pen );
			m_canvas.drawLine( 0, y, m_client_rect_wd, y );
			m_pen.setColor( GRID_COLOR );
			m_canvas.setPen( m_pen );
		}
		else
			m_canvas.drawLine( 0, y, m_client_rect_wd, y );
	}

	// draw all digits
	m_pen.setColor( GRID_FONT_COLOR );
	m_canvas.setPen( m_pen );
	int grid_max = (m_gridx_N > m_gridy_N) ? m_gridx_N : m_gridy_N;
	for( i = 0;i < grid_max;i++ )
	{
		x = sx - (double)i*m_grid_view_step_x;
		y = (double)i*m_grid_view_step_y;

		if( (val = grid_column - i*(int)m_grid_step_x) >= 0 )
		{
			str.setNum( val );
			m_canvas.drawText( x, m_half_buffer_size_ht + m_font_ht_k, str );
		}
		str.sprintf( "%d\"", (int)(m_half_vis_range_y - m_grid_step_y*i) );
		m_canvas.drawText( 2, y + m_font_ht_k, str );
	}
}
