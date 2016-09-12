/*
 * target_graph.cpp
 *
 *  Created on: 21 Jan 2016.
 *  Authors: Andrew Stepanenko, Rumen G.Bogdanovski
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
#include "target_graph.h"
#include "utils.h"


target_graph::target_graph( int client_width, int client_height, int cell_nx, int cell_ny, bool use_lines ) :
	cdrift_graph( client_width, client_height, cell_nx, cell_ny )
{
	m_use_lines = use_lines;
}


target_graph::~target_graph()
{

}


void target_graph::refresh(void)
{
	double k, data_ra, data_dec;
	int px, py, x, y, i;

	if( !m_need_refresh )
		return;

	// fill background
	m_brush.setColor( BKGD_COLOR );
	m_canvas.fillRect( 0, 0, m_client_rect_wd, m_client_rect_ht, m_brush );

	// Rasterizing coefficients
	k = (double)m_client_rect_ht / m_vis_range_y;

	draw_grid();

	// Draw points
	m_pen.setColor( RA_COLOR );
	m_canvas.setPen( m_pen );
	int count_max = (m_data_len <= m_data_count) ? m_data_len : m_data_idx;

	if (m_use_lines) {
		if (count_max > 0) {
			get_point(0, &data_ra, &data_dec);
			x = m_half_buffer_size_wd + (int)(data_ra * k);
			y = m_half_buffer_size_ht - (int)(data_dec * k);
			m_canvas.drawEllipse(QPointF(x, y), 1, 1);
		}
		for( i = 1; i < count_max; i++ ) {
			get_point(i, &data_ra, &data_dec);
			px = m_half_buffer_size_wd + (int)(data_ra * k);
			py = m_half_buffer_size_ht - (int)(data_dec * k);
			get_point(i-1, &data_ra, &data_dec);
			x = m_half_buffer_size_wd + (int)(data_ra * k);
			y = m_half_buffer_size_ht - (int)(data_dec * k);
			m_canvas.drawLine( px, py, x, y );
			m_canvas.drawEllipse(QPointF(x, y), 1, 1);
		}
	} else {
		for( i = 0; i < count_max; i++ ) {
			get_point(i, &data_ra, &data_dec);
			x = m_half_buffer_size_wd + (int)(data_ra * k);
			y = m_half_buffer_size_ht - (int)(data_dec * k);
			m_canvas.drawEllipse(QPointF(x, y), 1, 1);
		}
	}
	if (count_max > 1) {
		// Draw the last movement
		m_pen.setColor( DEC_COLOR );
		m_canvas.setPen( m_pen );
		get_point(i-2, &data_ra, &data_dec);
		px = m_half_buffer_size_wd + (int)(data_ra * k);
		py = m_half_buffer_size_ht - (int)(data_dec * k);
		get_point(i-1, &data_ra, &data_dec);
		x = m_half_buffer_size_wd + (int)(data_ra * k);
		y = m_half_buffer_size_ht - (int)(data_dec * k);
		m_canvas.drawLine( px, py, x, y );
		m_canvas.drawEllipse(QPointF(x, y), 5, 5);
	}
	m_need_refresh = false;
}


void target_graph::draw_grid( void )
{
	int i, x, y;
	QString str;

	m_pen.setColor( GRID_COLOR );
	m_canvas.setPen( m_pen );

	x =  m_client_rect_wd / 2;

	for( i = 0;i <= m_gridy_N/2;i++ )
	{
		y = (double)i*m_grid_view_step_y;
		if( i == m_gridy_N/2 )
		{
			m_pen.setColor( GRID_COLOR );
			m_canvas.setPen( m_pen );
			m_canvas.drawLine( 0, y, m_client_rect_wd, y );
			m_canvas.drawLine( x, 0, x , m_client_rect_ht );
			m_canvas.setPen( m_pen );
			m_canvas.drawEllipse(QPointF(m_half_buffer_size_wd, m_half_buffer_size_ht),
			                    (double)i*m_grid_view_step_x, (double)i*m_grid_view_step_y);
		}
		else
		{
			m_pen.setColor( GRID_COLOR );
			m_canvas.setPen( m_pen );
			m_canvas.drawEllipse(QPointF(m_half_buffer_size_wd, m_half_buffer_size_ht),
			                    (double)i*m_grid_view_step_x, (double)i*m_grid_view_step_y);
		}
	}

	// draw digits
	m_pen.setColor( GRID_FONT_COLOR );
	m_canvas.setPen( m_pen );
	x = m_client_rect_wd / 2 + 2;
	for( i = 0; i < m_gridy_N; i++ )
	{
		y = (double)i * m_grid_view_step_y;
		str.sprintf("%d\"", (int)(m_half_vis_range_y - m_grid_step_y*i) );
		m_canvas.drawText( x, y + m_font_ht_k, str );
	}
}
