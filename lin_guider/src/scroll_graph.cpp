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
	int i, j, k;
	double kx, ky, step;
	double *data_ptr;
	int start_idx;
	int /*band1_wd,*/ band1_start, band1_end;
	int band2_wd, band2_start, band2_end;
	int x, y;
	int px, py;

	if( !m_need_refresh )
		return;

	// fill background
	m_canvas.fillRect( 0, 0, m_client_rect_wd, m_client_rect_ht, m_brush );

	start_idx = (m_data_idx + m_data_cnt - m_vis_range_x) % m_data_cnt;
	// split visible region in 2 ranges
	if( m_data_idx > start_idx ) // only 1 band
	{
		//band1_wd 	= m_data_idx - start_idx; // = m_vis_range_x
		band1_start = start_idx-1;
		band1_end	= m_data_idx-1; // -1;
		band2_start = band2_end = band2_wd = 0;
	}
	else // 2 bands
	{
		//band1_wd 	= m_data_idx;
		band1_start = 0;
		band1_end 	= m_data_idx-1; //-1;

		band2_wd 	= m_data_cnt - start_idx;
		band2_start = start_idx;
		band2_end	= m_data_cnt-1;
	}

	// Rasterizing coefficients
	kx = (double)m_client_rect_wd / m_vis_range_x;
	ky = (double)m_client_rect_ht / m_vis_range_y;

	draw_grid( kx );

	// analyse kx and select optimal algorithm
	if( m_client_rect_wd <= m_vis_range_x )
	{
		step = 1.0 / kx;

		for( k = 0;k < 2;k++ )
		{
			data_ptr = m_data.line[k];

			if( k == RA_LINE )
				m_pen.setColor( RA_COLOR );
			else
				m_pen.setColor( DEC_COLOR );

			m_canvas.setPen( m_pen );

			// process band 1
			px = m_client_rect_wd;
			int p_idx = band1_end;
			if( p_idx < 0 )p_idx += m_data_cnt;
			py = m_half_buffer_size_ht - (int)(data_ptr[p_idx] * ky);

			x = m_client_rect_wd;

			for( i = band1_end, j = 0;i > band1_start; )
			{
				y = m_half_buffer_size_ht - (int)(data_ptr[i] * ky);
				x--;

				m_canvas.drawLine( px, py, x, y );

				px = x;
				py = y;

				//------------------------------------------
				j++;
				i = band1_end - (int)((double)j*step);
			}

			// process band 2
			for( i = band2_end, j = 0;i > band2_start; )
			{
				y = m_half_buffer_size_ht - (int)(data_ptr[i] * ky);
				x--;

				m_canvas.drawLine( px, py, x, y );

				px = x;
				py = y;

				//------------------------------------------
				j++;
				i = band2_end - (int)((double)j*step);
			}
		}
	}
	else
	{
		step = kx;

		for( k = 0;k < 2;k++ )
		{
			data_ptr = m_data.line[k];

			if( k == RA_LINE )
				m_pen.setColor( RA_COLOR );
			else
				m_pen.setColor( DEC_COLOR );

			m_canvas.setPen( m_pen );

			// process band 1
			px = m_client_rect_wd;
			int p_idx = band1_end-1;
			if( p_idx < 0 )p_idx += m_data_cnt;
			py = m_half_buffer_size_ht - (int)(data_ptr[p_idx] * ky);

			x = m_client_rect_wd;

			for( i = band1_end, j = 0;i > band1_start;i--, j++ )
			{
				y = m_half_buffer_size_ht - (int)(data_ptr[i] * ky);
				x = m_client_rect_wd - (int)((double)j*step) - 1;

				m_canvas.drawLine( px, py, x, y );

				px = x;
				py = y;
			}

			// process band 2
			for( i = band2_end;i > band2_start;i--, j++ )
			{
				y = m_half_buffer_size_ht - (int)(data_ptr[i] * ky);
				x = m_client_rect_wd - (int)((double)j*step) - 1;

				m_canvas.drawLine( px, py, x, y );

				px = x;
				py = y;
			}

		}
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

	grid_column = m_data_idx / (int)m_grid_step_x * (int)m_grid_step_x;
	sx = m_client_rect_wd - (double)(m_data_idx % (int)m_grid_step_x)*kx;

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
		str.setNum( (int)(m_half_vis_range_y - m_grid_step_y*i) );
		m_canvas.drawText( 2, y + m_font_ht_k, str );
	}
}
