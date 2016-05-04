/*
 * scroll_graph.cpp
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

#include <QtGui>
#include <QWidget>

#include "scroll_graph.h"
#include "utils.h"
#include "maindef.h"



cscroll_graph::cscroll_graph( int client_width, int client_height, int cell_nx, int cell_ny )
{
	m_client_rect_wd = client_width;
	m_client_rect_ht = client_height;

	m_buffer = new QImage( m_client_rect_wd, m_client_rect_ht, QImage::Format_RGB32 );

	m_vis_range_x = m_client_rect_wd; // horizontal range in ticks
	m_vis_range_y = 100;	// whole visible vertical range in arcsecs!

	m_gridx_N = cell_nx;
	m_gridy_N = cell_ny & (~1);

	m_data_cnt = 10*m_gridx_N*10;
	m_data.line[ RA_LINE ] = new double[ m_data_cnt ];
	m_data.line[ DEC_LINE ] = new double[ m_data_cnt ];
	reset_data();

	//graphics...
	m_pen.setStyle( Qt::SolidLine );
	m_pen.setWidth(1);
	m_brush.setStyle(Qt::SolidPattern);

	RA_COLOR 		= QColor( DEF_RA_COLOR[0], DEF_RA_COLOR[1], DEF_RA_COLOR[2] );
	DEC_COLOR 		= QColor( DEF_DEC_COLOR[0], DEF_DEC_COLOR[1], DEF_DEC_COLOR[2] );
	GRID_COLOR 		= QColor( DEF_GRID_COLOR[0], DEF_GRID_COLOR[1], DEF_GRID_COLOR[2] );
	BKGD_COLOR 		= QColor( DEF_BKGD_COLOR[0], DEF_BKGD_COLOR[1], DEF_BKGD_COLOR[2] );
	WHITE_COLOR 	= QColor( DEF_WHITE_COLOR[0], DEF_WHITE_COLOR[1], DEF_WHITE_COLOR[2] );
	GRID_FONT_COLOR	= QColor( DEF_GRID_FONT_COLOR[0], DEF_GRID_FONT_COLOR[1], DEF_GRID_FONT_COLOR[2] );
	m_brush.setColor( BKGD_COLOR );

	// init...
	init_render_vars();

	m_need_refresh = true;

	m_canvas.begin( get_buffer() );
	m_font_ht_k = m_canvas.fontMetrics().ascent();
	m_canvas.end();
}

cscroll_graph::~cscroll_graph()
{
	delete m_buffer;
	delete [] m_data.line[ RA_LINE ];
	delete [] m_data.line[ DEC_LINE ];
}


void cscroll_graph::init_render_vars( void )
{
	m_half_buffer_size_wd = m_client_rect_wd / 2;
	m_half_buffer_size_ht = m_client_rect_ht / 2;

	m_grid_view_step_x = (double)m_client_rect_wd / (double)m_gridx_N;
	m_grid_view_step_y = (double)m_client_rect_ht / (double)m_gridy_N;

	m_grid_step_x = (double)m_vis_range_x / (double)m_gridx_N;
	m_grid_step_y = (double)m_vis_range_y / (double)m_gridy_N;

	m_half_vis_range_x = m_vis_range_x / 2;
	m_half_vis_range_y = m_vis_range_y / 2;
}


void cscroll_graph::set_visible_ranges( int rx, int ry )
{
	if( rx < 10*m_gridx_N ) rx = 10*m_gridx_N;
	if( rx > m_data_cnt ) rx = m_data_cnt;

	if( m_vis_range_x != rx )
		m_need_refresh = true;
	m_vis_range_x = rx;

	if( ry < 5*m_gridy_N ) ry = 5*m_gridy_N;

	if( m_vis_range_x != ry )
		m_need_refresh = true;
	m_vis_range_y = ry;

	init_render_vars();
}


void cscroll_graph::get_visible_ranges( int *rx, int *ry ) const
{
	*rx = m_vis_range_x;
	*ry = m_vis_range_y;
}


int cscroll_graph::get_gridx_N( void ) const
{
	return m_gridx_N;
}


int cscroll_graph::get_gridy_N( void ) const
{
	return m_gridy_N;
}


void cscroll_graph::reset_data( void )
{
	memset( m_data.line[RA_LINE], 0, sizeof(double)*m_data_cnt );
	memset( m_data.line[DEC_LINE], 0, sizeof(double)*m_data_cnt );
	m_data_idx = 0;
	m_need_refresh = true;
}



QImage *cscroll_graph::get_buffer( void ) const
{
	return m_buffer;
}


void cscroll_graph::get_screen_size( int *sx, int *sy ) const
{
	*sx = m_client_rect_wd;
	*sy = m_client_rect_ht;
}


void cscroll_graph::on_paint( void )
{
	m_canvas.begin( get_buffer() );

	refresh();

	m_canvas.end();
}


/*************
*
* Main Drawing function
*
**************/
void cscroll_graph::refresh( void )
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

	draw_grid( kx, ky );

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


void cscroll_graph::draw_grid( double kx, double )
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


void cscroll_graph::add_point( double ra, double dec )
{
	m_data.line[ RA_LINE ][ m_data_idx ]  = ra;
	m_data.line[ DEC_LINE ][ m_data_idx ] = dec;

	m_data_idx++;

	if( m_data_idx == m_data_cnt )
		m_data_idx = 0;

	m_need_refresh = true;
}

