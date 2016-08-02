/*
 * drift_graph.cpp
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

#include "drift_graph.h"
#include "utils.h"
#include "maindef.h"


cdrift_graph::cdrift_graph( int client_width, int client_height, int cell_nx, int cell_ny )
{
	m_client_rect_wd = client_width;
	m_client_rect_ht = client_height;

	m_buffer = new QImage( m_client_rect_wd, m_client_rect_ht, QImage::Format_RGB32 );

	m_vis_range_x = m_client_rect_wd; // horizontal range in ticks
	m_vis_range_y = 100;	// whole visible vertical range in arcsecs!

	m_gridx_N = cell_nx;
	m_gridy_N = cell_ny & (~1);

	m_data_len = 100*m_gridx_N;
	//m_data_len =10;
	m_data.line[ RA_LINE ] = new double[ m_data_len ];
	m_data.line[ DEC_LINE ] = new double[ m_data_len ];
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

cdrift_graph::~cdrift_graph()
{
	delete m_buffer;
	delete [] m_data.line[ RA_LINE ];
	delete [] m_data.line[ DEC_LINE ];
}


void cdrift_graph::init_render_vars( void )
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


void cdrift_graph::set_visible_ranges( int rx, int ry )
{
	if( rx < 10*m_gridx_N ) rx = 10*m_gridx_N;
	if( rx > m_data_len ) rx = m_data_len;

	if( m_vis_range_x != rx )
		m_need_refresh = true;
	m_vis_range_x = rx;

	if( ry < m_gridy_N ) ry = m_gridy_N;

	if( m_vis_range_x != ry )
		m_need_refresh = true;
	m_vis_range_y = ry;

	init_render_vars();
}


void cdrift_graph::get_visible_ranges( int *rx, int *ry ) const
{
	*rx = m_vis_range_x;
	*ry = m_vis_range_y;
}


int cdrift_graph::get_gridx_N( void ) const
{
	return m_gridx_N;
}


int cdrift_graph::get_gridy_N( void ) const
{
	return m_gridy_N;
}


void cdrift_graph::reset_data( void )
{
	memset( m_data.line[RA_LINE], 0, sizeof(double)*m_data_len );
	memset( m_data.line[DEC_LINE], 0, sizeof(double)*m_data_len );
	m_data_idx = 0;
	m_data_count = 0;
	m_need_refresh = true;
}


QImage *cdrift_graph::get_buffer( void ) const
{
	return m_buffer;
}


void cdrift_graph::get_screen_size( int *sx, int *sy ) const
{
	*sx = m_client_rect_wd;
	*sy = m_client_rect_ht;
}


void cdrift_graph::on_paint( void )
{
	m_canvas.begin( get_buffer() );

	refresh();

	m_canvas.end();
}


void cdrift_graph::add_point( double ra, double dec )
{
	m_data.line[ RA_LINE ][ m_data_idx ]  = ra;
	m_data.line[ DEC_LINE ][ m_data_idx ] = dec;

	m_data_idx++;
	m_data_count++;

	if( m_data_idx == m_data_len )
		m_data_idx = 0;

	m_need_refresh = true;
}
