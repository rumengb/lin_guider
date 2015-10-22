/*
 * scroll_graph.h
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

#ifndef CSCROLL_GRAPH_H_
#define CSCROLL_GRAPH_H_

#include <QtGui>
#include <QWidget>
#include <QImage>
#include <QPainter>


#define RA_LINE    0
#define DEC_LINE   1

typedef struct
{
	double *line[2];
}delta_data_t;



class cscroll_graph
{
public:
	cscroll_graph( QWidget *own, int client_width, int client_height, int cell_nx, int cell_ny );
	virtual ~cscroll_graph();
	
	QImage *get_buffer( void ) const;
	bool add_point( double ra, double dec );
	void set_visible_ranges( int rx, int ry );
	void get_visible_ranges( int *rx, int *ry ) const;
	int  get_gridx_N( void ) const;
	int  get_gridy_N( void ) const;
	void reset_view( void );
	void reset_data( void );
	void on_paint( void );
	void get_screen_size( int *sx, int *sy ) const;
	
private:
	// view
	QWidget *owner;
	QImage *buffer;
	QPainter canvas;
	QColor BKGD_COLOR, RA_COLOR, DEC_COLOR, GRID_COLOR, WHITE_COLOR, GRID_FONT_COLOR;
	QPen pen;	
	QBrush brush;
	int half_buffer_size_wd;
	int half_buffer_size_ht;
	int client_rect_wd;
	int client_rect_ht;
	bool need_refresh;
	
	// data
	delta_data_t data;
	int data_cnt;
	int	data_idx;
	int gridx_N;
	int gridy_N;
	
	// grid vars...
	double grid_step_x, grid_step_y, grid_view_step_x, grid_view_step_y;
	int font_ht_k;
	
	// control
	int vis_range_x, vis_range_y;
	int half_vis_range_x, half_vis_range_y;
	
	void refresh( void );
	void draw_grid( double kx, double ky );
	void init_render_vars( void );
	
};

#endif /*CSCROLL_GRAPH_H_*/
