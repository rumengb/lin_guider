/*
 * drift_graph.h
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

#ifndef CDRIFT_GRAPH_H_
#define CDRIFT_GRAPH_H_

#include <QtGui>
#include <QWidget>
#include <QImage>
#include <QPainter>


class cdrift_graph
{
public:
	cdrift_graph( int client_width, int client_height, int cell_nx, int cell_ny );
	virtual ~cdrift_graph();
	
	QImage *get_buffer( void ) const;
	void add_point( double ra, double dec );
	void set_visible_ranges( int rx, int ry );
	void get_visible_ranges( int *rx, int *ry ) const;
	int  get_gridx_N( void ) const;
	int  get_gridy_N( void ) const;
	void reset_data( void );
	void on_paint( void );
	void get_screen_size( int *sx, int *sy ) const;
	
protected:
	#define RA_LINE    0
	#define DEC_LINE   1

	typedef struct
	{
		double *line[2];
	}delta_data_t;

	// view
	QPainter m_canvas;
	QPen     m_pen;
	QBrush   m_brush;
	QColor   BKGD_COLOR, RA_COLOR, DEC_COLOR, GRID_COLOR, WHITE_COLOR, GRID_FONT_COLOR;
	int  m_half_buffer_size_wd;
	int  m_half_buffer_size_ht;
	int  m_client_rect_wd;
	int  m_client_rect_ht;
	bool m_need_refresh;
	
	// data
	delta_data_t m_data;
	int m_data_len;
	int m_data_idx;
	int m_data_count;
	int m_gridx_N;
	int m_gridy_N;
	
	// grid vars...
	double m_grid_step_x, m_grid_step_y, m_grid_view_step_x, m_grid_view_step_y;
	int m_font_ht_k;
	
	// control
	int m_vis_range_x, m_vis_range_y;
	int m_half_vis_range_x, m_half_vis_range_y;

	// may be overloaded in inherited classes
	virtual void refresh( void ) {};
	virtual void draw_grid( void ) {};
	virtual void init_render_vars( void );
	inline void get_point(unsigned int index, double *data_ra, double *data_dec) const
    {
        //prevent overindexing
        if (index >= (unsigned)m_data_len)
            index = m_data_len-1;

        if (m_data_count > m_data_len) {
            int offset = index + m_data_idx;
            if(offset < m_data_len) {
                *data_ra = m_data.line[RA_LINE][offset];
                *data_dec = m_data.line[DEC_LINE][offset];
                return;
            } else {
                *data_ra = m_data.line[RA_LINE][offset - m_data_len];
                *data_dec = m_data.line[DEC_LINE][offset - m_data_len];
                return;
            }
        }
        *data_ra = m_data.line[RA_LINE][index];
        *data_dec = m_data.line[DEC_LINE][index];
	}
private:
	QImage  *m_buffer;
};

#endif /*CDRIFT_GRAPH_H_*/
