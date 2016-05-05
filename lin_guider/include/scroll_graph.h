/*
 * scroll_graph.h
 *
 *  Created on: 21 Jan 2016.
 *  Author: Rumen G.Bogdanovski
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

#ifndef SCROLL_GRAPH_H_
#define SCROLL_GRAPH_H_

#include "drift_graph.h"


class scroll_graph : public cdrift_graph
{
public:
	scroll_graph( int client_width, int client_height, int cell_nx, int cell_ny);
	virtual ~scroll_graph();

protected:
	virtual void refresh( void );
	virtual void draw_grid( double kx );
};

#endif /* SCROLL_GRAPH_H_ */
