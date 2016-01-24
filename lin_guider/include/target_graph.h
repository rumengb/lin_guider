/*
 * target_graph.h
 *
 *      Author: gm
 */

#ifndef TARGET_GRAPH_H_
#define TARGET_GRAPH_H_

#include "scroll_graph.h"


class target_graph : public cscroll_graph
{
public:
	target_graph( int client_width, int client_height, int cell_nx, int cell_ny );
	virtual ~target_graph();

protected:
	virtual void refresh( void );
	virtual void draw_grid( double kx, double ky );
	virtual void init_render_vars( void );
};

#endif /* TARGET_GRAPH_H_ */
