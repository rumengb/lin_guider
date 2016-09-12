/*
 * guider.h
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

#ifndef GUIDER_H
#define GUIDER_H

#include <QtGui>

#include "common.h"
#include "ui_guider.h"
#include "drift_graph.h"
#include "gmath.h"
#include "guider.h"
#include "fio.h"

class lin_guider;
namespace io_drv
{
	class cio_driver_base;
}


class guider : public QDialog
{
    Q_OBJECT

public:
	typedef enum
	{
        GRAPH_SCROLL = 0,
        GRAPH_TARGET_POINTS,
        GRAPH_TARGET_LINES,
        GRPAH_MAX
	} graph_type_t;

	struct drift_view_params_s
	{
		drift_view_params_s() :
			graph_type (GRAPH_SCROLL),
			drift_graph_xrange( -1 ),
			drift_graph_yrange( -1 ),
			cell_nx( 6 ),
			cell_ny( 6 )
		{}
		graph_type_t graph_type;
		int drift_graph_xrange;
		int drift_graph_yrange;
		int cell_nx;
		int cell_ny;
	};
	static const int cell_size = 50;

	guider( lin_guider *parent, io_drv::cio_driver_base *drv, struct guider::drift_view_params_s *dv_params, const common_params &comm_params );
	~guider();

	void guide( void );
	void set_half_refresh_rate( bool is_half );
	void set_math( lg_math::cgmath *math );
	void on_remote_start_stop( bool start );

protected slots:
	void onXscaleChanged( int i );
	void onYscaleChanged( int i );
	void onSquareSizeChanged( int index );
	void onThresholdChanged( int i );
	void onSwapDEC( int state );
	void onNormalizeGain( int state );
	void onSaveLog( int state );
	void onFileNameChanged();
	void onQualityControlChanged( int index );
	void onInfoRateChanged( double val );
	void onEnableDirRA( bool on );
	void onEnableDirDEC( bool on );
	void onEnableDirRAPlus( int state );
	void onEnableDirRAMinus( int state );
	void onEnableDirDECPlus( int state );
	void onEnableDirDECMinus( int state );
	void onInputParamChanged();

	void onStartStopButtonClick();

protected:
	void showEvent ( QShowEvent * event );
	void closeEvent( QCloseEvent *event );
	void hideEvent ( QHideEvent * event );

private:
	lg_math::cgmath *m_math;

	custom_drawer *m_drift_out;
	cdrift_graph *m_drift_graph;
	fio	*m_logger;
	bool is_started;
	bool half_refresh_rate;
	bool save_drift;
	int  quality_rate;
	int  guiding_stable;

	void initialize_graph( void );
	void fill_interface( void );
	void update_gains( void );
	void check_for_events( void );
	void update_status( enum lg_math::cgmath::status_level level, const std::string &txt );

	lin_guider *pmain_wnd;
	io_drv::cio_driver_base *m_driver;
	struct guider::drift_view_params_s *m_drift_view_params;
	const common_params &m_common_params;

	graph_type_t m_prev_graph_type;

	unsigned int m_status_key;

private:
    Ui::guiderClass ui;
};

#endif // GUIDER_H
