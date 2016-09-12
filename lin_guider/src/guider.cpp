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

#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include <utility>

#include "guider.h"
#include "lin_guider.h"
#include "gmath.h"
#include "server.h"
#include "scroll_graph.h"
#include "target_graph.h"

#include "utils.h"


guider::guider( lin_guider *parent, io_drv::cio_driver_base *drv, struct guider::drift_view_params_s *dv_params, const common_params &comm_params ) :
	QDialog( parent ),
	m_math( NULL ),
	m_drift_out( NULL ),
	m_drift_graph( NULL ),
	m_logger( NULL ),
	is_started( false ),
	half_refresh_rate( false ),
	save_drift( false ),
	quality_rate( 0 ),
	guiding_stable( 0 ),
	pmain_wnd( parent ),
	m_driver( drv ),
	m_drift_view_params( dv_params ),
	m_common_params( comm_params ),
	m_prev_graph_type( GRPAH_MAX ),
	m_status_key( 0 )
{
	int i;

	ui.setupUi(this);

	this->adjustSize();

	ui.l_Status->setAutoFillBackground( true );

	ui.comboBox_SquareSize->clear();
	for( i = 0;lg_math::guide_squares[i].size != -1;i++ )
		ui.comboBox_SquareSize->addItem( QString().setNum( lg_math::guide_squares[i].size ) );

	ui.comboBox_ThresholdAlg->clear();
	for( i = 0;lg_math::guide_square_alg[i].idx != -1;i++ )
		ui.comboBox_ThresholdAlg->addItem( QString( lg_math::guide_square_alg[i].name ) );

	ui.comboBox_QualityControl->clear();
	for( i = 0;lg_math::q_control_mtd[i].idx != -1;i++ )
		ui.comboBox_QualityControl->addItem( QString( lg_math::q_control_mtd[i].name ) );

	ui.spinBox_AccFramesRA->setMaximum( lg_math::MAX_ACCUM_CNT );
	ui.spinBox_AccFramesDEC->setMaximum( lg_math::MAX_ACCUM_CNT );

	// connect ui
	connect( ui.spinBox_XScale, 		SIGNAL(valueChanged(int)), this, SLOT(onXscaleChanged(int)) );
	connect( ui.spinBox_YScale, 		SIGNAL(valueChanged(int)), this, SLOT(onYscaleChanged(int)) );
	connect( ui.comboBox_SquareSize, 	SIGNAL(activated(int)),    this, SLOT(onSquareSizeChanged(int)) );
	connect( ui.comboBox_ThresholdAlg, 	SIGNAL(activated(int)),    this, SLOT(onThresholdChanged(int)) );
	connect( ui.comboBox_QualityControl,SIGNAL(activated(int)),	   this, SLOT(onQualityControlChanged(int)) );
	connect( ui.doubleSpinBox_QualityThreshold1, SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.doubleSpinBox_QualityThreshold2, SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.checkBox_SwapDec, 		SIGNAL(stateChanged(int)), this, SLOT(onSwapDEC(int)) );
	connect( ui.checkBox_normalizeGain, SIGNAL(stateChanged(int)), this, SLOT(onNormalizeGain(int)) );
	connect( ui.checkBox_SaveLog, 		SIGNAL(stateChanged(int)), this, SLOT(onSaveLog(int)) );
	connect( ui.lineEdit_DriftFileName,	SIGNAL(editingFinished()), this, SLOT(onFileNameChanged()) );
	connect( ui.spinBox_GuideRate, 		SIGNAL(valueChanged(double)), this, SLOT(onInfoRateChanged(double)) );
	connect( ui.groupBox_DirRA, 		SIGNAL(toggled(bool)), 	   this, SLOT(onEnableDirRA(bool)) );
	connect( ui.groupBox_DirDEC, 		SIGNAL(toggled(bool)), 	   this, SLOT(onEnableDirDEC(bool)) );
	connect( ui.checkBox_DirRAPlus,		SIGNAL(stateChanged(int)), this, SLOT(onEnableDirRAPlus(int)) );
	connect( ui.checkBox_DirRAMinus,	SIGNAL(stateChanged(int)), this, SLOT(onEnableDirRAMinus(int)) );
	connect( ui.checkBox_DirDECPlus,	SIGNAL(stateChanged(int)), this, SLOT(onEnableDirDECPlus(int)) );
	connect( ui.checkBox_DirDECMinus,	SIGNAL(stateChanged(int)), this, SLOT(onEnableDirDECMinus(int)) );
	connect( ui.spinBox_AccFramesRA, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_AccFramesDEC, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_PropGainRA, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_PropGainDEC, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_IntGainRA, 		SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_IntGainDEC, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_DerGainRA, 		SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_DerGainDEC, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_MaxPulseRA, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_MaxPulseDEC, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_MinPulseRA, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );
	connect( ui.spinBox_MinPulseDEC, 	SIGNAL(editingFinished()), this, SLOT(onInputParamChanged()) );

	connect( ui.pushButton_StartStop, SIGNAL(clicked()), this, SLOT(onStartStopButtonClick()) );

	// init drift widget
	m_drift_out = new custom_drawer( ui.frame_Graph );
	m_drift_out->move( ui.frame_Graph->frameWidth(), ui.frame_Graph->frameWidth() );
	m_drift_out->setAttribute( Qt::WA_NoSystemBackground, true );
	ui.frame_Graph->setAttribute( Qt::WA_NoSystemBackground, true );

	initialize_graph();

	// not UI vars
	is_started = false;
	half_refresh_rate = false;

	// init logger
	m_logger = new fio();
	save_drift = false;
	ui.checkBox_SaveLog->setChecked( save_drift );

	quality_rate   = -1;
	guiding_stable = -1;

}

guider::~guider()
{
	delete m_drift_out;
	delete m_drift_graph;
	delete m_logger;
}


void guider::initialize_graph()
{
	if( m_drift_graph )
	{
		delete m_drift_graph;
		m_drift_graph = NULL;
	}

	int cell_nx = m_drift_view_params->cell_nx < 2 ? 2 : m_drift_view_params->cell_nx;
	cell_nx = cell_nx <= 10 ? cell_nx : 10;
	int cell_ny = m_drift_view_params->cell_ny < 2 ? 2 : m_drift_view_params->cell_ny;
	cell_ny = cell_ny <= 10 ? cell_ny : 10;
	int DRIFT_GRAPH_WIDTH = cell_nx * cell_size;
	int DRIFT_GRAPH_HEIGHT = cell_ny * cell_size;

	switch  (m_drift_view_params->graph_type) {
	case GRAPH_TARGET_POINTS:
		m_drift_graph = new target_graph( DRIFT_GRAPH_WIDTH, DRIFT_GRAPH_HEIGHT, cell_nx, cell_ny, false );
		break;
	case GRAPH_TARGET_LINES:
		m_drift_graph = new target_graph( DRIFT_GRAPH_WIDTH, DRIFT_GRAPH_HEIGHT, cell_nx, cell_ny, true );
		break;
	default:
		m_drift_graph = new scroll_graph( DRIFT_GRAPH_WIDTH, DRIFT_GRAPH_HEIGHT, cell_nx, cell_ny );
	}

	m_drift_graph->set_visible_ranges( m_drift_view_params->drift_graph_xrange > 0 && m_drift_view_params->drift_graph_xrange <= DRIFT_GRAPH_WIDTH ? m_drift_view_params->drift_graph_xrange : DRIFT_GRAPH_WIDTH,
									   //DRIFT_GRAPH_WIDTH,
									   m_drift_view_params->drift_graph_yrange > 0 ? m_drift_view_params->drift_graph_yrange : 60 );

	m_drift_out->set_source( m_drift_graph->get_buffer(), NULL );
	m_drift_graph->on_paint();

	ui.frame_Graph->setMaximumSize( DRIFT_GRAPH_WIDTH + 2*ui.frame_Graph->frameWidth(), DRIFT_GRAPH_HEIGHT + 2*ui.frame_Graph->frameWidth() );
	ui.frame_Graph->setMinimumSize( ui.frame_Graph->maximumSize() );

	m_prev_graph_type = m_drift_view_params->graph_type;
}


void guider::showEvent ( QShowEvent * event )
{
	if( event->spontaneous() )
		return;

	if( !m_drift_graph ||
		m_prev_graph_type != m_drift_view_params->graph_type ||
		m_drift_graph->get_gridx_N() != m_drift_view_params->cell_nx ||
		m_drift_graph->get_gridy_N() != m_drift_view_params->cell_ny )
		initialize_graph();

	pmain_wnd->lock_toolbar( true );

	fill_interface();
}


void guider::closeEvent( QCloseEvent * )
{
	// trick to stop guiging
	is_started = true;
	onStartStopButtonClick();

	pmain_wnd->lock_toolbar( false );
}


void guider::hideEvent( QHideEvent * event )
{
	if( event->spontaneous() )
		return;

	close();
}


void guider::set_half_refresh_rate( bool is_half )
{
	half_refresh_rate = is_half;
}


void guider::set_math( lg_math::cgmath *math )
{
	assert( math );
	m_math = math;

	ui.comboBox_SquareSize->setEnabled( m_math->get_type() == lg_math::GA_CENTROID );
	setWindowTitle( tr("Guider - ") + QString(m_math->get_name()) );
}


void guider::on_remote_start_stop( bool start )
{
	// start
	if( (is_started && start) || (!is_started && !start) )
		return;

	onStartStopButtonClick();
}


void guider::fill_interface( void )
{
	const lg_math::cproc_in_params *in_params;
	const lg_math::cproc_out_params *out_params;
	lg_math::info_params_t	info_params;
	QString str;
	int rx, ry;

	assert( m_math );

	info_params = m_math->get_info_params();
	in_params   = m_math->get_in_params();
	out_params  = m_math->get_out_params();

	m_drift_graph->get_visible_ranges( &rx, &ry );
	ui.spinBox_XScale->setValue( rx / m_drift_graph->get_gridx_N() );
	ui.spinBox_YScale->setValue( ry / m_drift_graph->get_gridy_N() );

	ui.comboBox_SquareSize->setCurrentIndex( m_math->get_square_index() );
	ui.comboBox_ThresholdAlg->setCurrentIndex( m_math->get_square_algorithm_index() );

	ui.comboBox_QualityControl->setCurrentIndex( m_math->get_q_control_index() );
	ui.doubleSpinBox_QualityThreshold1->setValue( in_params->quality_threshold1 );
	ui.doubleSpinBox_QualityThreshold2->setValue( in_params->quality_threshold2 );

	//ui.checkBox_SwapDec->setChecked( swap_dec );
	ui.lineEdit_DriftFileName->setEnabled( ui.checkBox_SaveLog->isChecked() );

	ui.l_RecommendedGain->setText( tr("P:") + QString().setNum(in_params->guiding_normal_coef, 'f', 2 ) );

	ui.spinBox_GuideRate->setValue( in_params->guiding_rate );

	// info params...
	ui.l_Focal->setText( str.setNum( (int)info_params.focal) );
	ui.l_Aperture->setText( str.setNum( (int)info_params.aperture) );
	ui.l_FbyD->setText( QString().setNum( info_params.focal_ratio, 'f', 1) );
	str = QString().setNum(info_params.fov_wd, 'f', 1) + "x" + QString().setNum(info_params.fov_ht, 'f', 1);
	ui.l_FOV->setText( str );
	ui.l_Status->setText( QString() );
	ui.l_Status->setPalette( palette() );
	ui.l_Status->setMargin(2);

	ui.groupBox_DirRA->setChecked( in_params->enabled_dir[lg_math::RA] );
	ui.checkBox_DirRAPlus->setChecked( in_params->enabled_dir_sign[lg_math::RA][lg_math::SGN_POS] );
	ui.checkBox_DirRAMinus->setChecked( in_params->enabled_dir_sign[lg_math::RA][lg_math::SGN_NEG] );

	ui.groupBox_DirDEC->setChecked( in_params->enabled_dir[lg_math::DEC] );
	ui.checkBox_DirDECPlus->setChecked( in_params->enabled_dir_sign[lg_math::DEC][lg_math::SGN_POS] );
	ui.checkBox_DirDECMinus->setChecked( in_params->enabled_dir_sign[lg_math::DEC][lg_math::SGN_NEG] );

	ui.checkBox_AverageFrames->setChecked( in_params->average );
	ui.checkBox_normalizeGain->setChecked( in_params->normalize_gain );

	ui.spinBox_AccFramesRA->setValue( (int)in_params->accum_frame_cnt[lg_math::RA] );
	ui.spinBox_AccFramesDEC->setValue( (int)in_params->accum_frame_cnt[lg_math::DEC] );

	update_gains();

	ui.spinBox_MaxPulseRA->setValue( in_params->max_pulse_length[lg_math::RA] );
	ui.spinBox_MaxPulseDEC->setValue( in_params->max_pulse_length[lg_math::DEC] );

	ui.spinBox_MinPulseRA->setValue( in_params->min_pulse_length[lg_math::RA] );
	ui.spinBox_MinPulseDEC->setValue( in_params->min_pulse_length[lg_math::DEC] );

	ui.l_DeltaRA->setText(QString().setNum(out_params->delta[lg_math::RA], 'f', 2) );
	ui.l_DeltaDEC->setText(QString().setNum(out_params->delta[lg_math::DEC], 'f', 2) );

	ui.l_PulseRA->setText(QString().setNum(out_params->pulse_length[lg_math::RA]) );
	ui.l_PulseDEC->setText(QString().setNum(out_params->pulse_length[lg_math::DEC]) );

	ui.l_ErrRA->setText( QString().setNum(out_params->sigma[lg_math::RA]) );
	ui.l_ErrDEC->setText( QString().setNum(out_params->sigma[lg_math::DEC]) );

	ui.l_Quality->setText( QString().setNum(out_params->quality, 'f', 1) );
}


void guider::update_gains( void )
{
	if( !m_math )
		return;
	const lg_math::cproc_in_params *in_params = m_math->get_in_params();

	ui.spinBox_PropGainRA->setValue( in_params->normalize_gain ? in_params->proportional_gain[lg_math::RA] / in_params->guiding_normal_coef : in_params->proportional_gain[lg_math::RA]);
	ui.spinBox_PropGainDEC->setValue(in_params->normalize_gain ? in_params->proportional_gain[lg_math::DEC] / in_params->guiding_normal_coef : in_params->proportional_gain[lg_math::DEC]);
	ui.spinBox_IntGainRA->setValue(in_params->normalize_gain ? in_params->integral_gain[lg_math::RA] / in_params->guiding_normal_coef : in_params->integral_gain[lg_math::RA]);
	ui.spinBox_IntGainDEC->setValue(in_params->normalize_gain ? in_params->integral_gain[lg_math::DEC] / in_params->guiding_normal_coef : in_params->integral_gain[lg_math::DEC]);
	ui.spinBox_DerGainRA->setValue(in_params->normalize_gain ? in_params->derivative_gain[lg_math::RA] / in_params->guiding_normal_coef : in_params->derivative_gain[lg_math::RA]);
	ui.spinBox_DerGainDEC->setValue(in_params->normalize_gain ? in_params->derivative_gain[lg_math::DEC] / in_params->guiding_normal_coef : in_params->derivative_gain[lg_math::DEC]);
}


void guider::onXscaleChanged( int i )
{
	int rx, ry;
	int x_range = i*m_drift_graph->get_gridx_N();

	m_drift_graph->get_visible_ranges( &rx, &ry );
	m_drift_graph->set_visible_ranges( x_range, ry );

	// refresh if not started
	if( !is_started )
	{
		m_drift_graph->on_paint();
		m_drift_out->update();
	}
	m_drift_view_params->drift_graph_xrange = x_range;
}


void guider::onYscaleChanged( int i )
{
	int rx, ry;
	int y_range =i*m_drift_graph->get_gridy_N();

	m_drift_graph->get_visible_ranges( &rx, &ry );
	m_drift_graph->set_visible_ranges( rx, y_range );

	// refresh if not started
	if( !is_started )
	{
		m_drift_graph->on_paint();
		m_drift_out->update();
	}
	m_drift_view_params->drift_graph_yrange = y_range;
}


void guider::onSquareSizeChanged( int index )
{
	if( m_math )
		m_math->resize_square( index );
}


void guider::onThresholdChanged( int index )
{
	if( m_math )
		m_math->set_square_algorithm_index( index );
}


void guider::onSwapDEC( int state )
{
	(void)state;
	m_driver->swap_dec_bits();
	log_i( "DEC control bits swapped" );
}

void guider::onNormalizeGain( int state )
{
	lg_math::cproc_in_params *in_params;

	if( !m_math ) return;

	in_params = m_math->get_in_params();

	bool is_checked = (state != Qt::Unchecked);
	// for some reson state == 2 if cheked, need to cast to bool to compare!
	if(( in_params->normalize_gain == is_checked ) || (in_params->guiding_normal_coef < 0.001)) return;

	in_params->normalize_gain = is_checked;

	/*
	if( in_params->normalize_gain ) {
		in_params->proportional_gain[RA] = in_params->proportional_gain[RA] / in_params->guiding_normal_coef;
		in_params->proportional_gain[DEC] = in_params->proportional_gain[DEC] / in_params->guiding_normal_coef;
		in_params->integral_gain[RA] = in_params->integral_gain[RA] / in_params->guiding_normal_coef;
		in_params->integral_gain[DEC] = in_params->integral_gain[DEC] / in_params->guiding_normal_coef;
		in_params->derivative_gain[RA] = in_params->derivative_gain[RA] / in_params->guiding_normal_coef;
		in_params->derivative_gain[DEC] = in_params->derivative_gain[DEC] / in_params->guiding_normal_coef;
	} else {
		in_params->proportional_gain[RA] = in_params->proportional_gain[RA] * in_params->guiding_normal_coef;
		in_params->proportional_gain[DEC] = in_params->proportional_gain[DEC] * in_params->guiding_normal_coef;
		in_params->integral_gain[RA] = in_params->integral_gain[RA] * in_params->guiding_normal_coef;
		in_params->integral_gain[DEC] = in_params->integral_gain[DEC] * in_params->guiding_normal_coef;
		in_params->derivative_gain[RA] = in_params->derivative_gain[RA] * in_params->guiding_normal_coef;
		in_params->derivative_gain[DEC] = in_params->derivative_gain[DEC] * in_params->guiding_normal_coef;
	}
	*/

	update_gains();
	//m_math->set_in_params(in_params);
}

void guider::onSaveLog( int state )
{
	save_drift = (state == Qt::Checked);
	ui.lineEdit_DriftFileName->setEnabled( save_drift );
}


void guider::onFileNameChanged()
{
	ui.lineEdit_DriftFileName->setText( ui.lineEdit_DriftFileName->text().trimmed() );
}


void guider::onQualityControlChanged( int index )
{
	if( !m_math )
		return;

	m_math->set_q_control_index( index );
}


// params changing stuff
void guider::onInfoRateChanged( double val )
{
	if( !m_math )
		return;

	lg_math::cproc_in_params *in_params = m_math->get_in_params();

	in_params->guiding_rate = val;
	in_params->guiding_normal_coef = m_math->precalc_proportional_gain(in_params->guiding_rate);

	ui.l_RecommendedGain->setText( tr("P:") + QString().setNum(in_params->guiding_normal_coef, 'f', 2 ) );

	update_gains();
}


void guider::onEnableDirRA( bool on )
{
	if( !m_math )
		return;

	lg_math::cproc_in_params *in_params = m_math->get_in_params();
	in_params->enabled_dir[lg_math::RA] = on;
	m_math->calc_dir_checker();
}


void guider::onEnableDirDEC( bool on )
{
	if( !m_math )
		return;

	lg_math::cproc_in_params *in_params = m_math->get_in_params();
	in_params->enabled_dir[lg_math::DEC] = on;
	m_math->calc_dir_checker();
}


void guider::onEnableDirRAPlus( int state )
{
	if( !m_math )
		return;

	lg_math::cproc_in_params *in_params = m_math->get_in_params();
	in_params->enabled_dir_sign[lg_math::RA][lg_math::SGN_POS] = (state == Qt::Checked);
	m_math->calc_dir_checker();
}


void guider::onEnableDirRAMinus( int state )
{
	if( !m_math )
		return;

	lg_math::cproc_in_params *in_params = m_math->get_in_params();
	in_params->enabled_dir_sign[lg_math::RA][lg_math::SGN_NEG] = (state == Qt::Checked);
	m_math->calc_dir_checker();
}


void guider::onEnableDirDECPlus( int state )
{
	if( !m_math )
		return;

	lg_math::cproc_in_params *in_params = m_math->get_in_params();
	in_params->enabled_dir_sign[lg_math::DEC][lg_math::SGN_POS] = (state == Qt::Checked);
	m_math->calc_dir_checker();
}


void guider::onEnableDirDECMinus( int state )
{
	if( !m_math )
		return;

	lg_math::cproc_in_params *in_params = m_math->get_in_params();
	in_params->enabled_dir_sign[lg_math::DEC][lg_math::SGN_NEG] = (state == Qt::Checked);
	m_math->calc_dir_checker();
}


void guider::onInputParamChanged()
{
	QObject *obj;
	QSpinBox *pSB;
	QDoubleSpinBox *pDSB;
	lg_math::cproc_in_params *in_params;

	if( !m_math )
		return;

	obj = sender();

	in_params = m_math->get_in_params();

	if( (pSB = dynamic_cast<QSpinBox *>(obj)) )
	{
		if( pSB == ui.spinBox_AccFramesRA )
			in_params->accum_frame_cnt[lg_math::RA] = pSB->value();
		else
		if( pSB == ui.spinBox_AccFramesDEC )
			in_params->accum_frame_cnt[lg_math::DEC] = pSB->value();
		else
		if( pSB == ui.spinBox_MaxPulseRA )
			in_params->max_pulse_length[lg_math::RA] = pSB->value();
		else
		if( pSB == ui.spinBox_MaxPulseDEC )
			in_params->max_pulse_length[lg_math::DEC] = pSB->value();
		else
		if( pSB == ui.spinBox_MinPulseRA )
			in_params->min_pulse_length[lg_math::RA] = pSB->value();
		else
		if( pSB == ui.spinBox_MinPulseDEC )
			in_params->min_pulse_length[lg_math::DEC] = pSB->value();
	}
	else
	if( (pDSB = dynamic_cast<QDoubleSpinBox *>(obj)) )
	{
		if( pDSB == ui.spinBox_PropGainRA )
			in_params->proportional_gain[lg_math::RA] = in_params->normalize_gain ? pDSB->value() * in_params->guiding_normal_coef : pDSB->value();
		else
		if( pDSB == ui.spinBox_PropGainDEC )
			in_params->proportional_gain[lg_math::DEC] = in_params->normalize_gain ? pDSB->value() * in_params->guiding_normal_coef : pDSB->value();
		else
		if( pDSB == ui.spinBox_IntGainRA )
			in_params->integral_gain[lg_math::RA] = in_params->normalize_gain ? pDSB->value() * in_params->guiding_normal_coef : pDSB->value();
		else
		if( pDSB == ui.spinBox_IntGainDEC )
			in_params->integral_gain[lg_math::DEC] = in_params->normalize_gain ? pDSB->value() * in_params->guiding_normal_coef : pDSB->value();
		else
		if( pDSB == ui.spinBox_DerGainRA )
			in_params->derivative_gain[lg_math::RA] = in_params->normalize_gain ? pDSB->value() * in_params->guiding_normal_coef : pDSB->value();
		else
		if( pDSB == ui.spinBox_DerGainDEC )
			in_params->derivative_gain[lg_math::DEC] = in_params->normalize_gain ? pDSB->value() * in_params->guiding_normal_coef : pDSB->value();
		else
		if( pDSB == ui.doubleSpinBox_QualityThreshold1 )
			in_params->quality_threshold1 = pDSB->value();
		else
		if( pDSB == ui.doubleSpinBox_QualityThreshold2 )
			in_params->quality_threshold2 = pDSB->value();
	}
}





// processing stuff
void guider::onStartStopButtonClick()
{
	int res;

	assert( m_math );

	// start
	if( !is_started )
	{
		// start log
		if( save_drift )
		{
			res = fio::check_file_name( ui.lineEdit_DriftFileName->text().toAscii().data() );
			switch( res )
			{
			case FIO_EXIST:
				u_msg( "File '%s' exists", ui.lineEdit_DriftFileName->text().toAscii().data() );
				return;
			case FIO_ERROR:
				u_msg( "Unable to write file '%s'. Check permissions", ui.lineEdit_DriftFileName->text().toAscii().data() );
				return;
			}

			m_logger->start( ui.lineEdit_DriftFileName->text().toAscii().data() );
		}
		ui.pushButton_StartStop->setText( tr("Stop") );

		quality_rate   = -1;
		guiding_stable = -1;

		m_drift_graph->reset_data();
		m_math->start();
		is_started = true;
	}
	// stop
	else
	{
		m_math->stop();
		// stop pulses immediately
		m_driver->reset();

		// stop log
		m_logger->stop();

		is_started = false;

		ui.checkBox_SaveLog->setChecked( false );
		ui.pushButton_StartStop->setText( tr("Start") );
	}

	// check for status change
	{
		const std::pair< enum lg_math::cgmath::status_level, std::string > *status = m_math->get_status_info_for_key( &m_status_key );
		if( status ) update_status( status->first, status->second );
	}
}


void guider::guide( void )
{
	const lg_math::cproc_out_params *out = NULL;
	QString str;
	uint32_t tick = 0;
	double drift_x = 0, drift_y = 0;

	assert( m_math );

	// calc math. it tracks square
	m_math->do_processing();

	if( !isVisible() || !is_started )
		return;

	// do pulse
	out = m_math->get_out_params();
	m_driver->do_pulse( out->pulse_dir[lg_math::RA], out->pulse_length[lg_math::RA], out->pulse_dir[lg_math::DEC], out->pulse_length[lg_math::DEC] );

	m_math->get_star_drift( &drift_x, &drift_y );

	m_drift_graph->add_point( drift_x, drift_y );

	tick = m_math->get_ticks();

	if( tick & 1 )
	{
		// draw some params in window
		ui.l_DeltaRA->setText(str.setNum(out->delta[lg_math::RA], 'f', 2) );
		ui.l_DeltaDEC->setText(str.setNum(out->delta[lg_math::DEC], 'f', 2) );

		ui.l_PulseRA->setText(str.setNum(out->pulse_length[lg_math::RA]) );
		ui.l_PulseDEC->setText(str.setNum(out->pulse_length[lg_math::DEC]) );

		ui.l_ErrRA->setText( str.setNum(out->sigma[lg_math::RA], 'f', 2) );
		ui.l_ErrDEC->setText( str.setNum(out->sigma[lg_math::DEC], 'f', 2) );

		ui.l_Quality->setText( str.setNum(out->quality, 'f', 1) );
	}

	// do log
	if( save_drift )
		m_logger->add_drift( drift_x, drift_y );

	// broadcast drift
	if( m_common_params.udp_send_drift_data )
		server::send_bcast_msg( BCM_DRIFT_DATA, "%.2lf %.2lf", drift_x, drift_y );

	// determine miscellaneous events
	check_for_events();

	// skip half frames
	if( half_refresh_rate && (tick & 1) )
		return;

	m_drift_graph->on_paint();
	m_drift_out->update();

}


void guider::check_for_events( void )
{
	int q_rate = -1;
	int stability = -1;

	// Quality control
	if( m_math->get_q_control_index() != lg_math::Q_CTRL_OFF && (q_rate = m_math->calc_quality_rate()) != quality_rate )
	{
		quality_rate = q_rate;
		switch( quality_rate )
		{
		case lg_math::QUALITY_OK:
			server::send_bcast_msg( BCM_NORMAL_IMAGE_QUALITY );
			update_status( lg_math::cgmath::STATUS_LEVEL_INFO, "Guiding..." );
			break;
		case lg_math::QUALITY_NOTIFY:
			server::send_bcast_msg( BCM_LOW_IMAGE_QUALITY );
			update_status( lg_math::cgmath::STATUS_LEVEL_WARNING, "Quality is low." );
			break;
		case lg_math::QUALITY_CRITICAL:
			server::send_bcast_msg( BCM_CRITICAL_IMAGE_QUALITY );
			if( m_math->get_q_control_index() == lg_math::Q_CTRL_FULL )
			{
				log_i( "quality is too low, stopping guiding" );
				onStartStopButtonClick();
				// forcefully write to status - otherwise it will not show
				update_status( lg_math::cgmath::STATUS_LEVEL_ERROR, "Quality is too low, guiding stopped");
				return;
			}
			update_status( lg_math::cgmath::STATUS_LEVEL_ERROR, "Quality is too low." );
			break;
		}
	}
	// check stability
	if( m_common_params.udp_send_guiding_stability )
	{
		stability = m_math->calc_stability_rate();
		if( guiding_stable != stability )
		{
			guiding_stable = stability;
			switch( guiding_stable )
			{
			case lg_math::STABILITY_GOOD:
				server::send_bcast_msg( BCM_GUIDING_STABLE );
				break;
			case lg_math::STABILITY_BAD:
				server::send_bcast_msg( BCM_GUIDING_UNSTABLE );
				break;
			}
		}
	}

	// check for status change
	{
		const std::pair< enum lg_math::cgmath::status_level, std::string > *status = m_math->get_status_info_for_key( &m_status_key );
		if( status ) update_status( status->first, status->second );
	}
}


void guider::update_status( enum lg_math::cgmath::status_level level, const std::string &txt )
{
	static QColor bg_color_err( 255, 32, 32, 255 );
	static QColor bg_color_wrn( 255, 128, 0, 255 );
	static QColor bg_color_inf( 0, 0, 0, 0 );

	QPalette pal( ui.l_Status->palette() );
	switch( level )
	{
	case lg_math::cgmath::STATUS_LEVEL_ERROR:
		pal.setColor( QPalette::Background, bg_color_err );
		break;
	case lg_math::cgmath::STATUS_LEVEL_WARNING:
		pal.setColor( QPalette::Background, bg_color_wrn );
		break;
	default:
		pal.setColor( QPalette::Background, bg_color_inf );
	}
	ui.l_Status->setPalette( pal );
	ui.l_Status->setText( QString::fromUtf8( txt.data(), txt.size() ) );
}
