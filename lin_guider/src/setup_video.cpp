/*
 * setup_video.cpp
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
#include <unistd.h>

#include <map>
#include <string>

#include "lin_guider.h"
#include "setup_video.h"
#include "utils.h"


setup_video::setup_video(lin_guider *parent)
    : QDialog(parent), pmain_wnd(parent)
{
	ui.setupUi(this);

	setWindowTitle( tr("Camera Settings") );

	// init device list
	ui.comboBox_DeviceList->clear();
	int cnt = ARRAY_SIZE( video_drv::device_desc_list );

	for( int i = 0;i < cnt;i++ )
	{
		QString str = QString( video_drv::device_desc_list[i].desc );
		ui.comboBox_DeviceList->addItem( str, (int)video_drv::device_desc_list[i].type );
	}
	ui.comboBox_DeviceList->setCurrentIndex( 0 );

	const std::map< unsigned int, const std::string > &ext_ctls = pmain_wnd->m_video->get_cam_ext_ctl_list();
	for( std::map< unsigned int, const std::string >::const_iterator it = ext_ctls.begin();
		it != ext_ctls.end();++it )
	{
		ui.comboBox_ExtParamList->addItem( QString::fromStdString( it->second ), it->first );
	}
	if( ext_ctls.empty() )
	{
		ui.label_ExtParams->setEnabled( false );
		ui.comboBox_ExtParamList->setEnabled( false );
		ui.spinBox_ExtValue->setEnabled( false );
		ui.horizontalSlider_ExtValue->setEnabled( false );
	}
	else
		ui.comboBox_ExtParamList->setCurrentIndex( 0 );

	// connect...
	connect( ui.spinBox_Aperture,	   		SIGNAL(valueChanged(double)),		this, SLOT(onApertureChanged(double)) );
	connect( ui.spinBox_Focal,		   		SIGNAL(valueChanged(double)),		this, SLOT(onFocalChanged(double)) );
	connect( ui.spinBox_CCD_Width, 	   		SIGNAL(valueChanged(int)),			this, SLOT(onMatrixWidthChanged(int)) );
	connect( ui.spinBox_CCD_Height,    		SIGNAL(valueChanged(int)),			this, SLOT(onMatrixHeightChanged(int)) );
	connect( ui.spinBox_PixelWidth,	   		SIGNAL(valueChanged(double)),		this, SLOT(onPixeWidthChanged(double)) );
	connect( ui.spinBox_PixelHeight,   		SIGNAL(valueChanged(double)),		this, SLOT(onPixeHeightChanged(double)) );
	connect( ui.checkBox_AutoSensorInfo,    SIGNAL(stateChanged(int)),          this, SLOT(onAutoInfoChecked(int)) );
	connect( ui.comboBox_DeviceList,   		SIGNAL(activated(int)),         	this, SLOT(onDeviceListChanged(int)) );
	connect( ui.comboBox_FPS, 		   		SIGNAL(activated(int)), 			this, SLOT(onFPSChanged(int)) );
	connect( ui.comboBox_FrameSize,    		SIGNAL(activated(int)), 			this, SLOT(onFrameSizeChanged(int)) );
	connect( ui.checkBox_BW,                SIGNAL(stateChanged(int)),          this, SLOT(onBWChecked(int)) );
	connect( ui.checkBox_HalfOutFPS,        SIGNAL(stateChanged(int)),          this, SLOT(onHalfFPSChecked(int)) );
	connect( ui.checkBox_UseCalibration,    SIGNAL(stateChanged(int)),          this, SLOT(onUseCalibrationChecked(int)) );
	connect( ui.checkBox_AutoGain,     		SIGNAL(stateChanged(int)),			this, SLOT(onAutogainChanged(int)) );
	connect( ui.spinBox_Gain,		   		SIGNAL(valueChanged(int)), 			this, SLOT(onSpinGainChanged(int)) );
	connect( ui.horizontalSlider_Gain, 		SIGNAL(valueChanged(int)), 			this, SLOT(onSliderGainChanged(int)) );
	connect( ui.spinBox_Expo,		   		SIGNAL(valueChanged(int)), 			this, SLOT(onSpinExpoChanged(int)) );
	connect( ui.horizontalSlider_Expo, 		SIGNAL(valueChanged(int)), 			this, SLOT(onSliderExpoChanged(int)) );
	connect( ui.comboBox_ExtParamList, 		SIGNAL(currentIndexChanged(int)), 	this, SLOT(onExtParamChanged(int)) );
	connect( ui.spinBox_ExtValue,	   		SIGNAL(valueChanged(int)), 			this, SLOT(onSpinExtParamChanged(int)) );
	connect( ui.horizontalSlider_ExtValue,	SIGNAL(valueChanged(int)), 			this, SLOT(onSliderExtParamChanged(int)) );

	connect( ui.pushButton_OK, 				SIGNAL(clicked()), 					this, SLOT(onOkButtonClick()) );
	connect( ui.pushButton_Cancel, 			SIGNAL(clicked()), 					this, SLOT(onCancelButtonClick()) );

	first_show = true;
	applied = false;
}

setup_video::~setup_video()
{
}


void setup_video::showEvent( QShowEvent * event )
{
 video_drv::cam_control_t *control = NULL;

	is_filling_ui = false;

	if( event->spontaneous() )
		return;

	// check validity
	if( !pmain_wnd )
	{
		u_msg("setup_video::showEvent: main wnd not initialized");
		event->ignore();
		return;
	}

	params			    = pmain_wnd->m_video->get_capture_params();
	next_params			= pmain_wnd->m_video->get_next_params();

	guider_params 		= pmain_wnd->m_guider_params;
	half_refresh_rate   = pmain_wnd->m_ui_params.half_refresh_rate;
	snprintf( dev_name_video, sizeof(dev_name_video), "%s", pmain_wnd->dev_name_video );
	capture_sz.x		= next_params.width ? next_params.width : params.width;
	capture_sz.y		= next_params.height ? next_params.height : params.height;

	control = pmain_wnd->m_video->get_cam_control( video_drv::CI_GAIN );
	if( control )
	{
		params.gain = params.gain < control->min ? control->min : params.gain;
		params.gain = params.gain > control->max ? control->max : params.gain;
	}
	control = pmain_wnd->m_video->get_cam_control( video_drv::CI_EXPO );
	if( control )
	{
		params.exposure = params.exposure < control->min ? control->min : params.exposure;
		params.exposure = params.exposure > control->max ? control->max : params.exposure;
	}
	for( std::map< unsigned int, int >::iterator it = params.ext_params.begin();
		it != params.ext_params.end();++it )
	{
		control = pmain_wnd->m_video->get_cam_control( video_drv::CI_EXTCTL, it->first );
		if( control )
		{
			it->second = it->second < control->min ? control->min : it->second;
			it->second = it->second > control->max ? control->max : it->second;
		}
	}

	fill_interface();

	applied = false;
}


void setup_video::closeEvent ( QCloseEvent * /*event*/ )
{
	//if( applied )
	{
		pmain_wnd->m_guider_params = guider_params;
		pmain_wnd->m_ui_params.half_refresh_rate = half_refresh_rate;
		snprintf( pmain_wnd->dev_name_video, sizeof(pmain_wnd->dev_name_video), "%s", dev_name_video );

		pmain_wnd->guider_wnd->set_half_refresh_rate( half_refresh_rate );
		pmain_wnd->m_math->set_guider_params( guider_params.ccd_pixel_width, guider_params.ccd_pixel_height, guider_params.aperture, guider_params.focal );

		pmain_wnd->m_capture_params = params;
		pmain_wnd->m_video->set_use_calibration( params.use_calibration );
	}
}


void setup_video::hideEvent ( QHideEvent * event )
{
	if( event->spontaneous() )
		return;

	close();
}


void setup_video::fill_interface( void )
{
 int i, cur_frm, cur_fps;
 video_drv::current_format_state_t format_state;
 video_drv::cam_control_t *control = NULL;

	is_filling_ui = true;

	ui.spinBox_Aperture->setValue( guider_params.aperture );
	ui.spinBox_Focal->setValue( guider_params.focal );

	const struct video_drv::sensor_info_s &si = pmain_wnd->m_video->get_sensor_info();
	if( si.is_available && guider_params.auto_info )
		fill_sensor_info();
	else
	{
		ui.spinBox_CCD_Width->setValue( guider_params.matrix_width );
		ui.spinBox_CCD_Height->setValue( guider_params.matrix_height );
		ui.spinBox_PixelWidth->setValue( guider_params.ccd_pixel_width );
		ui.spinBox_PixelHeight->setValue( guider_params.ccd_pixel_height );
	}
	ui.checkBox_AutoSensorInfo->setChecked( si.is_available && guider_params.auto_info );
	ui.checkBox_AutoSensorInfo->setEnabled( si.is_available );

	//select from device list
	int actual_dev_type = next_params.type ? next_params.type : params.type;
	for( i = 0;i < ui.comboBox_DeviceList->count();i++ )
	{
		if( ui.comboBox_DeviceList->itemData(i).toInt() == actual_dev_type )
		{
			ui.comboBox_DeviceList->setCurrentIndex( i );
			break;
		}
	}

	ui.lineEdit_VideoDevice->setText( QString(dev_name_video) );
	update_dev_string_visibility( actual_dev_type );
	ui.checkBox_BW->setChecked( guider_params.bw_video );
	ui.checkBox_HalfOutFPS->setChecked( half_refresh_rate );
	ui.checkBox_UseCalibration->setChecked( params.use_calibration );
	ui.checkBox_UseCalibration->setEnabled( pmain_wnd->m_video->is_calibrated() );

	format_state = pmain_wnd->m_video->get_current_format_params();
	if( first_show )
	{
		ui.comboBox_FrameSize->clear();
		ui.comboBox_FPS->clear();

		if( format_state.format_desc )
		{
			// fill frame list
			for( i = 0;format_state.format_desc->frame_table[i].size.x > 0 && i < MAX_FMT;i++ )
				ui.comboBox_FrameSize->addItem( QString().setNum(format_state.format_desc->frame_table[i].size.x) + "x" + QString().setNum(format_state.format_desc->frame_table[i].size.y) );

			// fill fps list
			cur_frm = format_state.frame_idx < 0 ? 0 : format_state.frame_idx;
			for( i = 0;format_state.format_desc->frame_table[cur_frm].fps_table[i].denominator > 0 && i < MAX_FMT;i++ )
				ui.comboBox_FPS->addItem( QString().setNum( video_drv::time_fract::to_msecs( format_state.format_desc->frame_table[cur_frm].fps_table[i] ) / 1000.0, 'g', 2 ) );

		}

		first_show = false;
	}

	cur_frm = cur_fps = -1;
	if( format_state.format_desc )
	{
		// frame size
		for( i = 0;format_state.format_desc->frame_table[i].size.x > 0 && i < MAX_FMT;i++ )
			if( format_state.format_desc->frame_table[i].size.x == capture_sz.x &&  format_state.format_desc->frame_table[i].size.y == capture_sz.y )
				cur_frm = i;

		// fps
		cur_fps = format_state.fps_idx;
	}
	ui.comboBox_FrameSize->setCurrentIndex( cur_frm );
	ui.comboBox_FPS->setCurrentIndex( cur_fps );

	// get autogain
	control = pmain_wnd->m_video->get_cam_control( video_drv::CI_AUTOGAIN );
	if( control && control->enabled )
	{
		ui.checkBox_AutoGain->setEnabled( control->enabled ? true : false );
		ui.checkBox_AutoGain->setChecked( params.autogain );
	}
	else
	{
		ui.checkBox_AutoGain->setEnabled( false );
		ui.checkBox_AutoGain->setChecked( false );
	}

	// get gain
	control = pmain_wnd->m_video->get_cam_control( video_drv::CI_GAIN );
	if( control && control->enabled )
	{
		bool en = control->enabled ? true : false;
		if( en && ui.checkBox_AutoGain->isChecked() )
			en = false;
		ui.label_Gain->setEnabled( en );
		ui.spinBox_Gain->setEnabled( en );
		ui.spinBox_Gain->setRange( control->min, control->max );
		ui.spinBox_Gain->setValue( params.gain );
		ui.horizontalSlider_Gain->setEnabled( en );
		ui.horizontalSlider_Gain->setRange( control->min, control->max );
		ui.horizontalSlider_Gain->setValue( params.gain );
	}
	else
	{
		ui.label_Gain->setEnabled( false );
		ui.spinBox_Gain->setEnabled( false );
		ui.spinBox_Gain->setRange( 0, 0 );
		ui.spinBox_Gain->setValue( params.gain );
		ui.horizontalSlider_Gain->setEnabled( false );
		ui.horizontalSlider_Gain->setRange( 0, 0 );
		ui.horizontalSlider_Gain->setValue( 0 );
	}

	// get exposure
	control = pmain_wnd->m_video->get_cam_control( video_drv::CI_EXPO );
	if( control && control->enabled )
	{
		bool en = control->enabled ? true : false;
		ui.label_Expo->setEnabled( en );
		ui.spinBox_Expo->setEnabled( en );
		ui.spinBox_Expo->setRange( control->min, control->max );
		ui.spinBox_Expo->setValue( params.exposure );
		ui.horizontalSlider_Expo->setEnabled( en );
		ui.horizontalSlider_Expo->setRange( control->min, control->max );
		ui.horizontalSlider_Expo->setValue( params.exposure );
	}
	else
	{
		ui.label_Expo->setEnabled( false );
		ui.spinBox_Expo->setEnabled( false );
		ui.spinBox_Expo->setRange( 0, 0 );
		ui.spinBox_Expo->setValue( params.exposure );
		ui.horizontalSlider_Expo->setEnabled( false );
		ui.horizontalSlider_Expo->setRange( 0, 0 );
		ui.horizontalSlider_Expo->setValue( 0 );
	}
	if( !pmain_wnd->m_video->get_cam_ext_ctl_list().empty() )
	{
		onExtParamChanged( ui.comboBox_ExtParamList->currentIndex() );
	}

	is_filling_ui = false;
}


void setup_video::fill_sensor_info( void )
{
	const struct video_drv::sensor_info_s &si = pmain_wnd->m_video->get_sensor_info();
	if( !si.is_available )
		return;
	ui.spinBox_CCD_Width->setValue( si.matrix_width );
	ui.spinBox_CCD_Height->setValue( si.matrix_height );
	ui.spinBox_PixelWidth->setValue( si.pixel_width );
	ui.spinBox_PixelHeight->setValue( si.pixel_height );
}


void setup_video::update_dev_string_visibility( int dev_type )
{
	ui.lineEdit_VideoDevice->setVisible( false );
	ui.label_Information->setText( QString() );

	int cnt = ARRAY_SIZE( video_drv::device_desc_list );
	for( int i = 0;i < cnt;i++ )
		if( dev_type == video_drv::device_desc_list[i].type )
		{
			ui.lineEdit_VideoDevice->setVisible( video_drv::device_desc_list[i].show_dev_string_ui );
			if( video_drv::device_desc_list[i].hyper_info )
				ui.label_Information->setText( QString::fromUtf8(video_drv::device_desc_list[i].hyper_info) );
			if( video_drv::device_desc_list[i].info )
				log_i( "Video device info: \"%s\"", video_drv::device_desc_list[i].info );
		}
}


double setup_video::calc_arc( int pix, double pix_sz, double focal )
{
	if( focal == 0 )
		return -1;

 return  206264.8062470963552 * (double)pix * pix_sz / 1000.0 / focal;
}


void setup_video::onApertureChanged( double val )
{
 QString str;

	ui.l_FbyD->setText( str.setNum( ui.spinBox_Focal->value() / val, 'f', 1) );
	ui.l_Resolution->setText( str.setNum( 120.0 / val, 'f', 2) );
}


void setup_video::onFocalChanged( double val )
{
 QString str;

	ui.l_FbyD->setText( str.setNum( val / ui.spinBox_Aperture->value(), 'f', 1) );
	ui.l_PixResolution->setText( QString().setNum(calc_arc( 1, ui.spinBox_PixelWidth->value(), val ), 'f', 2) + "x" +
									QString().setNum(calc_arc( 1, ui.spinBox_PixelHeight->value(), val ), 'f', 2 ) );
	ui.l_FOV->setText( QString().setNum(calc_arc( ui.spinBox_CCD_Width->value(), ui.spinBox_PixelWidth->value(), val ) / 60.0, 'f', 1) + "x" +
			 QString().setNum(calc_arc( ui.spinBox_CCD_Height->value(), ui.spinBox_PixelHeight->value(), val ) / 60.0, 'f', 1 ) );

}


void setup_video::onMatrixWidthChanged( int val )
{
	ui.l_PixResolution->setText( QString().setNum(calc_arc( 1, ui.spinBox_PixelWidth->value(), ui.spinBox_Focal->value() ), 'f', 2) + "x" +
								QString().setNum(calc_arc( 1, ui.spinBox_PixelHeight->value(), ui.spinBox_Focal->value() ), 'f', 2 ) );
	ui.l_FOV->setText( QString().setNum(calc_arc( val, ui.spinBox_PixelWidth->value(), ui.spinBox_Focal->value() ) / 60.0, 'f', 1) + "x" +
					QString().setNum(calc_arc( ui.spinBox_CCD_Height->value(), ui.spinBox_PixelHeight->value(), ui.spinBox_Focal->value() ) / 60.0, 'f', 1 ) );

}


void setup_video::onMatrixHeightChanged( int val )
{
	ui.l_PixResolution->setText( QString().setNum(calc_arc( 1, ui.spinBox_PixelWidth->value(), ui.spinBox_Focal->value() ), 'f', 2) + "x" +
								QString().setNum(calc_arc( 1, ui.spinBox_PixelHeight->value(), ui.spinBox_Focal->value() ), 'f', 1 ) );
	ui.l_FOV->setText( QString().setNum(calc_arc( ui.spinBox_CCD_Width->value(), ui.spinBox_PixelWidth->value(), ui.spinBox_Focal->value() ) / 60.0, 'f', 1) + "x" +
					QString().setNum(calc_arc( val, ui.spinBox_PixelHeight->value(), ui.spinBox_Focal->value() ) / 60.0, 'f', 1 ) );
}


void setup_video::onPixeWidthChanged( double val )
{
	ui.l_PixResolution->setText( QString().setNum(calc_arc( 1, val, ui.spinBox_Focal->value() ), 'f', 2) + "x" +
									QString().setNum(calc_arc( 1, ui.spinBox_PixelHeight->value(), ui.spinBox_Focal->value() ), 'f', 2 ) );
	ui.l_FOV->setText( QString().setNum(calc_arc( ui.spinBox_CCD_Width->value(), val, ui.spinBox_Focal->value() ) / 60.0, 'f', 1) + "x" +
					QString().setNum(calc_arc( ui.spinBox_CCD_Height->value(), ui.spinBox_PixelHeight->value(), ui.spinBox_Focal->value() ) / 60.0, 'f', 1 ) );
}


void setup_video::onPixeHeightChanged( double val )
{
	ui.l_PixResolution->setText( QString().setNum(calc_arc( 1, ui.spinBox_PixelWidth->value(), ui.spinBox_Focal->value() ), 'f', 2) + "x" +
									QString().setNum(calc_arc( 1, val, ui.spinBox_Focal->value() ), 'f', 2 ) );
	ui.l_FOV->setText( QString().setNum(calc_arc( ui.spinBox_CCD_Width->value(), ui.spinBox_PixelWidth->value(), ui.spinBox_Focal->value() ) / 60.0, 'f', 1) + "x" +
					QString().setNum(calc_arc( ui.spinBox_CCD_Height->value(), val, ui.spinBox_Focal->value() ) / 60.0, 'f', 1 ) );
}


void setup_video::onAutoInfoChecked( int state )
{
	if( state == Qt::Checked )
		fill_sensor_info();

	ui.spinBox_CCD_Width->setEnabled( state != Qt::Checked );
	ui.spinBox_CCD_Height->setEnabled( state != Qt::Checked );
	ui.spinBox_PixelWidth->setEnabled( state != Qt::Checked );
	ui.spinBox_PixelHeight->setEnabled( state != Qt::Checked );
}


void setup_video::onDeviceListChanged( int index )
{
 bool ok;

 	int next_type = ui.comboBox_DeviceList->itemData( index ).toInt(&ok);
 	if( next_type == -1 )
 		next_type = params.type;

 	next_params.type = next_type;

 	update_dev_string_visibility( next_params.type );

	if( next_params.type != params.type )
		u_msg( "Restart program to apply changes." );
}


void setup_video::onFPSChanged( int index )
{
 video_drv::current_format_state_t format_state;
 video_drv::time_fract tmp;
 video_drv::post_param_t prm;


 	if( is_filling_ui )
 		return;

 	memset( &prm, 0, sizeof(video_drv::post_param_t) );

 	format_state = pmain_wnd->m_video->get_current_format_params();

	if( index != -1 && format_state.format_desc )
	{
		tmp = format_state.format_desc->frame_table[format_state.frame_idx].fps_table[index];

		if( params.fps != tmp )
		{
			video_drv::param_val_t val;
			val.set( tmp.numerator, tmp.denominator );

			pmain_wnd->m_video->pack_params( video_drv::CI_FPS, val, &prm );
			// send params to video thread
			pmain_wnd->m_video->post_params( prm );

			pmain_wnd->update_sb_video_info( index );
		}

		params.fps = guider_params.fps = tmp;
	}
}


void setup_video::onFrameSizeChanged( int /*index*/ )
{
	// try to interact with camera...
}


void setup_video::onBWChecked( int state )
{
	pmain_wnd->m_guider_params.bw_video = state == Qt::Checked;
}


void setup_video::onHalfFPSChecked( int state )
{
	pmain_wnd->m_ui_params.half_refresh_rate = state == Qt::Checked;
}


void setup_video::onUseCalibrationChecked( int state )
{
	pmain_wnd->m_video->set_use_calibration( state == Qt::Checked );
}


void setup_video::onAutogainChanged ( int state )
{
 video_drv::post_param_t prm;
 video_drv::param_val_t val;
 video_drv::cam_control_t *control = NULL;

	if( is_filling_ui )
		return;

	memset( &prm, 0, sizeof(video_drv::post_param_t) );

	val.set( state == Qt::Checked ? 1 : 0 );
	pmain_wnd->m_video->pack_params( video_drv::CI_AUTOGAIN, val, &prm );

	bool en = state == Qt::Checked ? false : true;

	control = pmain_wnd->m_video->get_cam_control( video_drv::CI_GAIN );
	if( control && control->enabled )
	{
		ui.label_Gain->setEnabled( en );
		ui.spinBox_Gain->setEnabled( en );
		ui.horizontalSlider_Gain->setEnabled( en );

		if( state == Qt::Unchecked )
		{
			val.set( params.gain );
			pmain_wnd->m_video->pack_params( video_drv::CI_GAIN, val, &prm );
		}
	}

	pmain_wnd->m_video->post_params( prm );
}


void setup_video::onSpinGainChanged( int value )
{
	ui.horizontalSlider_Gain->setValue( value );
}


void setup_video::onSliderGainChanged( int value )
{
 video_drv::post_param_t prm;
 video_drv::param_val_t val;

	if( is_filling_ui )
 		return;

	memset( &prm, 0, sizeof(video_drv::post_param_t) );

	params.gain = value;

	val.set( params.gain );
	pmain_wnd->m_video->pack_params( video_drv::CI_GAIN, val, &prm );
	pmain_wnd->m_video->post_params( prm );
	ui.spinBox_Gain->setValue( value );
}


void setup_video::onSpinExpoChanged( int value )
{
	ui.horizontalSlider_Expo->setValue( value );
}


void setup_video::onSliderExpoChanged( int value )
{
 video_drv::post_param_t prm;
 video_drv::param_val_t val;

	if( is_filling_ui )
 		return;

	memset( &prm, 0, sizeof(video_drv::post_param_t) );

	params.exposure = value;

	val.set( params.exposure );
	pmain_wnd->m_video->pack_params( video_drv::CI_EXPO, val, &prm );
	pmain_wnd->m_video->post_params( prm );
	ui.spinBox_Expo->setValue( value );
}


void setup_video::onExtParamChanged( int index )
{
	if( index == -1 )
		return;

	is_filling_ui = true;

	unsigned int ctl_id = ui.comboBox_ExtParamList->itemData( index ).toUInt();
	video_drv::cam_control_t *control = pmain_wnd->m_video->get_cam_control( video_drv::CI_EXTCTL, ctl_id );
	if( control )
	{
		ui.spinBox_ExtValue->setMinimum( control->min );
		ui.spinBox_ExtValue->setMaximum( control->max );
		ui.spinBox_ExtValue->setValue( params.ext_params[ control->id ] );

		ui.horizontalSlider_ExtValue->setMinimum( control->min );
		ui.horizontalSlider_ExtValue->setMaximum( control->max );
		ui.horizontalSlider_ExtValue->setValue( params.ext_params[ control->id ] );
	}

	is_filling_ui = false;
}


void setup_video::onSpinExtParamChanged( int value )
{
	ui.horizontalSlider_ExtValue->setValue( value );
}


void setup_video::onSliderExtParamChanged( int value )
{
	if( is_filling_ui )
		return;

	unsigned int ctl_id = ui.comboBox_ExtParamList->itemData( ui.comboBox_ExtParamList->currentIndex() ).toUInt();
	video_drv::cam_control_t *control = pmain_wnd->m_video->get_cam_control( video_drv::CI_EXTCTL, ctl_id );
	std::map< unsigned int, int >::iterator pit = params.ext_params.find( ctl_id );
	if( control && pit != params.ext_params.end() )
	{
		video_drv::post_param_t prm;
		video_drv::param_val_t val;

		memset( &prm, 0, sizeof(video_drv::post_param_t) );

		pit->second = value;

		val.set( (int)ctl_id, pit->second );
		pmain_wnd->m_video->pack_params( video_drv::CI_EXTCTL, val, &prm );
		pmain_wnd->m_video->post_params( prm );
		ui.spinBox_ExtValue->setValue( value );
	}
}


void setup_video::onOkButtonClick()
{
 char fname[64];
 video_drv::current_format_state_t format_state;

	guider_params.auto_info = ui.checkBox_AutoSensorInfo->isChecked();

	guider_params.aperture = ui.spinBox_Aperture->value();
	guider_params.focal = ui.spinBox_Focal->value();
	guider_params.matrix_width = ui.spinBox_CCD_Width->value();
	guider_params.matrix_height = ui.spinBox_CCD_Height->value();
	guider_params.ccd_pixel_width = ui.spinBox_PixelWidth->value();
	guider_params.ccd_pixel_height = ui.spinBox_PixelHeight->value();

	int actual_dev_type = next_params.type ? next_params.type : params.type;
	if( actual_dev_type > 0 && video_drv::device_desc_list[ actual_dev_type-1 ].show_dev_string_ui )
	{
		snprintf( fname, sizeof(fname), "%s", ui.lineEdit_VideoDevice->text().toAscii().data() );
		if( access( fname, R_OK|W_OK ) != 0 )
		{
			QMessageBox::warning( this, tr("Error"), tr("Selected video device is not available."), QMessageBox::Ok );
			return;
		}
		memmove( dev_name_video, fname, sizeof(dev_name_video) );
	}

	format_state = pmain_wnd->m_video->get_current_format_params();

	if( ui.comboBox_FrameSize->currentIndex() != -1 && format_state.format_desc )
	{
		capture_sz.x = format_state.format_desc->frame_table[ui.comboBox_FrameSize->currentIndex()].size.x;
		capture_sz.y = format_state.format_desc->frame_table[ui.comboBox_FrameSize->currentIndex()].size.y;
		
		next_params.width  = capture_sz.x;
		next_params.height = capture_sz.y;
	}

	// drop if no change
	next_params.type = next_params.type == params.type ? 0: next_params.type;

	pmain_wnd->m_video->set_next_params( next_params );

	guider_params.bw_video = ui.checkBox_BW->isChecked();

	half_refresh_rate = ui.checkBox_HalfOutFPS->isChecked();

	params.use_calibration = ui.checkBox_UseCalibration->isChecked();

/*
	// autogain has higher priority than gain
	if( ui.checkBox_AutoGain->isChecked() != autogain )
	{
		pmain_wnd->video->pack_params( CI_AUTOGAIN, ui.checkBox_AutoGain->isChecked() ? 1 : 0, &prm );
		autogain = ui.checkBox_AutoGain->isChecked();
	}
	else
	{
		pmain_wnd->video->pack_params( CI_GAIN, gain, &prm );
	}
	pmain_wnd->video->pack_params( CI_EXPO, exposure, &prm );
*/

	applied = true;

	close();
}


void setup_video::onCancelButtonClick()
{
	close();
}

