/*
 * params.cpp
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

#include <QSettings>
#include <stdio.h>
#include <unistd.h>

#include <map>

#include "params.h"
#include "utils.h"



params::params()
{
	save_device_cfg = true;	// true if device was successuly initialized

	// guider params
	memset( &m_guider_params, 0, sizeof(guiderparams_t) );
	m_guider_params.aperture = 75.0;
	m_guider_params.focal = 600.0;
	m_guider_params.matrix_width = 640;
	m_guider_params.matrix_height = 480;
	m_guider_params.ccd_pixel_width = m_guider_params.ccd_pixel_height = 5.6;
	m_guider_params.fps = video_drv::time_fract::mk_fps( 1, 10 );
	m_guider_params.bw_video = false;
	snprintf( m_dev_name_video, sizeof(m_dev_name_video)-1, "/dev/video0" );
	snprintf( m_dev_name_io, sizeof(m_dev_name_io)-1, "/dev/parport0" );

	m_ui_params.half_refresh_rate = false;

	m_capture_next_params.width   = 0;
	m_capture_next_params.height  = 0;
	
	// math
	m_math_in_params.reset();

	// io
	memset( &m_device_params, 0, sizeof(io_drv::device_init_params_t) );
	m_device_params.dir_map.bit_direction[0] = io_drv::RA_INC_DIR;
	m_device_params.dir_map.bit_direction[1] = io_drv::DEC_INC_DIR;
	m_device_params.dir_map.bit_direction[2] = io_drv::DEC_DEC_DIR;
	m_device_params.dir_map.bit_direction[3] = io_drv::RA_DEC_DIR;
	m_device_params.dir_map.bit_direction[4] = io_drv::NO_DIR;
	m_device_params.dir_map.bit_direction[5] = io_drv::NO_DIR;
	m_device_params.dir_map.bit_direction[6] = io_drv::NO_DIR;
	m_device_params.dir_map.bit_direction[7] = io_drv::NO_DIR;
	m_device_params.dir_map.is_inverse = false;

	m_device_params.death_time.use = false;
	m_device_params.death_time.delay = 100;

	m_device_params.type = io_drv::DT_NULL;
	m_device_params.next_device_type = -1;

	//calibration
	m_calibration_params.two_axis    = false;
	m_calibration_params.auto_mode   = false;
	m_calibration_params.dift_time   = 25;
	m_calibration_params.frame_count = 10;

	//net
	snprintf( m_net_params.bcast_ip, sizeof(m_net_params.bcast_ip), "127.0.0.1" );
	m_net_params.bcast_port = 5001;

	// common params initialized by ctor()

}


params::~params()
{
}


bool params::load( void )
{
 int i;
 QSettings settings( "GM_software", "lin-guider" );
 QString str;
 bool ok;

 	if( access(settings.fileName().toAscii().data(), R_OK|W_OK) != 0 )
 	{
 		log_e( "Unable to find config file %s\nusing defaults", settings.fileName().toAscii().data() );
 		return false;
 	}

	// device names
	settings.beginGroup("devices");
		str = settings.value("video_device", "/dev/video0").toString();
		snprintf( m_dev_name_video, sizeof(m_dev_name_video), "%s", str.toAscii().data() );
		str = settings.value("io_device", "/dev/parport0").toString();
		snprintf( m_dev_name_io, sizeof(m_dev_name_io), "%s", str.toAscii().data() );
	settings.endGroup();

	// io bit map
	settings.beginGroup("io");
		for( i = 0;i < 8;i++ )
		{
			m_device_params.dir_map.bit_direction[i] = (io_drv::guide_dir)settings.value( QString("dir_bit") + QString().setNum(i) ).toInt(&ok);
			if( m_device_params.dir_map.bit_direction[i] < io_drv::NO_DIR ||
				m_device_params.dir_map.bit_direction[i] > io_drv::DEC_DEC_DIR )
				m_device_params.dir_map.bit_direction[i] = io_drv::NO_DIR;
		}
		m_device_params.dir_map.is_inverse = settings.value( "inverse" ).toBool();
		m_device_params.death_time.use = settings.value("use_deathtime", false).toBool();
		m_device_params.death_time.delay = settings.value("deathtime_delay", 100).toInt(&ok);
		m_device_params.type = settings.value( "io_device_type" ).toInt(&ok);
	settings.endGroup();

	// math params
	settings.beginGroup("math");
		m_math_in_params.threshold_alg_idx = settings.value( "threshold_alg_idx", 0 ).toInt(&ok);
		m_math_in_params.q_control_idx = settings.value( "q_control_idx", 0 ).toInt();
		m_math_in_params.quality_threshold1 = settings.value( "quality_threshold1", 50 ).toDouble(&ok);
		m_math_in_params.quality_threshold2 = settings.value( "quality_threshold2", 15 ).toDouble(&ok);
		m_math_in_params.stability_limit_factor = settings.value( "stability_limit_factor", STABILITY_LIMIT_FACTOR ).toDouble(&ok);
		m_math_in_params.guiding_rate = settings.value( "guiding_rate" ).toDouble(&ok);
		m_math_in_params.enabled_dir[RA] = settings.value( "enabled_RA", true ).toBool();
		m_math_in_params.enabled_dir[DEC] = settings.value( "enabled_DEC", true ).toBool();
		m_math_in_params.enabled_dir_sign[RA][SGN_POS] = settings.value( "enabled_RA+", true ).toBool();
		m_math_in_params.enabled_dir_sign[RA][SGN_NEG] = settings.value( "enabled_RA-", true ).toBool();
		m_math_in_params.enabled_dir_sign[DEC][SGN_POS] = settings.value( "enabled_DEC+", true ).toBool();
		m_math_in_params.enabled_dir_sign[DEC][SGN_NEG] = settings.value( "enabled_DEC-", true ).toBool();
		m_math_in_params.average = settings.value( "average", true ).toBool();
		m_math_in_params.accum_frame_cnt[RA] = settings.value( "accum_frame_cnt_RA", 1 ).toInt(&ok);
		m_math_in_params.accum_frame_cnt[DEC] = settings.value( "accum_frame_cnt_DEC", 1 ).toInt(&ok);
		m_math_in_params.proportional_gain[RA] = settings.value( "proportional_gain_RA" ).toDouble(&ok);
		m_math_in_params.proportional_gain[DEC] = settings.value( "proportional_gain_DEC" ).toDouble(&ok);
		m_math_in_params.integral_gain[RA] = settings.value( "integral_gain_RA" ).toDouble(&ok);
		m_math_in_params.integral_gain[DEC] = settings.value( "integral_gain_DEC" ).toDouble(&ok);
		m_math_in_params.derivative_gain[RA] = settings.value( "derivative_gain_RA" ).toDouble(&ok);
		m_math_in_params.derivative_gain[DEC] = settings.value( "derivative_gain_DEC" ).toDouble(&ok);
		m_math_in_params.max_pulse_length[RA] = settings.value( "max_pulse_length_RA" ).toInt(&ok);
		m_math_in_params.max_pulse_length[DEC] = settings.value( "max_pulse_length_DEC" ).toInt(&ok);
		m_math_in_params.min_pulse_length[RA] = settings.value( "min_pulse_length_RA" ).toInt(&ok);
		m_math_in_params.min_pulse_length[DEC] = settings.value( "min_pulse_length_DEC" ).toInt(&ok);
	settings.endGroup();

	// capture params
	settings.beginGroup("capture");
		//m_capture_params.io_mtd = (io_method)settings.value( "io_mtd" ).toInt(&ok);
		//m_capture_params.pixel_format = settings.value( "pixel_format" ).toUInt(&ok);
		m_capture_params.type = m_capture_next_params.width = settings.value( "video_device_type" ).toInt(&ok);
		m_capture_params.width = m_capture_next_params.width = settings.value( "width" ).toInt(&ok);
		m_capture_params.height = m_capture_next_params.height = settings.value( "height" ).toInt(&ok);
		m_capture_params.fps.numerator = settings.value( "fps.numerator" ).toInt(&ok);
		m_capture_params.fps.denominator = settings.value( "fps.denominator" ).toInt(&ok);
		m_capture_params.use_calibration = settings.value( "use_calibration" ).toBool();
		{
			m_capture_params.ext_params.clear();
			int ext_size = settings.beginReadArray( "ext_params" );
			for( int i = 0;i < ext_size;i++ )
			{
				settings.setArrayIndex( i );
				unsigned int id = settings.value( "id", 0 ).toUInt();
				int value = settings.value( "value", 0 ).toInt();
				m_capture_params.ext_params[ id ] = value;
			}
			settings.endArray();
		}
	settings.endGroup();

	// ui params
	settings.beginGroup("ui");
		m_ui_params.half_refresh_rate = settings.value( "half_refresh_rate" ).toBool();
	settings.endGroup();

	// guider params
	settings.beginGroup("guider");
		m_guider_params.aperture = settings.value( "aperture" ).toDouble(&ok);
		m_guider_params.focal = settings.value( "focal" ).toDouble(&ok);
		m_guider_params.matrix_width = settings.value( "matrix_width" ).toInt(&ok);
		m_guider_params.matrix_height = settings.value( "matrix_height" ).toInt(&ok);
		m_guider_params.ccd_pixel_width = settings.value( "ccd_pixel_width" ).toDouble(&ok);
		m_guider_params.ccd_pixel_height = settings.value( "ccd_pixel_height" ).toDouble(&ok);
		m_guider_params.bw_video = settings.value( "bw_video" ).toBool();
	settings.endGroup();

	// calibration params
	settings.beginGroup("calibration");
		m_calibration_params.two_axis = settings.value( "two_axis", false ).toBool();
    	m_calibration_params.auto_mode = settings.value( "auto_mode", false ).toBool();
	    m_calibration_params.dift_time = settings.value( "dift_time", 25 ).toInt(&ok);
		m_calibration_params.frame_count = settings.value( "frame_count", 10 ).toInt(&ok);
	settings.endGroup();

	// network
	settings.beginGroup("net");
		str = settings.value("bcast_ip", "127.0.0.1").toString();
		snprintf( m_net_params.bcast_ip, sizeof(m_net_params.bcast_ip), "%s", str.toAscii().data() );
		m_net_params.bcast_port = settings.value("bcast_port", "5001").toInt(&ok);
	settings.endGroup();

	// common
	settings.beginGroup("common");
		m_common_params.udp_send_start_stop = settings.value( "udp_send_start_stop", false ).toBool();
		m_common_params.udp_send_image_quality = settings.value( "udp_send_image_quality", false ).toBool();
		m_common_params.udp_send_guiding_stability = settings.value( "udp_send_guiding_stability", false ).toBool();
		m_common_params.udp_send_drift_data = settings.value( "udp_send_drift_data", false ).toBool();
		m_common_params.dithering_range = settings.value( "dithering_range", 5 ).toInt();
		m_common_params.dithering_rest_tout = settings.value( "dithering_rest_tout", 3 ).toInt();
		DBG_VERBOSITY = settings.value( "dbg_verbosity", false ).toBool();
	settings.endGroup();

	// load device config
	QSettings dev_settings( "GM_software", QString("devconf")+QString().setNum(m_device_params.type) );
	if( access(dev_settings.fileName().toAscii().data(), R_OK|W_OK) != 0 )
	{
		log_e( "Unable to find config file '%s'\nusing defaults", dev_settings.fileName().toAscii().data() );

		// fill by out-of-range value
		for( i = 0;i < 8;i++ )
			m_device_params.bit_map_template.bit_map[i] = 0xFF;

		return true;
	}
	dev_settings.beginGroup("io_bit_mapping");
		for( i = 0;i < 8;i++ )
			m_device_params.bit_map_template.bit_map[i] = (unsigned char)dev_settings.value( QString("tmpl_bit") + QString().setNum(i), 255 ).toInt(&ok);
	dev_settings.endGroup();

 return true;
}


bool params::save( void )
{
 int i;
 QSettings settings("GM_software", "lin-guider");

 	// device names
	settings.beginGroup("devices");
		settings.setValue( "video_device", m_dev_name_video);
		settings.setValue( "io_device", m_dev_name_io );
	settings.endGroup();

	// io bit map
	settings.beginGroup( "io" );
		for( i = 0;i < 8;i++ )
			settings.setValue( QString("dir_bit") + QString().setNum(i), m_device_params.dir_map.bit_direction[i] );
		settings.setValue( "inverse", m_device_params.dir_map.is_inverse );
		settings.setValue( "use_deathtime", m_device_params.death_time.use );
		settings.setValue( "deathtime_delay", (int)m_device_params.death_time.delay );
		settings.setValue( "io_device_type", (int)m_device_params.next_device_type );
	settings.endGroup();

	// math params
	settings.beginGroup( "math" );
		settings.setValue( "threshold_alg_idx", m_math_in_params.threshold_alg_idx );
		settings.setValue( "q_control_idx", m_math_in_params.q_control_idx );
		settings.setValue( "quality_threshold1", m_math_in_params.quality_threshold1 );
		settings.setValue( "quality_threshold2", m_math_in_params.quality_threshold2 );
		settings.setValue( "stability_limit_factor", m_math_in_params.stability_limit_factor );
		settings.setValue( "guiding_rate", m_math_in_params.guiding_rate );
		settings.setValue( "enabled_RA", m_math_in_params.enabled_dir[RA] );
		settings.setValue( "enabled_DEC", m_math_in_params.enabled_dir[DEC] );
		settings.setValue( "enabled_RA+", m_math_in_params.enabled_dir_sign[RA][SGN_POS] );
		settings.setValue( "enabled_RA-", m_math_in_params.enabled_dir_sign[RA][SGN_NEG] );
		settings.setValue( "enabled_DEC+", m_math_in_params.enabled_dir_sign[DEC][SGN_POS] );
		settings.setValue( "enabled_DEC-", m_math_in_params.enabled_dir_sign[DEC][SGN_NEG] );
		settings.setValue( "average", m_math_in_params.average );
		settings.setValue( "accum_frame_cnt_RA", (unsigned)m_math_in_params.accum_frame_cnt[RA] );
		settings.setValue( "accum_frame_cnt_DEC", (unsigned)m_math_in_params.accum_frame_cnt[DEC] );
		settings.setValue( "proportional_gain_RA", m_math_in_params.proportional_gain[RA] );
		settings.setValue( "proportional_gain_DEC", m_math_in_params.proportional_gain[DEC] );
		settings.setValue( "integral_gain_RA", m_math_in_params.integral_gain[RA] );
		settings.setValue( "integral_gain_DEC", m_math_in_params.integral_gain[DEC] );
		settings.setValue( "derivative_gain_RA", m_math_in_params.derivative_gain[RA] );
		settings.setValue( "derivative_gain_DEC", m_math_in_params.derivative_gain[DEC] );
		settings.setValue( "max_pulse_length_RA", m_math_in_params.max_pulse_length[RA] );
		settings.setValue( "max_pulse_length_DEC", m_math_in_params.max_pulse_length[DEC] );
		settings.setValue( "min_pulse_length_RA", m_math_in_params.min_pulse_length[RA] );
		settings.setValue( "min_pulse_length_DEC", m_math_in_params.min_pulse_length[DEC] );
	settings.endGroup();

	// capture params
	settings.beginGroup("capture" );
		//settings.setValue( "io_mtd", m_capture_params.io_mtd );
		//settings.setValue( "pixel_format", m_capture_params.pixel_format );
		settings.setValue( "video_device_type", m_capture_next_params.type ? m_capture_next_params.type :  m_capture_params.type );
		settings.setValue( "width", m_capture_next_params.width ? m_capture_next_params.width : m_capture_params.width );
		settings.setValue( "height", m_capture_next_params.height ? m_capture_next_params.height : m_capture_params.height );
		settings.setValue( "fps.numerator", m_capture_params.fps.numerator );
		settings.setValue( "fps.denominator", m_capture_params.fps.denominator );
		settings.setValue( "use_calibration", m_capture_params.use_calibration );
		{
			settings.beginWriteArray( "ext_params" );
			int idx = 0;
			for( std::map< unsigned int, int >::const_iterator it = m_capture_params.ext_params.begin();
					it != m_capture_params.ext_params.end();++it )
			{
				settings.setArrayIndex( idx );
				settings.setValue( "id", it->first );
				settings.setValue( "value", it->second );
				++idx;
			}
			settings.endArray();
		}
	settings.endGroup();

	// ui params
	settings.beginGroup( "ui" );
		settings.setValue( "half_refresh_rate", m_ui_params.half_refresh_rate );
	settings.endGroup();

	// guider params
	settings.beginGroup( "guider" );
		settings.setValue( "aperture", m_guider_params.aperture );
		settings.setValue( "focal", m_guider_params.focal );
		settings.setValue( "matrix_width", m_guider_params.matrix_width );
		settings.setValue( "matrix_height", m_guider_params.matrix_height );
		settings.setValue( "ccd_pixel_width", m_guider_params.ccd_pixel_width );
		settings.setValue( "ccd_pixel_height", m_guider_params.ccd_pixel_height );
		settings.setValue( "bw_video", m_guider_params.bw_video );
	settings.endGroup();

	// calibration params
	settings.beginGroup( "calibration" );
		settings.setValue( "two_axis", m_calibration_params.two_axis );
		settings.setValue( "auto_mode", m_calibration_params.auto_mode );
		settings.setValue( "dift_time", m_calibration_params.dift_time );
		settings.setValue( "frame_count", m_calibration_params.frame_count );
	settings.endGroup();

	// network
	settings.beginGroup("net");
		settings.setValue( "bcast_ip", m_net_params.bcast_ip );
		settings.setValue( "bcast_port", m_net_params.bcast_port );
	settings.endGroup();

	// common
	settings.beginGroup("common");
		settings.setValue( "udp_send_start_stop", m_common_params.udp_send_start_stop );
		settings.setValue( "udp_send_image_quality", m_common_params.udp_send_image_quality );
		settings.setValue( "udp_send_guiding_stability", m_common_params.udp_send_guiding_stability );
		settings.setValue( "udp_send_drift_data", m_common_params.udp_send_drift_data );
		settings.setValue( "dithering_range", m_common_params.dithering_range );
		settings.setValue( "dithering_rest_tout", m_common_params.dithering_rest_tout );
		settings.setValue( "dbg_verbosity", DBG_VERBOSITY );
	settings.endGroup();

	// save device config
	if( m_device_params.type == m_device_params.next_device_type )
	{
		if( save_device_cfg )
		{
			QSettings dev_settings( "GM_software", QString("devconf")+QString().setNum(m_device_params.type) );

			dev_settings.beginGroup("io_bit_mapping");
				for( i = 0;i < 8;i++ )
					dev_settings.setValue( QString("tmpl_bit") + QString().setNum(i), m_device_params.bit_map_template.bit_map[i] );
			dev_settings.endGroup();
			log_i( "Device config saved" );
		}
		else
			log_i( "IO is not initialized, so - current device config is not saved" );
	}
	else
	{
		log_i( "Next IO device was changed, so - current device config is not saved" );
	}

 return true;
}





