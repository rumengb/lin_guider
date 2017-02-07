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

#include <stdio.h>
#include <unistd.h>
#include <assert.h>

#include <map>

#include <QSettings>

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
	snprintf( m_net_params.listen_socket, sizeof(m_net_params.listen_socket), "/tmp/lg_ss" );
	m_net_params.listen_port = 5656;
	m_net_params.use_tcp = false;

	// common params are initialized by ctor()

	// drift view params are initialized by ctor()

	m_wnd_geometry_state.clear();
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
		m_math_in_params.stability_limit_factor = settings.value( "stability_limit_factor", lg_math::STABILITY_LIMIT_FACTOR ).toDouble(&ok);
		m_math_in_params.guiding_rate = settings.value( "guiding_rate" ).toDouble(&ok);
		m_math_in_params.guiding_normal_coef = lg_math::cgmath::precalc_proportional_gain(m_math_in_params.guiding_rate);
		m_math_in_params.normalize_gain = settings.value( "normalize_gain" ).toBool();
		m_math_in_params.enabled_dir[lg_math::RA] = settings.value( "enabled_RA", true ).toBool();
		m_math_in_params.enabled_dir[lg_math::DEC] = settings.value( "enabled_DEC", true ).toBool();
		m_math_in_params.enabled_dir_sign[lg_math::RA][lg_math::SGN_POS] = settings.value( "enabled_RA+", true ).toBool();
		m_math_in_params.enabled_dir_sign[lg_math::RA][lg_math::SGN_NEG] = settings.value( "enabled_RA-", true ).toBool();
		m_math_in_params.enabled_dir_sign[lg_math::DEC][lg_math::SGN_POS] = settings.value( "enabled_DEC+", true ).toBool();
		m_math_in_params.enabled_dir_sign[lg_math::DEC][lg_math::SGN_NEG] = settings.value( "enabled_DEC-", true ).toBool();
		m_math_in_params.average = settings.value( "average", true ).toBool();
		m_math_in_params.accum_frame_cnt[lg_math::RA] = settings.value( "accum_frame_cnt_RA", 1 ).toInt(&ok);
		m_math_in_params.accum_frame_cnt[lg_math::DEC] = settings.value( "accum_frame_cnt_DEC", 1 ).toInt(&ok);
		m_math_in_params.proportional_gain[lg_math::RA] = settings.value( "proportional_gain_RA" ).toDouble(&ok);
		m_math_in_params.proportional_gain[lg_math::DEC] = settings.value( "proportional_gain_DEC" ).toDouble(&ok);
		m_math_in_params.integral_gain[lg_math::RA] = settings.value( "integral_gain_RA" ).toDouble(&ok);
		m_math_in_params.integral_gain[lg_math::DEC] = settings.value( "integral_gain_DEC" ).toDouble(&ok);
		m_math_in_params.derivative_gain[lg_math::RA] = settings.value( "derivative_gain_RA" ).toDouble(&ok);
		m_math_in_params.derivative_gain[lg_math::DEC] = settings.value( "derivative_gain_DEC" ).toDouble(&ok);
		m_math_in_params.max_pulse_length[lg_math::RA] = settings.value( "max_pulse_length_RA" ).toInt(&ok);
		m_math_in_params.max_pulse_length[lg_math::DEC] = settings.value( "max_pulse_length_DEC" ).toInt(&ok);
		m_math_in_params.min_pulse_length[lg_math::RA] = settings.value( "min_pulse_length_RA" ).toInt(&ok);
		m_math_in_params.min_pulse_length[lg_math::DEC] = settings.value( "min_pulse_length_DEC" ).toInt(&ok);
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
		m_capture_params.gain = settings.value( "gain" ).toInt(&ok);
		m_capture_params.exposure = settings.value( "exposure" ).toInt(&ok);
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
		m_ui_params.show_helper_TB = settings.value( "show_helper_TB", false ).toBool();
		{
			QByteArray wnd_geometry = settings.value("main_wnd_geometry").toByteArray();
			QByteArray wnd_state = settings.value("main_wnd_state").toByteArray();
			set_wnd_geometry_state( "main_wnd", std::make_pair( wnd_geometry, wnd_state ) );
		}
		{
			QByteArray wnd_geometry = settings.value("guider_wnd_geometry").toByteArray();
			QByteArray wnd_state = settings.value("guider_wnd_state").toByteArray();
			set_wnd_geometry_state( "guider_wnd", std::make_pair( wnd_geometry, wnd_state ) );
		}
		m_ui_params.viewport_scale = settings.value( "viewport_scale", 1.0 ).toFloat(&ok);
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
		m_guider_params.auto_info = settings.value( "auto_info" ).toBool();
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
		m_net_params.use_tcp = settings.value( "use_tcp", false ).toBool();
		str = settings.value("listen_socket", "/tmp/lg_ss").toString();
		snprintf( m_net_params.listen_socket, sizeof(m_net_params.listen_socket), "%s", str.toAscii().data() );
		m_net_params.listen_port = settings.value("listen_port", "5656").toInt(&ok);
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
		m_common_params.hfd_on = settings.value( "hfd_on", false ).toBool();
		m_common_params.square_index = settings.value( "square_index", lg_math::cgmath::DEFAULT_SQR ).toInt();
		m_common_params.reticle_angle = settings.value( "reticle_angle", 0 ).toDouble(&ok);
		m_common_params.osf_size_kx = settings.value( "osf_size_kx", 1 ).toDouble(&ok);
		m_common_params.osf_size_ky = settings.value( "osf_size_ky", 1 ).toDouble(&ok);
		m_common_params.guider_algorithm = settings.value( "guider_algorithm", (int)lg_math::GA_CENTROID ).toInt();
	settings.endGroup();

	// drift view
	settings.beginGroup("guider_drift_view");
		m_drift_view_params.graph_type = (guider::graph_type_t)settings.value( "graph_type", guider::GRAPH_SCROLL ).toInt();
		m_drift_view_params.drift_graph_xrange = settings.value( "drift_graph_xrange", 300 ).toInt();
		m_drift_view_params.drift_graph_yrange = settings.value( "drift_graph_yrange", 60 ).toInt();
		m_drift_view_params.cell_nx = settings.value( "cell_nx", 6 ).toInt();
		m_drift_view_params.cell_ny = settings.value( "cell_ny", 6 ).toInt();
	settings.endGroup();

	// load device config
	QSettings dev_settings( "GM_software", QString("devconf")+QString().setNum(m_device_params.type) );
	if( access(dev_settings.fileName().toAscii().data(), R_OK|W_OK) != 0 )
	{
		log_e( "Unable to find devconf file '%s'\nusing defaults", dev_settings.fileName().toAscii().data() );

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
		settings.setValue( "normalize_gain", m_math_in_params.normalize_gain );
		settings.setValue( "enabled_RA", m_math_in_params.enabled_dir[lg_math::RA] );
		settings.setValue( "enabled_DEC", m_math_in_params.enabled_dir[lg_math::DEC] );
		settings.setValue( "enabled_RA+", m_math_in_params.enabled_dir_sign[lg_math::RA][lg_math::SGN_POS] );
		settings.setValue( "enabled_RA-", m_math_in_params.enabled_dir_sign[lg_math::RA][lg_math::SGN_NEG] );
		settings.setValue( "enabled_DEC+", m_math_in_params.enabled_dir_sign[lg_math::DEC][lg_math::SGN_POS] );
		settings.setValue( "enabled_DEC-", m_math_in_params.enabled_dir_sign[lg_math::DEC][lg_math::SGN_NEG] );
		settings.setValue( "average", m_math_in_params.average );
		settings.setValue( "accum_frame_cnt_RA", (unsigned)m_math_in_params.accum_frame_cnt[lg_math::RA] );
		settings.setValue( "accum_frame_cnt_DEC", (unsigned)m_math_in_params.accum_frame_cnt[lg_math::DEC] );
		settings.setValue( "proportional_gain_RA", m_math_in_params.proportional_gain[lg_math::RA] );
		settings.setValue( "proportional_gain_DEC", m_math_in_params.proportional_gain[lg_math::DEC] );
		settings.setValue( "integral_gain_RA", m_math_in_params.integral_gain[lg_math::RA] );
		settings.setValue( "integral_gain_DEC", m_math_in_params.integral_gain[lg_math::DEC] );
		settings.setValue( "derivative_gain_RA", m_math_in_params.derivative_gain[lg_math::RA] );
		settings.setValue( "derivative_gain_DEC", m_math_in_params.derivative_gain[lg_math::DEC] );
		settings.setValue( "max_pulse_length_RA", m_math_in_params.max_pulse_length[lg_math::RA] );
		settings.setValue( "max_pulse_length_DEC", m_math_in_params.max_pulse_length[lg_math::DEC] );
		settings.setValue( "min_pulse_length_RA", m_math_in_params.min_pulse_length[lg_math::RA] );
		settings.setValue( "min_pulse_length_DEC", m_math_in_params.min_pulse_length[lg_math::DEC] );
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
		settings.setValue( "gain", m_capture_params.gain );
		settings.setValue( "exposure", m_capture_params.exposure );
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
		settings.setValue( "show_helper_TB", m_ui_params.show_helper_TB );
		{
			std::map< std::string, std::pair< QByteArray, QByteArray > >::const_iterator it = m_wnd_geometry_state.find( "main_wnd" );
			if( it != m_wnd_geometry_state.end() )
			{
				settings.setValue("main_wnd_geometry", it->second.first );
				settings.setValue("main_wnd_state", it->second.second );
			}
		}
		{
			std::map< std::string, std::pair< QByteArray, QByteArray > >::const_iterator it = m_wnd_geometry_state.find( "guider_wnd" );
			if( it != m_wnd_geometry_state.end() )
			{
				settings.setValue("guider_wnd_geometry", it->second.first );
				settings.setValue("guider_wnd_state", it->second.second );
			}
		}
		settings.setValue( "viewport_scale", (double)m_ui_params.viewport_scale );
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
		settings.setValue( "auto_info", m_guider_params.auto_info );
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
		settings.setValue( "use_tcp", m_net_params.use_tcp );
		settings.setValue( "listen_port", m_net_params.listen_port );
		settings.setValue( "listen_socket", m_net_params.listen_socket );
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
		settings.setValue( "hfd_on", m_common_params.hfd_on );
		settings.setValue( "square_index", m_common_params.square_index );
		settings.setValue( "reticle_angle", m_common_params.reticle_angle );
		settings.setValue( "osf_size_kx", m_common_params.osf_size_kx );
		settings.setValue( "osf_size_ky", m_common_params.osf_size_ky );
		settings.setValue( "guider_algorithm", m_common_params.guider_algorithm );
	settings.endGroup();

	// drift view
	settings.beginGroup("guider_drift_view");
		settings.setValue( "graph_type", m_drift_view_params.graph_type );
		settings.setValue( "drift_graph_xrange", m_drift_view_params.drift_graph_xrange );
		settings.setValue( "drift_graph_yrange", m_drift_view_params.drift_graph_yrange );
		settings.setValue( "cell_nx", m_drift_view_params.cell_nx );
		settings.setValue( "cell_ny", m_drift_view_params.cell_ny );
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





