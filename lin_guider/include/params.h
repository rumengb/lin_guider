/*
 * params.h
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
#ifndef PARAMS_H_
#define PARAMS_H_

#include <string.h>

#include <string>
#include <map>
#include <utility>

#include "lin_guider.h"
#include "common.h"
#include "io_driver.h"
#include "video.h"
#include "gmath.h"
#include "setup_video.h"
#include "rcalibration.h"
#include "server.h"
#include "guider.h"


class params
{
public:
	params();
	virtual ~params();

	bool load( void );
	bool save( void );

	bool save_device_cfg;

	// video
	guiderparams_t get_guider_params( void ) const { return m_guider_params;};
	void set_guider_params( const guiderparams_t &v ){ m_guider_params = v;};

	uiparams_t get_ui_params( void ) const { return m_ui_params;};
	void set_ui_params( const uiparams_t &v ){ m_ui_params = v;};

	video_drv::captureparams_t get_capture_params( void ) const { return m_capture_params;};
	void set_capture_params( const video_drv::captureparams_t &v ){ m_capture_params = v;};

	video_drv::capture_next_params_t get_capture_next_params( void ) const { return m_capture_next_params;};
	void set_capture_next_params( const video_drv::capture_next_params_t &v ){ m_capture_next_params = v;};

	// math
	const lg_math::cproc_in_params *get_math_in_params( void ) const { return &m_math_in_params;};
	void set_math_in_params( const lg_math::cproc_in_params &v ){ m_math_in_params = v;};

	// io
	io_drv::device_init_params_t get_device_params( void ) const { return m_device_params;};
	void set_device_params( const io_drv::device_init_params_t &v ){ m_device_params = v;};

	//dev names
	void get_video_dev( char *dst, int len ) const { snprintf( dst, len, "%s", m_dev_name_video ); };
	void set_video_dev( const char *src ){ snprintf( m_dev_name_video, sizeof(m_dev_name_video), "%s", src ); };

	void get_io_dev( char *dst, int len ) const { snprintf( dst, len, "%s", m_dev_name_io ); };
	void set_io_dev( const char *src ){ snprintf( m_dev_name_io, sizeof(m_dev_name_io), "%s", src ); };

	// calibration
	calibrationparams_t get_calibration_params( void ) const { return m_calibration_params; };
	void set_calibration_params( const calibrationparams_t &v ){ m_calibration_params = v;};

	// network
	net_params_t get_net_params( void ) const { return m_net_params; };
	void set_net_params( const net_params_t &v ) { memcpy( &m_net_params, &v, sizeof(net_params_t) ); };

	// common
	common_params get_common_params( void ) const { return m_common_params; };
	void set_common_params( const common_params &v ) { m_common_params = v; };

	// drift view
	guider::drift_view_params_s get_drift_view_params( void ) { return m_drift_view_params; };
	void set_drift_view_params( const guider::drift_view_params_s &v ) { m_drift_view_params = v; }

	const std::pair< QByteArray, QByteArray >& get_wnd_geometry_state( const std::string &name ) const
	{
		static std::pair< QByteArray, QByteArray > empty_stub;
		std::map< std::string, std::pair< QByteArray, QByteArray > >::const_iterator it = m_wnd_geometry_state.find( name );
		if( it != m_wnd_geometry_state.end() )
			return it->second;
		return empty_stub;
	}
	void set_wnd_geometry_state( const std::string &name, const std::pair< QByteArray, QByteArray > &geometry_state )
	{
		std::map< std::string, std::pair< QByteArray, QByteArray > >::iterator it = m_wnd_geometry_state.find( name );
		if( it != m_wnd_geometry_state.end() )
			m_wnd_geometry_state.erase( it );
		m_wnd_geometry_state.insert( std::make_pair( name, geometry_state ) );
	}

private:
	// video
	guiderparams_t m_guider_params;
	uiparams_t m_ui_params;
	video_drv::captureparams_t m_capture_params;
	video_drv::capture_next_params_t m_capture_next_params;

	// math
	lg_math::cproc_in_params m_math_in_params;

	// io
	io_drv::device_init_params_t m_device_params;

	// calibration
	calibrationparams_t m_calibration_params;

	// network
	net_params_t m_net_params;

	// common params
	common_params m_common_params;

	// guider drift view params
	guider::drift_view_params_s m_drift_view_params;

	// dev names
	char m_dev_name_video[64];
	char m_dev_name_io[64];

	// name->(geom, state)
	std::map< std::string, std::pair< QByteArray, QByteArray > > m_wnd_geometry_state;
};


#endif /*PARAMS_H_*/
