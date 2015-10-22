/*
 * settings.cpp
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

#include "lin_guider.h"
#include "server.h"
#include "common.h"
#include "settings.h"
#include "utils.h"


settings::settings( lin_guider *parent,
		net_params_t *net_params,
		common_params *comm_params,
		struct uiparams_s * ui_params,
		guider::drift_view_params_s *dv_params ) :
	QDialog(parent),
    m_pnet_params( net_params ),
    m_pcommon_params( comm_params ),
    m_ui_params( ui_params ),
    m_drift_view_params( dv_params )
{
	ui.setupUi(this);

	setWindowTitle( tr("Settings") );

	// fill drift graph combos
	ui.comboBox_DriftGraph_nx->clear();
	ui.comboBox_DriftGraph_ny->clear();
	for( int i = 0;i <= 10;i++ )
	{
		if( i > 1 )
		{
			ui.comboBox_DriftGraph_nx->addItem( QString().setNum(i), i );
			if( !(i & 1) )
				ui.comboBox_DriftGraph_ny->addItem( QString().setNum(i), i );
		}
	}

	connect( ui.pushButton_OK, SIGNAL(clicked()), this, SLOT(onOkButtonClick()) );
	connect( ui.pushButton_Cancel, SIGNAL(clicked()), this, SLOT(onCancelButtonClick()) );
}

settings::~settings()
{

}

void settings::showEvent( QShowEvent * event )
{
	if( event->spontaneous() )
		return;

	if( !m_pnet_params || !m_pcommon_params )
	{
		u_msg("settings::showEvent: Critical error! pointers not initialized");
		event->ignore();
		return;
	}
	m_net_params    = *m_pnet_params;
	m_common_params = *m_pcommon_params;

	fill_interface();
}


void settings::closeEvent ( QCloseEvent * event )
{
	(void)event;
}


void settings::hideEvent ( QHideEvent * event )
{
	if( event->spontaneous() )
		return;

	close();
}


void settings::fill_interface( void )
{
	// UDP
	ui.lineEdit_IP->setText( QString(m_net_params.bcast_ip) );
	ui.lineEdit_Port->setText( QString().setNum( (unsigned short)m_net_params.bcast_port) );

	ui.checkBox_StartStop->setChecked( m_common_params.udp_send_start_stop );
	ui.checkBox_ImageQuality->setChecked( m_common_params.udp_send_image_quality );
	ui.checkBox_GuidingStability->setChecked( m_common_params.udp_send_guiding_stability );
	ui.checkBox_DriftData->setChecked( m_common_params.udp_send_drift_data );

	ui.spinBox_DitherRange->setValue( m_common_params.dithering_range );
	ui.spinBox_DitherRestTout->setValue( m_common_params.dithering_rest_tout );

	// TCP
	ui.lineEdit_TCPPort->setText( QString().setNum( (unsigned short)m_net_params.listen_port) );
	ui.lineEdit_TCPPort->setEnabled( m_net_params.use_tcp );
	ui.lineEdit_lsocket->setText( QString(m_net_params.listen_socket) );
	ui.lineEdit_lsocket->setDisabled( m_net_params.use_tcp );
	ui.checkBox_useTCP->setChecked( m_net_params.use_tcp );

	// debug verbosity
	ui.checkBox_DBGVerbosity->setChecked( DBG_VERBOSITY );

	// UI toolbars
	ui.checkBox_ShowHelperTB->setChecked( m_ui_params->show_helper_TB );

	// HFD
	ui.checkBox_HFD_on->setChecked( m_common_params.hfd_on );

	// drift graph
	for( int i = 0;i < ui.comboBox_DriftGraph_nx->count();i++ )
	{
		if( ui.comboBox_DriftGraph_nx->itemData( i ).toInt() == m_drift_view_params->cell_nx )
		{
			ui.comboBox_DriftGraph_nx->setCurrentIndex( i );
			break;
		}
	}
	for( int i = 0;i < ui.comboBox_DriftGraph_ny->count();i++ )
	{
		if( ui.comboBox_DriftGraph_ny->itemData( i ).toInt() == m_drift_view_params->cell_ny )
		{
			ui.comboBox_DriftGraph_ny->setCurrentIndex( i );
			break;
		}
	}

}


void settings::onOkButtonClick()
{
	char  bcast_ip[16];
	int bcast_ip_len = 0;

	memset( bcast_ip, 0, sizeof(bcast_ip) );
	bcast_ip_len = ui.lineEdit_IP->text().length();
	memcpy( bcast_ip, ui.lineEdit_IP->text().toAscii().data(), MIN((int)sizeof(bcast_ip)-1, bcast_ip_len) );

	unsigned int parsed = 0, arg_len = 0;
	const char *arg = NULL;
	for( int n = 0;n < 4; n++ )
	{
		int res = u_memtok( bcast_ip, bcast_ip_len, '.', &arg, &arg_len, &parsed );
		if( !res || !arg || arg_len == 0 )
		{
			QMessageBox::warning( this, tr("Error"), tr("Emplty value in IP address."), QMessageBox::Ok );
			return;
		}
		long v = strtol( arg, NULL, 10 );
		if( v > 255 )
		{
			QMessageBox::warning( this, tr("Error"), tr("Invalid value in IP address."), QMessageBox::Ok );
			return;
		}
	}
	memcpy( m_net_params.bcast_ip, bcast_ip, sizeof(bcast_ip) );

	bool ok = false;
	int port = ui.lineEdit_Port->text().toInt( &ok );
	if( !ok || port < 1 || port > 65534 )
	{
		QMessageBox::warning( this, tr("Error"), tr("Invalid port."), QMessageBox::Ok );
		return;
	}
	m_net_params.bcast_port = port;

	port = ui.lineEdit_TCPPort->text().toInt( &ok );
	if( !ok || port < 1000 || port > 65534 )
	{
		QMessageBox::warning( this, tr("Error"), tr("Invalid port."), QMessageBox::Ok );
		return;
	}
	m_net_params.listen_port = port;

	snprintf( m_net_params.listen_socket, sizeof(m_net_params.listen_socket), "%s", ui.lineEdit_lsocket->text().toAscii().data() );

	m_net_params.use_tcp = ui.checkBox_useTCP->isChecked();

	m_common_params.udp_send_start_stop = ui.checkBox_StartStop->isChecked();
	m_common_params.udp_send_image_quality = ui.checkBox_ImageQuality->isChecked();
	m_common_params.udp_send_guiding_stability = ui.checkBox_GuidingStability->isChecked();
	m_common_params.udp_send_drift_data = ui.checkBox_DriftData->isChecked();

	m_common_params.dithering_range = ui.spinBox_DitherRange->value();
	m_common_params.dithering_rest_tout = ui.spinBox_DitherRestTout->value();

	DBG_VERBOSITY = ui.checkBox_DBGVerbosity->isChecked();

	// apply params
	std::map<int, bool> msg_map;

	msg_map.insert( std::make_pair(BCM_SRV_STARTED, m_common_params.udp_send_start_stop) );
	msg_map.insert( std::make_pair(BCM_SRV_STOPPED, m_common_params.udp_send_start_stop) );

	msg_map.insert( std::make_pair(BCM_NORMAL_IMAGE_QUALITY, m_common_params.udp_send_image_quality) );
	msg_map.insert( std::make_pair(BCM_LOW_IMAGE_QUALITY, m_common_params.udp_send_image_quality) );
	msg_map.insert( std::make_pair(BCM_CRITICAL_IMAGE_QUALITY, m_common_params.udp_send_image_quality) );

	msg_map.insert( std::make_pair(BCM_GUIDING_STABLE, m_common_params.udp_send_guiding_stability) );
	msg_map.insert( std::make_pair(BCM_GUIDING_UNSTABLE, m_common_params.udp_send_guiding_stability) );

	msg_map.insert( std::make_pair(BCM_DRIFT_DATA, m_common_params.udp_send_drift_data) );

	server::set_msg_map( msg_map );

	*m_pnet_params = m_net_params;

	m_ui_params->show_helper_TB = ui.checkBox_ShowHelperTB->isChecked();

	m_common_params.hfd_on = ui.checkBox_HFD_on->isChecked();

	// final
	*m_pcommon_params = m_common_params;

	if( ui.comboBox_DriftGraph_nx->currentIndex() != -1 )
		m_drift_view_params->cell_nx = ui.comboBox_DriftGraph_nx->itemData( ui.comboBox_DriftGraph_nx->currentIndex() ).toInt();
	if( ui.comboBox_DriftGraph_ny->currentIndex() != -1 )
		m_drift_view_params->cell_ny = ui.comboBox_DriftGraph_ny->itemData( ui.comboBox_DriftGraph_ny->currentIndex() ).toInt();

	close();
}


void settings::onCancelButtonClick()
{
	close();
}
