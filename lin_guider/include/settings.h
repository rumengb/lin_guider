/*
 * settings.h
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

#ifndef SETTINGS_H
#define SETTINGS_H

#include <QtGui/QDialog>
#include "ui_settings.h"
#include "server.h"
#include "common.h"
#include "guider.h"

//class lin_guider;
//struct uiparams_s;

class settings : public QDialog
{
	Q_OBJECT

public:
	settings( lin_guider *parent,
			net_params_t *net_params,
			common_params * comm_params,
			struct uiparams_s * ui_params,
			guider::drift_view_params_s *dv_params );
	virtual ~settings();


protected slots:
	void onGuiderAlgorithmChanged( int idx );

	void onOkButtonClick();
	void onCancelButtonClick();

protected:
	void showEvent( QShowEvent * event );
	void closeEvent ( QCloseEvent * event );
	void hideEvent ( QHideEvent * event );

private:
	void fill_interface( void );

	net_params_t m_net_params;
	common_params m_common_params;
	struct uiparams_s m_ui_params;
	guider::drift_view_params_s m_drift_view_params;

	net_params_t *m_pnet_params;
	common_params * const m_pcommon_params;
	struct uiparams_s *m_pui_params;
	guider::drift_view_params_s *m_pdrift_view_params;

private:
	Ui::settingsClass ui;
};

#endif // SETTINGS_H
