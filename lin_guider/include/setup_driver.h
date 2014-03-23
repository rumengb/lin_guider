/*
 * setup_driver.h
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

#ifndef SETUP_DRIVER_H
#define SETUP_DRIVER_H

#include <QtGui>
#include <QtGui/QDialog>
#include "ui_setup_driver.h"
#include "io_driver.h"

class lin_guider;

class setup_driver : public QDialog
{
    Q_OBJECT

public:
    setup_driver(lin_guider *parent = 0);
    ~setup_driver();

protected slots:
	void onBitSelected(int);
	void onInverseChanged( int state );
	void onDeviceListChanged( int index );
	void onRA_INC_ButtonPressed();
	void onRA_DEC_ButtonPressed();
	void onDEC_INC_ButtonPressed();
	void onDEC_DEC_ButtonPressed();
	void onRA_DEC_ButtonReleased();
	void onOkButtonClick();
	void onCancelButtonClick();
	void onUseDTChanged( int state );
	void onTestButtonClick();
private:
	void showEvent( QShowEvent * event );
	void closeEvent ( QCloseEvent * event );
	void hideEvent ( QHideEvent * event );

	void init_combo( QComboBox *pCombo );
	void create_dirmap( void );
	void fill_interface( void );
	void update_dev_string_visibility( int dev_type );
	io_drv::device_init_params_t params;

	char dev_name_io[64];

	bool apply_map;

	lin_guider *pmain_wnd;
private:
    Ui::setup_driverClass ui;
};

#endif // SETUP_DRIVER_H
