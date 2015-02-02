/*
 * setup_video.h
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

#ifndef SETUP_VIDEO_H
#define SETUP_VIDEO_H

#include <QtGui/QDialog>
#include "ui_setup_video.h"

#include "video.h"
#include "maindef.h"


typedef struct
{
	double ccd_pixel_width, ccd_pixel_height;
	double aperture, focal;
	int matrix_width, matrix_height;
	video_drv::time_fract_t fps;
	bool bw_video;
	bool auto_info;
}guiderparams_t;


class lin_guider;


class setup_video : public QDialog
{
    Q_OBJECT

public:
    setup_video(lin_guider *parent = 0);
    ~setup_video();

    bool is_applied( void );

protected slots:
	void onApertureChanged( double val );
	void onFocalChanged( double val );
	void onMatrixWidthChanged( int val );
	void onMatrixHeightChanged( int val );
	void onPixeWidthChanged( double val );
	void onPixeHeightChanged( double val );
	void onAutoInfoChecked( int state );
	void onDeviceListChanged( int index );
	void onFPSChanged( int index );
	void onFrameSizeChanged( int index );
	void onBWChecked( int state );
	void onHalfFPSChecked( int state );
	void onUseCalibrationChecked( int state );
	void onAutogainChanged ( int state );
	void onSpinGainChanged( int value );
	void onSliderGainChanged( int value );
	void onSpinExpoChanged( int value );
	void onSliderExpoChanged( int value );
	void onExtParamChanged( int index );
	void onSpinExtParamChanged( int value );
	void onSliderExtParamChanged( int value );

	void onOkButtonClick();
	void onCancelButtonClick();

protected:
	void showEvent( QShowEvent * event );
	void closeEvent ( QCloseEvent * event );
	void hideEvent ( QHideEvent * event );
	// ALL params
private:
	void fill_interface( void );
	void fill_sensor_info( void );
	void update_dev_string_visibility( int dev_type );
	double calc_arc( int pix, double pix_sz, double focal );

	guiderparams_t  guider_params;
	bool half_refresh_rate;
	video_drv::captureparams_t params;
	char dev_name_video[64];
	point_t capture_sz;
	video_drv::capture_next_params_t next_params;

	bool first_show;
	bool applied;

	bool is_filling_ui;

	lin_guider *pmain_wnd;
private:
    Ui::setup_videoClass ui;
};

#endif // SETUP_VIDEO_H
