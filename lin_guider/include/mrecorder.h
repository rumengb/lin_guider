/*
 * mrecorder.h
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

#ifndef MRECORDER_H
#define MRECORDER_H

#include <QtGui/QDialog>
#include <sys/types.h>
#include "ui_mrecorder.h"
#include "avilib.h"


class lin_guider;

class mrecorder : public QDialog
{
    Q_OBJECT

public:
    mrecorder(lin_guider *parent = 0);
    ~mrecorder();

    void add_frame( void );
    bool set_video_params( u_char *pvid, int vid_wd, int vid_ht );

protected slots:
	void onRecordButtonClick();
	void onStopButtonClick();

protected:
	void showEvent ( QShowEvent * event );
	void closeEvent( QCloseEvent *event );
	void hideEvent ( QHideEvent * event );

private:
	u_char *pvideo;					// pointer to source video frame
	u_char *v_buf;					// actual video frame
	int video_width, video_height;	// video frame dimensions
	int srow_len, drow_len;			// row lengths of src and dst buffers
	unsigned frame_cnt;				// movie frame counter
	avi_t *pavi;

	void fill_interface( void );
	bool start( char *fname );
	void stop( void );
	void convert_frame( void );
	bool is_started;


	lin_guider *pmain_wnd;

private:
    Ui::mrecorderClass ui;
};

#endif // MRECORDER_H
