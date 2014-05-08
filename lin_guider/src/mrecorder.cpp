/*
 * mrecorder.cpp
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

#include <assert.h>
#include <stdlib.h>

#include "mrecorder.h"
#include "lin_guider.h"
#include "utils.h"


mrecorder::mrecorder(lin_guider *parent)
    : QDialog(parent), pmain_wnd(parent)
{
	ui.setupUi(this);

	// setup ui
	setWindowTitle( tr("Movie recorder") );

	pvideo = NULL;
	v_buf = NULL;
	video_width = video_height = 0;
	frame_cnt = 0;

	is_started = false;

	// connect ui
	connect( ui.pushButton_Record, SIGNAL(clicked()), this, SLOT(onRecordButtonClick()) );
	connect( ui.pushButton_Stop,   SIGNAL(clicked()), this, SLOT(onStopButtonClick()) );
}


mrecorder::~mrecorder()
{
	if( v_buf )
		free( v_buf );
}


void mrecorder::showEvent ( QShowEvent * event )
{
	if( event->spontaneous() )
		return;

	//pmain_wnd->lock_toolbar( true );

	fill_interface();
}


void mrecorder::closeEvent( QCloseEvent * )
{
	stop();

	//pmain_wnd->lock_toolbar( false );

	//ui.pushButton_StartCalibration->setText( tr("Start") );
	//ui.pushButton_StartCalibration->setEnabled( true );
}


void mrecorder::hideEvent ( QHideEvent * event )
{
	if( event->spontaneous() )
	{
		return;
	}
	//event->ignore();
	close();
}


void mrecorder::onRecordButtonClick()
{
 bool ret;

	if( ui.lineEdit_FileName->text().isEmpty() )
	{
		u_msg( "Empty file name" );
		return;
	}

	const char *home_dir = getenv( "HOME" );
	ret = start( ((home_dir ? (QString(home_dir) + "/") : "" ) + ui.lineEdit_FileName->text()+".avi").toAscii().data() );
	if( !ret )
		return;

	ui.pushButton_Record->setEnabled( false );
	ui.pushButton_Stop->setEnabled( true );
}


void mrecorder::onStopButtonClick()
{
	stop();

	ui.pushButton_Record->setEnabled( true );
	ui.pushButton_Stop->setEnabled( false );
}


bool mrecorder::set_video_params( u_char *pvid, int vid_wd, int vid_ht )
{
	if( pvid == NULL )
		return false;
	if( vid_wd <= 0 || vid_ht <= 0 )
		return false;

	pvideo 		 = pvid;
	video_width  = vid_wd;
	video_height = vid_ht;

	if( v_buf )
		free( v_buf );
	v_buf = (u_char *)malloc( video_width * video_height * 3 );

	srow_len = video_width*4;
	drow_len = video_width*3;

 return true;
}


void mrecorder::fill_interface( void )
{
	ui.pushButton_Record->setEnabled( true );
	ui.pushButton_Stop->setEnabled( false );

	ui.l_FrameCnt->setText( QString().setNum( 0 ) );
}


bool mrecorder::start( char *fname )
{
 int res;

	if( is_started )
		return true;

	res = fio::check_file_name( fname );
	switch( res )
	{
	case FIO_EXIST:
		u_msg( "File '%s' exists", fname );
		return false;
	case FIO_ERROR:
		u_msg( "Unable to write file '%s'. Check permissions", fname );
		return false;
	}

	pavi = AVI_open_output_file(fname);
	if( !pavi )
	{
		log_e("%s\n", AVI_strerror());
		return false;
	}
	AVI_set_video( pavi, video_width, video_height, video_drv::time_fract::to_fps( pmain_wnd->m_capture_params.fps ), (char *)"RGB");

	frame_cnt = 0;

	is_started = true;

 return true;
}


void mrecorder::stop( void )
{
	if( !is_started )
		return;

	AVI_close( pavi );

	pavi = NULL;

	is_started = false;
}


void mrecorder::add_frame( void )
{
 int ret;


	if( !isVisible() || !is_started )
		return;

	assert( pvideo != NULL && v_buf != NULL );

	convert_frame();

	// add frame to movie...
	ret = AVI_write_frame( pavi, (char *)v_buf, video_width*video_height*3, frame_cnt );
	if( ret != 0 )
	{
		log_e("%s\n", AVI_strerror());
		return;
	}

	frame_cnt++;
	ui.l_FrameCnt->setText( QString().setNum( frame_cnt ) );
}


void mrecorder::convert_frame( void )
{
 int i, j;
 u_char *psrc, *pdst;

	for( j = 0;j < video_height;j++ )
	{
		pdst = v_buf + j*drow_len;
		psrc = pvideo + srow_len * (video_height-j-1);
		for( i = 0;i < video_width;i++ )
		{
			*pdst = *psrc;
			pdst++;
			psrc++;
			*pdst = *psrc;
			pdst++;
			psrc++;
			*pdst = *psrc;
			pdst++;
			psrc+=2;
		}
	}

}
