/*
 * setup_driver.cpp
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

#include <Qt>
#include <QtGui>

#include "setup_driver.h"
#include "lin_guider.h"
#include "io_driver.h"
#include "utils.h"


void setup_driver::init_combo( QComboBox *pCombo )
{
 int i, cnt;
 QString str;

	// index = guide_dir !!!
 	pCombo->clear();
 	cnt = ARRAY_SIZE( io_drv::bit_list_data );

 	for( i = 0;i < cnt;i++ )
 	{
 		str = QString( io_drv::bit_list_data[i].desc );
 		pCombo->addItem( str, (int)io_drv::bit_list_data[i].dir );
 	}

 	pCombo->setCurrentIndex( 0 );

}



setup_driver::setup_driver(lin_guider *parent)
    : QDialog(parent), pmain_wnd(parent)
{
 int i, cnt;
 QString str;


	ui.setupUi(this);

	setWindowTitle( tr("Pulse Device Settings") );

	// dbg
	ui.testButton->setVisible( false );

	// Init bit boxes
	init_combo( ui.bitBox0 );
	init_combo( ui.bitBox1 );
	init_combo( ui.bitBox2 );
	init_combo( ui.bitBox3 );
	init_combo( ui.bitBox4 );
	init_combo( ui.bitBox5 );
	init_combo( ui.bitBox6 );
	init_combo( ui.bitBox7 );


	// init device list
	ui.comboBox_DeviceList->clear();
	cnt = ARRAY_SIZE( io_drv::device_desc_list );

	for( i = 0;i < cnt;i++ )
	{
		str = QString( io_drv::device_desc_list[i].desc );
		ui.comboBox_DeviceList->addItem( str, (int)io_drv::device_desc_list[i].type );
	}
	ui.comboBox_DeviceList->setCurrentIndex( 0 );

	// connect ui
	connect( ui.bitBox0, SIGNAL( activated(int) ), this, SLOT(onBitSelected(int)) );
	connect( ui.bitBox1, SIGNAL( activated(int) ), this, SLOT(onBitSelected(int)) );
	connect( ui.bitBox2, SIGNAL( activated(int) ), this, SLOT(onBitSelected(int)) );
	connect( ui.bitBox3, SIGNAL( activated(int) ), this, SLOT(onBitSelected(int)) );
	connect( ui.bitBox4, SIGNAL( activated(int) ), this, SLOT(onBitSelected(int)) );
	connect( ui.bitBox5, SIGNAL( activated(int) ), this, SLOT(onBitSelected(int)) );
	connect( ui.bitBox6, SIGNAL( activated(int) ), this, SLOT(onBitSelected(int)) );
	connect( ui.bitBox7, SIGNAL( activated(int) ), this, SLOT(onBitSelected(int)) );

	connect( ui.inverseBits, SIGNAL( stateChanged(int) ), this, SLOT(onInverseChanged(int)) );

	ui.inverseBits->setChecked( true );

	connect( ui.comboBox_DeviceList, SIGNAL(activated(int)), this, SLOT(onDeviceListChanged(int)) );

	// connect buttons
	connect( ui.RA_INC_Button, SIGNAL( pressed() ),   this, SLOT( onRA_INC_ButtonPressed() ) );
	connect( ui.RA_DEC_Button, SIGNAL( pressed() ),   this, SLOT( onRA_DEC_ButtonPressed() ) );
	connect( ui.DEC_INC_Button, SIGNAL( pressed() ),  this, SLOT( onDEC_INC_ButtonPressed() ) );
	connect( ui.DEC_DEC_Button, SIGNAL( pressed() ),  this, SLOT( onDEC_DEC_ButtonPressed() ) );

	connect( ui.RA_INC_Button, SIGNAL( released() ),  this, SLOT( onRA_DEC_ButtonReleased() ) );
	connect( ui.RA_DEC_Button, SIGNAL( released() ),  this, SLOT( onRA_DEC_ButtonReleased() ) );
	connect( ui.DEC_INC_Button, SIGNAL( released() ), this, SLOT( onRA_DEC_ButtonReleased() ) );
	connect( ui.DEC_DEC_Button, SIGNAL( released() ), this, SLOT( onRA_DEC_ButtonReleased() ) );

	connect( ui.okButton, SIGNAL( clicked() ),        this, SLOT( onOkButtonClick() ) );
	connect( ui.cancelButton, SIGNAL( clicked() ),    this, SLOT( onCancelButtonClick() ) );

	// DT
	connect( ui.checkBox_UseDT, SIGNAL( stateChanged(int) ), this, SLOT(onUseDTChanged(int)) );

	connect( ui.testButton, SIGNAL( clicked() ), this, SLOT( onTestButtonClick() ) );

}

setup_driver::~setup_driver()
{

}


void setup_driver::showEvent( QShowEvent * event )
{
	if( event->spontaneous() )
		return;

	// check validity
	if( !pmain_wnd )
	{
		u_msg("setup_driver::showEvent: main wnd not initialized");
		return;
	}

	// stop any guiding pulse
	pmain_wnd->m_driver->reset();


	params = pmain_wnd->m_driver->get_deviceparams();
	snprintf( dev_name_io, sizeof(dev_name_io), "%s", pmain_wnd->dev_name_io );

	apply_map = false;

	fill_interface();
}


void setup_driver::closeEvent ( QCloseEvent * event )
{
 bool res;

	if( apply_map && pmain_wnd )
	{
		res = io_drv::cio_driver_base::check_device_dir_map( params.dir_map );
		if( !res )
		{
			QMessageBox::warning( this, tr("Warning"), tr("Pulse-driver direction map possibly incorrect.\nIt may be cause of guiding errors or device corruption!"), QMessageBox::Ok );
			if( !u_yes( tr("Do you want to accept incorrect parameters?") ) )
			{
				apply_map = false;
				event->ignore();
				return;
			}
		}

		params.death_time.use 		= ui.checkBox_UseDT->isChecked();
		params.death_time.delay 	= ui.spinBox_DT->value();

		pmain_wnd->m_driver->set_deviceparams( params );
		pmain_wnd->m_driver->reset();

		snprintf( pmain_wnd->dev_name_io, sizeof(pmain_wnd->dev_name_io), "%s", dev_name_io );
	}
}


void setup_driver::hideEvent ( QHideEvent * event )
{
	if( event->spontaneous() )
		return;

	close();
}


void setup_driver::onBitSelected( int /*index*/ )
{
 bool ok;
 io_drv::guide_dir tmp;


 	tmp = (io_drv::guide_dir)ui.bitBox0->itemData(ui.bitBox0->currentIndex()).toInt( &ok );
 	if( !ok )
 		u_msg("bit0 - combo data error");
 	params.dir_map.bit_direction[0] = ok ? tmp : io_drv::NO_DIR;

	tmp = (io_drv::guide_dir)ui.bitBox1->itemData(ui.bitBox1->currentIndex()).toInt( &ok );
	if( !ok )
		u_msg("bit1 - combo data error");
	params.dir_map.bit_direction[1] = ok ? tmp : io_drv::NO_DIR;

	tmp = (io_drv::guide_dir)ui.bitBox2->itemData(ui.bitBox2->currentIndex()).toInt( &ok );
	if( !ok )
 		u_msg("bit2 - combo data error");
	params.dir_map.bit_direction[2] = ok ? tmp : io_drv::NO_DIR;

	tmp = (io_drv::guide_dir)ui.bitBox3->itemData(ui.bitBox3->currentIndex()).toInt( &ok );
 	if( !ok )
 		u_msg("bit3 - combo data error");
 	params.dir_map.bit_direction[3] = ok ? tmp : io_drv::NO_DIR;

	tmp = (io_drv::guide_dir)ui.bitBox4->itemData(ui.bitBox4->currentIndex()).toInt( &ok );
 	if( !ok )
 		u_msg("bit4 - combo data error");
 	params.dir_map.bit_direction[4] = ok ? tmp : io_drv::NO_DIR;

	tmp = (io_drv::guide_dir)ui.bitBox5->itemData(ui.bitBox5->currentIndex()).toInt( &ok );
	if( !ok )
 		u_msg("bit5 - combo data error");
	params.dir_map.bit_direction[5] = ok ? tmp : io_drv::NO_DIR;

	tmp = (io_drv::guide_dir)ui.bitBox6->itemData(ui.bitBox6->currentIndex()).toInt( &ok );
 	if( !ok )
 		u_msg("bit6 - combo data error");
 	params.dir_map.bit_direction[6] = ok ? tmp : io_drv::NO_DIR;

	tmp = (io_drv::guide_dir)ui.bitBox7->itemData(ui.bitBox7->currentIndex()).toInt( &ok );
 	if( !ok )
 		u_msg("bit7 - combo data error");
 	params.dir_map.bit_direction[7] = ok ? tmp : io_drv::NO_DIR;

}


void setup_driver::onInverseChanged( int state )
{
	params.dir_map.is_inverse = (state != Qt::Unchecked);
}


void setup_driver::onDeviceListChanged( int index )
{
 bool ok;

 	params.next_device_type = ui.comboBox_DeviceList->itemData( index ).toInt(&ok);

 	update_dev_string_visibility( params.next_device_type );

 	if( params.next_device_type != params.type )
 		u_msg( "Restart program to apply changes." );
}


void setup_driver::onRA_INC_ButtonPressed()
{
	if( !pmain_wnd )
		return;
	pmain_wnd->m_driver->test_dir_map( params.dir_map, io_drv::RA_INC_DIR );
	//pmain_wnd->m_driver->do_pulse( io_drv::RA_INC_DIR, 5000, io_drv::DEC_INC_DIR, 5000 );
}


void setup_driver::onRA_DEC_ButtonPressed()
{
	if( !pmain_wnd )
		return;
	pmain_wnd->m_driver->test_dir_map( params.dir_map, io_drv::RA_DEC_DIR );
}


void setup_driver::onDEC_INC_ButtonPressed()
{
	if( !pmain_wnd )
		return;
	pmain_wnd->m_driver->test_dir_map( params.dir_map, io_drv::DEC_INC_DIR );
}


void setup_driver::onDEC_DEC_ButtonPressed()
{
	if( !pmain_wnd )
		return;
	pmain_wnd->m_driver->test_dir_map( params.dir_map, io_drv::DEC_DEC_DIR );
}


void setup_driver::onRA_DEC_ButtonReleased()
{
	if( !pmain_wnd )
		return;
	pmain_wnd->m_driver->test_dir_map( params.dir_map, io_drv::NO_DIR );
}


void setup_driver::onUseDTChanged( int state )
{
	ui.spinBox_DT->setVisible( state == Qt::Checked );
	ui.label_10->setVisible( state == Qt::Checked );
}


void setup_driver::onOkButtonClick()
{
 char fname[64];


	snprintf( fname, sizeof(fname), "%s", ui.lineEdit_IoDevice->text().toAscii().data() );
	if( params.next_device_type == io_drv::DT_LPT )
	{
		if( access( fname, R_OK|W_OK ) != 0 )
		{
			QMessageBox::warning( this, tr("Error"), tr("Selected io device is not available."), QMessageBox::Ok );
			return;
		}
	}
	snprintf( dev_name_io, sizeof(dev_name_io), "%s", fname );

	apply_map = true;

	close();
}


void setup_driver::onCancelButtonClick()
{
	close();
}

#include "server.h"
void setup_driver::onTestButtonClick()
{
//	server::send_bcast_msg( BCM_DEBUG, "test message" );
//	return;

	//for( int i = 0;i < 10;i++ )
	{
	pmain_wnd->m_driver->do_pulse( io_drv::RA_INC_DIR, 3000, io_drv::NO_DIR, 0 );
	usleep(100000);
	pmain_wnd->m_driver->do_pulse( io_drv::RA_DEC_DIR, 3000, io_drv::NO_DIR, 0 );
	usleep(150000);
	pmain_wnd->m_driver->do_pulse( io_drv::RA_INC_DIR, 3000, io_drv::NO_DIR, 0 );
	}

/*
	pmain_wnd->driver->write_data( 1 );
	usleep(100000);
	pmain_wnd->driver->write_data( 0 );
	usleep(150000);
	pmain_wnd->driver->write_data( 1 );

	usleep(3000000);
	pmain_wnd->driver->write_data( 0 );
*/
}


void setup_driver::fill_interface( void )
{
 int i;
 bool ok;

	ui.bitBox0->setCurrentIndex( (int)params.dir_map.bit_direction[0] );
	ui.bitBox1->setCurrentIndex( (int)params.dir_map.bit_direction[1] );
	ui.bitBox2->setCurrentIndex( (int)params.dir_map.bit_direction[2] );
	ui.bitBox3->setCurrentIndex( (int)params.dir_map.bit_direction[3] );
	ui.bitBox4->setCurrentIndex( (int)params.dir_map.bit_direction[4] );
	ui.bitBox5->setCurrentIndex( (int)params.dir_map.bit_direction[5] );
	ui.bitBox6->setCurrentIndex( (int)params.dir_map.bit_direction[6] );
	ui.bitBox7->setCurrentIndex( (int)params.dir_map.bit_direction[7] );

	ui.inverseBits->setChecked( params.dir_map.is_inverse );

	//select from device list
	for( i = 0;i < ui.comboBox_DeviceList->count();i++ )
	{
		if( ui.comboBox_DeviceList->itemData(i).toInt(&ok) == params.next_device_type )
		{
			ui.comboBox_DeviceList->setCurrentIndex( i );
			break;
		}
	}

	ui.lineEdit_IoDevice->setText( QString(dev_name_io) );

	update_dev_string_visibility( params.next_device_type );

	ui.checkBox_UseDT->setChecked( params.death_time.use );
	ui.spinBox_DT->setValue( params.death_time.delay );

	ui.spinBox_DT->setVisible( params.death_time.use );
	ui.label_10->setVisible( params.death_time.use );

}


void setup_driver::update_dev_string_visibility( int dev_type )
{
	ui.lineEdit_IoDevice->setVisible( false );
	ui.label_Information->setText( QString() );

	int cnt = ARRAY_SIZE( io_drv::device_desc_list );
	for( int i = 0;i < cnt;i++ )
		if( dev_type == io_drv::device_desc_list[i].type )
		{
			ui.lineEdit_IoDevice->setVisible( io_drv::device_desc_list[i].show_dev_string_ui );
			if( io_drv::device_desc_list[i].hyper_info )
				ui.label_Information->setText( QString::fromUtf8(io_drv::device_desc_list[i].hyper_info) );
			if( io_drv::device_desc_list[i].info )
				log_i( "IO device info: \"%s\"", io_drv::device_desc_list[i].info );
		}
}
