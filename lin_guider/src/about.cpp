/*
 * about.cpp
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

#include "about.h"
#include "maindef.h"


about::about(QWidget *parent)
    : QDialog(parent)
{
	ui.setupUi(this);
	
	setWindowTitle( tr("About Lin-guider") );
	
	QString str = ui.label_Info->text();
	str.replace( "*VERSION*", QString(CPY_RIGHT(VERSION)) );
	ui.label_Info->setText( str );

	// connect controls
	connect( ui.pushButton_Ok, SIGNAL( released() ), this, SLOT( onOk_ButtonPressed() ) );
}

about::~about()
{
}


void about::showEvent( QShowEvent * event )
{
	if( event->spontaneous() )
		return;

	ui.tabWidget_Pages->setCurrentIndex( 0 );
}


void about::onOk_ButtonPressed()
{
	close();
}

