/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology

    This library is free software; you can redistribute it and/or modify it under
    the terms of the GNU Lesser General Public License as published by the
    Free Software Foundation; either version 2.1 of the License, or (at your option)
    any later version.

    This library is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
    details.

    You should have received a copy of the GNU Lesser General Public License along
    with this library; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

//OpenCV
#include <opencv2/opencv.hpp>
//TrobotQt
#include "QtGps.h"

using namespace std;
using namespace cv;

QtGps::QtGps(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug) :
				ui(iui),
				robot(irobot),
				debug(idebug)
{
	QObject::connect(ui->gpsConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->gpsDisconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
	QObject::connect(&(this->refreshTimer), SIGNAL(timeout()), this, SLOT(refresh()));
	QObject::connect(ui->gpsZeroPointButton, SIGNAL(clicked()), this, SLOT(setZeroPoint()));
	refreshTimer.setInterval(500);
	refreshTimer.start();
}

QtGps::~QtGps(){
	disconnect();
}

void QtGps::connect(){
	if(ui->gpsPortCombo->count() != 0){
		robot->openGps(ui->gpsPortCombo->currentText().toAscii().data());
	}
}

void QtGps::disconnect(){
	robot->closeGps();
}

void QtGps::refresh(){
	if(robot->isGpsOpen()){
		const Mat tmp = debug->getGpsData();
		ui->gpsXLabel->setText(QString("%1").arg(tmp.at<float>(0)));
		ui->gpsYLabel->setText(QString("%1").arg(tmp.at<float>(1)));
		ui->gpsFixLabel->setText(QString("%1").arg(debug->getGpsFixStatus()));
		ui->gpsSatelitesLabel->setText(QString("%1").arg(debug->getGpsSatelitesUsed()));
	}
}

void QtGps::setZeroPoint(){
	const Mat tmp = debug->getGpsData();
	debug->setGpsZeroPoint(tmp.at<float>(2), tmp.at<float>(3));
}
