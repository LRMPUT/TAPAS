/*
 * QtGps.cpp
 *
 *  Created on: 15-07-2013
 *      Author: jachu
 */

//OpenCV
#include <opencv2/opencv.hpp>
//TrobotQt
#include "QtGps.h"

using namespace std;
using namespace cv;

QtGps::QtGps(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug) :
				ui(iui),
				robot(irobot),
				debug(idebug)
{
	QObject::connect(ui->gpsConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->gpsDisconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
	QObject::connect(&(this->refreshTimer), SIGNAL(timeout()), this, SLOT(refresh()));
	QObject::connect(ui->gpsZeroPointButton, SIGNAL(clicked()), this, SLOT(setZeroPoint()));
	refreshTimer.setInterval(500);
}

QtGps::~QtGps(){
	disconnect();
}

void QtGps::connect(){
	if(ui->gpsPortCombo->count() != 0){
		robot->openGps(ui->gpsPortCombo->currentText().toAscii().data());
		refreshTimer.start();
	}
}

void QtGps::disconnect(){
	refreshTimer.stop();
	robot->closeGps();
}

void QtGps::refresh(){
	const Mat tmp = debug->getGpsData();
	ui->gpsXLabel->setText(QString("%1").arg(tmp.at<double>(0)));
	ui->gpsYLabel->setText(QString("%1").arg(tmp.at<double>(1)));
	ui->gpsFixLabel->setText(QString("%1").arg(debug->getGpsFixStatus()));
	ui->gpsSatelitesLabel->setText(QString("%1").arg(debug->getGpsSatelitesUsed()));
}

void QtGps::setZeroPoint(){
	const Mat tmp = debug->getGpsData();
	debug->setGpsZeroPoint(tmp.at<double>(2), tmp.at<double>(3));
}
