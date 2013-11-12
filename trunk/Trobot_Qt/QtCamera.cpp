/*
 * QtCamera.cpp
 *
 *  Created on: 12-11-2013
 *      Author: jachu
 */

#include "QtCamera.h"

QtCamera::QtCamera(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug) :
	ui(iui),
	robot(irobot),
	debug(idebug)
{
	QObject::connect(ui->cameraConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->cameraDisonnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
	QObject::connect(&refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
}

QtCamera::~QtCamera(){

}

cv::Mat QtCamera::getFrame(){

}

void QtCamera::refresh(){

}

void QtCamera::connect(){

}

void QtCamera::disconnect(){

}
